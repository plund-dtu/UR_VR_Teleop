from ur_teleop_utils import URTeleop
import pyrealsense2 as rs
from multiprocessing import Process, Event, set_start_method
from multiprocessing import shared_memory
import numpy as np
import time
import datetime

# Logging parameters
LOG_FREQUENCY = 15
LOG_INTERVAL = 1.0 / LOG_FREQUENCY
last_log_time = time.perf_counter()
episode_save_path = r""
episode_task = ""
log_data_format = {
                   "joint_states": [],
                   "gripper_state": [],
                   "eef_poses": [],
                   "rgb_images": [],
                   "depth_images": [],
                   "forces": []
                  }

assert episode_save_path != "", "Episode data save path not defined!"
assert episode_task != "", "Episode task is not defined!"

# Robot parameters
ROBOT_IP = ""
assert ROBOT_IP != "", "Robot IP is not defined!"

# Camera parameters
resolution = (640, 480)
fps = 30

# Construct multiprocess workers to ensure high frequency of data stream without introducing latency to teleoperation controller
def realsense_worker(resolution, fps, serial, rgb_name, depth_name, ready_event):
    # Specific imports for individual image collection processes
    import pyrealsense2 as rs
    import numpy as np
    from multiprocessing import shared_memory

    # Load shared memory buffers
    shm_rgb = shared_memory.SharedMemory(name=rgb_name)
    shm_depth = shared_memory.SharedMemory(name=depth_name)
    rgb = np.ndarray((resolution[1], resolution[0], 3), dtype=np.uint8, buffer=shm_rgb.buf)
    depth = np.ndarray((resolution[1], resolution[0]), dtype=np.uint16, buffer=shm_depth.buf)

    # Initialize RealSense pipeline for camera defined by specific serial
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)

    pipeline.start(config)
    align = rs.align(rs.stream.color)

    print(f"RealSense worker {serial} initialized!")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            rgb[:] = np.asanyarray(color_frame.get_data())
            depth[:] = np.asanyarray(depth_frame.get_data())

            ready_event.set()

    except Exception as e:
        print(f"RealSense worker {serial} error:", e)
    finally:
        # Clean shutdown
        pipeline.stop()
        shm_rgb.close()
        shm_depth.close()

# Main control loop
if __name__ == "__main__": # This is important when running multiprocess scripts on Windows
    # Instantiate teleop controller
    teleop = URTeleop(IP=ROBOT_IP)

    # Configure RealSense image buffering in shared memory
    set_start_method("spawn", force=True)
    ctx = rs.context() # Detect connected RealSense cameras
    serials = [ctx.devices[i].get_info(rs.camera_info.serial_number) for i in range(len(ctx.devices))] # Extract serial number for each camera (should load in the same order each time)
    processes = []
    shms_rgb = []
    shms_depth = []
    events = []
    rgb_arrays = []
    depth_arrays = []

    # Create shared memory buffer for each camera and intialize multi processing
    for serial in serials:
        shm_rgb = shared_memory.SharedMemory(create=True, size=resolution[0] * resolution[1] * 3)
        shm_depth = shared_memory.SharedMemory(create=True, size=resolution[0] * resolution[1] * 2)

        rgb = np.ndarray((resolution[1], resolution[0], 3), dtype=np.uint8, buffer=shm_rgb.buf)
        depth = np.ndarray((resolution[1], resolution[0]), dtype=np.uint16, buffer=shm_depth.buf)

        event = Event()

        p = Process(target=realsense_worker, args=(resolution, fps, serial, shm_rgb.name, shm_depth.name, event))
        p.start()

        processes.append(p)
        shms_rgb.append(shm_rgb)
        shms_depth.append(shm_depth)
        events.append(event)
        rgb_arrays.append(rgb)
        depth_arrays.append(depth)

    # Utilities for teleoperation controller
    pause = True
    episode_saved = False
    episode_data = log_data_format

    # Main control loop
    print("Controller started: Wait for RealSense workers to start, then press X to unpause robot!")
    try:
        while True:
            current_time = time.perf_counter()

            rgb_images = []
            depth_images = []

            for i in range(len(serials)):
                if events[i].is_set():
                    events[i].clear()

                    rgb_images.append(rgb_arrays[i].copy())
                    depth_images.append(depth_arrays[i].copy())

            # Read states of right VR controller
            right_hand, left_hand = teleop.get_vr_controller_readings()
            left_hand_button = left_hand["button"]
            left_x_button_active = left_hand_button == 128
            left_y_button_active = left_hand_button == 2

            # Pause the robot when 
            if left_y_button_active:
                pause = True

            # Unpause robot and reset to current controller and robot position
            if left_x_button_active:
                if pause:
                    teleop.reset_teleop_state()
                pause = False

            # Step teleop controller and log data if not paused
            if not pause:
                teleop.teleop_step(right_hand["transform"], right_hand["trigger"])
                # Read states as often as possible to reduce "RTDEReceiveInterface boost system Exception: (asio.misc:2) End of file" full buffer issues, but log at lower frequency
                joint_state = teleop.rtde_r.getActualQ()
                tcp_pose = teleop.rtde_r.getActualTCPPose()
                tcp_forces = teleop.rtde_r.getActualTCPForce()

                if current_time - last_log_time >= LOG_INTERVAL:
                    last_log_time = current_time
                    episode_data["joint_states"].append(joint_state)
                    episode_data["gripper_state"].append(float(teleop.gripper_open))
                    episode_data["eef_poses"].append(tcp_pose)
                    episode_data["rgb_images"].append(rgb_images)
                    episode_data["depth_images"].append(depth_images)
                    episode_data["forces"].append(tcp_forces)

            # Check if episode should be saved or environment should be reset
            right_hand_button = right_hand["button"]
            right_hand_a_button = right_hand_button == 128
            right_hand_b_button = right_hand_button == 2
            if right_hand_a_button and not episode_saved: # Save if A button is pressed an episode has not already been saved
                datetime_tag = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
                task_string = episode_task.replace(" ", "")
                save_string = episode_save_path + f'{task_string}_{datetime_tag}'
                print(f"Saving episode data as '{save_string}'")
                np.save(save_string, episode_data)
                print("Done!")
                episode_saved = True # To ensure save process only runs once
            
            if right_hand_b_button: # Reset environment if B button is pressed
                rgb_images = []
                depth_images = []
                episode_saved = False
                episode_data = log_data_format
                teleop.rtde_c.servoStop()
                time.sleep(0.5)
                print("Moving robot to home position...")
                teleop.move_robot_home()
                pause = True
                teleop.gripper.move(teleop.grip_min_pos, 100, 0)
                print("Controller started: press X to unpause robot!")

    finally:
        # Clean shutdown
        for p in processes:
            p.terminate()
            p.join()

        for shm in shms_rgb:
            shm.close()
            shm.unlink()

        for shm in shms_depth:
            shm.close()
            shm.unlink()

        teleop.rtde_c.servoStop()

