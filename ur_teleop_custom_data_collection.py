from ur_teleop_utils import URTeleop

ROBOT_IP = ""
assert ROBOT_IP != "", "Robot IP is not defined!"
teleop = URTeleop(IP=ROBOT_IP)

if __name__ == "__main__":
    pause = True
    try:
        while True:
            print("Starting teleop controller, press X to unpause!")
            # Read states of right VR controller
            right_hand, left_hand = teleop.get_vr_controller_readings()
            left_hand_button = left_hand["button"]
            left_y_button_active = left_hand_button == 2
            left_x_button_active = left_hand_button == 128

            # Pause the robot when 
            if left_y_button_active:
                pause = True

            # Unpause robot and reset to current controller and robot position
            if left_x_button_active:
                if pause:
                    teleop.reset_teleop_state()
                pause = False

            # Step teleop controller if not paused
            if not pause:
                teleop.teleop_step(right_hand["transform"], right_hand["trigger"])

            """
            ADD DATA COLLECTION PIPELINE HERE
             - Create a timer to log data at desired frequency
             - Pull images from cameras
             - Pull data from sensors or robot
             - Convert to LeRobot/RLDS/etc.
            """

    except KeyboardInterrupt as e:
        teleop.rtde_c.servoStop()