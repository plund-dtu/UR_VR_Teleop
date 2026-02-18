import rtde_control, rtde_receive
import numpy as np
import openvr
from scipy.spatial.transform import Rotation as R
import robotiq_gripper

class URTeleop:
    def __init__(self, IP: str):
        self.ROBOT_HOME_POSITION = [np.pi/2.0, 
                                    -np.pi/1.65, 
                                    np.pi/1.65,
                                    -np.pi/2.0,
                                    -np.pi/2.0,
                                    np.pi]

        # Connect to robot
        rtde_c, rtde_r = self.connect_robot_rtde(IP=IP)
        self.rtde_c = rtde_c
        self.rtde_r = rtde_r

        # Connect to gripper
        self.gripper = self.connect_robotiq_gripper(IP=IP)
        self.grip_min_pos = self.gripper.get_min_position()
        self.grip_max_pos = self.gripper.get_max_position()

        # Connect to Meta Quest
        self.vr_system = self.connect_meta_quest()

        # Move robot home
        self.move_robot_home()
        robot_home_pose = self.rtde_r.getActualTCPPose()
        self.T_robot_t0 = robot_home_pose[:3]
        self.R_robot_t0 = R.from_rotvec(robot_home_pose[3:]).as_matrix()

        # Teleop state utils
        self.first_controller_reading = True
        self.gripper_open = True
        self.T_right_t0 = None
        self.R_right_t0 = None

    def run_teleop(self):
        """
        Run teleoperation controller in a pause/unpause compliant closed loop with no data collection pipeline. 
        The teleop starts in pause, press X to unpause and press Y to pause again.
        """
        pause = True

        try:
            while True:
                # Read states of right VR controller
                right_hand, left_hand = self.get_vr_controller_readings()
                left_hand_button = left_hand["button"]
                left_y_button_active = left_hand_button == 2
                left_x_button_active = left_hand_button == 128

                # Pause the robot when 
                if left_y_button_active:
                    pause = True

                # Unpause robot and reset to current controller and robot position
                if left_x_button_active:
                    if pause:
                        self.reset_teleop_state()
                    pause = False

                # Step teleop controller if not paused
                if not pause:
                    self.teleop_step(right_hand["transform"], right_hand["trigger"])

        except KeyboardInterrupt as e:
            self.rtde_c.servoStop()

    def reset_teleop_state(self):
        """
        Utility function used for unpausing the robot. Resets controller and robot start poses to align with current poses.
        """
        robot_home_pose = self.rtde_r.getActualTCPPose()
        self.T_robot_t0 = robot_home_pose[:3]
        self.R_robot_t0 = R.from_rotvec(robot_home_pose[3:]).as_matrix()
        self.first_controller_reading = True

    def teleop_step(self, transform, trigger):
        """
        Step the teleoperation controller once towards the input transform (array of [3,4]) and opens or closes the gripper based on trigger value
        """
        right_tf = transform

        # Compute world XYZ translations for controller (Meta Quest uses a coordinate frame like a camera, therefore, z=y and y=z in a right handed coordinate frame, so we rotate to flip axis)
        T_right = np.array([right_tf[0][3], right_tf[1][3], right_tf[2][3]])
        T_right = np.array([[1,  0,  0], [0,  0, -1], [0,  1,  0]]) @ T_right

        # Extract rotation matrix for controller
        R_right = np.array([[right_tf[0][0], right_tf[0][1], right_tf[0][2]],
                            [right_tf[1][0], right_tf[1][1], right_tf[1][2]],
                            [right_tf[2][0], right_tf[2][1], right_tf[2][2]]])

        # If first reading, zero controller pose to make movement relative to current pose
        if self.first_controller_reading:
            self.T_right_t0 = np.array(T_right)
            self.R_right_t0 = R_right
            self.first_controller_reading = False

        # Get current controller position relative to starting position
        T_controller = np.array(T_right) - self.T_right_t0

        # Get current controller orientation relative to starting position and transform to robot end-effector frame
        delta_R_controller = R_right @ self.R_right_t0.T
        axis_remap = self.rotmat("x", -90) # Flip Z and Y axis
        delta_R_controller_axis_swap = axis_remap @ delta_R_controller @ axis_remap.T
        axis_remap_2 = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]) # Flip y rotations
        delta_R_controller_axis_swap = axis_remap_2 @ delta_R_controller_axis_swap @ axis_remap_2.T
        axis_remap_3 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]) # Flip z rotations
        delta_R_controller_axis_swap = axis_remap_3 @ delta_R_controller_axis_swap @ axis_remap_3.T

        # Get rotvec target and cartesian target and combine for servo controller
        R_target = delta_R_controller_axis_swap @ self.R_robot_t0 # Axis aligned rotations of controller transferred to robot end effector
        R_target = R.from_matrix(R_target).as_rotvec()
        T_target = self.T_robot_t0 + T_controller
        target = np.zeros(6)
        target[0:3] = T_target
        target[3:] = R_target

        # Run servo controller to target
        self.robot_servo_target(target)

        # Action for Robotiq gripper
        right_trigger_value = trigger
        right_trigger_active = right_trigger_value > 0.8
        if right_trigger_active and self.gripper_open: # If trigger is pulled and the gripper is opened, the gripper will close
            self.gripper.move(self.grip_max_pos, 100, 0)
            self.gripper_open = False
        if not right_trigger_active and not self.gripper_open: # If the trigger is not pulled and the gripper is closed, the gripper will open
            self.gripper.move(self.grip_min_pos, 100, 0)
            self.gripper_open = True

    def robot_servo_target(self, target, velocity: float = 0.15, acceleration: float = 0.25, timestep: float = 1.0/50.0, lookahead_time: float = 0.1, gain: int = 150):
        """
        Servos the robot linearly towards a cartesian target for a timestep
        """
        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoL(target, velocity, acceleration, timestep, lookahead_time, gain) # Pose, velocity, acceleration, timestep, lookahead time, gain
        self.rtde_c.waitPeriod(t_start)

    def get_vr_controller_readings(self):
        """
        Returns readings from both handheld Meta Quest controllers, 0 is left hand and 1 is right hand
        """
        # Get controller position
        poses = openvr.VRCompositor().waitGetPoses([], None)[0]
        
        right_hand = {}
        left_hand = {}
        
        for device_index in range(openvr.k_unMaxTrackedDeviceCount): # Get devices
            pose = poses[device_index]
            
            if pose.bPoseIsValid: # Check if pose is valid
                device_class = self.vr_system.getTrackedDeviceClass(device_index)

                if device_class == openvr.TrackedDeviceClass_Controller: # Check if valid pose is from a controller object
                    # Check if left (1) or right (2) controller
                    role = self.vr_system.getControllerRoleForTrackedDeviceIndex(device_index)
                    if role == openvr.TrackedControllerRole_LeftHand:
                        hand = "Left"
                    elif role == openvr.TrackedControllerRole_RightHand:
                        hand = "Right"
                    else: 
                        hand = "Unknown"

                    # Extract pose matrix
                    tf = pose.mDeviceToAbsoluteTracking

                    # Get controller trigger and button state
                    _, state = self.vr_system.getControllerState(device_index)
                    trigger = state.rAxis[1].x
                    button = state.ulButtonPressed

                    if hand == "Right":
                        # Populate dict for controller
                        right_hand["transform"] = tf
                        right_hand["trigger"] = trigger
                        right_hand["button"] = button

                    if hand == "Left":
                        # Populate dict for controller
                        left_hand["transform"] = tf
                        left_hand["trigger"] = trigger
                        left_hand["button"] = button

        return right_hand, left_hand

    def move_robot_home(self):
        """
        Utility function to move the robot to a hard-coded home pose defined in joint angles
        """
        home_q = self.ROBOT_HOME_POSITION
        self.rtde_c.moveJ(home_q, 1.05, 1.4)

    def rotmat(self, axis: str, theta: float, degree: bool = True) -> np.array:
        """
        Returns a 3D rotation matrix around either x, y or z, at an angle of theta which can be either a degree or radian
        """
        assert axis == "x" or axis == "y" or axis == "z", 'Axis must be either "x", "y", or "z"!'

        if degree:
            theta = theta*np.pi/180.0

        if axis == "x":
            return np.array([[1.0, 0.0, 0.0],
                            [0.0, np.cos(theta), -np.sin(theta)],
                            [0.0, np.sin(theta), np.cos(theta)]])

        if axis == "y":
            return np.array([[np.cos(theta), 0.0, np.sin(theta)],
                            [0.0, 1.0, 0.0],
                            [-np.sin(theta), 0.0, np.cos(theta)]])

        if axis == "z":
            return np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta), np.cos(theta), 0.0],
                            [0.0, 0.0, 1.0]])
        
    def connect_robot_rtde(self, IP: str):
        """
        Returns a control and receive interface for a UR robot at a given IP address
        """
        rtde_controller = rtde_control.RTDEControlInterface(IP)
        rtde_receiver = rtde_receive.RTDEReceiveInterface(IP)
        return rtde_controller, rtde_receiver
    
    def connect_robotiq_gripper(self, IP: str):
        """
        Returns a Robotiq gripper object for controlling the gripper through ur_rtde
        """
        gripper = robotiq_gripper.RobotiqGripper()
        gripper.connect(IP, 63352)
        gripper.activate()
        return gripper

    def connect_meta_quest(self):
        """
        Returns a VR System object from the OpenVR API - make sure that Meta Quest Link is established and SteamVR is running
        """
        try:
            openvr.init(openvr.VRApplication_Scene)
            vr_system = openvr.VRSystem()
        except Exception as e:
            print("Failed to connect to Meta Quest: Make sure Meta Quest Link is established and SteamVR is running.")
        return vr_system