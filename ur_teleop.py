from ur_teleop_utils import URTeleop

ROBOT_IP = ""
assert ROBOT_IP != "", "Robot IP is not defined!"
teleop = URTeleop(IP=ROBOT_IP)

print("Starting teleop controller, press X to unpause!")
teleop.run_teleop()