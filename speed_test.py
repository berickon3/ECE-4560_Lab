from so101_utils import load_calibration, setup_motors, offset_config, move_to_pose
import time
# CONFIGURATION VARIABLES
PORT_ID = "COM4" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "Jack_the_Gripper" # REPLACE WITH YOUR ROBOT NAME! 

desired_position = {
    'shoulder_pan': 0.0,   # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 50.0           # 0-100 range
}

calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)
starting_pose = bus.sync_read("Present_Position")
offset_dict = offset_config(desired_position)
bus.sync_write("Goal_Position", offset_dict, normalize=True)
threshold = 0.17  
while True:
    positions = bus.sync_read("Present_Position")
    error = {motor: abs(positions[motor] - offset_dict[motor]) for motor in bus.motors}
    print(error)
    if all(e < threshold for e in error.values()):
        break
    time.sleep(0.1)
move_to_pose(bus, starting_pose, 2.0)
bus.disable_torque()