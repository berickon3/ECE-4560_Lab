from so101_utils import load_calibration, setup_motors
# CONFIGURATION VARIABLES
PORT_ID = "COM7" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "Jack_the_Gripper" # REPLACE WITH YOUR ROBOT NAME! 
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)
bus.disable_torque()