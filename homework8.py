from so101_utils import load_calibration, move_to_pose, setup_motors, pick_up_block, place_block

# CONFIGURATION VARIABLES
PORT_ID = "COM4" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "Jack_the_Gripper" # REPLACE WITH YOUR ROBOT NAME! 

# --- Specified Parameters ---
move_time = 0.5  # seconds to reach desired position

# ------------------------
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)
starting_pose = bus.sync_read("Present_Position")
pick_up_block(bus, [0.2, 0.1, 0.025], move_time, False)
place_block(bus, [0.2, -0.1, 0], move_time, False)

pick_up_block(bus, [0.2, 0.1, 0], move_time, False)
place_block(bus, [0.2, -0.1, 0.025], move_time, False)
move_to_pose(bus, starting_pose, move_time)
bus.disable_torque()