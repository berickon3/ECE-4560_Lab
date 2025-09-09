# read_position.py

import time
from so101_utils import load_calibration, setup_motors

# CONFIGURATION VARIABLES
PORT_ID = "/dev/tty.usbmodem5A460848371" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1"

# Setup calibration and motor bus
calibration = load_calibration(ROBOT_NAME)
bus = setup_motors(calibration, PORT_ID)

# Disable motors so that you can move them freely
bus.disable_torque()

# PRINT POSITIONS CONSTANTLY
while True:
    present_pos = bus.sync_read("Present_Position")
    print(present_pos)
    time.sleep(0.02)  # 50 Hz loop
