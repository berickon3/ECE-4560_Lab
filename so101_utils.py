# so101-utils.py
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from pathlib import Path
import draccus
import time

from so101_inverse_kinematics import get_inverse_kinematics

def offset_config(config):
    offset_dict = config.copy()
    offset_dict['shoulder_pan'] += 0.0
    offset_dict['shoulder_lift'] += 12 # Example offset of 2 degrees
    offset_dict['elbow_flex'] -= 3 # Example offset of -4 degrees
    offset_dict['wrist_flex'] -= 3
    offset_dict['wrist_roll'] += 0.0
    offset_dict['gripper'] += 0.0
    return offset_dict


def load_calibration(ROBOT_NAME) -> None:
    """
    Helper to load calibration data from the specified file.

    Args:
        fpath (Path | None): Optional path to the calibration file.
    """
    fpath = Path(f'Robot Calibration Files/{ROBOT_NAME}.json')
    with open(fpath) as f, draccus.config_type("json"):
        calibration = draccus.load(dict[str, MotorCalibration], f)
        return calibration

def setup_motors(calibration, PORT_ID):
    norm_mode_body = MotorNormMode.DEGREES
    bus = FeetechMotorsBus(
        port=PORT_ID,
        motors={
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        calibration=calibration,
    )
    bus.connect(True)

    with bus.torque_disabled():
        bus.configure_motors()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            bus.write("P_Coefficient", motor, 20)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            bus.write("I_Coefficient", motor, 0)
            bus.write("D_Coefficient", motor, 5)
    return bus

def move_to_pose(bus, desired_position, duration, step_alpha: float = 0.01):
    """
    Move the arm from current position to `desired_position` over `duration` seconds.

    Instead of computing alpha from elapsed time, this function increments alpha by
    `step_alpha` each loop. Default `step_alpha=0.02` results in 50 steps (0.0..0.98)
    plus a final command to set the exact target pose. The sleep between steps is
    computed so the total time spent is approximately `duration`.

    Args:
        bus: motor bus instance
        desired_position: dict of joint target values
        duration: total seconds to move
        step_alpha: incremental alpha per loop (0 < step_alpha <= 1). Typical 0.02.
    """
    # Quick sanity checks / early-exit
    if duration <= 0:
        # immediate set
        bus.sync_write("Goal_Position", offset_config(desired_position), normalize=True)
        return

    # read current pose and prepare target
    starting_pose = bus.sync_read("Present_Position")
    calibrated_position = offset_config(desired_position)

    # sanitize step_alpha
    try:
        step_alpha = float(step_alpha)
    except Exception:
        step_alpha = 0.02
    if step_alpha <= 0 or step_alpha > 1:
        step_alpha = 0.02

    # compute number of steps and per-step sleep so total duration is honored
    num_steps = max(1, int(round(1.0 / step_alpha)))
    sleep_per_step = duration / num_steps

    alpha = 0.0
    for _ in range(num_steps):
        # Interpolate each joint for current alpha
        position_dict = {}
        for joint in calibrated_position:
            p0 = starting_pose[joint]
            pf = calibrated_position[joint]
            position_dict[joint] = (1 - alpha) * p0 + alpha * pf

        # Send command
        bus.sync_write("Goal_Position", position_dict, normalize=True)

        time.sleep(sleep_per_step)
        alpha += step_alpha

    # Ensure final exact target is sent (alpha might not hit 1.0 exactly)
    bus.sync_write("Goal_Position", calibrated_position, normalize=True)
    
def hold_position(bus, duration):
    current_pos = bus.sync_read("Present_Position")
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        bus.sync_write("Goal_Position", current_pos, normalize=True)
        time.sleep(0.02)  # 50 Hz loop


def pick_up_block(bus, block_position, move_to_duration):
    
    # Move above block with gripper open
    block_raised = block_position.copy()
    block_raised[2] += 0.03  # raise block height amount
    block_configuration_raised = get_inverse_kinematics(block_raised)
    block_configuration_raised['gripper'] = 50
    move_to_pose(bus, block_configuration_raised, move_to_duration)
    
    
    # Move down to block with gripper open
    block_configuration = get_inverse_kinematics(block_position)
    block_configuration['gripper'] = 50
    move_to_pose(bus, block_configuration, 2.0)
    
    # Close gripper
    block_configuration_closed = block_configuration.copy()
    block_configuration_closed['gripper'] = 5
    move_to_pose(bus, block_configuration_closed, 2.0)

    # Lift up again
    block_configuration_raised['gripper'] = 5
    move_to_pose(bus, block_configuration_raised, 2.0)

    return bus

def place_block(bus, target_position, move_to_duration):
    
    # Move above target with gripper closed
    block_raised = target_position.copy()
    block_raised[2] += 0.03  # raise 1 inch
    block_configuration_raised = get_inverse_kinematics(block_raised)
    block_configuration_raised['gripper'] = 5
    move_to_pose(bus, block_configuration_raised, move_to_duration)
    
    # Move down to block
    block_configuration = get_inverse_kinematics(target_position)
    block_configuration['gripper'] = 5
    move_to_pose(bus, block_configuration, 1.0)
    
    # Open gripper 
    block_configuration_open = block_configuration.copy()
    block_configuration_open['gripper'] = 50
    move_to_pose(bus, block_configuration_open, 1.0)
    
    # Return to raised position
    block_configuration_raised['gripper'] = 50
    move_to_pose(bus, block_configuration_raised, 1.0)

    return bus
