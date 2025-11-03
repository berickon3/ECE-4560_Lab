# so101-utils.py
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from pathlib import Path
import draccus
import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
            bus.write("P_Coefficient", motor, 10)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            bus.write("I_Coefficient", motor, 0)
            bus.write("D_Coefficient", motor, 32)
    return bus

def move_to_pose_stepped(bus, desired_position, duration, step_alpha: float = 0.01):
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
    
def move_to_pose(bus, desired_position, duration, logging=False):
    start_time = time.time()
    starting_pose = bus.sync_read("Present_Position")
    offset_dict = offset_config(desired_position)
    if logging:
        # Initialize log fields
        times = []
        targets = []
        actuals = []
        joint_names = list(desired_position.keys())
    
    while True:
        t = time.time() - start_time
        if t > duration:
            break

        # Interpolation factor [0,1] (make sure it doesn't exceed 1)
        alpha = min(t / duration, 1)

        # Interpolate each joint
        position_dict = {}
        for joint in offset_dict:
            p0 = starting_pose[joint]
            pf = offset_dict[joint]
            position_dict[joint] = (1 - alpha) * p0 + alpha * pf
            
        # Store Actual Position
        present_pos = bus.sync_read("Present_Position")
        
        # Send command
        bus.sync_write("Goal_Position", position_dict, normalize=True)
        
        if logging:
            # Log time, target, and actual
            times.append(time.time())
            targets.append([position_dict[j] for j in joint_names])
            actuals.append([present_pos[j] for j in joint_names])
        
        time.sleep(0.02)  # 50 Hz loop
    if logging:
        # Convert to DataFrame
        df = pd.DataFrame({
            "time": times,
            **{f"target_{j}": [t[i] for t in targets] for i, j in enumerate(joint_names)},
            **{f"actual_{j}": [a[i] for a in actuals] for i, j in enumerate(joint_names)},
        })
        return df
    else:
        return None

def hold_position(bus, duration):
    current_pos = bus.sync_read("Present_Position")
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t > duration:
            break
        bus.sync_write("Goal_Position", current_pos, normalize=True)
        time.sleep(0.02)  # 50 Hz loop

def pick_up_block(bus, block_position, move_to_duration, logging=False):
    # Calculate all configurations
    
    ## Move above block with gripper open
    block_raised = block_position.copy()
    block_raised[2] += 0.03  # raise block height amount
    block_configuration_raised_initial = get_inverse_kinematics(block_raised)
    block_configuration_raised_initial['gripper'] = 50

    ## Move down to block with gripper open
    block_configuration = get_inverse_kinematics(block_position)
    block_configuration['gripper'] = 50
    
    ## Close gripper
    block_configuration_closed = block_configuration.copy()
    block_configuration_closed['gripper'] = 5
    
    ## Lift up again
    block_configuration_raised_final = get_inverse_kinematics(block_raised)
    block_configuration_raised_final['gripper'] = 5

    alldata = pd.DataFrame()
    
    df = move_to_pose(bus, block_configuration_raised_initial, move_to_duration, logging=logging)
    alldata = pd.concat([alldata, df], ignore_index=True)
    df = move_to_pose(bus, block_configuration, move_to_duration, logging=logging)
    alldata = pd.concat([alldata, df], ignore_index=True)
    df = move_to_pose(bus, block_configuration_closed, move_to_duration, logging=logging)
    alldata = pd.concat([alldata, df], ignore_index=True)
    df = move_to_pose(bus, block_configuration_raised_final, move_to_duration, logging=logging)
    alldata = pd.concat([alldata, df], ignore_index=True)

    if logging:
        for df in alldata:
            plot_data(df)
    
    return bus

def place_block(bus, target_position, move_to_duration):
    
    # Move above target with gripper closed
    block_raised = target_position.copy()
    block_raised[2] += 0.03  # raise 1 inch
    block_configuration_raised_initial = get_inverse_kinematics(block_raised)
    block_configuration_raised_initial['gripper'] = 5

    
    # Move down to block
    block_configuration = get_inverse_kinematics(target_position)
    block_configuration['gripper'] = 5

    
    # Open gripper 
    block_configuration_open = block_configuration.copy()
    block_configuration_open['gripper'] = 50

    
    # Return to raised position
    block_configuration_raised_final = block_configuration_raised_initial.copy()
    block_configuration_raised_final['gripper'] = 50

    
    move_to_pose(bus, block_configuration_raised_initial, move_to_duration)
    move_to_pose(bus, block_configuration, move_to_duration)
    move_to_pose(bus, block_configuration_open, move_to_duration)
    move_to_pose(bus, block_configuration_raised_final, move_to_duration)

    return bus

def plot_data(df):

    zero_configuration = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.00,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0          
    }
    joint_limits = {
        'shoulder_pan': (-1.919, 1.919),
        'shoulder_lift': (-1.74, 1.74),
        'elbow_flex': (-1.69, 1.69),
        'wrist_flex': (-1.65, 1.65),
        'wrist_roll': (-2.74, 2.84),
        'gripper': (0, 100)
    }
    joint_names = [j for j in zero_configuration.keys()]
    joint_limits = {j: (np.degrees(lim[0]), np.degrees(lim[1])) for j, lim in joint_limits.items()}

    # Plot target vs actual for each joint
    plt.figure(figsize=(10, 2 * len(joint_names)))
    for i, j in enumerate(joint_names):
        plt.subplot(len(joint_names), 1, i + 1)
        plt.plot(df["time"], df[f"target_{j}"], label=f"Target {j}")
        plt.plot(df["time"], df[f"actual_{j}"], label=f"Actual {j}", linestyle="--")
        plt.ylabel("Position")
        plt.legend(loc="upper right")
        plt.ylim(joint_limits[f"{j}"][0], joint_limits[f"{j}"][1])
    plt.xlabel("Time (s)")
    plt.suptitle("Target vs Actual Joint Positions")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()