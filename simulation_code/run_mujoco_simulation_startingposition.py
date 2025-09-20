import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, send_position_command
import os

model_path = os.path.join(os.path.dirname(__file__), 'model', 'scene.xml')
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

starting_position = {
    'shoulder_pan': 0.0,   # in degrees
    'shoulder_lift': -180.0,
    'elbow_flex': 180.0,
    'wrist_flex': 45.0,
    'wrist_roll': 0.0,
    'gripper': 0.0           # 0-100 range
}
set_initial_pose(d, starting_position)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    send_position_command(d, starting_position)

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
