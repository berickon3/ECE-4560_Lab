import mujoco
import mujoco.viewer
from so101_mujoco_utils import set_initial_pose, move_to_pose, hold_position
import os

model_path = os.path.join(os.path.dirname(__file__), 'model', 'scene.xml')
m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

starting_position = {
    'shoulder_pan': 0.0,   # in degrees
    'shoulder_lift': -90.0,
    'elbow_flex': 90.0,
    'wrist_flex': 45.0,
    'wrist_roll': 0.0,
    'gripper': 0.0           # 0-100 range
}

desired_position = {
    'shoulder_pan': 0.0,   # in degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0           # 0-100 range
}
with mujoco.viewer.launch_passive(m, d) as viewer:

  # Go to desired position
  set_initial_pose(d, starting_position)
  move_to_pose(m, d, viewer, desired_position, 2.0)
  
  
  # Hold Position
  hold_position(m, d, viewer, 2.0)

  # Return to starting position
  move_to_pose(m, d, viewer, starting_position, 2.0)  