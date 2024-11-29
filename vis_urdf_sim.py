import numpy as np

import sapien.core as sapien
from sapien.utils.viewer import Viewer
from sapien.asset import create_dome_envmap

def create_env(urdf_path):
  # Create a SAPIEN Engine instance
  engine = sapien.Engine()

  # Create a renderer instance (optional)
  renderer = sapien.SapienRenderer(offscreen_only=False)
  engine.set_renderer(renderer)
  scene_config = sapien.SceneConfig()
  scene = engine.create_scene(scene_config)
  scene.set_timestep(1 / 240)
  scene.set_environment_map(create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2]))
  scene.add_directional_light([-1, 0.5, -1], color=[2.0, 2.0, 2.0], shadow=True, shadow_map_size=4096)

  # Create a URDF loader
  urdf_loader = scene.create_urdf_loader()

  # Load the URDF file
  robot = urdf_loader.load(urdf_path)

  # Set initial pose for the robot
  initial_pose = sapien.Pose([0, 0, 0])
  robot.set_pose(initial_pose)

  cam = scene.add_camera(name="Cheese!", width=1280, height=720, fovy=1, near=0.1, far=10)
  cam.set_local_pose(sapien.Pose([0.413487, 0.0653831, 0.1111697], [0.088142, -0.0298786, -0.00264502, -0.995656]))

  viewer = Viewer(renderer)
  viewer.set_scene(scene)
  viewer.focus_camera(cam)

  return robot, viewer

if __name__ == "__main__":
  urdf_path = "assets/jakamini_leaphand.urdf"
  robot, viewer = create_env(urdf_path)

  arm_dof = np.array([-1.5707487,   0.24192421, -1.4037328,   0.02739489, -1.8208425,  -2.1729174])
  hand_dof = np.array([-0.01073527,  0.02301216,  0.00613856,  0.00000262,  0.00000262,  0.02147818,
    0.00613856,  0.23470163, -0.00613332,  0.02301216,  0.00307059,  0.00000262,
    0.02301216, -0.00306535, -0.00306535, -0.00153136])
  robot_dof = np.concatenate([arm_dof, hand_dof])

  while 1:
      # Keep the window open (if renderer is used)
      robot.set_qpos(robot_dof)
      viewer.render()