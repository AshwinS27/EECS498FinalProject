import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name, get_aabb, get_aabbs
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
from robot import Robot

# starts demo
def main():
    # Intialize PyBullet
    connect(use_gui=True)

    # Intialize the Robot
    robot = Robot()
    robot.load_world('pr2doorway.json')

   # Set goal configuration
    goal_config = (2.6, -1.3, -np.pi / 2)
    robot.set_goal_config(goal_config)

    # Go to goal state
    robot.lidar_test()

    # Keep graphics window opened
    wait_if_gui()
    disconnect()


if __name__ == "__main__":
    main()