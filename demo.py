import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name, get_aabb, get_aabbs
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
from robot import Robot
from pybullet_tools.utils import wait_for_user

# starts demo
def main():
    # Intialize PyBullet
    connect(use_gui=True)

    bigmap_goal_config = (7, 7, np.pi / 2)
    doorway_goal_config = (1.8329999999999997, -1.3, -np.pi)
    debug_goal_config = (4, 4, np.pi / 2)

    ################## PART 1 of 4 WITH REPEATED A* #####################3#
    print("Running part 1 of 4...")
    print("Repeated A* on small map...")
    # Intialize the Robot
    planner_type1 = 'A*'
    robot1 = Robot(planner_type1)
    robot1.load_world('pr2doorway.json')

    robot1.set_goal_config(doorway_goal_config)

    # Go to goal state
    robot1.run(debug=True)
    time_taken1 = robot1.get_runtime()
    distance_travelled1 = robot1.get_total_distance()
    print(planner_type1 + " distance travelled " + str(distance_travelled1))
    print(planner_type1 + " time taken: " + str(time_taken1))
    robot1.delete_map_and_robot()

    wait_for_user()
    ################# Part 2 of 4 with D* lite on small map ################
    print("Running part 2 of 4...")
    print("D* Lite on small map...")
    # Intialize the Robot
    planner_type2 = 'D*'
    robot2 = Robot(planner_type2)
    robot2.load_world('pr2doorway.json')

    robot2.set_goal_config(doorway_goal_config)

    # Go to goal state
    robot2.run(debug=False)
    time_taken2 = robot2.get_runtime()
    distance_travelled2 = robot2.get_total_distance()
    print(planner_type2 + " distance travelled " + str(distance_travelled2))
    print(planner_type2 + " time taken: " + str(time_taken2))

    robot2.delete_map_and_robot()

    wait_for_user()
    ############## Part 3 of 4 with Repeated A* on big map ###############
    print("Running part 3 of 4...")
    print("Repeated A* on big map...")
    # Intialize the Robot
    planner_type3 = 'A*'
    robot3 = Robot(planner_type3)
    robot3.load_world('pr2bigmap.json')

    robot3.set_goal_config(bigmap_goal_config)

    # Go to goal state
    robot3.run(debug=False)
    time_taken3 = robot3.get_runtime()
    distance_travelled3 = robot3.get_total_distance()
    print(planner_type3 + " distance travelled " + str(distance_travelled3))
    print(planner_type3 + " time taken: " + str(time_taken3))

    robot3.delete_map_and_robot()

    wait_for_user()
    ############# Part 4 of 4 with D* Lite on big map
    print("Running part 4 of 4...")
    print("D* Lite on big map...")
    # Intialize the Robot
    planner_type4 = 'D*'
    robot4 = Robot(planner_type4)
    robot4.load_world('pr2bigmap.json')

    robot4.set_goal_config(bigmap_goal_config)

    # Go to goal state
    robot4.run(debug=False)
    time_taken4 = robot4.get_runtime()
    distance_travelled4 = robot4.get_total_distance()
    print(planner_type4 + " distance travelled " + str(distance_travelled4))
    print(planner_type4 + " time taken: " + str(time_taken4))

    # Keep graphics window opened
    wait_if_gui()
    disconnect()


if __name__ == "__main__":
    main()