import numpy as np
from map import Map
from lidar import Lidar
from pybullet_tools.utils import get_joint_positions, joint_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from utils import load_env, execute_trajectory, draw_sphere_marker
from plot_utils import plot_points

class Robot:
    def __init__(self):
        # Set all movement params
        self.step_size = 0.05
        self.rot_step = np.pi/4

        # Create map and set map resolution
        self.resolution = round(np.sqrt((self.step_size**2)/2) + self.step_size/16, 3)
        self.global_map = Map(self.resolution)

        # Create lidar and configure
        self.lidar = Lidar()

        # State variables
        self.curr_state = None
        self.goal_state = None
        self.start_config = None
        self.goal_config = None
        self.seen_obstacles = []

        # Obstacle and Robot resources
        self.obstacles = None
        self.robots = None
        self.base_joints = None

        # Shared Algorithm Variables
        self.curr_path = []
        self.curr_path_idx = None

    ########### Public Member Functions ###########

    ###### For setting of start and goal positions
    # Sets goal configuration from tuple
    def set_goal_config(self, goal_config):
        self.goal_config = goal_config
        self.goal_state = list(goal_config)

    # Loads world and sets parameters
    def load_world(self, world_name):
        # load robot and obstacle resources
        robots, obstacles = load_env(world_name)
        self.robots = robots
        self.obstacles = obstacles

        # define active DoFs
        self.base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

        # set start config
        self.start_config = tuple(get_joint_positions(robots['pr2'], self.base_joints))
        self.curr_state = list(self.start_config)

    # Returns 1 if end of path is reached or path is empty
    # Returns 0 for succesfully incrementing along path
    def move_in_path(self):
        if len(self.curr_path) == 0:
            print("Path is Empty")
            return 1
        elif self.curr_path_idx == len(self.curr_path):
            print("End of path reached")
            return 1
        else:
            sub_path = [tuple(self.curr_path[self.curr_path_idx]), tuple(self.curr_path[self.curr_path_idx + 1])]
            self.curr_path_idx += 1
            execute_trajectory(self.robots['pr2'], self.base_joints, sub_path, sleep=0.2)
            self.curr_state = self.curr_path[self.curr_path_idx]
            return 0


    def lidar_test(self):
        # Straight line trajectory to test Lidar
        start = self.start_config
        trans_step = 0.1
        movements = 500

        # Create path
        path = []
        path.append(list(start))
        state = list(start)
        for i in range(0, movements):
            new_state = np.copy(state)
            new_state[0] += trans_step
            path.append(new_state)
            state = new_state
        self.curr_path = path
        self.curr_path_idx = 0

        # keep moving along path
        while not self.move_in_path():
            # Get points from map
            new_points = self.lidar.getLidarScan(self.curr_state, self.obstacles)
            plot_points(new_points)


    #### For path searching ####
    def repeated_astar(self):
        pass

    def dstar_lite(self):
        pass

    def lpastar(self):
        pass


    ##### For Getting state parameters #########
    def get_current_path(self):
        return self.curr_path

    def get_current_path_idx(self):
        return self.curr_path_idx

    def get_current_state(self):
        return self.curr_state

    ############ Private Member Functions #############

    """
    CALL THIS TO SCAN AREA AND UPDATE MAP WITH NEW OBSTACLE VALUES
    Updates Map through Process:
    1.) Gets Lidar scan from position
    2.) Puts all obstacles seen into maps obstacle dict
    3.) updates variable(self.seen_obstacles) with all points that have new obstacles  
    """
    def updateMap(self):
        # Get lidar Scan
        obstacle_points = self.lidar.getLidarScan(self.curr_state, self.obstacles)

        # draw on map

        # Update global map
        self.global_map.updateMap(obstacle_points)

        # Get novel obstacle points
        new_obs_points = self.global_map.getNewPoints()

        #TODO: What to do with the new obstacle points?
        # Maybe check if they interfere with path that is already planned?

