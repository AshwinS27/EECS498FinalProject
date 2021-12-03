import numpy as np
from map import Map
from lidar import Lidar
from pybullet_tools.utils import get_joint_positions, joint_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
from utils import load_env, execute_trajectory, draw_sphere_marker

class Robot:
    def __init__(self):
        # Set all movement params
        self.step_size = 0.5
        self.rot_step = np.pi/4

        # Create map and set map resolution
        self.resolution = 0.5
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




    #### For path searching ####
    def repeated_astar(self):
        pass

    def dstar_lite(self):
        pass

    def lpastar(self):
        pass

    #### For setting Parameters ####
    # Setting parameters for the Robot
    def set_step_size(self, step_size):
        self.step_size = step_size
    def set_rot_step(self, rot_step):
        self.rot_step = rot_step

    # Setting parameters for the Map
    def set_resolution(self, resolution):
        self.resolution = round(resolution, 2)
        self.global_map.set_resolution(self.resolution)

    ############ Private Member Functions #############

    """
    Updates Map through Process:
    1.) Gets Lidar scan from position
    2.) Puts all obstacles seen into maps obstacle dict
    3.) updates variable(self.seen_obstacles) with all points that have new obstacles  
    """
    def updateMap(self):
        dists = self.lidar.getLidarRaw(self.curr_state, self.obstacles)
        self.global_map.update

