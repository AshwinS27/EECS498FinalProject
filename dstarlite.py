import numpy as np
from map import Map
from utils import dist
from plot_utils import plot_points, destroy_points
from queue import PriorityQueue

km = 0
start_node = None

class Node:
    def __init__(self, state, id_in, pred_id, g_in, rhs_in):
        # Parameters
        self.state = state
        self.id = id_in
        self.g = g_in
        self.rhs = rhs_in
        self.predecessors = []
        if pred_id is not None:
            self.predecessors.append(pred_id)
        self.successors = []

    def calculate_key(self):
        return [min(self.g, self.rhs) + dist(self.state, start_node.get_state()) + km, min(self.g, self.rhs)]

    def __lt__(self, other):
        my_key = self.calculate_key()
        other_key = other.calculate_key()
        if my_key[0] == other_key[0]:
            return my_key[1] < other_key[1]
        return my_key[0] < other_key[0]

    def __eq__(self, other):
        if self.id == other.id:
            return True
        return False


    ########### Getters ##########
    def get_rhs(self):
        return self.rhs

    def get_id(self):
        return self.id

    def get_pred(self):
        return self.predecessors

    def get_state(self):
        return self.state

    def get_g(self):
        return self.g

    def get_predecessors(self):
        return self.predecessors

    ############# Setting values #########3
    def set_rhs(self, rhs_in):
        self.rhs = rhs_in

    def set_g(self, g_in):
        self.g = g_in

    def add_to_pred(self, unique_id):
        self.predecessors.append(unique_id)

    def add_to_succ(self, unique_id):
        self.successors.append(unique_id)


class Dstarlite:
    def __init__(self, map, robot):
        # Map and Robot parameters
        self.myrobot = robot
        self.global_map = map
        self.trans_step_size = self.myrobot.get_trans_step_size()
        self.rot_step_size = self.myrobot.get_rot_step_size()

        # Metrics
        self.total_distance = 0

        # start and goal states
        self.goal_state = None  # set by Robot in setting goal state
        self.start_state = None  # set by Robot when loading world

        # debugging
        self.debug = False

        # D* lite parameters
        self.goal_node = None
        self.pq = None
        self.unique_id = 0
        self.nodeSet = {}
        self.openSet = {}
        # start is a global variable
        # km is a global variable


    def set_start_state(self, state):
        self.start_state = state
        global start_node
        start_node = Node(self.start_state, self.unique_id, None, np.inf, np.inf)
        self.openSet[tuple(self.start_state)] = self.unique_id
        self.nodeSet[self.unique_id] = start_node
        self.unique_id += 1

    def updateGoal(self):
        self.goal_state = self.myrobot.get_goal_state()
        self.openSet[tuple(self.goal_state)] = -1
        self.goal_node = Node(self.goal_state, -1, None, np.inf, 0)
        self.nodeSet[-1] = self.goal_node

    # returns true if key1 is less than key2
    def compare_keys(self, key1, key2):
        if key1[0] == key2[0]:
            return key1[1] < key2[1]
        return key1[0] < key2[0]

    def run(self, debug=False): #the "MAIN" from pseduocode
        ######### Intitialize parameters #############3
        self.debug = debug
        obs_marker_ids = []
        path_marker_ids = []
        self.pq.put(start_node)

        self.compute_shortest_path()

        while not self.is_goal(start_node):
            if start_node.g == np.inf:
                print("No solution")
                return

            # find successor to move towards
            # generate successors in case there are none
            self.generate_and_add_successors(start_node)
            temp_path = [start_node.get_state()]
            new_start_node = None
            min_val = np.inf
            for succ_id in start_node.successors:
                if dist(start_node.get_state(), self.nodeSet[succ_id].get_state()) + self.nodeSet[succ_id].get_g() < min_val:
                    global start_node
                    start_node = new_start_node
                    temp_path.append(start_node.get_state())

            # move in path
            if self.debug:
                # remove previous path and generate new one
                destroy_points(path_marker_ids)
                path_marker_ids.clear()
                path_marker_ids = plot_points(self.generate_path())

            path_marker_ids = self.generate_path

            self.myrobot.move_in_path(temp_path, 0)
            self.total_distance += dist(temp_path[0], temp_path[1])

            #scan area and update map
            new_obs_points = self.myrobot.update_map(start_node.get_state())
            if self.debug:
                obs_marker_ids.extend(plot_points(new_obs_points))


            #use new_obs_points to check for edge costs
            km = km + dist(temp_path[0], temp_path[1])


            ############## What was there before ######### I think a better solution is available
            for obs in new_obs_points:

                # TODO: Add more neighbors to the obs points
                xy_obs_0 = (obs[1] + self.global_map.resolution, obs[2] + self.global_map.resolution, 0)
                xy_obs_pih = (obs[1] + self.global_map.resolution, obs[2] + self.global_map.resolution, np.pi / 2.0)
                xy_obs_pi = (obs[1] + self.global_map.resolution, obs[2] + self.global_map.resolution, np.pi)
                xy_obs_twopi = (obs[1] + self.global_map.resolution, obs[2] + self.global_map.resolution, 2 * np.pi)
                xy_obs = [xy_obs_0, xy_obs_pih, xy_obs_pi, xy_obs_twopi]

                for orient in xy_obs:
                    if self.openSet.get(orient) is not None:  # i.e there is a node on an obstacle:
                        id = self.openSet.get(orient)
                        nodeChange = self.nodeSet[id]
                        nodeChange.g = np.inf
                        self.updateNode(nodeChange)
            ############# End of what existed #################





    def compute_shortest_path(self):
        while self.pq.queue[0] < start_node or start_node.get_rhs() != start_node.get_g():
            key_old = self.pq.queue[0].calculate_key()
            u = self.pq.get()
            if self.compare_keys(key_old, u.calculate_key()):
                self.pq.put(u)
            elif u.get_g() < u.get_rhs():
                u.g = u.get_rhs()
                for pred in u.get_predecessors():
                    self.update_node(pred)
            else:
                u.g = np.inf
                for pred in u.get_predecessors():
                    self.update_node(pred)
                self.update_node(u)

    def update_node(self, node):
        if self.is_goal(node):
            # rhs correlated to successors, therefore generate and add any new successors
            self.generate_and_add_successors(node)
            rhs_min = np.inf
            for succ_id in node.successors:
                rhs_min = min(rhs_min, dist(node.get_state(), self.nodeSet[succ_id].get_state()) + self.nodeSet[succ_id].get_g())

        # check to see if node is contained within priority queue and remove if it is
        for i in range(0, self.pq.qsize()):
            if self.pq.queue[i].get_id() == node.get_id():
                self.pq.queue.remove(node)

        if node.get_g() != node.get_rhs():
            self.pq.put(node)

    def generate_and_add_successors(self, node):
        # This node has already generated its neighbors
        if not node.successors.empty():
            return
        # Generate and check 4 connected successors for being
        # 1.) already created
        # 2.) an obstacle

        # Create #1
        new_state = [x for x in node.get_state()]
        new_state[0] += self.trans_step_size
        new_state = self.global_map.roundPointToCell(new_state)
        # Check #1 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state):
            if self.openSet.get(tuple(new_state)) is None:
                # Doesn't exist, so let's create it
                new_node = Node(new_state, self.unique_id, node.get_id(), node.get_g() + dist(node.get_state(), new_state), node.get_g() + dist(node.get_state(), new_state))
                self.openSet[new_state] = self.unique_id
                self.nodeSet[self.unique_id] = new_node
                self.unique_id += 1
                node.add_to_succ(new_node.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(), node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

        # Create #2
        new_state2 = [x for x in node.get_state()]
        new_state2[0] -= self.trans_step_size
        new_state2 = self.global_map.roundPointToCell(new_state2)
        # Check #2 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state2):
            if self.openSet.get(tuple(new_state2)) is None:
                # Doesn't exist, so let's create it
                new_node2 = Node(new_state2, self.unique_id, node.get_id(),
                                node.get_g() + dist(node.get_state(), new_state2),
                                node.get_g() + dist(node.get_state(), new_state2))
                self.openSet[new_state2] = self.unique_id
                self.nodeSet[self.unique_id] = new_node2
                self.unique_id += 1
                node.add_to_succ(new_node2.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state2]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(),
                                       node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

        # Create #3
        new_state3 = [x for x in node.get_state()]
        new_state3[1] += self.trans_step_size
        new_state3 = self.global_map.roundPointToCell(new_state3)
        # Check #3 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state3):
            if self.openSet.get(tuple(new_state3)) is None:
                # Doesn't exist, so let's create it
                new_node3 = Node(new_state3, self.unique_id, node.get_id(),
                                node.get_g() + dist(node.get_state(), new_state3),
                                node.get_g() + dist(node.get_state(), new_state3))
                self.openSet[new_state3] = self.unique_id
                self.nodeSet[self.unique_id] = new_node3
                self.unique_id += 1
                node.add_to_succ(new_node3.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state3]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(),
                                       node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

        # Create #4
        new_state4 = [x for x in node.get_state()]
        new_state4[1] -= self.trans_step_size
        new_state4 = self.global_map.roundPointToCell(new_state4)
        # Check #1 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state4):
            if self.openSet.get(tuple(new_state4)) is None:
                # Doesn't exist, so let's create it
                new_node4 = Node(new_state4, self.unique_id, node.get_id(),
                                node.get_g() + dist(node.get_state(), new_state4),
                                node.get_g() + dist(node.get_state(), new_state4))
                self.openSet[new_state4] = self.unique_id
                self.nodeSet[self.unique_id] = new_node4
                self.unique_id += 1
                node.add_to_succ(new_node4.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state4]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(),
                                       node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

        # Create #5
        new_state5 = [x for x in node.get_state()]
        new_state5[2] += self.rot_step_size
        new_state5 = self.global_map.roundPointToCell(new_state5)
        # Check #5 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state5):
            if self.openSet.get(tuple(new_state5)) is None:
                # Doesn't exist, so let's create it
                new_node5 = Node(new_state5, self.unique_id, node.get_id(),
                                node.get_g() + dist(node.get_state(), new_state5),
                                node.get_g() + dist(node.get_state(), new_state5))
                self.openSet[new_state5] = self.unique_id
                self.nodeSet[self.unique_id] = new_node5
                self.unique_id += 1
                node.add_to_succ(new_node5.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state5]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(),
                                       node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

        # Create #6
        new_state6 = [x for x in node.get_state()]
        new_state6[2] -= self.rot_step_size
        new_state6 = self.global_map.roundPointToCell(new_state6)
        # Check #1 for existing already or for being an obstacle
        if not self.global_map.isObstacle(new_state6):
            if self.openSet.get(tuple(new_state6)) is None:
                # Doesn't exist, so let's create it
                new_node6 = Node(new_state6, self.unique_id, node.get_id(),
                                node.get_g() + dist(node.get_state(), new_state6),
                                node.get_g() + dist(node.get_state(), new_state6))
                self.openSet[new_state6] = self.unique_id
                self.nodeSet[self.unique_id] = new_node6
                self.unique_id += 1
                node.add_to_succ(new_node.get_id())
            else:
                # exists so lets just add as predecessor
                node_that_exists = self.nodeSet[self.openSet[new_state6]]
                node_that_exists.add_to_pred(node.get_id())
                node.add_to_succ(node_that_exists.get_id())

                # if g(node) + c(node, node_that_exists) < g(node_that_exists) then update node_that_exists.g
                if node.get_g() + dist(node.get_state(), node_that_exists.get_state()) < node_that_exists.get_g():
                    node_that_exists.g = node.get_g() + dist(node.get_state(), node_that_exists.get_state())

    def is_goal(self, node):
        if dist(node.get_state(), self.goal_state) <= self.myrobot.goal_threshold:
            return True
        return False

    def get_total_distance(self):
        return self.total_distance

    def generate_path(self):
        curr_node = start_node
        path = []
        while not curr_node == self.goal_node:
            path.append(curr_node.get_state())
            # next node is the minimum of it's predecessors
            min_node = None
            min_score = np.inf
            for succ_id in curr_node.successors:
                if dist(self.nodeSet[succ_id].get_state(), curr_node.get_state()) + self.nodeSet[succ_id].get_g() < min_score:
                    min_score = dist(self.nodeSet[succ_id].get_state(), curr_node.get_state()) + self.nodeSet[succ_id].get_g()
                    min_node = self.nodeSet[succ_id]
            curr_node = min_node
        path = path.reverse()
        return path

