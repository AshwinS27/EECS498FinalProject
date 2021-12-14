import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
### YOUR IMPORTS HERE ###
from queue import PriorityQueue
import math

from pybullet_tools.utils import wait_for_user
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker, draw_line


class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in, cost_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.cost = cost_in
    
    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y and self.theta == other.theta)


    def printme(self):
        print("\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid, "cost:", self.cost)

#########################

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2bigmap.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, -1.3, -np.pi/2)))

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))

    # Example of draw 
    # draw_sphere_marker((0, 0, 1), 0.1, (1, 0, 0, 1))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    print(start_config)
    #goal_config = (2.6, -1.3, -np.pi/2) //orig
    goal_config = (-7, -7.4, 0)
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###
    #Action Cost
    def c(n,m):
        return math.sqrt((n.x-m.x)**2 + (n.y-m.y)**2 + (min(abs(n.theta - m.theta),2*math.pi - (n.theta - m.theta))**2))
    #Heuristic
    def h(n):
        return math.sqrt((n.x-goal_config[0])**2 + (n.y-goal_config[1])**2 + (min(abs(n.theta - goal_config[2]),2*math.pi - (n.theta - goal_config[2]))**2))
 
    q = PriorityQueue()
    DIST = 0.5
    ANG = math.pi/2.0
    epsilon = 0.3
    #x,y,theta,id=0,parent=-1,cost-to-come=0
    startNode = Node(start_config[0],start_config[1],start_config[2],0,-1,0)
    goalNode = Node(goal_config[0],goal_config[1],goal_config[2],-1,-1,-1)
    #priority = 0, id=0, node=startNode
    q.put((0, 0, startNode))
    #node,succesorF
    open_list = [(startNode,0)]
    nodeCounter = 1
    nodeParent = startNode
    visited = [] #closed list
    colliding = []
    collfree = []
    current = start_config
    #Format: node & succesorF
    at_goal = False
    while not q.empty():
        next_item = q.get()
        nodeParent = next_item[2]
        open_list.remove((nodeParent,next_item[0]))
        at_goal = False
        if(c(nodeParent,goalNode) < epsilon):
            collision_fn(current)
            print("I'm at goal")
            at_goal = True
            print("Priority:", next_item[0])
            next_item[2].printme()
            break
        #CREATE 8 KIDS
        nodeId = 0
        x_inc = Node(nodeParent.x + DIST,nodeParent.y,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
        x_dec = Node(nodeParent.x - DIST,nodeParent.y,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
        y_dec = Node(nodeParent.x,nodeParent.y - DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
        y_inc = Node(nodeParent.x,nodeParent.y + DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
        th_dec = Node(nodeParent.x,nodeParent.y,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
        th_inc = Node(nodeParent.x,nodeParent.y,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)

        kids_4connected = [x_inc, x_dec, y_inc, y_dec, th_inc, th_dec]

        def make8conKids():
            #8 connceted in 3d is 8 + 9 + 9 = 26
            #take 8 connected earlier # need 20 more
            #X INCREASE
                #Theta decrease
            xi_yi_td = Node(nodeParent.x + DIST,nodeParent.y + DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            xi_yd_td = Node(nodeParent.x + DIST,nodeParent.y - DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            xi_yz_td = Node(nodeParent.x + DIST,nodeParent.y,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
                #Theta same
            xi_yi_tz = Node(nodeParent.x + DIST,nodeParent.y + DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
            xi_yd_tz = Node(nodeParent.x + DIST,nodeParent.y - DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
                #Theta increase
            xi_yi_ti = Node(nodeParent.x + DIST,nodeParent.y + DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xi_yd_ti = Node(nodeParent.x + DIST,nodeParent.y - DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xi_yz_ti = Node(nodeParent.x + DIST,nodeParent.y,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            
            #x Decrease
            xd_yi_td = Node(nodeParent.x - DIST,nodeParent.y + DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            xd_yd_td = Node(nodeParent.x - DIST,nodeParent.y - DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            xd_yz_td = Node(nodeParent.x - DIST,nodeParent.y,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            
            xd_yi_tz = Node(nodeParent.x - DIST,nodeParent.y + DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
            xd_yd_tz = Node(nodeParent.x - DIST,nodeParent.y - DIST,nodeParent.theta,nodeId,nodeParent.id,nodeParent.cost)
            
            xd_yi_ti = Node(nodeParent.x - DIST,nodeParent.y + DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xd_yd_ti = Node(nodeParent.x - DIST,nodeParent.y - DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xd_yz_ti = Node(nodeParent.x - DIST,nodeParent.y,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)

            #X zero
            xz_yi_ti = Node(nodeParent.x,nodeParent.y + DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xz_yd_ti = Node(nodeParent.x,nodeParent.y - DIST,nodeParent.theta + ANG,nodeId,nodeParent.id,nodeParent.cost)
            xz_yi_td = Node(nodeParent.x,nodeParent.y + DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            xz_yd_td = Node(nodeParent.x,nodeParent.y - DIST,nodeParent.theta - ANG,nodeId,nodeParent.id,nodeParent.cost)
            
            kids_8connected = [x_inc, x_dec, y_inc, y_dec, th_inc, th_dec, 
                     xi_yi_td, xi_yd_td, xi_yz_td,
                     xi_yi_tz, xi_yd_tz,
                     xi_yi_ti, xi_yd_ti, xi_yz_ti,
                     xd_yi_td, xd_yd_td, xd_yz_td,
                     xd_yi_tz, xd_yd_tz,
                     xd_yi_ti, xd_yd_ti, xd_yz_ti,
                     xz_yi_ti, xz_yd_ti,
                     xz_yi_td, xz_yd_td]
            return kids_8connected              
        
        #kids = kids_4connected
        kids = make8conKids()
        i = kids[1]
        current = (i.x, i.y, i.theta)
        
        succesor_f = 0
        for i in kids:
            current = (i.x, i.y, i.theta)
            print("HEYY")
            if (collision_fn(current)):
                colliding.append(current)
                continue
            collfree.append(current)
            print("SOMETHING PUT")
            cost_to_cum =  c(i,nodeParent) + nodeParent.cost 
            #cost of coming to current node + cost of coming to parent
            succesor_f = h(i) + cost_to_cum
            #succer_f is h from i, cost to go to i, + cost of getting to next_item
            i.cost = cost_to_cum
            visit_before = False
            open_before = False

            for vertex in open_list:
                if (c(vertex[0],i) < epsilon):
                    if (vertex[1] <= succesor_f):
                        open_before = True
            if (open_before):
                continue
            for vertex in visited:
                if(c(vertex[0],i) < epsilon):         
                    if (vertex[1] <= succesor_f):
                        visit_before = True
            if (visit_before):
                continue
            i.id = nodeCounter
            nodeCounter+=1
            q.put((succesor_f,i.id,i))
            open_list.append((i,succesor_f))
            
        print(q)
        visited.append((nodeParent,succesor_f))  
    
    #backtracking for path:
    if (not at_goal):
        print("No Solution Found")

    nodeCurr = nodeParent
    while(nodeCurr.parentid != 0):
        parent_node = nodeCurr.parentid
        path.append([nodeCurr.x,nodeCurr.y,nodeCurr.theta])
        for i in visited:
            if (i[0].id == parent_node):
                nodeCurr = i[0]
                break
    path.reverse()
    print("Planner run planning time: ", time.time() - start_time)

    collision_free_set = set(collfree)
    colliding_set = set(colliding)
    
    
    for i in collision_free_set:
        sphere_position = (i[0], i[1], 0)
        sphere_radius = 0.1
        sphere_color = (0, 0, 1, 0.4) # R, G, B, A
        draw_sphere_marker(sphere_position, sphere_radius, sphere_color)
    
    for i in path:
        set_joint_positions(robots['pr2'], base_joints, i)
        sphere_position = (i[0], i[1], 0)
        sphere_radius = 0.1
        sphere_color = (0, 0, 0, 1) # R, G, B, A
        draw_sphere_marker(sphere_position, sphere_radius, sphere_color)
    
    for i in colliding_set:
        sphere_position = (i[0], i[1], 0)
        sphere_radius = 0.1
        sphere_color = (1, 0, 0, 0.5) # R, G, B, A
        draw_sphere_marker(sphere_position, sphere_radius, sphere_color)
    
    ######################
    print("Planner run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()