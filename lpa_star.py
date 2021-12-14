def lpa_star():
    init()
    while(True):
        computeShortPath()
        while(not costChanges()):
            pass
            #IS THIS WHEN WE REACH MINIMUM?
            #WHATS THE POINT OF THIS WHILE LOOP
        edgeList = getEdgeList()
        for edge in edgeList:
            edge.setCost(newCost(edge))
            updateNode(edge.endNode)
    return

#In all functions,
#h(start,s) = heuristic of goal distance to s
#g(s) = distance to come (like g in a star)
def calculateKey(s):
    #return min of 
    #g(s), rhs(s) +
    #h(start,s) + k,
    #also return min of g(s), rhs(s) 
    
    return 

def init():
    #Make priority q U
    kM = 0
    #For all successors of current Node

# void main() {
#   initialize();
#   while (true) {
#     computeShortestPath();
#     while (!hasCostChanges())
#       sleep;
#     for (edge : getChangedEdges()) {
#         edge.setCost(getNewCost(edge));
#         updateNode(edge.endNode);
#     }
#   }
# }

# void initialize() {
#   queue = new PriorityQueue();
#   for (node : getAllNodes()) {
#     node.g = INFINITY;
#     node.rhs = INFINITY;
#   }
#   start.rhs = 0;
#   queue.insert(start, calculateKey(start));
# }

# /** Expands the nodes in the priority queue. */
# void computeShortestPath() {
#   while ((queue.getTopKey() < calculateKey(goal)) || (goal.rhs != goal.g)) {
#     node = queue.pop();
#     if (node.g > node.rhs) {
#       node.g = node.rhs;
#       for (successor : node.getSuccessors())
#         updateNode(successor);
#     } else {
#       node.g = INFINITY;
#       updateNode(node);
#       for (successor : node.getSuccessors())
#         updateNode(successor);
#     }
#   }
# }

# /** Recalculates rhs for a node and removes it from the queue.
#  * If the node has become locally inconsistent, it is (re-)inserted into the queue with its new key. */
# void updateNode(node) {
#   if (node != start) {
#     node.rhs = INFINITY;
#     for (predecessor: node.getPredecessors())
#       node.rhs = min(node.rhs, predecessor.g + predecessor.getCostTo(node));
#     if (queue.contains(node))
#       queue.remove(node);
#     if (node.g != node.rhs)
#       queue.insert(node, calculateKey(node));
#   }
# }

# int[] calculateKey(node) {
#   return {min(node.g, node.rhs) + node.getHeuristic(goal), min(node.g, node.rhs)};
# }