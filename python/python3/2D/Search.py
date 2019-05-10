import numpy as np
import copy
import sys
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import Visualize as vis
import OptimizationFunctions as op_funs

'''
Usage
python3 Search.py x y tolerance

This file contains a search code that takes in a goal and a tolerance and
searches through the control space of the spring node system to determine
which controls to use. Then it visualizes the determined path.

Note that you will likely have to Ctrl + C out of the program as a path
with your specified tolerance is likely not possible.
'''

# The goal x,y is the first two input args
goal = np.array([[float(sys.argv[1]), float(sys.argv[2])]])


'''
Objective function that returns the distance of the 9th free node  to
the specified goal node.

Args:
positions (1D numpy array) : Positions of the free nodes in the system
'''
def goal1(positions):
    global goal
    nodes_xy = np.reshape(positions, (10, 2))
    
    return np.linalg.norm(nodes_xy[9] - goal)

class Node(object):
    '''
        Node objects contain all of the information about a system. They also contain
        a list of control inputs that were applied to get to this configuration.
        
        Node objects can do collision checks. And determine their children.
        
        Args:
        positions (1D numpy array) : (x1, y1, x2, y2,....) node positions
        connections (List of tuples: [(int, int)] : Each tuple corresponds to a spring (in order).
            each element in the tuple is a node index. (a, b) -> this spring is connected 
            from node a to node b.
        l0_arr (1D numpy array) : unstretched lengths in order
        k_arr (1D numpy array) : stiffnesses in order
        control_arr (List of Lists of Ints) : list of unstretched length controls in order applied
        num_nodes (int) : number of free nodes
        fixed_nodes (Dictionary: {Int, 1D numpy array} ) : fixed node index maps to x, y, z
    '''

    def __init__(self, positions, fixed_nodes, num_nodes, connections, l0_arr, k_arr, control_arr):
        self.positions = positions
        self.fixed_nodes = fixed_nodes
        self.num_nodes = num_nodes
        self.connections = connections
        self.l0_arr = l0_arr
        self.k_arr = k_arr
        self.control_arr = control_arr
    
    
    '''
    Checks each node in the child to see if it touches or collides with the obstacle.
    If returns True -> one or more nodes collides
    
    Args:
    child_pos (1D numpy array) : (x1, y1, x2, y2,....) node positions
    obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius)
    '''        
    def pointCollisionCheck(self, child_pos, obstacles):
        my_xy = np.reshape(self.positions, (self.num_nodes, 2))
        child_xy = np.reshape(child_pos, (self.num_nodes, 2))
        
        # Check each distance compared to radius
        for obstacle in obstacles:
            term1 = (child_xy[:,0] - obstacle[0][0])**2
            term2 = (child_xy[:,1] - obstacle[0][1])**2
            dists = np.sqrt(term1 + term2)
            
            return np.min(dists) <= obstacle[1]
        
        
        
    '''
    Returns the nodes that can be produced by the current node.
    Each child is produced by making 1 passive unstretched length active.
    
    Args:
    l0s_allowed (List of Ints) : 1 -> can change
    l0_passive (Double) : Value of unstretched length when passive
    l0_active (Double) : Value of unstretched length when active
    '''    
    def getChildren(self, l0s_allowed, l0_passive, l0_active):
        # List of child nodes
        children = []
        
        # Loop through each spring
        for i in range(0, len(self.l0_arr)):
            # Check for passive spring and if it can change
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                # Add element to control array
                curr_control_arr.append(self.l0_arr)
                
                # Add child to list
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
    
    '''
    Returns the nodes that can be produced by the current node.
    Each child is produced by making 1 passive/active unstretched length passive/active.
    
    Args:
    l0s_allowed (List of Ints) : 1 -> can change
    l0_passive (Double) : Value of unstretched length when passive
    l0_active (Double) : Value of unstretched length when active
    '''    
    def getChildrenAll(self, l0s_allowed, l0_passive, l0_active):
        children = []
        
        # Loops through springs
        for i in range(0, len(self.l0_arr)):
            # Check for passive spring and if it can change
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                
                # Add child to list
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
            # Check for active spring and if it can change
            if (self.l0_arr[i] == l0_active and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_passive
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                # Add child to list
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
        
    '''
    Returns the nodes that can be produced by the current node.
    Each child is produced by making 1 passive unstretched length active.
    
    Args:
    l0s_allowed (List of Ints) : 1 -> can change
    l0_passive (Double) : Value of unstretched length when passive
    l0_active (Double) : Value of unstretched length when active
    obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius)
    '''
    def getChildrenWithObs(self, l0s_allowed, l0_passive, l0_active, obstacles):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                # Only adds child if obstacle is not in collision
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
    
    '''
    Returns the nodes that can be produced by the current node.
    Each child is produced by making 1 passive/active unstretched length passive/active.
    
    Args:
    l0s_allowed (List of Ints) : 1 -> can change
    l0_passive (Double) : Value of unstretched length when passive
    l0_active (Double) : Value of unstretched length when active
    obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius)
    '''    
    def getChildrenWithObsAll(self, l0s_allowed, l0_passive, l0_active, obstacles):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                # Only adds child if obstacle is not in collision
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
            if (self.l0_arr[i] == l0_active and l0s_allowed[i]==1):
                # Copy the l0 array for new child and change specified spring
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_passive
                # Move the nodes using potential energy minimization
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                # Set positions for this child
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                # Only adds child if obstacle is not in collision
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children

        
class BFSearch(object):

    '''
        This class performs best first search.
    '''
    def __init__(self):
        self.visited = []
        self.frontier = []
        
    '''
    Function that peforms best first search.
    
    Args:
    start_node (Node) : Starting configuration
    obj_fun (Function) : The objective function for evaluating states
    threshold (Double) : Search stops if objective function reaches below threshold
    ls_allowed (List of Ints) : which springs can be changed 1->can change
    ls_possible (Tuple of doubles) : passive and active unstretched lengths
    obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius) 
    '''
    def bfs(self, start_node, obj_fun, threshold, ls_allowed, ls_possible, obstacles=None):
    
        # Setup figure to plot on
        #fig = plt.figure()
        #ax = fig.add_subplot(111)
        
        global goal
        self.visited = [] # list of visited controls
        self.frontier = [] # list of nodes to explore
        
        # Start with best node
        best_node = start_node
        best_value = obj_fun(start_node.positions)
        
        #self.visited.append(start_node.l0_arr.tolist())
        
        # Add node to the frontier
        self.insertFrontier((start_node, obj_fun(start_node.positions)))
        
        # Within try/catch to catch the Ctrl + C for early termination
        try:
            # Continue search while frontier is not empty
            while (len(self.frontier) > 0):
                print(len(self.visited))
                # Take item off of the beginning of the frontier
                curr_itm = self.frontier.pop()
                curr_node = curr_itm[0] # Node
                curr_value = curr_itm[1] # Objective function value
            
                # Add control to visited
                self.visited.append(curr_node.l0_arr.tolist())
            
                # Get children w/ and w/out obstacles
                if obstacles == None:
                    children = curr_node.getChildren(ls_allowed, ls_possible[0], ls_possible[1])
                else:
                    children = curr_node.getChildrenWithObsAll(ls_allowed, ls_possible[0], ls_possible[1],
                                                     obstacles)

                # Evaluate Children and add them to the frontier
                for child in children:
                    # Add child if we havent seen the control before
                    if child.l0_arr.tolist() not in self.visited:
                        child_value = obj_fun(child.positions)
                        # Save best value
                        if (child_value < best_value):
                            print("new best")
                            best_value = child_value
                            best_node = child
                        # Stop if threshold is achieved
                        if (child_value < threshold):
                            best_value = child_value
                            best_node = child
                            self.frontier = []
                            # Animate control sequence
                            vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
                            break
                        self.insertFrontier((child, child_value))
                        
        except KeyboardInterrupt:
            # Enter this if Ctrl + C is used
            # Animate Control Sequence
            vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
            return (best_node, best_value)
        print (best_node, best_value)
        vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
        return (best_node, best_value)
    
    '''
    Adds a node and objective function value pair to the frontier.
    Note that the frontier is sorted with minimum value at the the front
    
    Args:
    pair (Tuple (Node, Double)) : Node and Objective function value pair
    '''    
    def insertFrontier(self, pair):
    
        if (len(self.frontier) == 0):
            self.frontier.append(pair)
        else:
            for i in range(0, len(self.frontier)):
                if self.frontier[i][1] < pair[1]:
                    self.frontier.insert(i, pair)
                    return
            self.frontier.append(pair)
           
                
                
# Initial Configuration
initial_nodes_free = np.array([[0.0, 9.0],
                               [0.0, 8.0],
                               [0.0, 7.0],
                               [0.0, 6.0],
                               [0.0, 5.0],
                               [1.0, 9.0],
                               [1.0, 8.0],
                               [1.0, 7.0],
                               [1.0, 6.0],
                               [1.0, 5.0]]).reshape(20,)
# Fixed Nodes
initial_nodes_fixed = {}
initial_nodes_fixed[10] = np.array([0.0, 10.0])
initial_nodes_fixed[11] = np.array([1.0, 10.0])

# Spring Connections
connections = [(0, 1), (0, 5), (0, 10), (0, 6),
               (1, 2), (1, 6), (1, 7),
               (2, 3), (2, 7), (2, 8),
               (3, 4), (3, 8), (3, 9),
               (4, 9),
               (5, 6), (5,11), (5,10), 
               (6, 7),
               (7, 8),
               (8, 9)]

# Possible spring characteristics               
l_groups = (1, 0.5, np.sqrt(2))
k_groups = (1000, 1000)

# Actual spring unstreched length values      
ls = np.array([l_groups[0], l_groups[0], l_groups[0], l_groups[2], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[0], l_groups[2],
               l_groups[0], l_groups[0], l_groups[2], l_groups[0], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[0], l_groups[0]])
               
# Which springs can change
ls_can_change = [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]

# Actual spring stiffness values
ks = np.array([k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0]])


# Obstacles
obstacles = [((4, 7), 1)]

# Minimize PE with node positions
res = minimize(op_funs.nodePositionObjective, initial_nodes_free,
               args=(connections, ls, ks, 10, initial_nodes_fixed),
               method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})

# Starting Node
node1 = Node(res.x, initial_nodes_fixed, 10,
             connections, ls, ks, [])
# Search
searcher = BFSearch()
searcher.bfs(node1, goal1, float(sys.argv[3]), ls_can_change, l_groups, obstacles)

plt.show()
