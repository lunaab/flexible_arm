import numpy as np
import copy
import sys
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import Visualize as vis
import OptimizationFunctions as op_funs
goal = np.array([[float(sys.argv[1]), float(sys.argv[2])]])

def goal1(positions):
    global goal
    nodes_xy = np.reshape(positions, (10, 2))
    
    return np.linalg.norm(nodes_xy[9] - goal) #+ np.linalg.norm(nodes_xy[8] - goal) + np.linalg.norm(nodes_xy[7] - goal)

class Node(object):

    def __init__(self, positions, fixed_nodes, num_nodes, connections, l0_arr, k_arr, control_arr):
        self.positions = positions
        self.fixed_nodes = fixed_nodes
        self.num_nodes = num_nodes
        self.connections = connections
        self.l0_arr = l0_arr
        self.k_arr = k_arr
        self.control_arr = control_arr
        
    def lineCollisionCheck(self, child_pos, obstacles):
        ''' Obstacles[i][0] (x,y) positions
            Obstacles[i][1] radius
        '''
        my_xy = np.reshape(self.positions, (self.num_nodes, 2))
        child_xy = np.reshape(child_pos, (self.num_nodes, 2))
        #print "PARENT"
        #print my_xy
        #print "CHILD"
        #print child_xy
        
        # child -> 1, self -> 0
        b_s = child_xy[:,0] - my_xy[:,0]
        a_s = my_xy[:,1] - child_xy[:,1]
        c_s = (-1*a_s*child_xy[:,0]) + (-1*b_s*child_xy[:,1])
        #print "As"
        #print a_s
        #print "Bs"
        #print b_s
        #print "Cs"
        #print c_s
        
        for obstacle in obstacles:
            dists = np.absolute(a_s * obstacle[0][0] + b_s * obstacle[0][1] + c_s) / np.sqrt(a_s**2 + b_s**2)
            
            #print np.min(dists)
            #print np.min(dists) <= obstacle[1]
            #if (np.min(dists) == obstacle[1]):
            #    print "EQUAL"
            #if (np.min(dists) > obstacle[1]):
            #    print dists
            print dists
            return np.min(dists) <= obstacle[1]
            
    def pointCollisionCheck(self, child_pos, obstacles):
        ''' Obstacles[i][0] (x,y) positions
            Obstacles[i][1] radius
        '''
        my_xy = np.reshape(self.positions, (self.num_nodes, 2))
        child_xy = np.reshape(child_pos, (self.num_nodes, 2))
        
        for obstacle in obstacles:
            term1 = (child_xy[:,0] - obstacle[0][0])**2
            term2 = (child_xy[:,1] - obstacle[0][1])**2
            dists = np.sqrt(term1 + term2)
            
            return np.min(dists) <= obstacle[1]
        
        
        
        
    def getChildren(self, l0s_allowed, l0_passive, l0_active):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
        
    def getChildrenAll(self, l0s_allowed, l0_passive, l0_active):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
            if (self.l0_arr[i] == l0_active and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_passive
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
        
    def getChildrenWithObs(self, l0s_allowed, l0_passive, l0_active, obstacles):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
        
    def getChildrenWithObsAll(self, l0s_allowed, l0_passive, l0_active, obstacles):
        children = []
        for i in range(0, len(self.l0_arr)):
            if (self.l0_arr[i] == l0_passive and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_active
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
            if (self.l0_arr[i] == l0_active and l0s_allowed[i]==1):
                curr_l0_arr = copy.deepcopy(self.l0_arr)
                curr_l0_arr[i] = l0_passive
                res = minimize(op_funs.nodePositionObjective, self.positions,
                   args=(self.connections, curr_l0_arr, self.k_arr,
                   self.num_nodes, self.fixed_nodes),
                   method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
                curr_positions = res.x
                curr_control_arr = copy.deepcopy(self.control_arr)
                curr_control_arr.append(self.l0_arr)
                if (not self.pointCollisionCheck(curr_positions, obstacles)):
                    children.append(Node(curr_positions, self.fixed_nodes, self.num_nodes,
                                     self.connections, curr_l0_arr, self.k_arr,
                                     curr_control_arr))
        return children
        
class BFSearch(object):

    def __init__(self):
        self.visited = []
        self.frontier = []
        
    def bfs(self, start_node, obj_fun, threshold, ls_allowed, ls_possible, obstacles=None):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        global goal
        self.visited = []
        self.frontier = []
        best_node = start_node
        best_value = obj_fun(start_node.positions)
        
        #self.visited.append(start_node.l0_arr.tolist())
        self.insertFrontier((start_node, obj_fun(start_node.positions)))
        count = 0
        try:
            while (len(self.frontier) > 0):
                print len(self.visited) #self.frontier
                curr_itm = self.frontier.pop()
                curr_node = curr_itm[0]
                curr_value = curr_itm[1]
            
            
                self.visited.append(curr_node.l0_arr.tolist())
            
                if obstacles == None:
                    children = curr_node.getChildren(ls_allowed, ls_possible[0], ls_possible[1])
                else:
                    children = curr_node.getChildrenWithObsAll(ls_allowed, ls_possible[0], ls_possible[1],
                                                     obstacles)
                count = count + len(children)
                #print count
                #print 'Parent'
                #print curr_node.l0_arr.tolist()
                #print 'Children'
                for child in children:
                    #print child.l0_arr.tolist()
                    if child.l0_arr.tolist() not in self.visited:
                        #print 'Accepted'
                        #vis.show_system_same_plot(child.positions, child.connections,
                                                  #child.num_nodes, child.fixed_nodes, ax)
                        child_value = obj_fun(child.positions)
                        #print child_value
                        if (child_value < best_value):
                            print "new best"
                            print child_value
                            #print len(child.control_arr)
                            best_value = child_value
                            best_node = child
                        if (child_value < threshold):
                        
                            best_value = child_value
                            best_node = child
                            self.frontier = []
                            vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
                            #vis.show_system_goal(child.positions, child.connections,
                                                  #child.num_nodes, child.fixed_nodes, goal, [9])
                            break
                        self.insertFrontier((child, child_value))
                #raw_input()
            print count
        except KeyboardInterrupt:
            #vis.show_system_goal(best_node.positions, best_node.connections,
                                #best_node.num_nodes, best_node.fixed_nodes, goal, [9])
            #print (best_node, best_value)
            vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
            return (best_node, best_value)
        print (best_node, best_value)
        vis.animate_node(start_node.positions, best_node, obstacles, goal, [9])
        return (best_node, best_value)
        
    def insertFrontier(self, pair):
    
        if (len(self.frontier) == 0):
            self.frontier.append(pair)
        else:
            for i in range(0, len(self.frontier)):
                if self.frontier[i][1] < pair[1]:
                    self.frontier.insert(i, pair)
                    return
            self.frontier.append(pair)
            #self.frontier.insert(0,pair)
           
                
                
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
initial_nodes_fixed = {}
initial_nodes_fixed[10] = np.array([0.0, 10.0])
initial_nodes_fixed[11] = np.array([1.0, 10.0])

connections = [(0, 1), (0, 5), (0, 10), (0, 6),
               (1, 2), (1, 6), (1, 7),
               (2, 3), (2, 7), (2, 8),
               (3, 4), (3, 8), (3, 9),
               (4, 9),
               (5, 6), (5,11), (5,10), 
               (6, 7),
               (7, 8),
               (8, 9)]
               
l_groups = (1, 0.5, np.sqrt(2))
k_groups = (1000, 1000)      
ls = np.array([l_groups[0], l_groups[0], l_groups[0], l_groups[2], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[0], l_groups[2],
               l_groups[0], l_groups[0], l_groups[2], l_groups[0], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[0], l_groups[0]])
ls_can_change = [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
ks = np.array([k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0]])


obstacles = [((4, 7), 1)]

# Minimize PE with node positions
res = minimize(op_funs.nodePositionObjective, initial_nodes_free,
               args=(connections, ls, ks, 10, initial_nodes_fixed),
               method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})

print goal1(res.x)
node1 = Node(res.x, initial_nodes_fixed, 10,
             connections, ls, ks, [])
searcher = BFSearch()
searcher.bfs(node1, goal1, float(sys.argv[3]), ls_can_change, l_groups, obstacles)

plt.show()
