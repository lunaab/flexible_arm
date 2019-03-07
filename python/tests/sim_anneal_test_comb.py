import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import copy
import time

import Optimizers
import BodiesFast as bds
import BodyBuilderFast as bld


def obj_func(system):

    ret_system = copy.deepcopy(system)
    
    x0_pos = n0.pos
    x0_orient = n0.orient
    for i in range(1,10):
        x0_pos = np.append(x0_pos, system.nodes[i].pos)
        x0_orient = np.append(x0_orient, system.nodes[i].orient)
    x0 = np.append(x0_pos, x0_orient)
    res = minimize(ret_system.modelFunction, x0, jac=ret_system.calculatePEGradient, method='BFGS', tol=1e-6)
    ret_system.moveNodes(res.x)
    
    x1_old = system.nodes[7].pos[0]
    x2_old = system.nodes[8].pos[0]
    x3_old = system.nodes[9].pos[0]
    
    old = (x1_old + x2_old + x3_old) / 3.0
    
    x1_new = ret_system.nodes[7].pos[0]
    x2_new = ret_system.nodes[8].pos[0]
    x3_new = ret_system.nodes[9].pos[0]
    
    new = (x1_new + x2_new + x3_new) / 3.0
    
    return -1*np.sqrt((old-new)**2)
    
    #x1 = np.absolute(x1_old - x1_new)
    #x2 = np.absolute(x2_old - x2_new)
    #x3 = np.absolute(x3_old - x3_new)
    
    #return -1 * (x1 + x2 + x3) / 3.0
    

def moveSystem(system):
    x0_pos = n0.pos
    x0_orient = n0.orient
    for i in range(1,10):
        x0_pos = np.append(x0_pos, system.nodes[i].pos)
        x0_orient = np.append(x0_orient, system.nodes[i].orient)
    x0 = np.append(x0_pos, x0_orient)
    res = minimize(system.modelFunction, x0, jac=system.calculatePEGradient, method='BFGS', tol=1e-6)
    system.moveNodes(res.x)    

def neighbor_func(system):
    bldr = bld.BodyBuilder(-9.81, 2)
    ls = [1.0, 0.5]
    ks = [10, 16] #10, 60
    betas = [0, 0]
    return bldr.buildRandomNeighborWithGivenSprings(system, ls, ks, betas)

# World Information
g = -9.81
g_coord = 2

# Anneal variables
step_size = 0.01

# Constants
l_passive = 1
l_active = None
k_passive = 10
k_active = None
beta_passive = 0
beta_active = 0
mass = 0.008377575

# Nodes Free
n0 = bds.Node(mass, np.array([0.0, 0.0, 4.0]), np.zeros(3,), isFixed=False)
n1 = bds.Node(mass, np.array([-1.0, 0.0, 4.0]), np.zeros(3,), isFixed=False)
n2 = bds.Node(mass, np.array([-0.5, 0.86603, 4.0]), np.zeros(3,), isFixed=False)
n3 = bds.Node(mass, np.array([0.5, 0.86603, 4.0]), np.zeros(3,), isFixed=False)
n4 = bds.Node(mass, np.array([1.0, 0.0, 4.0]), np.zeros(3,), isFixed=False)
n5 = bds.Node(mass, np.array([0.5, -0.86603, 4.0]), np.zeros(3,), isFixed=False)
n6 = bds.Node(mass, np.array([-0.5, -0.86603, 4.0]), np.zeros(3,), isFixed=False)
n7 = bds.Node(mass, np.array([-0.5, 0.28868, 3.1835]), np.zeros(3,), isFixed=False)
n8 = bds.Node(mass, np.array([0.5, 0.28868, 3.1835]), np.zeros(3,), isFixed=False)
n9 = bds.Node(mass, np.array([0.0, -0.57735, 3.1835]), np.zeros(3,), isFixed=False)
nodes_free = [n0, n1, n2, n3, n4, n5, n6, n7, n8, n9]

# Fixed Node
n10 = bds.Node(mass, np.array([-0.5, 0.28868, 4.8165]), np.zeros(3,), isFixed=False)
n11 = bds.Node(mass, np.array([0.5, 0.28868, 4.8165]), np.zeros(3,), isFixed=False)
n12 = bds.Node(mass, np.array([0.0, -0.57735, 4.8165]), np.zeros(3,), isFixed=False)
nodes_fixed = [n10, n11, n12]

# Springs
springs = []
for i in range(0, 36):
    springs.append(bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0])))

# Connections nodes = [free,fixed]
connections = [(0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (0,7), (0,8), (0,9), (0,10), (0,11), (0,12),  
               (1,2), (1,6), (1,7), (1,10),
               (2,3), (2,7), (2,10),
               (3,4), (3,8), (3,11), 
               (4,5), (4,8), (4,11),
               (5,6), (5,9), (5,12),
               (6,9), (6,12),
               (7,8), (7,9), 
               (8,9),
               (10,11), (10,12),
               (11,12)]
      
        

# Make the system
model = bds.System(nodes_free, nodes_fixed, springs, connections, 2, g)
annealer = Optimizers.SimulatedAnnealing(obj_func, neighbor_func, model)
best = annealer.optimize(max_iters=10, inner_iters=100, alpha=0.7, T=1)
moveSystem(best[0])
#
for i in range(0, 3):
    print best[0].nodes[best[0].num_free + i].pos
#
print best
best[0].visualize_specific_springs((30, 31, 32))
plt.show()
