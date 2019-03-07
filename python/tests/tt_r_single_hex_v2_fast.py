import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import copy
import time

import BodiesFast as bds
import BodyBuilder as bld

# World Information
g = -9.81
g_coord = 2

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
s0 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s1 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s2 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s3 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s4 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s5 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s6 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s7 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s8 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s9 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s10 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s11 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s12 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s13 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s14 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s15 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s16 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s17 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s18 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s19 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s20 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s21 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s22 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s23 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s24 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s25 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s26 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s27 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s28 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s29 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s30 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s31 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s32 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s33 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s34 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
s35 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0]))
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
model.visualize()
#plt.show()
print '_________________ORIGINAL___________________'
for node in nodes_free:
    print node.pos

x0_pos = n0.pos
x0_orient = n0.orient
for i in range(1,10):
    x0_pos = np.append(x0_pos, nodes_free[i].pos)
    x0_orient = np.append(x0_orient, nodes_free[i].orient)
x0 = np.append(x0_pos, x0_orient)
start = time.time()
res = minimize(model.modelFunction, x0, method='Nelder-Mead', tol=1e-6)
model.moveNodes(res.x)
end = time.time()
print 'TIME: %f' % (end-start)
print 'ITERATIONS: %d' % res.nit
print 'EVALS: %d' % res.nfev
print 'VALUE: %f' % res.fun
print '_________________AFTER___________________'
for node in nodes_free:
    print node.pos
print model.l0_mat
model.visualize()
#plt.show()



