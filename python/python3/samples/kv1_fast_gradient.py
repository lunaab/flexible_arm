import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import copy
import time

import BodiesFast as bds
import BodyBuilderFast as bld

'''
This file build a kv1 hexagonal close packed structure and activates the 6th tendon
'''

springs_of_interest = [6]

# World Information
g = -9.81
g_coord = 2

# Constants
l_passive = 1
l_active = 0.5
k_passive = 10
k_active = 60
beta_passive = 0
beta_active = 0
mass = 0.008377575

# Nodes Free
n0 = bds.Node(mass, np.array([-0.00000,	0.00000, 3.98789]), np.zeros(3,), isFixed=False)
n1 = bds.Node(mass, np.array([-0.00127, -0.99671, 3.98333]), np.zeros(3,), isFixed=False)
n2 = bds.Node(mass, np.array([-0.86254, -0.49945, 3.98333]), np.zeros(3,), isFixed=False)
n3 = bds.Node(mass, np.array([-0.86254, 0.49945, 3.98333]), np.zeros(3,), isFixed=False)
n4 = bds.Node(mass, np.array([-0.00127, 0.99671, 3.98333]), np.zeros(3,), isFixed=False)
n5 = bds.Node(mass, np.array([0.86381, 0.49725, 3.98333]), np.zeros(3,), isFixed=False)
n6 = bds.Node(mass, np.array([0.86381, -0.49725, 3.98333]), np.zeros(3,), isFixed=False)
n7 = bds.Node(mass, np.array([-0.28821, -0.49919, 3.16264]), np.zeros(3,), isFixed=False)
n8 = bds.Node(mass, np.array([-0.28821, 0.49919, 3.16264]), np.zeros(3,), isFixed=False)
n9 = bds.Node(mass, np.array([0.57641, 0.00000, 3.16264]), np.zeros(3,), isFixed=False)
nodes_free = [n0, n1, n2, n3, n4, n5, n6, n7, n8, n9]

# Fixed Node
n10 = bds.Node(mass, np.array([-0.28868, -0.50000, 4.81650]), np.zeros(3,), isFixed=False)
n11 = bds.Node(mass, np.array([-0.28868, 0.50000, 4.81650]), np.zeros(3,), isFixed=False)
n12 = bds.Node(mass, np.array([0.57735, 0.00000, 4.81650]), np.zeros(3,), isFixed=False)
nodes_fixed = [n10, n11, n12]

# Springs
springs = []
for i in range(0, 36):
    if i in springs_of_interest:
        springs.append(bds.Spring(None, None, l_active, k_active, 0, np.array([0, 1, 0]), np.array([0, 1, 0])))
    else:
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
model.visualize_springs(k_passive, k_active)
#plt.show()
# Show the initial node positions
print('_________________ORIGINAL___________________')
for node in nodes_free:
    print(node.pos)

# Order the poses
x0_pos = n0.pos
x0_orient = n0.orient
for i in range(1,10):
    x0_pos = np.append(x0_pos, nodes_free[i].pos)
    x0_orient = np.append(x0_orient, nodes_free[i].orient)
x0 = np.append(x0_pos, x0_orient)
start = time.time()

# Move the system by minimizing potential energy
res = minimize(model.modelFunction, x0, jac = model.calculatePEGradient, method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': True})
model.moveNodes(res.x)

end = time.time()
print('TIME: ', (end-start))
# Show the final node positions
print('_________________AFTER___________________')
for node in nodes_free:
    print(node.pos)
print('Gradient Norm: ', str(np.sqrt(np.sum(model.calculatePEGradient(res.x)**2))))
model.visualize_springs(k_passive, k_active)
plt.show()



