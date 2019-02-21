import numpy as np
from scipy.optimize import minimize

import Bodies as bds

# World Information
g = -9.81

# Constants
l_passive = 1
l_active = 0.9*l_passive
k_passive = 6000
k_active = 8000
mass = 1


# Nodes Free
n0 = bds.Node(mass, np.array([0.5, 0.5, 9]), np.zeros(3,), isFixed=False)
n1 = bds.Node(mass, np.array([-0.5, 0.5, 9]), np.zeros(3,), isFixed=False)
n2 = bds.Node(mass, np.array([-0.5, -0.5, 9]), np.zeros(3,), isFixed=False)
n3 = bds.Node(mass, np.array([0.5, -0.5, 9]), np.zeros(3,), isFixed=False)
nodes_free = [n0, n1, n2, n3]

# Fixed Node
n4 = bds.Node(mass, np.array([0.5, 0.5, 10]), np.zeros(3,), isFixed=True)
n5 = bds.Node(mass, np.array([-0.5, 0.5, 10]), np.zeros(3,), isFixed=True)
n6 = bds.Node(mass, np.array([-0.5, -0.5, 10]), np.zeros(3,), isFixed=True)
n7 = bds.Node(mass, np.array([0.5, -0.5, 10]), np.zeros(3,), isFixed=True)
nodes_fixed = [n4, n5, n6, n7]


#TODO
# Springs
s1 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
s2 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
s3 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
s4 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
springs = [s1, s2, s3, s4]

# Connections nodes = [free,fixed]
connections = [(0,4), (1,5), (2, 6), (3,7)]

# Make the system
model = bds.System(nodes_free, nodes_fixed, springs, connections, 2, g)
model.visualize()

x0_pos = np.append(n0.pos, n1.pos, n2.pos, n3.pos)
x0_orient = np.append(n0.orient, n1.orient, n2.orient, n3.orient)
x0 = np.append(x0_pos, x0_orient)
res = minimize(model.modelFunction, x0, method='Nelder-Mead', tol=1e-10)
model.visualize()
print model.modelFunction(res.x)
print '--------------------'
print n1.pos
print n2.pos
print n3.pos
print '--------------------'

