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

# Fixed Node
n1 = bds.Node(mass, np.array([0, 0, 10]), np.zeros(3,), isFixed=True)
nodes_fixed = [n1]

# Nodes Free
n2 = bds.Node(mass, np.array([0, 0, 9]), np.zeros(3,), isFixed=False)
n3 = bds.Node(mass, np.array([0, 1, 7]), np.zeros(3,), isFixed=False)
nodes_free = [n2, n3]

# Springs
s1 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
s2 = bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 0, 1]), np.array([0, 0, 1]))
springs = [s1, s2]

# Connections nodes = [free,fixed]
connections = [(2,0), (0,1)]

# Make the system
model = bds.System(nodes_free, nodes_fixed, springs, connections, 2, g)

x0_pos = np.append(n2.pos, n3.pos)
x0_orient = np.append(n2.orient, n3.orient)
x0 = np.append(x0_pos, x0_orient)
res = minimize(model.modelFunction, x0, method='Nelder-Mead', tol=1e-10)
print model.modelFunction(res.x)
print '--------------------'
print n1.pos
print n2.pos
print n3.pos
print '--------------------'

