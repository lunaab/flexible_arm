import numpy as np
from scipy.optimize import minimize

import Bodies as bds

# World Information
g = -9.81

# Constants
l_passive = 1
l_active = 0.9*l_passive
k_passive = 1000
k_active = 8000
mass = 0.008377575

# Nodes Free
n0 = bds.Node(mass, np.array([0.70707, 0, 0]), np.zeros(3,), isFixed=False)
n1 = bds.Node(mass, np.array([0.70707, 0.5, 0]), np.zeros(3,), isFixed=False)
n2 = bds.Node(mass, np.array([0.70707, 0.5, 0.5]), np.zeros(3,), isFixed=False)
n3 = bds.Node(mass, np.array([0.70707, 0, 0.5]), np.zeros(3,), isFixed=False)
n4 = bds.Node(mass, np.array([1.41421, 0, 0]), np.zeros(3,), isFixed=False)
n5 = bds.Node(mass, np.array([1.41421, 0.5, 0]), np.zeros(3,), isFixed=False)
n6 = bds.Node(mass, np.array([1.41421, 0.5, 0.5]), np.zeros(3,), isFixed=False)
n7 = bds.Node(mass, np.array([1.41421, 0, 0.5]), np.zeros(3,), isFixed=False)
n8 = bds.Node(mass, np.array([2.12132, 0, 0]), np.zeros(3,), isFixed=False)
n9 = bds.Node(mass, np.array([2.12132, 0.5, 0]), np.zeros(3,), isFixed=False)
n10 = bds.Node(mass, np.array([2.12132, 0.5, 0.5]), np.zeros(3,), isFixed=False)
n11 = bds.Node(mass, np.array([2.12132, 0, 0.5]), np.zeros(3,), isFixed=False)
n12 = bds.Node(mass, np.array([2.82843, 0, 0]), np.zeros(3,), isFixed=False)
n13 = bds.Node(mass, np.array([2.82843, 0.5, 0]), np.zeros(3,), isFixed=False)
n14 = bds.Node(mass, np.array([2.82843, 0.5, 0.5]), np.zeros(3,), isFixed=False)
n15 = bds.Node(mass, np.array([2.82843, 0, 0.5]), np.zeros(3,), isFixed=False)
nodes_free = [n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15]

# Fixed Node
n16 = bds.Node(mass, np.array([0, 0, 0]), np.zeros(3,), isFixed=True)
n17 = bds.Node(mass, np.array([0, 0.5, 0]), np.zeros(3,), isFixed=True)
n18 = bds.Node(mass, np.array([0, 0.5, 0.5]), np.zeros(3,), isFixed=True)
n19 = bds.Node(mass, np.array([0, 0, 0.5]), np.zeros(3,), isFixed=True)
nodes_fixed = [n16, n17, n18, n19]



# Springs
s0 = bds.Spring(None, None, l_passive, k_passive, 0)
s1 = bds.Spring(None, None, l_passive, k_passive, 0)
s2 = bds.Spring(None, None, l_passive, k_passive, 0)
s3 = bds.Spring(None, None, l_passive, k_passive, 0)
s4 = bds.Spring(None, None, l_passive, k_passive, 0)
s5 = bds.Spring(None, None, l_passive, k_passive, 0)
s6 = bds.Spring(None, None, l_passive, k_passive, 0)
s7 = bds.Spring(None, None, l_passive, k_passive, 0)
s8 = bds.Spring(None, None, l_passive, k_passive, 0)
s9 = bds.Spring(None, None, l_passive, k_passive, 0)
s10 = bds.Spring(None, None, l_passive, k_passive, 0)
s11 = bds.Spring(None, None, l_passive, k_passive, 0)
s12 = bds.Spring(None, None, l_passive, k_passive, 0)
s13 = bds.Spring(None, None, l_passive, k_passive, 0)
s14 = bds.Spring(None, None, l_passive, k_passive, 0)
s15 = bds.Spring(None, None, l_passive, k_passive, 0)
s16 = bds.Spring(None, None, l_passive, k_passive, 0)
s17 = bds.Spring(None, None, l_passive, k_passive, 0)
s18 = bds.Spring(None, None, l_passive, k_passive, 0)
s19 = bds.Spring(None, None, l_passive, k_passive, 0)
s20 = bds.Spring(None, None, l_passive, k_passive, 0)
s21 = bds.Spring(None, None, l_passive, k_passive, 0)
s22 = bds.Spring(None, None, l_passive, k_passive, 0)
s23 = bds.Spring(None, None, l_passive, k_passive, 0)
s24 = bds.Spring(None, None, l_passive, k_passive, 0)
s25 = bds.Spring(None, None, l_passive, k_passive, 0)
s26 = bds.Spring(None, None, l_passive, k_passive, 0)
s27 = bds.Spring(None, None, l_passive, k_passive, 0)
s28 = bds.Spring(None, None, l_passive, k_passive, 0)
s29 = bds.Spring(None, None, l_passive, k_passive, 0)
s30 = bds.Spring(None, None, l_passive, k_passive, 0)
s31 = bds.Spring(None, None, l_passive, k_passive, 0)
s32 = bds.Spring(None, None, l_passive, k_passive, 0)
s33 = bds.Spring(None, None, l_passive, k_passive, 0)
s34 = bds.Spring(None, None, l_passive, k_passive, 0)
s35 = bds.Spring(None, None, l_passive, k_passive, 0)
springs = [s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18, s19, s20, s21, s22, s23, s24, s25, s26, s27, s28, s29, s30, s31, s32, s33, s34, s35]

# Connections nodes = [free,fixed]
connections = [(16,17), (17,18), (18,19), (19,16), (19,0), (18,1), (16,1), (17,2), (19,3), (0,1), (1,2), (2,3), (3,0), (0,2), (1,3), (3,4), (2,5), (0,4), (1,5), (2,6), (3,7), (4,5), (5,6), (6,7), (7,4), (4,6), (5,7), (7,8), (6,9), (4,8), (5,9), (6,10), (7,11), (8,9), (9,10), (10,11), (11,8), (8,10), (9,11), (11,12), (10,13), (8,12), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,), (,)]

# Make the system
model = bds.System(nodes_free, nodes_fixed, springs, connections, 2, g)

x0 = np.append(n2.pos, n3.pos)
res = minimize(model.modelFunction, x0, method='Nelder-Mead', tol=1e-10)
