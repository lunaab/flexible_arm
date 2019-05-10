import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import BodiesFast as bds
import BodyBuilderFast as bld

# World Information
g = -9.81
g_coord = 2

# Constants
l_passive = 1
l_active = 0.9*l_passive
k_passive = 6000
k_active = 8000
beta_passive = 500
beta_active = 1000
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

bldr = bld.BodyBuilder(g, g_coord)
my_sys = bldr.buildRandomBody(nodes_free, nodes_fixed,
                             (l_passive, l_active),
                             (k_passive, k_active),
                             (beta_passive, beta_active))

my_sys.visualize()
#plt.show()

print "++++++++++++++++++++++++++++++INITIAL SYSTEM++++++++++++++++++++++++++++++++++++++"                            
print my_sys.connections
i = 0
print my_sys.l0_mat
for spring in my_sys.springs:
#    print "[%d]: %f" % (i, spring.l0)
    i = i+1
    
new_sys = bldr.buildRandomNeighborWithGivenSprings(my_sys,
                             (l_passive, l_active),
                             (k_passive, k_active),
                             (beta_passive, beta_active))
print "++++++++++++++++++++++++++++++NEW SYSTEM++++++++++++++++++++++++++++++++++++++" 
print new_sys.connections
i = 0
print new_sys.l0_mat
for spring in new_sys.springs:
#    print "[%d]: %f" % (i, spring.l0)
    i = i+1
