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
l_active = 0.5
k_passive = 10
k_active = 60
beta_passive = 0
beta_active = 0
mass = 0.008377575

for w in range(0, 36):
    for n in range(w+1, 36):

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
        springs_of_interest = [w, n]
        springs = []
        for j in range(0, 36):
            if j in springs_of_interest:
                springs.append(bds.Spring(None, None, l_active, k_active, 0, np.array([0, 1, 0]), np.array([0, 1, 0])))
            else:
                springs.append(bds.Spring(None, None, l_passive, k_passive, 0, np.array([0, 1, 0]), np.array([0, 1, 0])))
    

        # Connections nodes = [free,fixed]
        connections = [(0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (0,7), (0,8), (0,9), (0,10), (0,11), (0,12),  
               (1,2), (2,3), (3,4), (4,5), (5,6), (6,1), (7,8),
               (8,9), (9,7), (10,11), (11,12), (12,10),
               (1,7), (2,7), (3,8), (4,8), (5,9), (6,9), (1,10), (2,10),
               (3,11), (4,11), (5,12), (6,12)]

    # Make the system
        model = bds.System(nodes_free, nodes_fixed, springs, connections, 2, g)
        model.visualize_specific_springs(springs_of_interest)
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
        res = minimize(model.modelFunction, x0, jac = model.calculatePEGradient, method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': True}) #jac = model.calculatePEGradient
        model.moveNodes(res.x)
        end = time.time()
        print 'TIME: %f' % (end-start)
        #print 'ITERATIONS: %d' % res.nit
        #print 'EVALS: %d' % res.nfev
        #print 'VALUE: %f' % res.fun
        print '_________________AFTER___________________'
        for node in nodes_free:
            print node.pos
        file_name = 'k2_v2/k2_v2_luna/t_' + str(springs_of_interest[0]) + '_' + str(springs_of_interest[1]) + '.txt'
        model.storeState(file_name)
        #print model.l0_mat
        #model.visualize_springs(k_passive, k_active)
        #plt.show()



