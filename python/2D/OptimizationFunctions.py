import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def potentialEnergy(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes):
    m = 1
    g = 9.81
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    #print fixed_nodes
    spring_nodes_1 = np.zeros((len(connections), 2)) 
    spring_nodes_2 = np.zeros((len(connections), 2)) 
    for i in range(0, len(connections)):
            curr_connect = connections[i]
            if curr_connect[0] < num_nodes:
                spring_nodes_1[i,:] = nodes_xy[curr_connect[0],:]
            else:
                #print curr_connect[0]
                spring_nodes_1[i,:] = fixed_nodes[curr_connect[0]]
            if curr_connect[1] < num_nodes:
                spring_nodes_2[i,:] = nodes_xy[curr_connect[1],:]
            else:
                #print curr_connect[1]
                spring_nodes_2[i,:] = fixed_nodes[curr_connect[1]]
                
    # Gravitational PE
    g_pe = np.sum(m * nodes_xy[:,1] * g)
    
    # Spring PE
    #print spring_nodes_1
    dist = np.sqrt(np.sum((spring_nodes_1 - spring_nodes_2)**2, axis=1))
    #print dist
    s_pe = 0.5 * np.sum(spring_ks * (dist - spring_ls)**2)
    
    return g_pe + s_pe
    
def unstretchedLengthObjective(spring_ls, node_position_vector, connections, spring_ks, ls_possible, ks_possible, num_nodes, fixed_nodes):
    #print ls_possible
    alpha = 10
    pe = potentialEnergy(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes)
    l_error = np.zeros((len(connections),2))
    l_error[:,0] = np.absolute(spring_ls - ls_possible[0])
    l_error[:,1] = np.absolute(spring_ls - ls_possible[1])
    l_error = np.min(l_error, axis=1)
    
    #print "PE: %2.3f" % pe
    #print "LE: %2.3f" % np.sum(alpha*l_error)
    return pe + np.sum(alpha*l_error)
    
def nodePositionObjective(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes):
    return potentialEnergy(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes)
    
    
def show_system(node_position_vector, connections, num_nodes, fixed_nodes):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    for i in range(0, len(connections)):
        curr_connect = connections[i]
        if curr_connect[0] < num_nodes:
            x1.append(nodes_xy[curr_connect[0],0])
            y1.append(nodes_xy[curr_connect[0],1])
        else:
            x1.append(fixed_nodes[curr_connect[0]][0])
            y1.append(fixed_nodes[curr_connect[0]][1])
        if curr_connect[1] < num_nodes:
            x2.append(nodes_xy[curr_connect[1],0])
            y2.append(nodes_xy[curr_connect[1],1])
        else:
            
            x2.append(fixed_nodes[curr_connect[1]][0])
            y2.append(fixed_nodes[curr_connect[1]][1])
        ax.plot([x1[len(x1)-1], x2[len(x2)-1]], [y1[len(y1)-1], y2[len(y2)-1]], color='#797978')
    ax.scatter(x1 + x2, y1 + y2, color='b')
    
def show_system_goal(node_position_vector, connections, num_nodes, fixed_nodes, goal, targ_nodes):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    for i in range(0, len(connections)):
        curr_connect = connections[i]
        if curr_connect[0] < num_nodes:
            x1.append(nodes_xy[curr_connect[0],0])
            y1.append(nodes_xy[curr_connect[0],1])
        else:
            x1.append(fixed_nodes[curr_connect[0]][0])
            y1.append(fixed_nodes[curr_connect[0]][1])
        if curr_connect[1] < num_nodes:
            x2.append(nodes_xy[curr_connect[1],0])
            y2.append(nodes_xy[curr_connect[1],1])
        else:
            
            x2.append(fixed_nodes[curr_connect[1]][0])
            y2.append(fixed_nodes[curr_connect[1]][1])
        ax.plot([x1[len(x1)-1], x2[len(x2)-1]], [y1[len(y1)-1], y2[len(y2)-1]], color='#797978')
    ax.scatter(x1 + x2, y1 + y2, color='b')
    
    x = []
    y = []
    for i in range(0, len(targ_nodes)):
        x.append(nodes_xy[targ_nodes[i],0])
        y.append(nodes_xy[targ_nodes[i],1])
    ax.scatter(x, y, color='r')
    ax.scatter([goal[0,0]], [goal[0,1]], color='g')
    
def show_system_same_plot(node_position_vector, connections, num_nodes, fixed_nodes, ax):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    ax.set_xlim(-8.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    for i in range(0, len(connections)):
        curr_connect = connections[i]
        if curr_connect[0] < num_nodes:
            x1.append(nodes_xy[curr_connect[0],0])
            y1.append(nodes_xy[curr_connect[0],1])
        else:
            x1.append(fixed_nodes[curr_connect[0]][0])
            y1.append(fixed_nodes[curr_connect[0]][1])
        if curr_connect[1] < num_nodes:
            x2.append(nodes_xy[curr_connect[1],0])
            y2.append(nodes_xy[curr_connect[1],1])
        else:
            
            x2.append(fixed_nodes[curr_connect[1]][0])
            y2.append(fixed_nodes[curr_connect[1]][1])
        ax.plot([x1[len(x1)-1], x2[len(x2)-1]], [y1[len(y1)-1], y2[len(y2)-1]], color='#797978')
    ax.scatter(x1 + x2, y1 + y2, color='b')
    

'''# Initial Configuration
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
               l_groups[0], l_groups[2], l_groups[0], l_groups[1], l_groups[1]])
ls_can_change = [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
ks = np.array([k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0]])


# Goal Configuration'''
'''goal_nodes_free = np.array([[0.0, 9.0],
                            [0.0, 8.0],
                            [1.25, 6.5],
                            [2.25, 6.5],
                            [3, 7.1],
                            [1.0, 9.0],
                            [1.0, 8.0],
                            [1.5, 7.1],
                            [2.5,7.1],
                            [2.75, 7.5]]).reshape(20,) 
                            
# Minimize PE with unstretched lengths
res = minimize(op.unstretchedLengthObjective, ls,
               args=(goal_nodes_free, connections, ks, l_groups, k_groups, 10, initial_nodes_fixed),
               method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': True})
#ls = np.copy(res.x)
print ls '''
'''for i in range(0, len(ls)):
    high = np.absolute(ls[i] - l_groups[0])
    low = np.absolute(ls[i] - l_groups[1])
    if high < low:
        ls[i] = l_groups[0]
    else:
        ls[i] = l_groups[1]'''
'''# Minimize PE with node positions
res = minimize(nodePositionObjective, initial_nodes_free,
               args=(connections, ls, ks, 10, initial_nodes_fixed),
               method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': True})
#print res.x
#print connections
show_system(initial_nodes_free, connections, 10, initial_nodes_fixed)
show_system(res.x, connections, 10, initial_nodes_fixed)
plt.show()'''
