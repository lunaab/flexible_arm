import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

''' 
This file contains functions that can be used to calculate the potential energy of a 2D
mass-spring system.

It also has some local visualization tools: Note that for a more comprehensive visualization
suite reference the Visualize.py file.

'''


'''
Determines the potential energy of a mass spring system.

Args:
node_position_vector (1D numpy array) : (x1, y1, x2, y2,....) node positions
connections (List of tuples: [(int, int)] : Each tuple corresponds to a spring (in order).
    each element in the tuple is a node index. (a, b) -> this spring is connected from node a to
    node b.
spring_ls (1D numpy array) : unstretched lengths in order
spring_ks (1D numpy array) : stiffnesses in order
num_nodes (int) : number of free nodes
fixed_nodes (Dictionary: {Int, 1D numpy array} ) : fixed node index maps to x, y, z
'''
def potentialEnergy(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes):
    m = 1
    g = 9.81
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))

    # Getting all of the nodes on each end of the spring into two arrays
    spring_nodes_1 = np.zeros((len(connections), 2)) 
    spring_nodes_2 = np.zeros((len(connections), 2)) 
    for i in range(0, len(connections)):
            curr_connect = connections[i]
            if curr_connect[0] < num_nodes:
                spring_nodes_1[i,:] = nodes_xy[curr_connect[0],:]
            else:
                spring_nodes_1[i,:] = fixed_nodes[curr_connect[0]]
            if curr_connect[1] < num_nodes:
                spring_nodes_2[i,:] = nodes_xy[curr_connect[1],:]
            else:
                spring_nodes_2[i,:] = fixed_nodes[curr_connect[1]]
                
    # Gravitational PE
    g_pe = np.sum(m * nodes_xy[:,1] * g)
    
    # Spring PE
    dist = np.sqrt(np.sum((spring_nodes_1 - spring_nodes_2)**2, axis=1))
    s_pe = 0.5 * np.sum(spring_ks * (dist - spring_ls)**2)
    
    return g_pe + s_pe
 
'''
Does exactly the same thing as the function above. Was going to do something different.
'''  
def nodePositionObjective(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes):
    return potentialEnergy(node_position_vector, connections, spring_ls, spring_ks, num_nodes, fixed_nodes)
    

'''
Draws the spring mass system in matplotlib. The system is represented by dots and lines.

Args:
node_position_vector (1D numpy array) : (x1, y1, x2, y2,....) node positions
connections (List of tuples: [(int, int)] : Each tuple corresponds to a spring (in order).
    each element in the tuple is a node index. (a, b) -> this spring is connected from node a to
    node b.
num_nodes (int) : number of free nodes
fixed_nodes (Dictionary: {Int, 1D numpy array} ) : fixed node index maps to x, y, z

'''   
def show_system(node_position_vector, connections, num_nodes, fixed_nodes):

    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    # Axis limits, feel free to remove
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    
    # Draw the springs
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
    # Draw the nodes
    ax.scatter(x1 + x2, y1 + y2, color='b')

'''
Draws the spring mass system in matplotlib. The system is represented by dots and lines.
Also draws a dot at the goal location. Special nodes are colored red.

Args:
node_position_vector (1D numpy array) : (x1, y1, x2, y2,....) node positions
connections (List of tuples: [(int, int)] : Each tuple corresponds to a spring (in order).
    each element in the tuple is a node index. (a, b) -> this spring is connected from node a to
    node b.
num_nodes (int) : number of free nodes
fixed_nodes (Dictionary: {Int, 1D numpy array} ) : fixed node index maps to x, y, z
goal (List of tuples: [(Int, Int)]) : x and y of points to draw that are green
targ_nodes (List of Int) : Indices of nodes in system to draw red 

'''       
def show_system_goal(node_position_vector, connections, num_nodes, fixed_nodes, goal, targ_nodes):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    # Axis limits, feel free to remove
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    
    # Draw springs
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
    # Draw blue nodes
    ax.scatter(x1 + x2, y1 + y2, color='b')
    
    x = []
    y = []
    # Draw red nodes
    for i in range(0, len(targ_nodes)):
        x.append(nodes_xy[targ_nodes[i],0])
        y.append(nodes_xy[targ_nodes[i],1])
    ax.scatter(x, y, color='r')
    
    # Draw green goals
    ax.scatter([goal[0,0]], [goal[0,1]], color='g')

'''
Draws the spring mass system in matplotlib. The system is represented by dots and lines.
Uses the same ax so that all plots end up on the same plot.

Args:
node_position_vector (1D numpy array) : (x1, y1, x2, y2,....) node positions
connections (List of tuples: [(int, int)] : Each tuple corresponds to a spring (in order).
    each element in the tuple is a node index. (a, b) -> this spring is connected from node a to
    node b.
num_nodes (int) : number of free nodes
fixed_nodes (Dictionary: {Int, 1D numpy array} ) : fixed node index maps to x, y, z
ax (Matplotlib Ax) : previous ax for plotting on.

'''    
def show_system_same_plot(node_position_vector, connections, num_nodes, fixed_nodes, ax):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    
    # Axis limits, feel free to remove
    ax.set_xlim(-8.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    
    # Draw the springs
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
    # Draw nodes
    ax.scatter(x1 + x2, y1 + y2, color='b')
    

'''# Initial Configuration of node positions
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
# Fixed node dictionary
initial_nodes_fixed = {}
initial_nodes_fixed[10] = np.array([0.0, 10.0])
initial_nodes_fixed[11] = np.array([1.0, 10.0])

# List of spring connections
connections = [(0, 1), (0, 5), (0, 10), (0, 6),
               (1, 2), (1, 6), (1, 7),
               (2, 3), (2, 7), (2, 8),
               (3, 4), (3, 8), (3, 9),
               (4, 9),
               (5, 6), (5,11), (5,10), 
               (6, 7),
               (7, 8),
               (8, 9)]

# Unstretch Length possibilities (passive, active)               
l_groups = (1, 0.5, np.sqrt(2))
# Stiffnesses posibilities (passive, active)
k_groups = (1000, 1000)

# Unstretch lengths of each spring      
ls = np.array([l_groups[0], l_groups[0], l_groups[0], l_groups[2], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[0], l_groups[2],
               l_groups[0], l_groups[0], l_groups[2], l_groups[0], l_groups[0],
               l_groups[0], l_groups[2], l_groups[0], l_groups[1], l_groups[1]])
               
# List of if springs can have characteristics changed
# 1 is True, 0 is False
ls_can_change = [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]

# Stiffnesses of each spring
ks = np.array([k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0],
               k_groups[0], k_groups[0], k_groups[0], k_groups[0], k_groups[0]])

                            

# Minimize PE with node positions
res = minimize(nodePositionObjective, initial_nodes_free,
               args=(connections, ls, ks, 10, initial_nodes_fixed),
               method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': True})

# Draw and show the system before/after optimization
show_system(initial_nodes_free, connections, 10, initial_nodes_fixed)
show_system(res.x, connections, 10, initial_nodes_fixed)
plt.show()'''
