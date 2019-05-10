import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import OptimizationFunctions as op_fun
import time

'''
This file is used for visualizing the node spring system used in Search.py and
OptimizationFunctions.py

Some of the methods are similar/identical to those found in OptimizationFunctions.py
'''

'''
Creates an animation of the node following its control inputs

Args:
initial_positions (1D numpy array) : (x1, y1, x2, y2, ...) node positions at start
graph_node (Node) : Final node containing a control input list
obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius)
goal (List of tuples: [(Int, Int)]) : x and y of points to draw that are green
targ_nodes (List of Int) : Indices of nodes in system to draw red

'''
def animate_node(initial_positions, graph_node, obstacles, goal, targ_nodes):
    
    # Positions is a list of configuration positions
    positions = [initial_positions]
    
    # Add one position per control sequence
    for i in range(1, len(graph_node.control_arr)):
        res = minimize(op_fun.nodePositionObjective, positions[len(positions)-1],
                       args=(graph_node.connections, graph_node.control_arr[i], graph_node.k_arr,
                       graph_node.num_nodes, graph_node.fixed_nodes),
                       method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
        positions.append(np.copy(res.x))
        
    res = minimize(op_fun.nodePositionObjective, positions[len(positions)-1],
                       args=(graph_node.connections, graph_node.l0_arr, graph_node.k_arr,
                       graph_node.num_nodes, graph_node.fixed_nodes),
                       method='BFGS', tol=1e-6, options={'gtol': 1e-6, 'disp': False})
    positions.append(np.copy(res.x))
    
    
    # Prepare figure
    fig = plt.figure()
    plt.ion()
    ax = fig.add_subplot(111)
    i = 0
    
    # Loop through configurations in the positions variable
    while True:
        nodes_xy = np.reshape(positions[i], (10, 2))
        show_system_same_plot_goal_obs(positions[i], graph_node.connections, graph_node.num_nodes, graph_node.fixed_nodes, ax, goal, targ_nodes, obstacles)
        
        plt.show()
        plt.pause(0.2)
        ax.clear()
        # Cycles
        i = i + 1
        if i == len(positions):
            i = 0
            
    
            
    
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
    
    # Sets limits feel free to remove
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    # Adds springs
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
        
    # Adds nodes
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
    # Axis limits feel free to remove
    ax.set_xlim(-3.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    
    # Draws springs
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
    # Draws nodes
    ax.scatter(x1 + x2, y1 + y2, color='b')
    
    # Draws goal
    x = []
    y = []
    for i in range(0, len(targ_nodes)):
        x.append(nodes_xy[targ_nodes[i],0])
        y.append(nodes_xy[targ_nodes[i],1])
    ax.scatter(x, y, color='r')
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
    # Axis limits feel free to remove
    ax.set_xlim(-8.0, 8)
    ax.set_ylim(3, 11)
    x1 = []
    x2 = []
    y1 = []
    y2 = []
    # Draws springs
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
    # Draws nodes
    ax.scatter(x1 + x2, y1 + y2, color='b')

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
obstacles (List of ((Double, Double), Int) : Obstacles ((x position, y position), radius)
'''    
def show_system_same_plot_goal_obs(node_position_vector, connections, num_nodes, fixed_nodes, ax, goal, targ_nodes, obstacles):
    nodes_xy = np.reshape(node_position_vector, (num_nodes, 2))
    # Axis limits feel free to remove
    ax.set_xlim(-8.0, 8)
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
    # Draw nodes blue
    ax.scatter(x1 + x2, y1 + y2, color='b')
    
    x = []
    y = []
    # Draw target nodes red
    for i in range(0, len(targ_nodes)):
        x.append(nodes_xy[targ_nodes[i],0])
        y.append(nodes_xy[targ_nodes[i],1])
    ax.scatter(x, y, color='r')
    # Draw goal nodes green
    ax.scatter([goal[0,0]], [goal[0,1]], color='g')
    
    # Draw obstacles
    for obstacle in obstacles:
        ax.add_artist(plt.Circle((obstacle[0][0], obstacle[0][1]), obstacle[1], color='b'))
