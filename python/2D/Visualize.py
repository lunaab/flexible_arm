import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import OptimizationFunctions as op_fun
import time

def animate_node(initial_positions, graph_node, obstacles, goal, targ_nodes):
    
    positions = [initial_positions]
    #print len(graph_node.control_arr)
    for i in range(1, len(graph_node.control_arr)):
        #print graph_node.control_arr[i]
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
    
    
    fig = plt.figure()
    plt.ion()
    ax = fig.add_subplot(111)
    i = 0
    while True:
        nodes_xy = np.reshape(positions[i], (10, 2))
        #print np.linalg.norm(nodes_xy[9] - np.array([4, 9]))
        show_system_same_plot_goal_obs(positions[i], graph_node.connections, graph_node.num_nodes, graph_node.fixed_nodes, ax, goal, targ_nodes, obstacles)
        
        plt.show()
        plt.pause(0.2)
        ax.clear()
        # Cycles
        i = i + 1
        if i == len(positions):
            i = 0
            
    
            
    
    

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
    
def show_system_same_plot_goal_obs(node_position_vector, connections, num_nodes, fixed_nodes, ax, goal, targ_nodes, obstacles):
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
    
    x = []
    y = []
    for i in range(0, len(targ_nodes)):
        x.append(nodes_xy[targ_nodes[i],0])
        y.append(nodes_xy[targ_nodes[i],1])
    ax.scatter(x, y, color='r')
    ax.scatter([goal[0,0]], [goal[0,1]], color='g')
    
    for obstacle in obstacles:
        ax.add_artist(plt.Circle((obstacle[0][0], obstacle[0][1]), obstacle[1], color='b'))
