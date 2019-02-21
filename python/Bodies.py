import numpy as np
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

import time

class Node(object):

    def __init__(self, mass, pos, orient, isFixed=False):
        ''' 
            mass: real
            pos: (1,3) np array of doubles
            orient: (1,3) np array of doubles (mod 2pi)
            isFixed: boolean 
        '''
        self.mass = mass
        self.pos = pos
        self.orient = orient
        self.isFixed = isFixed
        
    def gravPotential(self, coord, g):
        return -1*self.mass*g*self.pos[coord]
        
    def move(self, pos, orient):
        if not self.isFixed:
            self.pos = pos
            self.orient = orient
            
    def transformAxis(self, axis):
        
        xrot = np.array([[1, 0, 0],
                         [0, np.cos(self.orient[0]), -1*np.sin(self.orient[0])],
                         [0, np.sin(self.orient[0]), np.cos(self.orient[0])]])
                             
        yrot = np.array([[np.cos(self.orient[1]), 0, np.sin(self.orient[1])],
                         [0, 1, 0],
                         [-1*np.sin(self.orient[1]), 0, np.cos(self.orient[1])]])
                             
        zrot = np.array([[np.cos(self.orient[2]), -1*np.sin(self.orient[2]), 0],
                         [np.sin(self.orient[2]), np.cos(self.orient[2]), 0],
                         [0, 0, 1]])
                             
        rot = np.dot(zrot, yrot)
        rot = np.dot(rot, xrot)

        return np.dot(rot, axis.reshape(3,1))
        
class Spring(object):

    def __init__(self, node1, node2, l0, k, beta, axis1, axis2, smartAxis=False):
        ''' 
            node1: Node
            node2: Node
            l0: real
            k: real
            beta: real
            axis1/2: (1,3) np array unit vector
        '''
        self.node1 = node1
        self.node2 = node2
        self.l0 = l0
        self.k = k
        self.beta = beta
        self.smartAxis = smartAxis
        if not self.smartAxis:
            self.axis1 = axis1
            self.axis2 = axis2
            
    def updateAxis(self):
        if self.smartAxis:
            self.axis1 = self.node2.pos - self.node1.pos
            self.axis1 = self.axis1 / (sum(self.axis1*self.axis1)**0.5)
            self.axis2 = np.copy(self.axis1)
            
    def springPotential(self):
        # Linear potential energy
        dist = np.sum((self.node2.pos - self.node1.pos)**2)**(0.5)
        dx = dist - self.l0
        linear_PE = 0.5*self.k*(dx**2)
        
        # Angular potential energy
        axis1 = self.node1.transformAxis(self.axis1)
        axis2 = self.node2.transformAxis(self.axis2)
        
        val = np.dot(axis1.transpose(), self.node2.pos - self.node1.pos)
        if val > 1.:
            val = 1.
        theta = np.arccos(val)
        
        angular_PE = 0.5*self.beta*(theta**2)
        
        return linear_PE + angular_PE
        
class System(object):

    def __init__(self, nodes_free, nodes_fixed, springs, connections, grav_coord, grav):
        ''' 
            nodes: list of nodes 
            springs: list of springs
            connections: list of (int, int) 
            grav_coord: int
            grav: real
        '''
        self.num_free = len(nodes_free)
        self.num_fixed = len(nodes_fixed)
        self.nodes = nodes_free + nodes_fixed
        self.dims = self.nodes[0].pos.shape[0]
        self.springs = springs
        self.connections = connections
        self.grav_coord = grav_coord
        self.grav =  grav
        
        self.connectNodes(connections)
        
    def calculatePE(self):
        start = time.time()
        PE = 0
        
        for spring in self.springs:
            PE = PE + spring.springPotential()
            
        for node in self.nodes:
            PE = PE + node.gravPotential(self.grav_coord, self.grav)
        end = time.time()
        #print 'FUNC TIME: %f' % (end-start)
        return PE
        
    def connectNodes(self, connections):
        for i in range(0, len(connections)):
            self.springs[i].node1 = self.nodes[connections[i][0]]
            self.springs[i].node2 = self.nodes[connections[i][1]]
            self.springs[i].updateAxis()
            
    def moveNodes(self, new_pos, new_orient):
        if new_pos.shape[0] > self.num_free or new_orient.shape[0] > self.num_free:
            print 'More positions or orientations than nodes in move'
            
        for i in range(0, self.num_free):
            self.nodes[i].move(new_pos[i,:], new_orient[i,:])
        
    def modelFunction(self, param_vec):
        '''
            param_vec: 1d array (x,y,z,x,y,z,...,r,p,y,r,p,y,...)
        '''
        new_pos = np.reshape(param_vec[0:self.num_free*self.dims], (self.num_free, self.dims))
        new_orient = np.reshape(param_vec[self.num_free*self.dims:len(param_vec)], (self.num_free, 3))
        new_orient = new_orient % (2*np.pi)
        
        self.moveNodes(new_pos, new_orient)
        return self.calculatePE()
        
    def visualize(self):
        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(111, projection = '3d')
        ax.set_xlim(-3, 5)
        ax.set_ylim(-3, 5)
        ax.set_zlim(-3, 5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        x1 = []
        x2 = []
        y1 = []
        y2 = []
        z1 = []
        z2 = []
        for spring in self.springs:
            x1.append(spring.node1.pos[0])
            y1.append(spring.node1.pos[1])
            z1.append(spring.node1.pos[2])
            x2.append(spring.node2.pos[0])
            y2.append(spring.node2.pos[1])
            z2.append(spring.node2.pos[2])
            ax.plot([x1[len(x1)-1], x2[len(x2)-1]], [y1[len(y1)-1], y2[len(y2)-1]], zs=[z1[len(z1)-1], z2[len(z2)-1]], color='r') 
        
        ax.scatter(x1 + x2, y1 + y2, z1 + z2, color = 'b')
        #ax.plot([x1, x2], [y1, y2], zs=[z1, z2], color = 'r')
         
        #matplotlib.pyplot.show()      
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
