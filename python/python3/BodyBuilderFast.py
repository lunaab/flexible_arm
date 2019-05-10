import numpy as np
from scipy.optimize import minimize

import copy

import BodiesFast as bds

'''
Factory class used to create spring-mass systems.

'''

class BodyBuilder(object):

    def __init__(self, grav, grav_coord):
        self.grav = grav
        self.grav_coord = grav_coord
    
    '''
    Creates a node-spring system
    
    '''
    def buildBody(self, nodes_free, nodes_fixed, connects):
        ''' nodes: list of (1,4) tuple. (m, (x, y, z), (r, p, y), Fixed)
                        (x,y,z) and (r,p,y) are (1,3) np array
            connections: list of (1,6) np array. (n1_idx, n2_idx, l0, k, beta, axis)
                           axis (1,3) np array
            grav_coord: int
            grav: real '''
            
        # Make the nodes
        free_nodes = []
        fixed_nodes = []
        for node in nodes_free:
            free_nodes.append(bds.Node(node[0], node[1], node[2], node[3]))
        for node in nodes_fixed:
            fixed_nodes.append(bds.Node(node[0], node[1], node[2], node[3]))
        
        # Make the springs        
        springs = []
        connections = []
        for item in connects:
            connections.append((item[0], item[1]))
            springs.append(bds.Spring(None, None, item[2], item[3], item[4], item[5]))
            
        return bds.System(free_nodes, fixed_nodes, springs, connections, self.grav, self.grav_coord)
    
    '''
    Given free and fixed nodes and possible spring characteristics randomly
    connects the nodes with randomly passive or active springs
    '''        
    def buildRandomBody(self, nodes_free, nodes_fixed, ls, ks, betas):
        ''' nodes_free: list of Nodes objects that can move
            nodes_fixed: list of Nodes objects that cannot move
            ls: 2 tuple (passive l0, active l0)
            ks: 2 tuple (passive k, active k)
            betas: 2 tuple (passive beta, active beta)
        '''
        num_nodes = len(nodes_free) + len(nodes_fixed)
        max_springs = int((num_nodes*(num_nodes - 1)) / 2.)
        
        # Connect top triangle of adj matrix (1: connected, 0: Not)
        connects = np.random.randint(2, size=max_springs)
        # List of active springs (1: active, 0: passive)
        active = np.random.randint(2, size = int(np.sum(connects)))
        
        connections = []
        springs = []
        active_count = 0
        n1 = 0 #node 1
        n2 = 1 #node 2
        # Iterates through all combinations of n1 and n2 order does not matter
        for i in range(0, max_springs):
            # Make spring
            if connects[i]:
                connections.append((n1, n2))
                # check if this spring is active
                if active[active_count]:
                    springs.append(bds.Spring(None, None, ls[1], ks[1], betas[1],
                                                 None, None, smartAxis=True))
                else:
                   springs.append(bds.Spring(None, None, ls[0], ks[0], betas[0],
                                                 None, None, smartAxis=True))
                active_count = active_count + 1
            n2 = n2 + 1
            if n2 == num_nodes:
                n1 = n1 + 1
                n2 = n1 + 1
                
        return bds.System(nodes_free, nodes_fixed, springs, connections, self.grav, self.grav_coord)
    
    '''
    generates a new system by adding one spring
    or changing the one spring from active/passive to passive/active
    
    Args:
    system (System) : the system you want to generate a neighbor for
    ls (Tuple (double, double)) : passive and active unstretched length
    ks (Tuple (double, double)) : passive and active stiffness
    betas (Tuple (double, double)) : passive and active angular stiffness
    '''
    def buildRandomNeighbor(self, system, ls, ks, betas):
        ret_system = copy.deepcopy(system)
        
        change = np.random.randint(3, size=1)
        
        # Add spring
        if change != 2:
            count = 0
            pair = np.random.randint(len(ret_system.nodes), size=2)
            while True:
                if pair[0] == pair[1]:
                    pair = np.random.randint(len(ret_system.nodes), size=2)
                    continue
                try:
                    ret_system.connections.index((pair[0], pair[1]))
                    isNew = False
                except:
                    count = count + 1
                try:
                    ret_system.connections.index((pair[1], pair[0]))
                    isNew = False
                except:
                    count = count + 1
                    
                if count == 2:
                    break
                count = 0
                pair = np.random.randint(len(ret_system.nodes), size=2)
            
            # Active Spring
            if change == 0:
                new_spring = bds.Spring(ret_system.nodes[pair[0]], ret_system.nodes[pair[1]],
                                           ls[1], ks[1], betas[1],
                                           None, None, smartAxis=True)
                new_spring.updateAxis()
                ret_system.addSpring(new_spring, (pair[0], pair[1]))
            
            # Passive Spring
            elif change == 1:
                new_spring = bds.Spring(ret_system.nodes[pair[0]], ret_system.nodes[pair[1]],
                                           ls[0], ks[0], betas[0],
                                           None, None, smartAxis=True)
                new_spring.updateAxis()
                ret_system.addSpring(new_spring, (pair[0], pair[1]))
                
            
        # Delete spring
        elif change == 2:
            idx = np.random.randint(len(ret_system.springs), size=1)[0]
            ret_system.removeSpring(idx)
        return ret_system
        
    '''
    generates a new system by
    changing one spring from active/passive to passive/active
    
    Args:
    system (System) : the system you want to generate a neighbor for
    ls (Tuple (double, double)) : passive and active unstretched length
    ks (Tuple (double, double)) : passive and active stiffness
    betas (Tuple (double, double)) : passive and active angular stiffness
    '''
    def buildRandomNeighborWithGivenSprings(self, system, ls, ks, betas):
        ''' system: System Object from bds.py
            ls: 2 tuple (passive l0, active l0)
            ks: 2 tuple (passive k, active k)
            betas: 2 tuple (passive beta, active beta)
        '''
        ret_system = copy.deepcopy(system)
        spring_idx = np.random.randint(len(ret_system.springs), size=1)[0]
        
        if ret_system.springs[spring_idx].l0 == ls[0]:
            ret_system.updateSpring(spring_idx, ls[1], ks[1], betas[1])
        else:
            ret_system.updateSpring(spring_idx, ls[0], ks[0], betas[0])
        
        return ret_system
    
    '''
    generates a random system, but the connections in the system passed in the argument
    are kept the same. IE randomly changes springs to active/passive
    
    Args:
    system (System) : the system you want to generate a neighbor for
    ls (Tuple (double, double)) : passive and active unstretched length
    ks (Tuple (double, double)) : passive and active stiffness
    betas (Tuple (double, double)) : passive and active angular stiffness
    '''
    def buildRandomWithGivenSprings(self, system, ls, ks, betas):
        ''' system: System Object from bds.py
            ls: 2 tuple (passive l0, active l0)
            ks: 2 tuple (passive k, active k)
            betas: 2 tuple (passive beta, active beta)
        '''
        ret_system = copy.deepcopy(system)            
        spring_count = len(ret_system.springs)
        active = np.random.randint(2, size=spring_count)
        for i in range(0, spring_count):
            if active[i]:
                ret_system.springs[i].l0 = ls[1]
                ret_system.springs[i].k = ks[1]
                ret_system.springs[i].beta = betas[1]
        ret_system.buildFastStructure()
        return ret_system
        
        
    '''
    Returns a tuple of lists of node objects
    '''
    def buildNodes(self, nodes_free, nodes_fixed):
        ''' nodes: list of (1,4) tuple. (m, (x, y, z), (r, p, y), Fixed)
                    (x,y,z) and (r,p,y) are (1,3) np array
        '''
        free_nodes = []
        fixed_nodes = []
        for node in nodes_free:
            free_nodes.append(bds.Node(node[0], node[1], node[2], node[3]))
        for node in nodes_fixed:
            fixed_nodes.append(bds.Node(node[0], node[1], node[2], node[3]))
        return (free_nodes, fixed_nodes)




















