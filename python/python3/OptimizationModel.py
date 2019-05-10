import numpy as np
from scipy.optimize import minimize

import copy

import BodiesFast as bds
import BodyBuilderFast as bld

'''
Currently useless
'''

class OptimizationModel(object):

    def __init__(self, system, obj_func, neighbor_func):
        '''
            system: System from Bodies.py
        '''
        self.system_save = copy.deepcopy(system)
        self.system = copy.deepcopy(system)
        
        self.obj_func = obj_func
        self.neighbor_func = neighbor_func
        
    def objectiveFunction(self):
        return self.obj_func()
        
    def genNeighborFunction(self):
        return self.neighbor_func()
