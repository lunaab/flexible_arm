import numpy as np
import copy

class SimulatedAnnealing(object):
    
    def __init__(self, obj_func, neighbor_func, model):
        ''' 
            Args:
            model-OptimizationModel-'''
        self.orig_model = model
        self.model = copy.deepcopy(model)
        self.obj_func = obj_func # Cannot modify original object
        self.neighbor_func = neighbor_func # Cannot modify original object
        
        
    def accept(self, old_value, new_value, T):
        exponent = (old_value - new_value) / T
        accept_prob = np.exp(exponent)
        return accept_prob >= np.random.rand()     
        
        
    def optimize(self, max_iters=10000, inner_iters=100, alpha=0.99, T=1.0):
        solution = self.model
        solution_value = self.obj_func(solution)
        for i in range(0, max_iters):
            print i
            for j in range(0, inner_iters):
                new_solution = self.neighbor_func(solution)
                new_value = self.obj_func(new_solution)
                if self.accept(solution_value, new_value, T):
                    #print '%d,%f,%f,%f' % (((i*inner_iters)+j), new_value, new_solution[0], new_solution[1])
                    solution = new_solution
                    solution_value = new_value
            T = T*alpha
            
        return (solution, solution_value)
        
    
                
                
            
            
        
        
