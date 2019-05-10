import numpy as np
import copy
from matplotlib import pyplot as plt

'''
File for optimization methods.

Currently only contains simulated annealing.

'''

class SimulatedAnnealing(object):
    '''
    Class used for simulated annealing
    '''
    def __init__(self, obj_func, neighbor_func, model):
        ''' 
            Args:
            model (Anything) - whatever you are trying to optimize on
            obj_func (Function) - function that takes model as the only parameter
                                    and returns a double
            neighbor_func (Function) - function that takes model as the only parameter
                                    anad returns a new model 
        '''
        self.orig_model = model
        self.model = copy.deepcopy(model)
        self.obj_func = obj_func # Cannot modify original object
        self.neighbor_func = neighbor_func # Cannot modify original object
        self.costs = []
        self.iters = []
        
    '''
    Returns whether or not a value is accepted in the process as a function
    of the new value, current value, and temperature
    '''  
    def accept(self, old_value, new_value, T):
        numer = old_value - new_value
        exponent = numer / T
        accept_prob = np.exp(exponent)
        return accept_prob > np.random.rand()     
        
    '''
    Returns the new temperature
    '''
    def cooldown(T, alpha, t):
        return T*alpha
    
    '''
    Runs the simulated annealing algorithm and plts the results
    
    max_iters (Int) : number of iterations of outer loop
    inner_iters (Int) : number of iterations of inner loop
    alpha (double) : cooling parameter
    T (double) : starting temperature
    cd (Function) : function of T, alpha, and max iters
    '''    
    def optimize(self, max_iters=10000, inner_iters=100, alpha=0.99, T=1.0, cd=cooldown):
        solution = self.model
        solution_value = self.obj_func(solution)
        best_solution = solution
        best_solution_value = solution_value
        # Outer loop changes temperature
        for i in range(0, max_iters):
            print(i)
            # inner loop has constant temperature
            for j in range(0, inner_iters):
                # Get Neighbor
                new_solution = self.neighbor_func(solution)
                new_value = self.obj_func(new_solution)
                # Try and accept neighbor
                if self.accept(solution_value, new_value, T):
                    self.costs.append(new_value)
                    self.iters.append((i*inner_iters)+j)
                    solution = new_solution
                    solution_value = new_value
                if new_value < best_solution_value:
                    best_solution = new_solution
                    best_solution_value = new_value
            T = cd(T, alpha, i)
            
        plt.plot(self.iters, self.costs)
        plt.xlabel('Iterations')
        plt.ylabel('Obj Function Value')
        plt.title('Objective Function: -|X_avg_old - X_avg_new|\nOuter: ' + str(max_iters) + ' Inner: ' + str(inner_iters) + ' Alpha: ' + str(alpha))
        #plt.show()
        return (best_solution, best_solution_value)
        
    
                
                
            
            
        
        
