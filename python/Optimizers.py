import numpy as np
import copy
from matplotlib import pyplot as plt

class SimulatedAnnealing(object):
    
    def __init__(self, obj_func, neighbor_func, model):
        ''' 
            Args:
            model-OptimizationModel-'''
        self.orig_model = model
        self.model = copy.deepcopy(model)
        self.obj_func = obj_func # Cannot modify original object
        self.neighbor_func = neighbor_func # Cannot modify original object
        self.costs = []
        self.iters = []
        
        
    def accept(self, old_value, new_value, T):
        numer = old_value - new_value
        exponent = numer / T
        accept_prob = np.exp(exponent)
        print 'T: %f' % T
        #print 'Numer: %f' % numer
        print 'Accept Prob: %f' % accept_prob
        #raw_input()
        return accept_prob > np.random.rand()     
        
    
    def cooldown(T, alpha, t):
        return T*alpha
        #return alpha/np.log(1+t)
        
    def optimize(self, max_iters=10000, inner_iters=100, alpha=0.99, T=1.0, cd=cooldown):
        solution = self.model
        solution_value = self.obj_func(solution)
        for i in range(0, max_iters):
            print i
            for j in range(0, inner_iters):
                new_solution = self.neighbor_func(solution)
                new_value = self.obj_func(new_solution)
                if self.accept(solution_value, new_value, T):
                    #print solution_value
                    #print '%d,%f,%f,%f' % (((i*inner_iters)+j), new_value, new_solution[0], new_solution[1])
                    self.costs.append(new_value)
                    self.iters.append((i*inner_iters)+j)
                    solution = new_solution
                    solution_value = new_value
            T = cd(T, alpha, i)
            #print T 
            
        plt.plot(self.iters, self.costs)
        plt.title('Outer: ' + str(max_iters) + ' Inner: ' + str(inner_iters) + ' Alpha: ' + str(alpha))
        #plt.show()
        return (solution, solution_value)
        
    
                
                
            
            
        
        
