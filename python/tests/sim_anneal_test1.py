import numpy as np

import Optimizers

step_size = 0.01

def obj_func_1(value):
    return value**2
    
def obj_func_2(value):
    term1 = (value[0]**2 + value[1] - 11)**2
    term2 = (value[0] + value[1]**2 - 7)**2
    return term1 + term2
    
def obj_func_3(value):
    term1 = np.sin(value[0])*np.cos(value[1])
    term2 = np.absolute(1 - (np.sqrt(value[0]**2 + value[1]**2)/np.pi))
    term3 = np.exp(term2)
    return -1 * np.absolute(term1 * term3)
    

def neighbor_func_1(value):
    which = round(np.random.rand())
    if which == 0:
        return value + step_size
    else:
        return value - step_size

def neighbor_func_2(value):
    which = round(np.random.randint(6, size=1))
    if which == 0:
        return (value[0] + step_size, value[1])
    elif which == 1:
        return (value[0], value[1] + step_size)
    elif which == 2:
        return (value[0] + step_size, value[1] + step_size)
    elif which == 3:
        return (value[0] - step_size, value[1])
    elif which == 4:
        return (value[0], value[1] - step_size)
    elif which == 5:
        return (value[0] - step_size, value[1] - step_size)
        
        
model_1 = 10.0
model_2 = (5.0, 5.0)
model_3 = np.random.randint(-10, 10, size=2)

annealer = Optimizers.SimulatedAnnealing(obj_func_3, neighbor_func_2, model_3)
best = annealer.optimize(max_iters=1000)
print model_3
print best
