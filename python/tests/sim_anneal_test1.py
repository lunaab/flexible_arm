import numpy as np

import Optimizers

def obj_func(value):
    term1 = (value[0]**2 + value[1] - 11)**2
    term2 = (value[0] + value[1]**2 - 7)**2
    return term1 + term2
    
def neighbor_func(value):
    which = round(np.random.randint(6, size=1))
    if which == 0:
        return (value[0] + 0.0001, value[1])
    elif which == 1:
        return (value[0], value[1] + 0.0001)
    elif which == 2:
        return (value[0] + 0.0001, value[1] + 0.0001)
    elif which == 3:
        return (value[0] - 0.0001, value[1])
    elif which == 4:
        return (value[0], value[1] - 0.0001)
    elif which == 5:
        return (value[0] - 0.0001, value[1] - 0.0001)
model = (5, 5)

annealer = Optimizers.SimulatedAnnealing(obj_func, neighbor_func, model)
best = annealer.optimize()
#print best
