import numpy as np
import copy
import sys
from scipy.optimize import minimize
from scipy.optimize import LinearConstraint
import matplotlib.pyplot as plt

'''
Solves Malik's 1D problem with 0 unstretched length
'''

class LinearSystem(object):

    def __init__(self, m_arr, L, g):
        self.m_arr = m_arr
        self.L = L
        self.g = g
        pass
       
    
    '''
    Args:
    abc_arr (1D numpy array) : a1, a2, ..., b1, b2,..., c1, c2...
    xy_target_arr (1D numpy array): target node positions (x1, y1, x2, y2,...)
    alphas (1D numpy array) : (passive, active)
    betas (1D numpy array) : (passive, active)
    gammas (1D numpy array) : (passive, active)
    num_nodes (Int) : number of nodes
    '''
    def obj_func_L2(self, abc_arr, xy_target_arr,
                          alphas, betas, gammas, num_nodes):
                          
        # Determine stiffness from a, b, c
        alpha_arr = abc_arr[0:num_nodes]*alphas[0] + (1-abc_arr[0:num_nodes])*alphas[1]
        beta_arr = abc_arr[num_nodes:2*num_nodes+1]*betas[0] + (1-abc_arr[num_nodes:2*num_nodes+1])*betas[1]
        gamma_arr = abc_arr[2*num_nodes+1:3*num_nodes+1]*gammas[0] + (1-abc_arr[2*num_nodes+1:3*num_nodes+1])*gammas[1]
        
        # Determine x, y, s from stiffnesses
        xys_arr = self.find_xys_given_abg(num_nodes, alpha_arr, beta_arr, gamma_arr)
        xys_arr = xys_arr.reshape(num_nodes, 3)
        
        xy_arr = xys_arr[:,0:2]
        xy_target_arr = xy_target_arr.reshape((num_nodes,2))
        
        # Squared error term
        error = np.sum(np.sum((xy_target_arr - xy_arr)**2, axis=1))
        
        return error
        
    def obj_func_L2_limited(self, abc_arr, xy_target_arr,
                          alphas, betas, gammas, num_nodes, which, is_print=True):
                          
        # Determine stiffness from a, b, c
        alpha_arr = abc_arr[0:num_nodes]*alphas[0] + (1-abc_arr[0:num_nodes])*alphas[1]
        beta_arr = abc_arr[num_nodes:2*num_nodes+1]*betas[0] + (1-abc_arr[num_nodes:2*num_nodes+1])*betas[1]
        gamma_arr = abc_arr[2*num_nodes+1:3*num_nodes+1]*gammas[0] + (1-abc_arr[2*num_nodes+1:3*num_nodes+1])*gammas[1]
        
        # Determine x, y, s from stiffnesses
        xys_arr = self.find_xys_given_abg(num_nodes, alpha_arr, beta_arr, gamma_arr)
        xys_arr = xys_arr.reshape(num_nodes, 3)
        
        xy_arr = xys_arr[:,0:2]
        
        for i in range(1, num_nodes):
            xy_arr[i, 0] = xy_arr[i, 0] + xy_arr[i-1, 0]
            xy_arr[i, 1] = xy_arr[i, 1] + xy_arr[i-1, 1]
        
        xy_arr = xys_arr[which,0:2]
        
        xy_targ = np.copy(xy_target_arr)
        xy_targ = xy_targ.reshape((int(xy_targ.shape[0]/2),2))
        for i in range(1, int(xy_targ.shape[0]/2)):
            xy_targ[i, 0] = xy_targ[i, 0] + xy_targ[i-1, 0]
            xy_targ[i, 1] = xy_targ[i, 1] + xy_targ[i-1, 1]
        
        
        
        if is_print:
            print(xy_arr)
            print(xy_targ)
        
        # Squared error term
        error = np.sum(np.sum((xy_targ - xy_arr)**2, axis=1))
        
        return error
  
    '''
    Args:
    n (Int) : number of free nodes
    alpha_arr (1D numpy array) : alpha1, alpha2, ...
    beta_arr (1D numpy array) : beta1, beta2, ...
    gamma_arr (1D numpy array) : gamma1, gamma2, ...
    '''
    def find_xys_given_abg(self, n, alpha_arr, beta_arr, gamma_arr):
        '''print(n)
        print(alpha_arr.shape)
        print(beta_arr.shape)
        print(gamma_arr.shape)'''
    
        A = np.zeros((3*n, 3*n))
        b = np.zeros((3*n, 1))
        
        # Build A and b
        for k in range(0, n):
            x_k_idx = (k*3)
            y_k_idx = (k*3) + 1
            s_k_idx = (k*3) + 2
            b[(k*3)] = 0
            b[(k*3) + 1] = self.m_arr[k]*self.g
            if (k != n-1):
                b[(k*3) + 2] = 0
            else:
                b[(k*3) + 2] = -1* beta_arr[k+1] * self.L
            
            # M - X Forces
            A[(k*3), x_k_idx] = -1*(alpha_arr[k] + gamma_arr[k])
            if (k != n-1):
                A[(k*3), (k+1)*3] = alpha_arr[k+1]
            for i in range(0, k):
                A[(k*3), i*3] = -1*gamma_arr[k]
            for i in range(0, k+1):
                A[(k*3), (i*3) + 2] = gamma_arr[k]
                
            # M - Y Forces
            A[(k*3) + 1, y_k_idx] = (alpha_arr[k] + gamma_arr[k])
            if (k != n-1):
                A[(k*3) + 1, ((k+1)*3) + 1] = -1*alpha_arr[k+1]
            for i in range(0, k):
                A[(k*3) + 1, (i*3) + 1] = gamma_arr[k]  
                
            # L -X Forces
            A[(k*3) + 2, s_k_idx] = -1*(beta_arr[k] + gamma_arr[k])
            if (k != n-1):
                A[(k*3) + 2, ((k+1)*3) + 2] = beta_arr[k+1]
            else:
                for i in range(0, k+1):
                    A[(k*3) + 2, (i*3) + 2] += -1*beta_arr[k+1]
            for i in range(0, k):
                A[(k*3) + 2, (i*3) + 2] += -1*gamma_arr[k] 
            for i in range(0, k+1):
                A[(k*3) + 2, (i*3)] = gamma_arr[k]
                
        # Solve system
        #print A
        return np.linalg.solve(A, b)



# Constants      
n = 10 # number of nodes
m_arr = 0.1*np.ones((n,)) # mass
L = 10 
g = -1 # gravity

ls = LinearSystem(m_arr, L, g)

# Random Target generations
stiff_max = 4
alpha_arr = stiff_max*np.random.random((n,))
beta_arr = stiff_max*np.random.random((n+1,))
gamma_arr = stiff_max*np.random.random((n,))

xys = ls.find_xys_given_abg(n, alpha_arr, beta_arr, gamma_arr)
xys = xys.reshape(n,3)
xys = xys[:,0:2]
xy_target_arr = xys.flatten()




# Initial guesses and bounds for stiffness
abc_arr = 0.5 + np.zeros((3*n+1,))
bounds = np.zeros((3*n+1,2))
bounds[:,1] = 1
alphas = [0, stiff_max] #[0, stiff_max]
betas = [0, stiff_max]
gammas = [0, stiff_max]

# Constraints on stiffness
cnts = LinearConstraint(np.identity(3*n+1), 0, 1)

# Determining alpha, beta, gamma without contraints
'''res = minimize(ls.obj_func_L2, abc_arr, args=(xy_target_arr,
                                              alphas, betas, gammas, n),
               method='BFGS', options={'gtol': 1e-6, 'disp': True})'''

# Determining alpha, beta, gamma with contraints               
res = minimize(ls.obj_func_L2, abc_arr, args=(xy_target_arr,
                                              alphas, betas, gammas, n),
               method='SLSQP', constraints=cnts, options={'gtol': 1e-6, 'disp': True})
               
# Determining alpha, beta, gamma with extra nodes
'''N = 37
abc_arr = 0.5 + np.zeros((3*N+1,))
cnts = LinearConstraint(np.identity(3*N+1), 0, 1)
ls.m_arr = 0.1*np.ones((N,))
which = [0, 4, 8, 12, 16, 20, 24, 28, 32, 36] 
res = minimize(ls.obj_func_L2_limited, abc_arr, args=(xy_target_arr,
                                              alphas, betas, gammas, N, which),
               method='SLSQP', constraints=cnts, options={'gtol': 1e-6, 'disp': True})

# Print a,b,c array and objective function value
#print(res.x)
#print(ls.obj_func_L2(res.x, xy_target_arr, alphas, betas, gammas, n))
print(ls.obj_func_L2_limited(res.x, xy_target_arr, alphas, betas, gammas, N, which, is_print=True))

alpha_arr = res.x[0:N]*alphas[0] + (1-res.x[0:N])*alphas[1]
beta_arr = res.x[N:2*N+1]*betas[0] + (1-res.x[N:2*N+1])*betas[1]
gamma_arr = res.x[2*N+1:3*N+1]*gammas[0] + (1-res.x[2*N+1:3*N+1])*gammas[1]

# get final x, y, s from given alpha, beta, gamma
xys = ls.find_xys_given_abg(N, alpha_arr, beta_arr, gamma_arr)

# split x and y
xy_target_arr = xy_target_arr.reshape((n,2))
x_target = xy_target_arr[:,0]
y_target = xy_target_arr[:,1]

# Plot the result
x = np.zeros((N,))
y = np.zeros((N,))
s = np.zeros((N,))
x[0] = xys[0]
y[0] = xys[1]
s[0] = xys[2]
for i in range(1, N):
    x[i] = xys[i*3] + x[i-1]
    y[i] = xys[i*3 + 1] + y[i-1]
    s[i] = xys[i*3 + 2] + s[i-1]
   
for i in range(1, n):
    x_target[i] = x_target[i-1]+x_target[i]
    y_target[i] = y_target[i-1]+y_target[i]
    
x_care = []
y_care = []
for i in range(0, len(which)):
    x_care.append(x[which[i]])
    y_care.append(y[which[i]])   

plt.figure()
plt.plot(x, y, color='b', marker='o', zorder=0)
x = [0, 10]
y = [0, 0]
plt.scatter(x, y, color='r')
plt.scatter(x_care, y_care, color='y', zorder=100)
plt.scatter(x_target, y_target, color='r', marker='^', zorder=1000)

y = np.zeros((N,))
plt.plot(s, y, color='g', marker='o')
#plt.xlim(-0.1, 20)
#plt.ylim(-1, 0)
plt.show()'''

# get alpha, beta, gamma, from a,b,c
alpha_arr = res.x[0:n]*alphas[0] + (1-res.x[0:n])*alphas[1]
beta_arr = res.x[n:2*n+1]*betas[0] + (1-res.x[n:2*n+1])*betas[1]
gamma_arr = res.x[2*n+1:3*n+1]*gammas[0] + (1-res.x[2*n+1:3*n+1])*gammas[1]

# get final x, y, s from given alpha, beta, gamma
xys = ls.find_xys_given_abg(n, alpha_arr, beta_arr, gamma_arr)

# split x and y
xy_target_arr = xy_target_arr.reshape((n,2))
x_target = xy_target_arr[:,0]
y_target = xy_target_arr[:,1]

# Plot the result
x = np.zeros((n,))
y = np.zeros((n,))
s = np.zeros((n,))
x[0] = xys[0]
y[0] = xys[1]
s[0] = xys[2]
for i in range(1, n):
    x[i] = xys[i*3] + x[i-1]
    y[i] = xys[i*3 + 1] + y[i-1]
    s[i] = xys[i*3 + 2] + s[i-1]
   
for i in range(1, n):
    x_target[i] = x_target[i-1]+x_target[i]
    y_target[i] = y_target[i-1]+y_target[i]
    

plt.plot(x, y, color='b', marker='o')
x = [0, 10]
y = [0, 0]
plt.scatter(x, y, color='r')
plt.scatter(x_target, y_target, color='r', marker='^')

y = np.zeros((n,))
plt.plot(s, y, color='g', marker='o')
#plt.xlim(-0.1, 20)
#plt.ylim(-20, 20)
plt.show()
