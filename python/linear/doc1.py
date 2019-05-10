import numpy as np
import copy
import sys
from scipy.optimize import minimize
from scipy.optimize import dual_annealing
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt

class LinearSystem(object):

    def __init__(self, m_arr, L, g):
        self.m_arr = m_arr
        self.L = L
        self.g = g
        pass
       
    
    
    def obj_func_L1(self, abc_arr, xy_target_arr,
                          alphas, betas, gammas, num_nodes):
        alpha_arr = abc_arr[0:n]*alphas[0] + (1-abc_arr[0:n])*alphas[1]
        beta_arr = abc_arr[n:2*n+1]*betas[0] + (1-abc_arr[n:2*n+1])*betas[1]
        gamma_arr = abc_arr[2*n+1:3*n+1]*gammas[0] + (1-abc_arr[2*n+1:3*n+1])*gammas[1]
        
        xys_arr = self.find_xys_given_abg(num_nodes, alpha_arr, beta_arr, gamma_arr)
        xys_arr = xys_arr.reshape(num_nodes, 3)
        
        xy_arr = xys_arr[:,0:2]
        #xy_arr = xy_arr.flatten()
        xy_target_arr = xy_target_arr.reshape((n,2))
        '''x_target = xy_target_arr[:,0]
        y_target = xy_target_arr[:,1]

        x = np.zeros((n,))
        y = np.zeros((n,))
        s = np.zeros((n,))
        x[0] = xys_arr[0]
        y[0] = xys_arr[1]
        s[0] = xys_arr[2]
        for i in range(1, n):
            x[i] = xys_arr[i*3] + x[i-1]
            y[i] = xys_arr[i*3 + 1] + y[i-1]
            s[i] = xys_arr[i*3 + 2] + s[i-1]
            x_target[i] = x_target[i-1]+x_target[i]
            y_target[i] = y_target[i-1]+y_target[i]'''
        
        #print xy_arr
        #print xy_target_arr
        #print np.absolute(xy_target_arr - xy_arr)
        
        error = np.sum(np.sum((xy_target_arr - xy_arr)**2, axis=1))
        #error = np.sum(np.sqrt(np.sum((xy_target_arr - xy_arr)**2, axis=1)))
        #error = 0
        #error = error + np.sum(np.absolute(x_target - x))
        #error = error + np.sum(np.absolute(y_target - y))
        
        return error
  
    def find_xys_given_abg(self, n, alpha_arr, beta_arr, gamma_arr):
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


        
n = 10
m_arr = 0.1*np.ones((n,))
L = 10
g = -1
alpha_arr = 5*np.random.random((n,))
beta_arr = 5*np.random.random((n+1,))
gamma_arr = 5*np.random.random((n,))


ls = LinearSystem(m_arr, L, g)


abc_arr = 0.5 + np.zeros((3*n+1,))
bounds = np.zeros((3*n+1,2))
bounds[:,1] = 1
alphas = [0.1, 5]
betas = [0.1, 5]
gammas = [0.1, 5]

xys = ls.find_xys_given_abg(n, alpha_arr, beta_arr, gamma_arr)
xys = xys.reshape(n,3)
xys = xys[:,0:2]
xy_target_arr = xys.flatten()
#xy_target_arr = np.random.random((n*2,))
#xy_target_arr = xy_target_arr.reshape((n,2))
#xy_target_arr[:,1] = -0.1 * (xy_target_arr[:,1] - 0.3)
#xy_target_arr = xy_target_arr.flatten()
res = minimize(ls.obj_func_L1, abc_arr, args=(xy_target_arr,
                                              alphas, betas, gammas, n),
               method='BFGS', options={'gtol': 1e-6, 'disp': True})
print(res.x)
#res = dual_annealing(ls.obj_func_L1, bounds, args=(xy_target_arr, alphas, betas, gammas, n),
#                     x0 = res.x)
#res = differential_evolution(ls.obj_func_L1, bounds, args=(xy_target_arr, alphas, betas, gammas, n))
print(res.x)
print(ls.obj_func_L1(res.x, xy_target_arr, alphas, betas, gammas, n))

alpha_arr = res.x[0:n]*alphas[0] + (1-res.x[0:n])*alphas[1]
beta_arr = res.x[n:2*n+1]*betas[0] + (1-res.x[n:2*n+1])*betas[1]
gamma_arr = res.x[2*n+1:3*n+1]*gammas[0] + (1-res.x[2*n+1:3*n+1])*gammas[1]

xys = ls.find_xys_given_abg(n, alpha_arr, beta_arr, gamma_arr)

xy_target_arr = xy_target_arr.reshape((n,2))
x_target = xy_target_arr[:,0]
y_target = xy_target_arr[:,1]

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
