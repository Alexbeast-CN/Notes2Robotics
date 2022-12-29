import math
import osqp
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg, sparse

class MinEffort:
    def __init__(self) -> None:
        '''
        args:

        '''
        
    def model_setup(self, way_points, n_order, n_obj, avg_v):
        '''
        args:
            way_points: a 2D np.array: [[x0,y0,(z0)],[x1,y1,(z1)],...]
            n_order: the order of polynomial
            n_obj: the order of derivative
            avg_v: average velocity
        '''
        n_seg  = way_points.shape[0] - 1
        n_dim  = way_points.shape[1]
        n_coef = n_order + 1
        self.time_set = self.arange_time(way_points, avg_v)
        

        for i in range(n_dim):
            p = self.single_axis_traj(n_seg, n_obj, n_coef)
        
    def get_traj(self):
        pass
    
    def arange_time(self, way_points, avg_v):
        pass
    
    def single_axis_traj(self, way_points, n_seg, n_obj, n_coef):
        # Compute QP cost function matrix
        q_i_cost = []
        for i in range(n_seg):
            q_i_cost.append(self.CompurteQ(n_coef, n_obj, self.time_set[i], self.time_set[i+1]))
        q_cost = linalg.block_diag(*q_i_cost)
        p_cost = np.zeros(q_cost.shape[0])
        
        # Setup equality constrains
        A_eq, b_eq = self.ComputeEquation(n_seg, n_obj, n_coef)
        
        # Solve qp(sqp) problem
        m = osqp.OSQP()
        m.setup(P=sparse.csc_matrix(q_cost), q=None, l=b_eq, A=sparse.csc_matrix(A_eq), u=b_eq, verbose=False)
        results = m.solve()
        x = results.x
        return x


    def factorial(self,n,r):
        '''
        return A_n^(n-r)
        '''
        if (n < r):
            return 0
        ans = 1
        for _ in range(r):
            ans = ans*n
            n -= 1
            
        return ans

    def ComputeQ(self,n,r,ts,te):
        '''
        args:
            n is the number of parameters
            r is the derivative order
            ts is the start time of this segment
            te is the end time of this segment
        return:
            one segment Q matrix of QP.
        '''
        Q = np.zeros((n,n))
        for i in range(n):
            for j in range(n):
                if i >= r and j >= r:
                    factor = i+j-2*r+1
                    Q[i][j] = self.factorial(i,r)*self.factorial(j,r)/factor*(te**factor-ts**factor)
        return Q   
    
    def ComputeEquation(self, n_seg, n_obj, n_coef):
        '''
        return the equality constrain matrix 'A' and vector 'b' in QP
        '''
        A = np.zeros((n_obj*(n_seg + 1) - 2, n_coef*n_seg))
        b = np.zeros(n_obj*(n_seg + 1) - 2)
        
        n_eq = 0
        # Start and End constrains: 2*(n_obj-1)
        for i in range(n_obj-1):
            A[i, 0:n_coef] = self.compute_t_vec(self.time_set[0], n_coef, i)
            b[i] = 0 
            A[i+n_obj-1, n_coef*(n_seg-1):n_coef*n_seg] = self.compute_t_vec(self.time_set[-1], n_coef, i)
            b[i+n_obj-1] = 0
            n_eq += 2
        
        # Points constrains and continous constraints: (n_seg - 1)*n_obj
        for i in range(1,n_seg):
            A[n_eq, n_coef*i:n_coef*(i+1)] = self.compute_t_vec(self.time_set[i], n_coef, 0)
            b[n_eq] = np.array([[self.way_points[i]]])
            n_eq += 1
            for j in range(n_obj-1):
                t_vec = self.compute_t_vec(self.time_set[i], n_coef, j)
                A[n_eq+n_seg, n_coef*(i-1):n_coef*i] = t_vec
                A[n_eq+n_seg, n_coef*i:n_coef*(i+1)] = -t_vec
                n_eq += 1
        
        return A, b
    
    def compute_t_vec(self, t, n_coef, k):
        t_vector = np.zeros(n_coef)
        for i in range(k, n_coef):
            t_vector[i] =  self.factorial(i,k) * pow(t, i-k)
        return t_vector