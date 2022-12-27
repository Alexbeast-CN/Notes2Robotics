import numpy as np
import osqp

class single_axis_minisnap:
    def __init__(self, n_order, derivertive, waypts, times) -> None:
        '''
        args:
            n_order: polynormial order, ie. 5
            derivertive: "acc", "jerk", "snap"
            waypts: an array of way points
            ts: an array of time interval
        '''
        self.n_order = n_order
        self.waypts = waypts
        self.ts = times
        
        self.v0 = 0
        self.a0 = 0
        self.ve = 0
        self.ae = 0
        
        if derivertive == "acc":
            if (n_order < 3):
                print("n_order should be larger than 3 for acc")
            self.r = 2
        elif derivertive == "jerk":
            if (n_order < 4):
                print("n_order should be larger than 4 for jerk")
            self.r = 3
        elif derivertive == "snap":
            if n_order < 5:
                print("n_order should be larger than 5 for snap")
            self.r = 4
            
    def minisnap(self):
        pass
    
    def timeArrangement(self):
        pass
    
    def sumQ(self):
        n = self.n_order + 1
        n_seg = self.waypts.size - 1
        r = self.r
        Q = np.zeros((n_seg*n, n_seg*n))
        for i in range(n_seg):
            ts = self.ts[i]
            te = self.ts[i+1]
            Q[i][i] = self.computeQ(n, r, ts, te)
    
    def computeQ(self, n, r, ts, te):
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
                    Q[n-1-i][n-1-j] = self.factorial(i,r)*self.factorial(j,r)/factor*(te**factor-ts**factor)
        return Q
    
    def factorial(self, n, r):
        '''
        n is the number of parameters
        r is the derivative order
        '''
        if (n < r):
            return 0
        ans = 1
        for _ in range(r):
            ans = ans*n
            n -= 1
            
        return ans

def main():
    ms = single_axis_minisnap(5, "jerk", np.array([0,1,2,3,4]),np.array([0,1,2,3,4]))    
    print(ms.sumQ())
    
if __name__ == "__main__":
    main()