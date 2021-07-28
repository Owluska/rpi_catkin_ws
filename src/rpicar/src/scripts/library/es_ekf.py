#! /usr/bin/env python3

import numpy as np
#from rotations import skew_symmetric, Quaternion
from ahrs import Quaternion
def skew_symmetric(v):
    """Skew symmetric form of a 3x1 vector."""
    return np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]], dtype=np.float64)

class ekf:
    def __init__(self):
        
        self.ROT = np.eye(3)
        self.acc_tol = 1.0
        
        self.var_f = np.array([0.01,0.01,0.01])
        self.var_w = np.array([0.01,0.01,0.01])
        self.var_m =  np.array([0.01,0.01,0.01])

       

        self.g = np.array([0, 0, 9.81])       # gravity
        #self.g = np.array([0.,0., 6.500022])
        self.l_jac = np.zeros([9, 9])
        self.l_jac[3:, 3:] = np.eye(6)          # motion model noise jacobian

        self.h_jac = np.zeros([9, 9])
        self.h_jac[6:, 6:] = np.eye(3)          # measurement model jacobian

        self.N = self.define_N()
        self.R = self.define_R()

        self.p_est = np.zeros([1, 3])          # position estimates
        self.v_est = np.zeros_like(self.p_est)  # velocity estimates
        self.q_est = np.zeros([1, 4])           # orientation estimates as quaternions
        self.p_cov = np.zeros([1, 9, 9])        # covariance matrices at each timestep
        #self.dt = np.zeros([1,1])
        self.a = np.zeros_like(self.p_est)
        self.norm = False
        
    def init_data(self, p, v, q, p_cov):
    # Set initial values.
        self.p_est[0] = p 
        self.v_est[0] = v
        self.q_est[0] = q
        self.p_cov[0] = p_cov  # covariance of estimate


    def measurement_update(self, y, p, v, q, p_cov):
        #Compute Kalman Gain
        S = self.h_jac @ p_cov @ self.h_jac.T + self.R
        #print(S)
        S_inv = np.linalg.pinv(S)
        K = p_cov @ self.h_jac.T @ S_inv
        # Compute error state
        phi = Quaternion(*q).to_euler()
        x = np.append(p, (v, phi)).reshape(1,9)
        #print("p: {}\tv: {}\tphi: {}\t\n\n".format(p,v,phi))
        
        #print("K: {}\nR: {}\n\n".format(K,R))
        err = K.T @ (y - x).T
        
        #print("x:    {}\ny:    {}\nerr: {}\n\n".format(x,y,err.T))


        dp = err[0:3].T[0]
        dv = err[3:6].T[0]
        d_phi = err[6:9].T[0]
        # 3.3 Correct predicted state
        p += dp
        v += dv      

        dq = Quaternion(axis_angle=d_phi) * q
        if self.norm:
            q =  np.linalg.norm(dq)
   
        #Compute corrected covariance
        p_cov = (np.eye(9) - K @ self.h_jac) @ p_cov
        return p, v, q, p_cov


    def get_F(self, dt, f):
        #(dt, C, f):
        ID = np.eye(3)
        
        F = np.eye((9), dtype = 'double')
        F[:3, 3:6] = ID
        #print(f)
        #F[3:6, 6:9] = -self.ROT @ skew_symmetric(f) * dt
        F[3:6, 6:9] = -self.skew_matrix(self.ROT @ f)
        F *= dt
        return F
    
    def skew_matrix(self, matrix_R3):
        m = np.zeros([3,3])
        
        m[0, 1] = -matrix_R3[2]
        m[0, 2] = matrix_R3[1]
        m[1, 0] = matrix_R3[2]
        m[1, 2] = -matrix_R3[0]
        m[2, 0] = -matrix_R3[1]
        m[2, 1] = matrix_R3[0]
        return m

    def define_N(self):
        z = np.array([0.,0.,0.])
        sigmas = np.append(z, (self.var_f, self.var_w))
        N = np.diag(sigmas)
        return N
    
    def define_R(self):
        z = np.array([0.,0.,0.])
        sigmas = np.append(z, (z,self.var_w))
        R = np.diag(sigmas)
        return R

    def propagate(self, f, w, dt):    
        #Linearize the motion model and compute Jacobians
        # * - gives argument as tuple, in this particular case: (q_est[0], q_est[1], and so on...)
        k = self.q_est.shape[0] - 1
        if self.norm:
            q_prev = np.linalg.norm(Quaternion(self.q_est[k]))
        else:
            #print(self.q_est, k)
            q_prev = Quaternion(self.q_est[k])
        
        #print()
        self.ROT = q_prev.to_DCM()
        #print(self.ROT)
        a = self.ROT @ f + self.g
        

        self.a = np.append(self.a, a.reshape(1,3), axis = 0)
        
#        self.v_est[k] = np.where(abs(self.a[k]) >= self.acc_tol, self.v_est[k], 0.00)
#        self.p_est[k] = np.where(abs(self.a[k]) >= self.acc_tol, self.p_est[k], 0.00)
        
        p = self.p_est[k] + dt * self.v_est[k] + 0.5 * dt ** 2 * self.a[k]      
        v =  dt * self.a[k] + self.v_est[k]
        #print("k {} dt:{:.2f} v:{:.2f} {:.2f} {:.2f} a:{:.2f} {:.2f} {:.2f} p:{:.2f} {:.2f} {:.2f}".format(k, dt, *a, *p, *v))
#        v_loc = np.array([float(od), 0, 0])
#        v = self.ROT @ v_loc


        qw = Quaternion(w * dt)
        q = Quaternion(q_prev).product(qw)
        #qw = Quaternion(axis_angle=angle).quat_mult_right(q_prev)
        #qw = q_prev.quat_mult_left(Quaternion(axis_angle=angle))
 
        if self.norm:
            q = np.linalg.norm(qw)
       

        F = self.get_F(dt, f)
        N = self.N * dt ** 2

        #Propagate uncertainty 
        #print(N.shape, self.l_jac.shape)
        p_cov = F @ self.p_cov[k] @ F.T + self.l_jac @ N @ self.l_jac.T
        
        return p, v, q, p_cov
    
    def update(self, f, w, dt, useFilter = False, sensors_data = 0.0):
        #print(k)
        p, v, q, p_cov = self.propagate(f, w, dt)
        if useFilter:
            p, v, q, p_cov = self.measurement_update(sensors_data, p, v, q, p_cov)
            
        #print(p, end = '\n\n')
        
        self.p_est = np.append(self.p_est, p.reshape(1,3), axis = 0)
        self.v_est = np.append(self.v_est, v.reshape(1,3), axis = 0)
        self.q_est = np.append(self.q_est, q.reshape(1,4), axis = 0)
        self.p_cov = np.append(self.p_cov, p_cov.reshape(1, 9, 9), axis = 0)