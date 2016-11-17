#!/usr/bin/env python
"""
Created on Wed Oct 26 09:01:16 2016

@author: dennis
"""

import numpy as np
import rospy 
import common_functions as cf
_FCUT = 5.0
_ACC_MAX = 2.0
_THR_MAX = 0.9
_THR_MIN = 0.12
_VEL_MAX_XY = 8.0
_VEL_MAX_Z_DOWN = -2.0
_VEL_MAX_Z_UP = 4.0
_ACC_MAX_XY = 1.0

class controller():
    def __init__(self, pid_coeff, offset):
       global FCUT, _ACC_MAX, _THR_MAX, _THR_MIN, _VEL_MAX_XY, _VEL_MAX_Z_UP, _VEL_MAX_Z_DOWN, _ACC_MAX_XY
        
       self._pid_coeff = pid_coeff# = {"Pxy_p": 0.0, "Pz_p": 0.0, "Pxy_v":0.0 , "Pz_v":0.0, "Dxy_v":0.0 , "Dz_v":0.0 ,  "Ixy_v":0.0 , "Iz_v":0.0, "M":0.0 }
       self._p_c = np.zeros(3)
       self._v_c = np.zeros(3)
       self._a_c = np.zeros(3)
       self._p_star = np.zeros(3)
       self._v_star = np.zeros(3)
       self._a_star = np.zeros(3)
       self._v_error = np.zeros((2,3)) # previous two errors
       self._a_error = np.zeros((2,3))
       
       self._acc_z_offset = offset

       self._acc_error_prev_f = np.zeros(3)
       
       self._dt = 0.0
       self._t_prev =  0.0
       
       
       self._fcut = _FCUT
       self._acc_max = _ACC_MAX
       self._thr_max = _THR_MAX
       self._thr_min = _THR_MIN
       self._vel_max_xy = _VEL_MAX_XY
       self._vel_max_z_up = _VEL_MAX_Z_UP
       self._vel_max_z_down = _VEL_MAX_Z_DOWN
       self._acc_max_xy = _ACC_MAX_XY
       
       self._vel_error_d_prev = np.zeros(3)
       self._acc_error_d_prev = np.zeros(3)
       
       self._vel_sp_prev = np.zeros(3)
       self._integral_v = np.zeros(3)
       self._integral_a = np.zeros(3)
       self._a_prev = np.zeros(3)
       self._thr_prev = np.zeros(3)
       self._thr_prev[2] = 0.5
       self._vel_error_d_prev = np.zeros(3)
       self._v_o = np.zeros(3)
       self._a_o = np.zeros(3)
       self._thrust_des = np.zeros(3)
       self._thrust_des[2] = 0.5
       

           
           
       
          
          
    def set_states(self, p_c, v_c, a_c,  p_star, v_star, a_star, pid_coeff):
       self._p_c = np.copy(p_c)
       self._v_c = np.copy(v_c)
       self._a_c = np.copy(a_c)
       self._a_c[2] -= self._acc_z_offset # subtract offset
      
       self._p_star = np.copy(p_star)
       self._v_star = np.copy(v_star)
       self._a_star = np.copy(a_star)
       self._pid_coeff = pid_coeff
       
       
       
    def _lowpass(self, inp, prev):
        
        a = self._fcut /8.0  # 7 is magic number
        out = (1-a) * prev + a * inp
        return out
        
        
        
    
        
    def update_thrust_old(self, time):
        
         # compute dt
        dt = 0.0
        if self._t_prev == 0.0:
            now = rospy.Time.now()
            self._t_prev = now.secs + now.nsecs * 10e-9
            dt = 1.0/500.0
        else:
            now = rospy.Time.now()
            dt = (now.secs + now.nsecs * 10e-9) - self._t_prev  
            self._t_prev = (now.secs + now.nsecs * 10e-9)
            
        #now = rospy.Time.now()
       
        # TODO: not hard coded
        dt = 0.04
        
        ### velocity along and across path
        if np.any(self._v_star):

            vel_along_path = np.dot(self._v_c, self._v_star)/np.linalg.norm(self._v_star) * self._v_star/np.linalg.norm(self._v_star)
            vel_across_path = self._v_c - vel_along_path
            
            ### cross controller
            pose_error = self._p_star - self._p_c
            vel_sp_across = self._pid_coeff.Pv * pose_error
            vel_sp_across[:2] = cf.saturate_vector(vel_sp_across[:2], self._vel_max_xy, 0.0)
            vel_sp_across[2] = min(self._vel_max_z_up, max(self._vel_max_z_down, vel_sp_across[2]))
            vel_error_across =  vel_sp_across -  vel_across_path
            thrust_across = self._pid_coeff.Dv * vel_error_across + self._integral_v
        
            ### along controller
            vel_sp_along = np.zeros(3)
            vel_sp_along[:2] = cf.saturate_vector(self._v_star[:2], self._vel_max_xy, 0.0)
            vel_sp_along[2] = min(self._vel_max_z_up, max(self._vel_max_z_down, self._v_star[2]))
            vel_error_along = vel_sp_along - vel_along_path
            thrust_along = self._pid_coeff.Dv * vel_error_along + self._integral_a
            
            self._integral_a += self._pid_coeff.Iv * vel_error_along * dt
            
            
        else:
            vel_across_path = np.copy(self._v_c)
            ### cross controller
            pose_error = self._p_star - self._p_c
            vel_sp_across = self._pid_coeff.Pv * pose_error
            vel_sp_across[:2] = cf.saturate_vector(vel_sp_across[:2], self._vel_max_xy, 0.0)
            vel_sp_across[2] = min(self._vel_max_z_up, max(self._vel_max_z_down, vel_sp_across[2]))
            vel_error_across =  vel_sp_across -  vel_across_path
            thrust_across = self._pid_coeff.Dv * vel_error_across + self._integral_v
            thrust_along = np.zeros(3)

        self._integral_v += self._pid_coeff.Iv * vel_error_across * dt
        
        
        
        # PID acceleration
        thrust_sp = self._pid_coeff.Pp * ( self._a_star ) + thrust_across + thrust_along
        
        # saturare
        mag = np.linalg.norm(thrust_sp)
        if mag > 0.9 :
            thrust_sp = thrust_sp *0.9/mag
        elif mag < 0.1 and not 0.0:
            thrust_sp = thrust_sp * 0.1/mag

        
        # print
        cf.print_arrays( [pose_error, self._v_c, thrust_sp]) 
        


        # return 
        return thrust_sp, thrust_across, thrust_along
            

            
     
        
        
           
    
    
    
     
#  node enter point
def main():
    

    c = controller()
    
    
if __name__ == '__main__':
    main()

           
           
           
       
      


