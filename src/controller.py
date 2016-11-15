#!/usr/bin/env python
"""
Created on Wed Oct 26 09:01:16 2016

@author: dennis
"""

import numpy as np
import rospy 
import common_functions as cf
_FCUT = 5.0
_ACC_MAX = 5.0
_THR_MAX = 0.9
_THR_MIN = 0.12
_VEL_MAX_XY = 8.0
_VEL_MAX_Z_DOWN = -3.0
_VEL_MAX_Z_UP = 5.0
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

       
        ### position controller
        pose_error = self._p_star - self._p_c
        vel_sp = self._pid_coeff.Pp * pose_error 
        
        # add feedforward
        #vel_sp += self._v_star
        
        # limit velocity sp
        if np.linalg.norm(vel_sp[:2]) > self._vel_max_xy:
            
            vel_sp[:2] = vel_sp[:2]/np.linalg.norm(vel_sp[:2]) * self._vel_max_xy
            
        if vel_sp[2] > self._vel_max_z_up:
            vel_sp[2] = self._vel_max_z_up
        elif vel_sp[2] < self._vel_max_z_down:
            vel_sp[2] = self._vel_max_z_down
        

    

        ### limit horizontal acceleration ToDo: these values make no sense
        '''acc_xy = ( vel_sp[:2]   - self._vel_sp_prev[:2] )/dt
        if np.linalg.norm(acc_xy) > self._acc_max_xy:
            print "acc xy sat"
            
            acc_xy = acc_xy/np.linalg.norm(acc_xy)
            acc_xy *= self._acc_max_xy
            vel_sp[:2] = acc_xy * dt + self._vel_sp_prev[:2]
            
        acc_z = ( vel_sp[2]- self._vel_sp_prev[2])/dt
        if np.abs(acc_z) > 2.0 * self._acc_max_xy:
            acc_z /= np.abs(acc_z)
            vel_sp[2] = acc_z * 2.0 * self._acc_max_xy * dt + self._vel_sp_prev[2]
            print "acc z sat"
            
            
        self._vel_sp_prev = vel_sp'''
        
        ## get vvelocity along path

        if self._v_star.any():
            vel_along_path_c = np.dot(self._v_c, self._v_star)/np.linalg.norm(self._v_star) * self._v_star/np.linalg.norm(self._v_star)
            vel_cross_path_c = self._v_c - vel_along_path_c #vel_np.dot(self._v_c, pose_error)/np.linalg.norm(pose_error) * pose_error/np.linalg.norm(pose_error)
        else:
            vel_along_path_c = np.zeros(3)
            vel_cross_path_c = self._v_c
        
        
        if self._a_star.any():
            acc_along_path_c = np.dot(self._a_c, self._a_star)/np.linalg.norm(self._a_star) * self._v_star/np.linalg.norm(self._a_star)
            acc_cross_path_c = self._a_c - acc_along_path_c #np.dot(self._v_c, pose_error)/np.linalg.norm(pose_error) * pose_error/np.linalg.norm(pose_error)
        else:
            acc_along_path_c = np.zeros(3)
            acc_cross_path_c = self._a_c
        
        
        #print vel_along_path_c, acc_along_path_c, vel_cross_path_c, acc_cross_path_c
  
            
        ### velocity controller
            
        # velocity error
        vel_error = vel_sp - vel_cross_path_c 
            
        # vel error derivative
        vel_error_d = self._lowpass((vel_cross_path_c - self._v_o)/dt, self._vel_error_d_prev)
         
        # PID
        acc_sp = self._pid_coeff.Pv * ( vel_error )# + self._pid_coeff.Dv * vel_error_d + self._integral_v
        if np.linalg.norm(acc_sp > 5.0):
            acc_sp = acc_sp/np.linalg.norm(acc_sp) * 5.0
        #a_long = np.copy(acc_sp)

        # add feedforward
        #acc_sp += self._a_star 
        

 
        
        #print "acc_sp", acc_sp
        #print "acc_c", self._a_c


        # Print the amount of thrust resulting from the feed-forward and the errors
        acc_thrust = np.linalg.norm(self._a_star) 
        vel_thrust = np.linalg.norm(self._pid_coeff.Pv * (self._v_star - self._v_c))
        pos_thrust = np.linalg.norm(self._pid_coeff.Pv * self._pid_coeff.Pp * pose_error)
        acc_err_thrust = np.linalg.norm(acc_sp - self._a_c)
        # print("acc_err: {:0.2f} \t acc: {:0.2f} \t vel: {:0.2f} \t pos {:0.2f}".format(acc_err_thrust, acc_thrust, vel_thrust, pos_thrust)) # DEBUG
        
        ### acceleration controller
        #acc_sp = np.array([0.0,0.5,0.0])
        # acc error
        acc_error = acc_sp - acc_cross_path_c
        
        # acc error derivative
        acc_error_d = self._lowpass((acc_cross_path_c - self._a_o)/dt , self._acc_error_d_prev)
        
        
        
        
        
        ### along path
        #PV = np.array([1.0,1.0,1.0])
        vel_error_path = self._v_star - vel_along_path_c
        acc_sp_path = self._pid_coeff.Pv * vel_error_path + self._a_star
        if np.linalg.norm(acc_sp_path > 5.0):
            acc_sp_path = acc_sp_path/np.linalg.norm(acc_sp_path) * 5.0
            
        acc_error_path = acc_sp_path - acc_along_path_c
        
        acc_sp = acc_sp_path + acc_sp
        
        
        
        # PID acceleration
        thrust_sp = self._pid_coeff.Pa * ( acc_error +acc_error_path ) + self._thrust_des#+ self._pid_coeff.Da * acc_error_d + self._integral_a + self._thrust_des
        
    

        #print acc_error
        
        #  corrected acceleration
        #thrust_delta_xy = self._pid_coeff.Pa[0] * acc_error_f[:2]
        #thrust_delta_z = self._pid_coeff.Pa[2] * acc_error_f[2]
        
        #self._thrust_des[:2] = thrust_delta_xy
        #self._thrust_des[2] = thrust_delta_z
        
        
        
        # map
        #thrust_sp = np.zeros(3)
        #thrust_sp[2] = self._pid_coeff.Mz * (acc_sp[2] + 9.9)
        #thrust_sp[:2] = self._pid_coeff.Mxy * acc_sp[:2]
        
        #cf.print_arrays([self._a_c, acc_sp, thrust_sp])

        #print("a_e:{}").format(acc_error )    
        #print("p_sp: {}, p_c: {},  v_sp: {}, v_c: {}, a_sp:{}, a_c:{}").format(self._p_star, self._p_c, vel_sp, self._v_c, acc_sp, self._a_c   )    
        #print("p_sp: {}, v_sp: {},  a_sp: {}, th: {}, v_error: {}, p_error: {}").format(self._p_star, vel_sp, acc_sp, thrust_sp, vel_error_norm, pose_error_norm  )
        #cf.print_arrays([self._a_c/np.linalg.norm(self._a_c), acc_sp/np.linalg.norm(acc_sp)])
        
        '''a_c_g= np.zeros(3)
        a_c_g[0] = self._a_c[0]
        a_c_g[1] = self._a_c[1]
        a_c_g[2] = self._a_c[2] + 9.91'''
        # cf.print_arrays([acc_sp, a_c_g, self._a_body, [np.linalg.norm(a_c_g), np.linalg.norm(self._a_body)]])
        #cf.print_arrays([acc_sp, self._a_c])
        
        # cf.print_arrays([vel_sp, self._v_c, a_long, self._a_star, self._a_c])
        
        
        #print("a_sp: {}, a_c:{}, a_e:{}").format(acc_sp, self._a_c, acc_error)
        #print("p_error: {}, v_error: {}, a_error:{}").format(pose_error, vel_error, acc_error)
        #print thrust_sp
        
        
        ### updates and resets
        
        # integral
     
        self._integral_v += self._pid_coeff.Iv * vel_error * dt
        self._integral_a += self._pid_coeff.Ia * acc_error * dt
        
        mag = np.linalg.norm(thrust_sp)
        
        if mag > 0.9: 
            
            if np.arccos(np.dot(vel_error_path, acc_error_path)/(np.linalg.norm(vel_error_path) * np.linalg.norm(acc_error_path))) < np.pi/2.0: # position not reached
                self._thrust_des = thrust_sp/np.linalg.norm(thrust_sp) * 0.9
                #print "max and change direction"
                #p.arccos(np.dot(vel_error_path, acc_error_path)/(np.linalg.norm(vel_error_path) * np.linalg.norm(acc_error_path)))
            else:
                self._thrust_des = thrust_sp
                #print "max and didnt change direction"
                
        elif np.linalg.norm(thrust_sp) <= 0.1:
            
            if np.arccos(np.dot(vel_error_path, acc_error_path)/(np.linalg.norm(vel_error_path) * np.linalg.norm(acc_error_path))) > np.pi/2.0: # position not reached
                self._thrust_des = thrust_sp/np.linalg.norm(thrust_sp) * 0.1
                #print "min and change direxction"
            else:
                self._thrust_Des = thrust_sp
                #print "min and not changed direction"
                
        else:
                self._thrust_des = thrust_sp
        
        self._vel_error_d_prev = vel_error_d
        self._acc_error_d_prev = acc_error_d
        
        self._v_o = self._v_c
        self._a_o = self._a_c

        # return 
        return thrust_sp, acc_sp, self._a_c
            

            
        
        
        
    def update_thrust_sp(self, time):
        
        # compute dt
        dt = 0.0
        if self._t_prev == 0.0:
            self._t_prev == time
            dt = 1.0/500.0
        else:
            dt = self.time - self._t_prev
            self._t_prev = time
            

        ### Position Cross Controller
        
        # cross position controller
        vel_sp = self._pid_coeff.Pp * ( self._p_star - self._p_c)
        
        
        
       
        # TODO do som saturation maybe for velocity
        
        # add feedforward velocity
        vel_sp += self._v_star
        
        #print self._pid_coeff.Pp
            
        
        ### PID velocity controller Controller for velocity
         
        # velocity error
        vel_error = vel_sp - self._v_c
        
        # p term
        Pv = self._pid_coeff.Pv * ( vel_error  - self._v_error[0,:] )
        
        # I term
        Iv = self._pid_coeff.Iv * vel_error * dt
        
        # D term
        Dv = self._pid_coeff.Dv / dt * ( vel_error - 2.0 * self._v_error[0,:] + self._v_error[1,:])
        
        # acceleration differece
        delta_a = Pv + Iv + Dv
        
        a_sp =  delta_a + self._a_star + self._a_prev 
        
        # anti windup
        '''a_sp = np.zeros(3)
        
        for idx in range(1, len(a_sp)):
            
            if np.abs( delta_a[idx] + self._a_prev[idx] + self._a_star[idx]) >= self._acc_max:
                a_sp[idx] = self._a_prev[idx] + self._a_star[idx]
            else:
                a_sp[idx] = self._a_prev[idx] + self._a_star[idx] + delta_a[idx]'''
            
            
            
            
        '''if ( np.linalg.norm(self._a_prev + delta_a + self._a_star) >= self._acc_max ):         
            a_sp = self._a_prev + self._a_star
        else:
            a_sp= self._a_prev + self._a_star + delta_a'''

            
        
        ### PID velocity controller for acceleration
    
        
      
        # acceleration error
        acc_error = a_sp - self._a_c
    
        
        # acc des
        thrust_sp =  self._pid_coeff.Pa * (acc_error) #+ self._thr_prev
        

        
        
        # p term
        '''Pa = self._pid_coeff.Pa * ( acc_error )#- self._a_error[0,:])
        
        # I term
        Ia = self._pid_coeff.Ia * acc_error * dt
        
        # D term
        Da =  self._pid_coeff.Da / dt * ( acc_error - 2.0 * self._a_error[0,:] + self._a_error[1,:])
        
        # thrust difference
        delta_thr = Pa #+ Ia + Da'''
        
        #print("acc_e:{}, delta_th:{}").format(acc_error, delta_thr)  '''  
        
        # desired thrust direction and magnitude
        '''delta_thr_dir = delta_thr / np.linalg.norm( delta_thr )
        delta_thr_mag = np.linalg.norm( delta_thr )
        
        # previous thrust direction and magnitude
        prev_thr_dir = self._thr_prev / np.linalg.norm( self._thr_prev )
        prev_thr_mag = np.linalg.norm( self._thr_prev)
        
        # desired thrust direction and mag
        thr_des_mag = np.linalg.norm( prev_thr_mag + delta_thr)
        thr_des_dir = (prev_thr_dir + delta_thr_dir)/np.l'''
        
        
        
        
        
        
        # anti windup + feedforward
        '''thrust_sp = np.zeros(3)
        if ( np.linalg.norm(self._thr_prev + delta_thr) >= self._thr_max ) or (np.linalg.norm(self._thr_prev + delta_thr) <= self._thr_min):         
            thrust_sp = thrust_sp/np.linalgnorm(thrust_sp) * 

            

        else:
        
            thrust_sp= self._thr_prev + delta_thr'''
        
        #thrust_sp =   self._pid_coeff.M * a_sp 
        #print("thr_delta:{}, thr_sp:{}").format(delta_thr, thrust_sp)    
        #print("a_c:{}, a_sp_g:{}").format(self._a_c, thrust_sp)    
        
        #print("a_star:{}, a_sp: {}, a_c: {}, thr: {}").format(self._a_star, a_sp_g, self._a_c, thrust_sp)
        
       
        #compute "thrust vector"
        #thrust_sp = self._pid_coeff.M * a_desired + 9.81
        
    
        
        # print 
        #print("e_p: {}, e_v: {}, a_e:{}, t_sp: {}, t_prev: {}").format(pos_error,vel_error, acc_error, thrust_sp, self._thr_prev )
        
        # update
        self._a_prev = a_sp
        self._v_error[1,:] = self._v_error[0,:]
        self._v_error[0,:] = vel_error
        self._thr_prev = thrust_sp
        self._a_error[1,:] = self._a_error[0,:]
        self._a_error[0,:] = acc_error
    
        return thrust_sp 
        
        
        
        
        
        
           
    
    
    
    
#  node enter point
def main():
    

    c = controller()
    
    
if __name__ == '__main__':
    main()

           
           
           
       
      


