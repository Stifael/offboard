#!/usr/bin/env python
"""
Created on Thu Sep 29 09:57:26 2016

@author: dennis
"""
import time 
import numpy as np
from operator import xor
from tf.transformations import * #quaternion_from_euler, quaternion_multiply, quaternion_matrix
#from __future__ import print_function
import numpy as np
import common_functions as f



class setpoint():
    def __init__(self, state, pub_target):
        
        '''self._pose_msg = pose_msg # pointer to the setpoint position msg
        self._driver = driver
        self._lock = pos_lock
        self._state = state'''
        self._state = state
        self.pub_target = pub_target
        
        
    def do_step(self, step):
        
        if self._state.state == "posctr":
            self.do_step_in_pose( step )
        else:
            self.do_step_in_vel( step )
        
        
    def do_step_in_pose(self, step):
        
        print 'do step'

        
        # convert yaw to desire orientation
        q_des = self.twist_in_yaw(step[3])
        
        # desire step 
        des_step = step[:3] + q_des
        
        # get current pose
        dummy = self._state.driver.local_pose.pose.position
        des_step[0] += dummy.x
        des_step[1] += dummy.y
        des_step[2] += dummy.z
        
        # set step
        self._state.set_msg(des_step)
        
        # publish desired point
        
        
        # check if it reaches goal in predefined time
        '''t0 = time.time()
        offset = 0.1
        offset_rad = 0.1
        
        while xor(time.time() - t0 < timeout,  ( f.is_at_position(self._state.driver., self._driver.local_pose.pose.position, offset)  and \
        f.is_at_orientation(self._pose_msg.pose.orientation,self._driver.local_pose.pose.orientation ,offset_rad))):
            
            time.sleep(0.2)

        # print if goal was reached    
        if f.is_at_position(self._pose_msg.pose.position, self._driver.local_pose.pose.position, offset) and \
        f.is_at_orientation(self._pose_msg.pose.orientation,self._driver.local_pose.pose.orientation ,offset_rad):
            print 'reached goal'
        else:
            print 'too slow'''
            
            
    def twist_in_yaw(self, yaw):
                
        # get current orientation
        dummy = self._state.driver.local_pose.pose.orientation
                    
        # x,y,z,w
        q_c = [0,0,0,0]
        q_c[0] = dummy.x
        q_c[1] = dummy.y
        q_c[2] = dummy.z
        q_c[3] = dummy.w
          
        # desire rotation
        q_rot = [0,0,0,0]
        q_rot[3] = np.cos(0.5*yaw)
        q_rot[2] = np.sin(0.5*yaw)
                    
        # desire attitude
        return quaternion_multiply(q_rot, q_c).tolist()
    
    def do_step_in_vel(self, step):
        
        print "do vel step"
        
        dummy = self._state.driver.local_vel.twist.linear
        step[0] += dummy.x
        step[1] += dummy.y
        step[2] += dummy.z
        
        self._state.set_msg(step)
        
        
        # get current vel
        #dummy = self._state.driver.local_pose.pose.position
    
    
    

        
    