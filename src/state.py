#!/usr/bin/env python
"""
Created on Tue Oct 11 09:49:40 2016

@author: dennis
"""

from geometry_msgs.msg import PoseStamped, Point
from offboard.msg import ThreePointMsg, CircleMsg
import rospy
import common_functions as cf
import mavros_driver
import numpy as np


class state():
    def __init__(self, nh, lock, driver):
        
        self._lock = lock
        self._driver = mavros_driver.mavros_driver(nh)
        
        ## states
        # pose
        rospy.Subscriber('state/pose_ctr', Point, self._relative_pose_setpoint_cb)
        self.pose_state = "pose_ctr"
        self._pose_sp = np.zeros(3)
        # vel 
        rospy.Subscriber('state/vel_ctr', Point, self._vel_setpoint_cb)
        self.vel_state = "vel_ctr"
        self._vel_sp = np.zeros(3)
        # acc
        rospy.Subscriber('state/acc_ctr', Point, self._acc_setpoint_cb)
        self.acc_state = "acc_ctr"
        self._acc_sp = np.zeros(3)
        # three point msg
        self.three_point_state = "three_point"
        rospy.Subscriber('state/three_point_message', ThreePointMsg, self._three_point_msg_cb)
        self._three_pt = [0,0,0]
        self._three_pt_duration = 1.0
        # circle 
        self.circle_state = "circle"
        rospy.Subscriber('state/circle',CircleMsg, self._circel_msg_cb)
        self._circle_axis = np.zero(3)
        self._circle_radius = 0.0
        
        '''
        ## initialization
        # default initialization
        self.set_state("pos_ctr")

        # initial desired position: position and orientation
        ps = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        self.set_msg(ps)'''
        
        
    # getters
    def get_state(self):
        return self._state
    def get_mode(self):
        return self.driver.current_state.mode
    def get_local_pose(self):
        return cf.p_ros_to_numpy(self._driver.local_pose.pose.position)
    def get_orientation(self): # from world to body
        return cf.q_ros_to_numpy(self._driver.local_pose.pose.orientation)
    def get_local_vel(self):
        return cf.p_ros_to_numpy(self._driver.local_vel.twist.linear)
    def get_body_acc(self):
        return cf.p_ros_to_numpy(self._driver.body_acc.linear_acceleration)
    def get_local_acc(self):
        q = self.get_orientation()
        acc_b = self.get_body_acc()
        return np.dot(cf.rotation_from_q_transpose(q), acc_b)
    
    
        
    def set_msg(self, arg):
        if self._state == "pose_ctr":
            if len(arg) == 7:
                self._lock.acquire()
                self.msg.pose.position.x = arg[0]
                self.msg.pose.position.y = arg[1]
                self.msg.pose.position.z = arg[2]
    
                self.msg.pose.orientation.x = arg[3]
                self.msg.pose.orientation.y = arg[4]
                self.msg.pose.orientation.z = arg[5]
                self.msg.pose.orientation.w = arg[6]   
                self._lock.release()
            else:
                print "posctr requires array of len 7"
                
            
        elif self._state == "vel_ctr":
            if len(arg) == 3:
                self._lock.acquire()
                self.msg.twist.linear.x = arg[0]
                self.msg.twist.linear.y = arg[1]
                self.msg.twist.linear.z = arg[2]
                
                #self._vel_msg.twist.angular.x = arg[3]
                #self._vel_msg.twist.angular.y = arg[4]
                #self._vel_msg.twist.angular.z = arg[5]
                self._lock.release()
            else:
                print "velctr requires array of len 3"
                
                
        elif self._state == "acc_ctr":
            if len(arg) == 3:
                self._lock.acquire()
                self.msg.vector.x = arg[0]
                self.msg.vector.y = arg[1]
                self.msg.vector.z = arg[2]
                self._lock.acquire()

            else:
                print "accctr requires array of len 3"
                
      
        elif self._state == "three_point":
            if len(arg) == 3:
                # initialize
                poses = []
                
                # loop through bezier points
                for idx, pose in enumerate(arg):
                    
                    p = PoseStamped()
                    p.pose.position.x = pose[0]
                    p.pose.position.y = pose[1]
                    p.pose.position.z = pose[2]
                    
                    poses.append(p)
                
                self._lock.acquire()
                self.msg.poses = poses
                self.msg.header.stamp = rospy.get_rostime()
                #self._bezier_msg.header.frame_id = "local_origin"
                self._lock.release()

                
    ### state callbacks   
    def _three_point_msg_cb(self, data):
        self._three_pt = [cf.p_ros_to_numpy(data.prev), \
                           cf.p_ros_to_numpy(data.ctrl), \
                           cf.p_ros_to_numpy(data.next)]
        self._three_pt_duration = data.duration
        if self._state is not self.three_point_state:
            self._lock.acquire()
            self._state = self.three_point_state
            self.msg = self._driver.get_acc_msg()
            self.pub = self._driver.get_acc_publisher()
            self._lock.release()
            
        
    def _circel_msg_cb(self, data):
        self._circle_axis = data.axis
        self._circle_radius = data.radius
        if self._state is not self.circle_state:
            self._lock.acquire()
            self._state = self.three_point_state
            self.msg = self._driver.get_acc_msg()
            self.pub = self._driver.get_acc_publisher()
            self._lock.release()
             
    def _acc_setpoint_cb(self, data):
        self._acc_sp = cf.p_ros_to_numpy(data)
        if self._state is not self.acc_state:
            self._lock.acquire()
            self._state = self.accel_state
            self.msg = self._driver.get_acc_msg()
            self.pub = self._driver.get_acc_publisher()
            self._lock.release()
        
    def _vel_setpoint_cb(self, data):
        self._vel_sp = cf.p_ros_to_numpy(data)
        if self._state is not self.vel_state:
            self._lock.acquire()
            self._state = self.vel_state            
            self.msg = self._driver.get_vel_msg()
            self.pub = self._driver.get_vel_publisher()
            self._lock.release()
    
    def _relative_pose_setpoint_cb(self, data):
        self._pose_sp = cf.p_ros_to_numpy(data)
        if self._state is not self.pose_state:
            self._lock.acquire()
            self._state = self.pose_state
            self.msg = self._driver.get_pose_msg()
            self.pub = self._driver.get_pose_publisher()
            self._lock.release()

    
        
            
