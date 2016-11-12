#!/usr/bin/env python
"""
Created on Tue Oct 11 09:49:40 2016

@author: dennis
"""

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Point
from offboard.msg import ThreePointMsg
from nav_msgs.msg import Path
import rospy

class state():
    def __init__(self, lock, driver):
        
        self._lock = lock
        self._driver = driver
        
        ## states
        # pose
        self._pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        sekf._pose_des_sub = rospy.s
        self._pose_msg = PoseStamped()
        self._pose_state = "posctr"
        # vel 
        self._vel_pub =  rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10 )
        self._vel_msg = TwistStamped()
        self._vel_state = "velctr"
        # acc
        self._accel_pub = rospy.Publisher('mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
        self._accel_msg = Vector3Stamped()
        self._accel_state = "accelctr"
        rospy.Subscriber('/mode/acceleration', Point, self._acceleration_msg_cb)
        # three point msg
        self._bezier_pub = rospy.Publisher('path/bezier_pt', Path, queue_size=10)
        self._bezier_msg = Path()
        self._bezier_state = "bezier"
        rospy.Subscriber('/mode/three_point_message', ThreePointMsg, self._three_point_msg_cb)
    


        

        # default initialization
        self.set_state("posctr")

        # initial desired position: position and orientation
        ps = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        self.set_msg(ps)
    
    def get_state(self):
        return self._state
        
    def set_state(self, arg):
        
        self._lock.acquire()
        if arg == "posctr":
            
            self._state = self._pose_state
            self.msg = self._driver.get_pose_msg()
            self.pub = self._driver.get_pose_publisher()
            
        elif arg == "velctr":
            self._state = self._vel_state
            self.msg = self._driver.get_vel_msg()
            self.pub = self._driver.get_vel_publisher()
            
        elif arg == "accelctr":
            self._state = self._accel_state
            self.msg = self._driver.get_acc_msg()
            self.pub = self._driver.get_acc_puclisher()
            
        elif arg == "bezier":
            self._state = self._bezier_state
            self.msg = self._driver.get_bezier_msg()
            self.pub = self._driver.get_bezerier_pub()
            
        else:
            print "this state is not supported"
        self._lock.release()  
        
        
        
    def set_msg(self, arg):
        
        
        if self._state == "posctr":
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
                
            
        elif self._state == "velctr":
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
                
                
        elif self._state == "accelctr":
            if len(arg) == 3:
                self._lock.acquire()
                self.msg.vector.x = arg[0]
                self.msg.vector.y = arg[1]
                self.msg.vector.z = arg[2]
                self._lock.acquire()

            else:
                print "accelctr requires array of len 3"
                
                   
                
        elif self._state == "bezier":
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
        self._state = "three_point"
        self._bezier_pt = [data.prev, data.ctrl, data.next]
        self._bezier_duration = data.duration

                
    def _acceleration_msg_cb(self, data):
        self._state = "acceleration""
        
            
