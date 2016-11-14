#!/usr/bin/env python
"""
Created on Tue Oct 11 09:49:40 2016

@author: dennis
"""

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
import rospy

class state():
    def __init__(self, lock, driver):
        
        self._lock = lock
        self.driver = driver
        
        ## states
        # pose
        self._pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
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
        # path
        self._bezier_pub = rospy.Publisher('path/bezier_pt', Path, queue_size=10)
        self._bezier_msg = Path()
        self._bezier_state = "bezier"

        

        # default initialization
        self.set_state("posctr")

        # initial desired position: position and orientation
        ps = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        self.set_msg(ps)
        
        
    def set_state(self, arg):
        
        self._lock.acquire()
        if arg == "posctr":
            
            self.state = self._pose_state
            self.msg = self._pose_msg
            self.pub = self._pose_pub
            
        elif arg == "velctr":
            self.state = self._vel_state
            self.msg = self._vel_msg
            self.pub = self._vel_pub
            
        elif arg == "accelctr":
            self.state = self._accel_state
            self.msg = self._accel_msg
            self.pub = self._accel_pub
            
        elif arg == "bezier":
            self.state = self._bezier_state
            self.msg = self._bezier_msg
            self.pub = self._bezier_pub
            
        else:
            print "this state is not supported"
        self._lock.release()  
        
        
        
    def set_msg(self, arg):
        
        
        if self.state == "posctr":
            if len(arg) == 7:
                self._lock.acquire()
                self._pose_msg.pose.position.x = arg[0]
                self._pose_msg.pose.position.y = arg[1]
                self._pose_msg.pose.position.z = arg[2]
    
                self._pose_msg.pose.orientation.x = arg[3]
                self._pose_msg.pose.orientation.y = arg[4]
                self._pose_msg.pose.orientation.z = arg[5]
                self._pose_msg.pose.orientation.w = arg[6]   
                self._lock.release()
            else:
                print "posctr requires array of len 7"
                
            
        elif self.state == "velctr":
            if len(arg) == 3:
                self._lock.acquire()
                self._vel_msg.twist.linear.x = arg[0]
                self._vel_msg.twist.linear.y = arg[1]
                self._vel_msg.twist.linear.z = arg[2]
                
                #self._vel_msg.twist.angular.x = arg[3]
                #self._vel_msg.twist.angular.y = arg[4]
                #self._vel_msg.twist.angular.z = arg[5]
                self._lock.release()
            else:
                print "velctr requires array of len 3"
                
                
        elif self.state == "accelctr":
            if len(arg) == 3:
                self._lock.acquire()
                self._accel_msg.vector.x = arg[0]
                self._accel_msg.vector.y = arg[1]
                self._accel_msg.vector.z = arg[2]
                self._lock.acquire()

            else:
                print "accelctr requires array of len 3"
                
                
        
            
                
        elif self.state == "bezier":
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
                self._bezier_msg.poses = poses
                self._bezier_msg.header.stamp = rospy.get_rostime()
                #self._bezier_msg.header.frame_id = "local_origin"
                self._lock.release()
                
        
            
