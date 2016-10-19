#!/usr/bin/env python
"""
Created on Fri Sep 16 23:28:53 2016

@author: dennis
"""

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped 
from nav_msgs.msg import Path
import time
from tf.transformations import *
import numpy as np
import common_functions as cf
import bezier_fn as bf


### constant
RATE_STATE = 1 # state rate subscription


### class for subscription ###
class mapping():
    def __init__(self, nh):

    
        ### subscriber ###
        
        # state subscriber 
        self._rate_state = rospy.Rate(RATE_STATE)
        self._current_state = State()
        rospy.Subscriber('/mavros/state', State , self._current_state_cb)
        
        # subscriber,
        self._local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self._local_vel = TwistStamped()
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._local_vel_cb)
        self._bezier_pt = Path()
        rospy.Subscriber('/path/bezier_pt', Path, self._bezier_cb)
        
        
        # vel pub
        self._vel_pub =  rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10 )
        self._vel_msg = TwistStamped()

        
        
        
    
    def _pub_v_desired(self):
        
        # get current position
        pose = cf.p_ros_to_numpy(self._local_pose.pose.position)
        
        
        bz = [cf.p_ros_to_numpy(self._bezier_pt.poses[0].pose.position), \
                cf.p_ros_to_numpy(self._bezier_pt.poses[1].pose.position), \
                cf.p_ros_to_numpy(self._bezier_pt.poses[2].pose.position)]
        
        '''if len(self._bezier_pt.poses) == 3:
            # current bezier curve
           
        
            
        else:
            bz = []
            
        print bz'''

        # get closest point and velocity to bezier
        p_des, v_des = bf.point_closest_to_bezier(bz, pose)
        
        # get desired velocity
        v_final = bf.vel_adjusted(p_des, v_des, pose)
        
        # get yaw angle error
        theta = self.angle_error(v_final)
        
        # assign to msg
        self._vel_msg.twist.linear = cf.p_numpy_to_ros(v_final)
        self._vel_msg.twist.angular.z = theta 
     
        
        # publish
        self._vel_pub.publish(self._vel_msg)
        
        
    
        
        
    # finds closest point on circel to a specific point
    def angle_error(self, v_des):
        
        # current orrientation
        q_c = cf.q_ros_to_numpy(self._local_pose.pose.orientation)
        
        # convert v_des to body frame
        vb_des = np.dot(cf.rotation_from_q(q_c), v_des)
        
        # body z axis x
        z = np.array([0.0,0.0,1.0])
        x = np.array([1.0,0.0,0.0])

        # project onto xy body plane
        vb_des_proj = vb_des - z * np.dot(z, np.transpose(vb_des))
        
        # normalize
        vb_proj_n = vb_des_proj / np.linalg.norm(vb_des_proj)
        
        
        # get angle 
        theta = np.arccos(np.dot(x, np.transpose(vb_proj_n)))
        
        
        # determine sign
        cross = np.cross(x, vb_proj_n)
        if ( cross[2] < 0.0 ):
            theta *= -1.0
        
        return theta
            
        
        

        
            
       
        
    ### callback functions ###
    
    def _current_state_cb(self, data):
        self._current_state = data


    def _local_pose_cb(self, data):
        self._local_pose = data
        
    def _local_vel_cb(self, data):
        self._local_vel = data
        
    def _bezier_cb(self, data):
        self._bezier_pt = data
        self._pub_v_desired()
        
        
        
        




#  node enter point
def main():
    
    # create ros node handle
    nh = rospy.init_node('beziermapping')
    
    # create mapping obj
    mapping(nh)
    
    # spin 
    rospy.spin()
    
if __name__ == '__main__':
    main()