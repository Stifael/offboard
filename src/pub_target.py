#!/usr/bin/env python
"""
Created on Sat Oct  8 18:49:48 2016

@author: stifael
"""

import rospy
from visualization_msgs.msg import Marker



class pub_target():
    def __init__(self):
        
        self.pub_pa = rospy.Publisher('path', Marker, queue_size=10 )
        self.pub_sp = rospy.Publisher('setpoint', Marker, queue_size=10)

        
        
        
    def pub_path(self, points):
        

        markers = Marker()
        markers.header.frame_id = "local_origin"
        markers.header.stamp = rospy.Time.now()
        markers.ns = "points"
        markers.action = markers.ADD
        markers.type = markers.LINE_STRIP
        markers.id = 0

        markers.scale.x = 0.05
        #markers.scale.y = 0.1
    
        markers.color.a = 1.0
        #markers.color.r = 1.0
        #markers.color.g = 1.0 
        markers.color.b = 1.0
        
        #markers.pose.orientation.w = 1.0
        
        # create vertices
        #for idx, point in points:
        markers.points = points
            
        
        # publish points
        self.pub_pa.publish(markers)
        
        
        
    def init_sp(self):
        
        self.pose = Marker()
        self.pose.header.frame_id = "local_origin"
        self.pose.ns = "points"
        self.pose.action = self.pose.ADD
        self.pose.type = self.pose.LINE_STRIP
        self.pose.id = 0
        self.pose.scale.x = 0.1
        self.pose.color.a = 1.0
        self.pose.color.g = 1.0
        
        
        
    def pub_setpoints(self, pose):
        
        self.pose.points.append(pose)
        self.pose.header.stamp = rospy.Time.now()
        
        # publish
        self.pub_sp.publish(self.pose)
        
        
        
        
        
    

        
        
    