#!/usr/bin/env python
"""
Created on Sat Oct  8 18:49:48 2016

@author: stifael
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class pub_bezier():
    def __init__(self):
        
        self.pub_vel = rospy.Publisher('vel_v', Marker, queue_size=10)
        self.pub_x = rospy.Publisher('veh_x', Marker, queue_size=10)
        self.pub_a = rospy.Publisher('veh_a', Marker, queue_size=10)
        self.init_vel()
        self.init_x()
        self.init_a()
        

    def init_vel(self):
        
        
        vel = Marker()
        vel.header.frame_id = "local_origin"
        vel.header.stamp = rospy.Time.now()
        vel.ns = "bezier"
        vel.action = vel.ADD
        vel.type = vel.ARROW
        vel.id = 0

        vel.scale.x = 0.1
        vel.scale.y = 0.2
        vel.scale.z = 0.7
        #
    
        vel.color.a = 1.0
        #markers.color.r = 1.0
        vel.color.g = 0.5 
        vel.color.b = 0.5
        
        self.vel = vel
        
        
    def init_x(self):
        
        
        x = Marker()
        x.header.frame_id = "local_origin"
        x.header.stamp = rospy.Time.now()
        x.ns = "bezier"
        x.action = x.ADD
        x.type = x.ARROW
        x.id = 0

        x.scale.x = 0.1
        x.scale.y = 0.2
        x.scale.z = 0.7
        #
    
        x.color.a = 1.0
        x.color.r = 0.5
        x.color.g = 0.5
        x.color.b = 0.5
        
        self.x= x
        
        
    def init_a(self):
        
        
        a = Marker()
        a.header.frame_id = "local_origin"
        a.header.stamp = rospy.Time.now()
        a.ns = "bezier"
        a.action = a.ADD
        a.type = a.ARROW
        a.id = 0

        a.scale.x = 0.1
        a.scale.y = 0.2
        a.scale.z = 0.2
        #
    
        a.color.a = 1.0
        a.color.r = 0.5
        a.color.g = 0.1
        a.color.b = 0.1
        
        self.a = a
        
        
    def pub_x_vec(self, xPts):
        
        self.x.points = xPts
        self.x.header.stamp = rospy.Time.now()
        
        self.pub_x.publish(self.x)
           
        
        
    def pub_velocity(self, velPts):
        

        self.vel.points = velPts
        self.vel.header.stamp = rospy.Time.now()   
        
        # publish points
        self.pub_vel.publish(self.vel)
        
        
    def pub_a_vec(self, accPts):
        
        self.a.points = accPts
        self.a.header.stamp = rospy.Time.now()
        self.pub_a.publish( self.a )
        
        
        
    
        

        
        
        
        
    

        
        