# -*- coding: utf-8 -*-
"""
Created on Mon Oct 17 10:07:17 2016

@author: dennis
"""

import cicle_functions as cf
from tf.transformations import * 
import numpy as np
import common_functions as f

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class paths():
    def __init__(self, state, pub_path):
        
        self._state = state
        self.pub = pub_path
      
        
        
    def circle(self, radius, normal,  ax_body):
        
        # assign values
        self._radius = radius
        self._normal = normal/np.linalg.norm(normal)
        
        # current orientation in nunmpy form
        q = f.q_ros_to_numpy(self._state.driver.local_pose.pose.orientation)
        
        # axis to world frame: w = R_T * b
        ax_world = np.transpose(np.dot(np.transpose(f.rotation_from_q(q)), np.transpose(ax_body)))
        #print "ax: ", ax_world
        
        #  point from current position + ax_world
        pose = f.p_ros_to_numpy(self._state.driver.local_pose.pose.position)
        point = pose + ax_world
        
        # center: closest point to "point" and "circle"
        self._center = cf.closest_to_circle_and_point(point, pose, normal, radius)
        

        #print "center: ", self._center
    
        # vector from center to pose
        delta_center_pose = np.transpose(pose - self._center)
        point = pose
        
        
        
        # angle from 0: 2 * pi
        thetas = np.linspace(0.0, 2.0*np.pi, 20)
        
        # create points on circle path
        q = np.array([0.0,0.0,0.0,0.0]) # inialize rotation quaternion
        points = [0.0 for i in  thetas]  # initialze points
        self.pts = [0.0 for i in points] 
        
        for idx, theta in enumerate(thetas):
            
            
            # create rotation quaternion
            q[:3] = np.sin(theta * 0.5) * self._normal
            q[3] = np.cos(theta * 0.5)
            
            # new point is rotated vector + center
            point_new = np.dot(f.rotation_from_q(q),delta_center_pose )+ self._center


            # fill array
            points[idx] = f.p_numpy_to_ros(point_new)
            
            self.pts[idx] = point_new
            

        # creat bezier points
        self.bezier_points()
        
        # send path
        self.pub.pub_path(points)
        
        
        
        
    def bezier_points(self):
        
        self.bz_pt = [0.0 for i in self.pts]
        
        self.bz_pt[0] = [self.pts[0], self.pts[0], ( self.pts[1] + self.pts[0]) * 0.5]
        
        for idx in range(len(self.pts)-2):
            
            self.bz_pt[idx+1] = [self.bz_pt[idx][2], self.pts[idx+1], (self.pts[idx+1] + self.pts[idx+2])*0.5]
            
        
        # last
        self.bz_pt[len(self.pts)-1] = [self.bz_pt[len(self.pts)-2][2], self.pts[len(self.pts)-1], self.pts[len(self.pts)-1]]
            
            
        
        
        
        
        

        
        
        
        
        
        