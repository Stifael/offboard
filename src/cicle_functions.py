#!/usr/bin/env python
"""
Created on Sat Oct  8 13:05:30 2016

@author: stifael
"""
import numpy as np

# finds closest point on circel to a specific point
def closest_to_circle_and_point(point, center, normal, radius):

    # central coordintate
    p = point - center
    
    # projection
    p_p = p - normal * np.dot(normal, p.reshape((3,1)))
     
    # check if p_p is on center
    if np.array_equal(p_p, np.array([0.0,0.0,0.0])):
        p_p = perpendicular(normal)
            
    # nearest point 
    p_closest = p_p * radius / np.linalg.norm(p_p)
        
    # return with original coorinate system
    return p_closest + center
    
    
    
# helper function for closest point function    
def perpendicular(n):
    if np.abs(n[1]) < np.abs(n[2]) :
        return np.cross(n, np.array([1,0,0]))
    else:
        return np.cross(n, np.array([0,1,0]))
        
        
# unit vector tangent to circle at specific point    
def tangent_to_circle(normal, point, center):
    
    # make sure normal is normal
    normal = normal / np.linalg.norm(normal)
    
    # unit vector to point from center coordinate
    p = (point - center) / np.linalg.norm(point - center)
    
    # return tangen unit vector
    return np.cross(normal, p)
    
    
    
    
    
    
    
if __name__ == '__main__':
    n = np.array([0,0,1])
    r = 1
    c = np.array([0,0,0])
    p = np.array([1,1,1])
    p_c = closest_to_circle_and_point(p,c,n,r)
    
    print tangent_to_circle(n, p_c, c)
    
    
    
    