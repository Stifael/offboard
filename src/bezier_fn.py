#!/usr/bin/env python
"""
Created on Wed Oct 19 11:20:40 2016

@author: dennis
"""

import numpy as np
import math

_P = 2.0

def point_closest_to_bezier(bezier, pose, duration=2.0):
    P0 = np.array(bezier[0])
    P1 = np.array(bezier[1])
    P2 = np.array(bezier[2])
    r = np.array(pose)
    
    # The distance from pose to the bezier curve at time t
    bz_delta = lambda t: np.linalg.norm((1.0-t)**2 * P0 + 2 * t * (1.0-t) * P1 + t**2 * P2 - r)

    # Find the point on the curve that is closest to pose
    t = golden_section_search(bz_delta, 0.0, 1.0, 0.001)
    
    # position, velocity and acceleration corresponding to t
    p = (1.0-t)**2.0 * P0 + 2.0 * (1.0-t)*t * P1 + t**2 * P2
    v = (2.0 * (1.0-t) * (P1-P0) + 2.0 * t * (P2-P1)) / duration
    a = 2.0 * (P2 - 2.0 * P1 + P0) / duration**2

    return p,v,a


def golden_section_search(f, a, b, e=1e-5):
    ''' Golden section search to find the minimum of f on [a,b] '''
    golden_ratio = (math.sqrt(5) + 1) / 2
    c = b - (b - a) / golden_ratio
    d = a + (b - a) / golden_ratio
    while abs(c - d) > e:
        if f(c) < f(d):
            b = d
        else:
            a = c
        c = b - (b - a) / golden_ratio
        d = a + (b - a) / golden_ratio

    return (b + a) / 2

        
def accel_adjusted(p_des, v_des, a_des, pose, vel):
    
    delta_p = 0.0*( p_des - pose)    
    delta_v = 0.0*(v_des - vel)
    
    a_adj = (a_des + delta_v + delta_p)
    
    return a_adj
    
    
# adds the distance from current positin and desired to velocity vector  
def vel_adjusted(pose_desired, vel_desired, pose_current):
    global _P
    
    # numpy arrays
    pd = np.array(pose_desired)
    vd = np.array(vel_desired)
    pc = np.array(pose_current)
    
    # difference in position
    delta = pd - pc
    
    # veloctiy cross term
    vcross = _P * delta
    
    # velocity vector
    v_adj = vcross + vd
    
    # return
    return v_adj
    

    