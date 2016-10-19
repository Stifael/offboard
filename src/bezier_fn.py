#!/usr/bin/env python
"""
Created on Wed Oct 19 11:20:40 2016

@author: dennis
"""

import numpy as np

_P = 2.0

def point_closest_to_bezier(bezier, pose):
    
    # assign variables
    P00 = bezier[0][0]
    P01 = bezier[0][1]
    P02 = bezier[0][2]
    P10 = bezier[1][0]
    P11 = bezier[1][1]
    P12 = bezier[1][2]
    P20 = bezier[2][0]
    P21 = bezier[2][1]
    P22 = bezier[2][2]
    r1 = pose[0]
    r2 = pose[1]
    r3 = pose[2]
    
    P0 = np.array([P00, P01, P02])
    P1 = np.array([P10, P11, P12])
    P2 = np.array([P20, P21, P22])
    #r = np.array([r1, r2, r3])
    
    # bezier distance fun
    bz_delta = lambda t: (P00*(-t + 1)**2 + P10*t*(-2*t + 2) + P20*t**2 - r1)**2 + (P01*(-t + 1)**2 + P11*t*(-2*t + 2) + P21*t**2 - r2)**2 + (P02*(-t + 1)**2 + P12*t*(-2*t + 2) + P22*t**2 - r3)**2
    
    # t of point bezier point closest to current position
    x = binary_2(bz_delta, 0.0, 1.0, 0.001)
    
    # velocity vector corresponding to x
    v = 2* (1-x) * (P1-P0)+2.0*x*(P2-P1)
    
    # position corresponding to x
    p = (1 - x)**2 * P0 + 2.0*(1-x)*x*P1 + x*x*P2
    
    # point on bezier correpondint to x
    return p,v
    
    
    

# return a number from 0 to 1: function needs be quadratic function 
def binary_2(fun, x_l, x_r, eps):
    
    y_r = fun(x_r)
    y_l = fun(x_l)
    
    x_m = 0.5 * (x_l + x_r)
    y_m = fun(x_m)
    
    while(abs(y_l - y_m) >= eps):
        
        if y_r < y_l:
            
            y_l = y_m
            x_l = x_m
            
        else:
            
            y_r = y_m
            x_r = x_m
            
            
        x_m = 0.5 * (x_l + x_r)
        y_m = fun(x_m)
        
    return x_m
        
        
    
    
    
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
    

    