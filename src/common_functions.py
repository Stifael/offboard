#!/usr/bin/env python
"""
Created on Fri Oct  7 13:27:14 2016

@author: dennis
"""
import numpy as np
from tf.transformations import *
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import Quaternion


def is_at_orientation(q_current, q_desired, offset_rad):
    q_des= np.array((q_desired.x, q_desired.y, q_desired.z, q_desired.w))
    q_c  = np.array((q_current.x, q_current.y, q_current.z, q_current.w))
    q_e = quaternion_multiply(q_des, quaternion_inverse(q_c))
    return 2.0 * np.arccos(np.abs(q_e[3])) < offset_rad
        #norm = np.sqrt(1 - (q_e[3] * q_e[3]))
        #v = [0,0,0]
        #v[0] = q_e[0]/norm
        #v[1] = q_e[1]/norm
        #v[2] = q_e[2]/norm
    
    
    
    
def is_at_position(p_current, p_desired, offset):
    
    des_pos = np.array((p_desired.x,
                        p_desired.y,
                        p_desired.z))
    cur_pos = np.array((p_current.x,
                        p_current.y,
                        p_current.z))
            
    return np.linalg.norm(des_pos - cur_pos) < offset


def q_ros_to_numpy(q_ros):
    # x,y,z,w
    q = np.array([0.0,0.0,0.0,0.0])
    q[0] = q_ros.x
    q[1] = q_ros.y
    q[2] = q_ros.z
    q[3] = q_ros.w
    return q
    
def p_numpy_to_ros(p_np):
    p = Point()
    p.x = p_np[0]
    p.y = p_np[1]
    p.z = p_np[2]
    return p

    
def p_numpy_to_ros_vector(p_np):
    p = Vector3()
    p.x = p_np[0]
    p.y = p_np[1]
    p.z = p_np[2]
    return p
    
    
def p_ros_to_numpy(p_ros):
    p = np.array([0.0,0.0,0.0])
    p[0] = p_ros.x
    p[1] = p_ros.y
    p[2] = p_ros.z
    return p

def print_arrays(arrays, format_str = '{0:.3f}'):
    
    array_formated = [None] * len(arrays)
    for idx, vec in enumerate(arrays):
        array_formated[idx] = ndprint(vec, format_str)
    
    print(' '.join('{}: {}'.format(*k) for k in enumerate(array_formated)))

        
    
    
def ndprint(a, format_string ='{0:.3f}'):
    return  [format_string.format(v,i) for i,v in enumerate(a)]

        
        
        
    

# from diebel with changed order: q = [x,y,z,w] 
def rotation_from_q(q):

    # change order if w is at index 3

    R = np.zeros([3,3])
    R[0,0] = q[3]*q[3] + q[0]*q[0] - q[1]*q[1] - q[2] * q[2]
    R[0,1] = 2*q[0]*q[1] + 2*q[3]*q[2] 
    R[0,2] = 2*q[0]*q[2] - 2*q[3]*q[1]
    
    R[1,0] = 2*q[0]*q[1] - 2*q[3]*q[2]
    R[1,1] = q[3]*q[3] - q[0]*q[0] + q[1]*q[1] - q[2] * q[2]
    R[1,2] = 2*q[1]*q[2] + 2*q[3]*q[0]
    
    R[2,0] = 2*q[0]*q[2] + 2*q[3]*q[1]
    R[2,1] = 2*q[1]*q[2] - 2*q[3]*q[0]
    R[2,2] = q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2] * q[2]
    
    return R
    
def rotation_from_q_transpose(q):
    
    return np.transpose(rotation_from_q(q))
    
def saturate_vector(vec, maximum, minimum):
    
    mag = np.linalg.norm(vec)
    if mag == 0.0:
        return vec
    mag_adjusted = min(maximum, max(minimum, mag))
    return vec/mag * mag_adjusted
    
    
    