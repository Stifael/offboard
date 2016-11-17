#!/usr/bin/env python
"""
#!/usr/bin/env python
Created on Fri Sep 16 23:28:53 2016

@author: dennis
"""

import rospy
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget, AvoidanceTriplet
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Quaternion, Vector3, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
import time
from tf.transformations import *
import numpy as np
import common_functions as cf
import bezier_fn as bf
import pub_bezier
from dynamic_reconfigure.server import Server
from offboard.cfg import PIDConfig
from offboard.msg import ThreePointMsg
import controller 

### constant
RATE_STATE = 1 # state rate subscription


class pidCoeff():
    def __init__(self):
        
        self.Pp = np.zeros(3)
        self.Pv = np.zeros(3)
        self.Iv = np.zeros(3)
        self.Dv = np.zeros(3)
        self.Pa = np.zeros(3)
        self.Ia = np.zeros(3)
        self.Da = np.zeros(3)
        self.Mxy = 0.0
        self.Mz = 0.0
        
        

### class for subscription ###
class mapping():
    def __init__(self, nh):
        
        
        
        
        
        
        self._run_bz_controller = False
        
        
        # vel pub
        self._vel_pub =  rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10 )
        self._vel_msg = TwistStamped()
        

        # acc pub
        self._accel_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10 )
        self._accel_msg = Vector3Stamped()
        
        # attitude 
        self._att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self._att_msg = AttitudeTarget()
        self._att_msg.type_mask = 7
        
        # local raw: send acceleration and yaw
        self._acc_yaw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size= 10)
        self._acc_yaw_msg = PositionTarget()
        self._acc_yaw_msg.type_mask = 2048 + 32 + 16 + 8 + 4 + 2 + 1  #+ 512
        
        # local raw: send velocity and yaw
        self._vel_yaw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size= 10)
        self._vel_yaw_msg = PositionTarget()
        self._vel_yaw_msg.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048
        
        # path bezier triplet send
        self._bezier_triplet_pub = rospy.Publisher('/mavros/avoidance_triplet', AvoidanceTriplet, queue_size=10)
        self._bezier_triplet_msg = AvoidanceTriplet()
        self._bezier_duration = 1.0
        
        
        
        # initlaize publisher for visualization
        self._pub_visualize = pub_bezier.pub_bezier()
        
        # dt
        self._dt = 0.0
        
                
        # call back variables
        self._pid_coeff = pidCoeff()
        
        #print self._pid_coeff
        
        # pid tuning parameters with first callback
        Server(PIDConfig, self._pidcallback)

        
        self._ctr = controller.controller(self._pid_coeff, 9.91)
    
        ### subscriber ###
        
        # state subscriber 
        self._rate_state = rospy.Rate(RATE_STATE)
        self._current_state = State()
        rospy.Subscriber('/mavros/state', State , self._current_state_cb)
        
        # subscriber,
        self._local_pose = PoseStamped()
        self._local_pose.pose.position = cf.p_numpy_to_ros([0.0,0.0,0.0])
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        
        
        self._local_vel = TwistStamped()
        self._local_vel.twist.linear = cf.p_numpy_to_ros([0.0,0.0,0.0])
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._local_vel_cb)
        
        self._bezier_pt = []
        '''self._bezier_pt[0] = cf.p_numpy_to_ros([0.0,0.0,0.0])
        self._bezier_pt[1] = cf.p_numpy_to_ros([0.0,0.0,0.0])
        self._bezier_pt[2] = cf.p_numpy_to_ros([0.0,0.0,0.0]'''
        #self._bezier_duration = 1.0
        rospy.Subscriber('/path/bezier_pt', Path, self._bezier_cb)
        rospy.Subscriber('/path/three_point_message', ThreePointMsg, self._three_point_msg_cb)
        
        self._linear_acc = Vector3()
        self._linear_acc = cf.p_numpy_to_ros_vector([0.0,0.0,0.0])
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)

 
        
        
        #= {"Pxy_p": 0.0, "Pz_p": 0.0, "Pxy_v":0.0 , "Pz_v":0.0, "Dxy_v":0.0 , "Dz_v":0.0 ,  "Ixy_v":0.0 , "Iz_v":0.0, "M":0.0 }
        # controller
       
        

        
        
        
    def _pub_thrust_sp_desired(self):
        
        #self._local_pose.pose.position = np.array([1,2,3])
        #self._local_vel_twist_linear = np.array([1,2,3])
        
        
        # get current position p_c, v_c, a_c in local frame
        p_c = cf.p_ros_to_numpy(self._local_pose.pose.position)
        q_c = cf.q_ros_to_numpy(self._local_pose.pose.orientation) 
        v_c =cf.p_ros_to_numpy(self._local_vel.twist.linear) 
        # v_c_body = 
        a_c = cf.p_ros_to_numpy(self._linear_acc) # bodyframe
        a_c = np.dot(cf.rotation_from_q_transpose(q_c), a_c) # world frame
        
      
        # bezier points
        bz = [cf.p_ros_to_numpy(self._bezier_pt[0]), \
                cf.p_ros_to_numpy(self._bezier_pt[1]), \
                cf.p_ros_to_numpy(self._bezier_pt[2])]

        
        # get closest point p*, velocity v* and acceleration a*
        p_star, v_star, a_star = bf.point_closest_to_bezier(bz, p_c, self._bezier_duration)
        
        
        '''p_star = np.array([0.0,0.0,5.0])
        v_star = np.array([0.0,0.0,0.0])
        a_star = np.array([0.0,0.0,0])'''

        # set states
        self._ctr.set_states(p_c, v_c, a_c, p_star, v_star, a_star, self._pid_coeff)
        
        # compute desired thrust
        thrust_des, v_sp, vc = self._ctr.update_thrust_old(time.time())
        
        # send vel and thrust vector
        self._visualize_vel(p_c, vc)
        self._visualize_acc(p_c, v_sp )
        self._visualize_x(p_c)
        self._visualize_target(p_star)

        
        # get correct yaw
        # get yaw angle error
        yaw_desired = 0.0
        v_star_norm= np.linalg.norm(v_star)
        z = np.array([0.0,0.0,1.0])
        if (v_star_norm > 0.0) and not (np.array_equal(np.abs(v_star/v_star_norm), z)): #yaw not defined if norm(v_des) or v_des == z 
            # get current yaw
            yaw_desired = self.get_desired_yaw(v_star) - np.pi/2.0
            
       
         
        
        
        # assign to msg
        self._acc_yaw_msg.acceleration_or_force =  cf.p_numpy_to_ros_vector(thrust_des)
        self._acc_yaw_msg.yaw = yaw_desired
        

        # publish
        self._acc_yaw_pub.publish(self._acc_yaw_msg)
        
        
        
        
    def _pub_att_desired(self):

        
        q = Quaternion()
        q.x =0.0
        q.y = 0.0
        q.z = 1.0
        q.w = 0.0
        
        self._att_msg.orientation = q
        self._att_msg.thrust =1.0
        
        self._att_pub.publish(self._att_msg)
        
        
    def _pub_acc_yaw_desired(self):
        
        a = Vector3()
        a.x = 0.0
        a.y = 0.0
        a.z = 0.2
        self._acc_yaw_msg.acceleration_or_force = a
        #self._local_msg.yaw = 0.0
        
        self._local_pub.publish(self._acc_yaw_msg)
        
        
    
    def _pub_v_desired(self):
        
        # get current position
        pose = cf.p_ros_to_numpy(self._local_pose.pose.position)
        
        
        bz = [cf.p_ros_to_numpy(self._bezier_pt[0]), \
                cf.p_ros_to_numpy(self._bezier_pt[1]), \
                cf.p_ros_to_numpy(self._bezier_pt[2])]
        

        # get closest point and velocity to bezier
        p_des, v_des, a_des = bf.point_closest_to_bezier(bz, pose, self._bezier_duration)
        
        print a_des
        
        # send velocity vector
        self._visualize_vel(p_des, v_des)
        self._visualize_x(pose)
        
        
        # get desired velocity
        v_final = bf.vel_adjusted(p_des, v_des, pose)
        v_final *= min(np.linalg.norm(v_final), 3.0) / np.linalg.norm(v_final)
        
        # get yaw angle error
        theta = 0.0
        v_des_norm= np.linalg.norm(v_des)
        z = np.array([0.0,0.0,1.0])
        if (v_des_norm > 0.0) and not (np.array_equal(np.abs(v_des/v_des_norm), z)): #yaw not defined if norm(v_des) or v_des == z 
            
            theta = self.angle_error(v_des)
            
        # get current yaw
        yaw_desired = self.get_desired_yaw(v_des) - np.pi/2.0
        
        
        # assign to msg
        self._vel_yaw_msg.velocity = cf.p_numpy_to_ros_vector(v_final)
        self._vel_yaw_msg.yaw = yaw_desired
        
        # publish
        self._vel_yaw_pub.publish(self._vel_yaw_msg)
        
    
    
    def _visualize_x(self, pose):
        
        # current orientation
        q_c = cf.q_ros_to_numpy(self._local_pose.pose.orientation)
        
        # body frame x
        x_b = np.array([1.0,0.0,0.0])
        
        # convert to world frame
        x = np.dot(cf.rotation_from_q_transpose(q_c), x_b)
        
        pt = cf.p_numpy_to_ros(pose)
        pt2 = cf.p_numpy_to_ros(pose + x)
        
        pts = [pt, pt2]
        
        
        self._pub_visualize.pub_x_vec(pts)
        
    def _visualize_target(self, p):
        
        pt = cf.p_numpy_to_ros(p)
        
        self._pub_visualize.pub_target(pt)
      
    def _visualize_vel(self, p, v):
        

        pt = cf.p_numpy_to_ros(p)
        pt2 = cf.p_numpy_to_ros(v + p)
        points = [pt, pt2]

        self._pub_visualize.pub_velocity(points)
        
        
        
    def _visualize_acc(self, p, a):    
        pt = cf.p_numpy_to_ros(p)
        pt2 = cf.p_numpy_to_ros( p + a)
        points = [pt, pt2]
        self._pub_visualize.pub_a_vec(points)        
        
        
    
    
    def _pub_a_desired(self):
        

        # get current position, velocity
        pose = cf.p_ros_to_numpy(self._local_pose.pose.position)
        velocity = cf.p_ros_to_numpy(self._local_vel.twist.linear)
        
        
        bz = [cf.p_ros_to_numpy(self._bezier_pt[0]), \
                cf.p_ros_to_numpy(self._bezier_pt[1]), \
                cf.p_ros_to_numpy(self._bezier_pt[2])]
        

        # get closest point and velocity and acceleration to bezier
        p_des, v_des, a_des = bf.point_closest_to_bezier(bz, pose, self._bezier_duration)
        
        
        # get desired velocity
        #a_final = bf.accel_adjusted(p_des, v_des, a_des, pose, velocity)
        
        
        print a_des
        
        #print "a_des : {}\t v_des : {}\t p_des: {}".format(a_des, v_des, p_des) 
        
        # get yaw angle error
        '''theta = 0.0
        v_des_norm= np.linalg.norm(v_des)
        z = np.array([0.0,0.0,1.0])
        if (v_des_norm > 0.0) and not (np.array_equal(np.abs(v_des/v_des_norm), z)): #yaw not defined if norm(v_des) or v_des == z 
            
            theta = self.angle_error(v_des)'''
            
        #a_final = np.array([0.0,0.0,0.52])
        
        # assign to msg
        self._accel_msg.vector = cf.p_numpy_to_ros(a_des)

     
        
        # publish
        self._accel_pub.publish(self._accel_msg)
        
        

        
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
            
        #print theta
        
        return theta
        
        
    # get desired yaw   
    def get_desired_yaw(self, v_des):
        
        # z axis
        z = np.array([0.0,0.0,1.0])
        x = np.array([1.0,0.0,0.0])
        
        # project v_des onto xy plane
        v_des_proj = v_des - z * np.dot(z, np.transpose(v_des))
        v_des_p_n = v_des_proj / np.linalg.norm(v_des_proj)
        
        # angle between v_des_prj and x
        angle = np.arccos(np.dot(x, np.transpose(v_des_p_n)))
        
        # sign
        cross = np.cross(x, v_des_p_n)
        if (cross[2] < 0.0):
            angle *= -1
             
        return angle
            
        
        
            
        
    # current yaw 
    def get_current_yaw(self):
       
        # current orrientation
        q_c = cf.q_ros_to_numpy(self._local_pose.pose.orientation)
        
        # body frame x
        x_b = np.array([1.0,0.0,0.0])
        
        # convert to world frame
        x_w = np.dot(cf.rotation_from_q_transpose(q_c), x_b)
        
        # norm of xy plane
        z = np.array([0.0,0.0,1.0])
        x = np.array([1.0,0.0,0.0])
        
        # pojecto on xy placne of world frame
        x_w_proj = x_w - z * np.dot(z, np.transpose(x_w))  
        
        # normalize
        x_w_proj_n = x_w_proj / np.linalg.norm(x_w_proj)
        
        # get angle 
        yaw = np.arccos(np.dot(x, np.transpose(x_w_proj_n)))
        
        
        #determine sign
        cross = np.cross(x, x_w_proj_n)
        if (cross[2] < 0.0):
            yaw *= -1.0
            
        return yaw
        
        
    def send_bezier_triplet(self):
        
        self._bezier_triplet_msg.prev = self._bezier_pt[0]
        self._bezier_triplet_msg.ctrl = self._bezier_pt[1]
        self._bezier_triplet_msg.next = self._bezier_pt[2]
        
        self._bezier_triplet_msg.acc_per_err = 0.0
        self._bezier_triplet_msg.duration = 1.0
        self._bezier_triplet_msg.max_acc   = 5.0

        print self._bezier_pt
        self._bezier_triplet_pub.publish(self._bezier_triplet_msg)
        
       
        
    ### callback functions ###
    
    def _current_state_cb(self, data):
        self._current_state = data


    def _local_pose_cb(self, data):
        self._local_pose = data
        
    def _local_vel_cb(self, data):
        self._local_vel = data
        
    def _imu_cb(self, data):
        self._linear_acc = data.linear_acceleration
        if self._run_bz_controller:
            self._pub_thrust_sp_desired()
        
        
    def _bezier_cb(self, data):
        self._bezier_pt = [pose.pose.position for pose in data.poses]
        #self.send_bezier_triplet()
        self._run_bz_controller = True
        
        # self._bezier_pt = [pose.pose.position for pose in data.poses]
        # self._run_bz_controller = True
        # #self._pub_a_desired()
    
    def _three_point_msg_cb(self, data):
        self._bezier_pt = [data.prev, data.ctrl, data.next]
        self._bezier_duration = data.duration
        self._run_bz_controller = True
        
         
    def _pidcallback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {Pxy_p}, {Pz_p},\ 
        #{Pxy_v}, {Pz_v}, {Ixy_v}, {Ixy_v}, {Iz_v}, {Dxy_v}, {Dz_v}, {Mz}, {Mxy}""".format(**config))
        
        # pid struct
        pid = pidCoeff()
        
        pid.Pp[0] = config.Pxy_p
        pid.Pp[1] = config.Pxy_p
        pid.Pp[2] = config.Pz_p
        
        pid.Pv[0] = config.Pxy_v
        pid.Pv[1] = config.Pxy_v
        pid.Pv[2] = config.Pz_v
        
        pid.Iv[0] = config.Ixy_v
        pid.Iv[1] = config.Ixy_v
        pid.Iv[2] = config.Iz_v
        
        pid.Dv[0] = config.Dxy_v
        pid.Dv[1] = config.Dxy_v
        pid.Dv[2] = config.Dz_v
        
         
        pid.Pa[0] = config.Pxy_a
        pid.Pa[1] = config.Pxy_a
        pid.Pa[2] = config.Pz_a
        
        pid.Ia[0] = config.Ixy_a
        pid.Ia[1] = config.Ixy_a
        pid.Ia[2] = config.Iz_a
        
        pid.Da[0] = config.Dxy_a
        pid.Da[1] = config.Dxy_a
        pid.Da[2] = config.Dz_a
        
        pid.Mxy= config.Mxy
        pid.Mz = config.Mz
    

        self._pid_coeff = pid
        
        print self._pid_coeff.Pp

          
        return config
        
        
        
        

       




#  node enter point
def start():
    
    # create ros node handle
    nh = rospy.init_node('beziermapping')
    #nh = "fff"
    
    # create mapping obj
    mp = mapping(nh)
    
    rospy.spin()
    
    '''r = rospy.Rate(150)

    
    while not rospy.is_shutdown():
        if mp._run_bz_controller:

            mp._pub_thrust_sp_desired()
    
        r.sleep()'''
    
if __name__ == '__main__':
    start()
