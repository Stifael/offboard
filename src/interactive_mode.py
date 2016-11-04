#!/usr/bin/env python
"""
Created on Thu Sep 29 09:22:58 2016

@author: dennis
"""




import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

import threading
import sys
import time 
import signal
import mavros_driver
import setpoint
import numpy as np
#import circle
import state
import pub_target
import paths
import follow

from tf.transformations import * #quaternion_from_euler, quaternion_multiply, quaternion_matrix
#from __future__ import print_function



# desire relative pose
pose_rel = [0.0,0.0,0.0,0.0]




# function to reset variable
def reset(ar):
    for i,x in enumerate(ar):
        ar[i] = 0.0
        

# checks if arg is an int   
def if_isNum(arg):
    try:
        float(arg)
        return True
    except ValueError:
        return False


# move thread: needs to sent position > 2 Hz
def move_pub(rate, st, run_event):
    
    # frequency of publishing
    rate = rospy.Rate(rate)
    
    # publish desire pose
    while run_event.is_set() and not rospy.is_shutdown():
                
        st.pub.publish(st.msg)
        
        rate.sleep()
        
        
        
# expected user input
def usage():
    print "usage: [p [x] [y] [z] [y]| c [r] [ax] [bx] [cx] | mode [MODE] | arm [BOOL] | exit]"
    
        
  
  
  
# main thread
def run_tests():

    ### initlaization


    # ros node initalization
    nh = rospy.init_node('interatction', anonymous=True)
    
    
        
    # create driver for receiving mavros msg
    drv = mavros_driver.mavros_driver(nh)
    
    
    # publisher for sp
    target = pub_target.pub_target()
    
    
    # lock for publisher thread
    lock = threading.Lock()
    
    # state: posctr, velctr
    st = state.state(lock, drv)

    
    # paths
    path = paths.paths(st, target)

    
    # modes
    sp = setpoint.setpoint(st, target) #set points
    #cl = circle.circle(st, target) #circle mode
    
    # follow threads
    follow_thr = follow.thread_control(st, path)
    
    # pose publisher rate
    rate = 20

    # signal flag for running threads
    run_event = threading.Event()
    run_event.set()
    



    # thread that sends position
    #move_t = threading.Thread(target=move_pub, args=(rate, pub_pose, pose_msg, pub_twist, twist_msg, state, run_event))
    move_t = threading.Thread(target=move_pub, args=(rate, st, run_event))
    move_t.start()
    
    
    #  ctrl-c handler: aslo used to shut down during flight
    def ctrlC_handler(a,b): 
        print 'teleop program:'
        print 'disarm'
        drv.arm(False)
        print '> start closing all threads'
        run_event.clear()
        move_t.join()
        follow_thr.stop_thread()
        print "> threads succesfully closed. Leaving program"
        sys.exit()
    
    
    # catch ctrl-c
    signal.signal(signal.SIGINT, ctrlC_handler)
    print "set offboard"
    
    # go into offboard mode
    drv.set_mode("OFFBOARD")
    
    # arm
    drv.arm(True)
    
    # wait some seconds until reaching hover position
    time.sleep(6.0)
    
    interactive_mode = True
    # main loop in manual mode
    if interactive_mode:
        
        # show usage
        usage()
        
        
        while 1:

            # read input from console
            user_input = sys.stdin.readline()
            
            
            # split input
            args = user_input.split()
            
            # cases
            if len(args) > 5:
                print "too many argumets"
                usage()
            
            elif len(args) == 0:
                print "no argument given"
                usage()
                
            else:
                
                # leave program
                if str(args[0]) == "exit":
                    print "leaving program"
                    ctrlC_handler(0,0)
               
                # set position    
                elif str(args[0]) == "p":
                    # reset relative position
                    reset(pose_rel)
                    
                    # close circle thread if running
                    follow_thr.stop_thread()
                
                    
                    # set new relative position 
                    
                    if len(args[1:]) > 4:
                        print "too many arguments"
                        usage()
                        
                    elif len(args[1:]) < 4:
                        print "not enough arguments"
                        usage()
                    
                    else:
                        for ind, arg in enumerate(args[1:]):
                        
                            if if_isNum(arg):   
                                pose_rel[ind] = float(arg)
                        
                            else:
                                print arg + " is not a number"
                                reset(pose_rel)
                                
                        st.set_state("posctr")      
                        sp.do_step(pose_rel)
                        
                        
                # bezier point
                elif str(args[0]) == "b":
                    
                    # reset relative position
                    reset(pose_rel)
                    
                    # close circle thread if running
                    follow_thr.stop_thread()
                
                    # set new relative position 
                    
                    if len(args[1:]) > 4:
                        print "too many arguments"
                        usage()
                        
                    elif len(args[1:]) < 4:
                        print "not enough arguments"
                        usage()
                    
                    else:
                        for ind, arg in enumerate(args[1:]):
                        
                            if if_isNum(arg):   
                                pose_rel[ind] = float(arg)
                        
                            else:
                                print arg + " is not a number"
                                reset(pose_rel)
                                
                        st.set_state("bezier")        
                        sp.do_step_bez(pose_rel)
                    
                    
                # set mode
                elif str(args[0]) == "mode":
                    
                    mode = args[1].upper()
                
                    if str(mode) == "OFFBOARD":
                        # go into offboard mode
                        drv.set_mode("OFFBOARD")
                    else:
                        print "This mode is not yet supported"
                
                # arm
                elif str(args[0]) == "arm":
                    drv.arm(True)
                    
                    
                # path mode
                elif str(args[0]) == "c":
                    
                    # close circle thread if running
                    follow_thr.stop_thread()
          
                        
                    correct_input = True
                    if len(args[1:]) > 4:
                        print "too many arguments"
                        usage()
                    
                    elif len(args[1:]) <4:
                        print "too few arguments"
                        usage()
                    else:
                        axis = []
                        for ind, arg in enumerate(args[2:]):
                            if if_isNum(arg):
                                axis.append(float(arg))
                            else:
                                print arg + "not a number"
                                correct_input = False
                        radius = 0.0        
                        if if_isNum(args[1]):
                            radius = float(args[1])
                        
                        if correct_input:    
                            path.circle(radius, axis, [1.0,0.0,0.0])
                    
                            # start thread
                            follow_thr.start_thread()
       
                else:
                    print "this input is not supported"
                    usage()
                        
                        
            
            # dont waste cpu           
            time.sleep(1)
    
   
        
    # start landing 
    print "start landing"
    drv.land()
    
   
    # join thread
    run_event.clear()
    move_t.join()

        
  

    
if __name__ == '__main__':
    try:
        
        run_tests()
        sys.exit()
        
    except rospy.ROSInterruptException:
        pass
