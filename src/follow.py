#!/usr/bin/env python
"""
Created on Mon Oct 17 10:34:05 2016

@author: dennis
"""
import common_functions as cf
import binary_search as bs
import numpy as np
from threading import Thread
import bezier_fn as bf




class follow_thread(Thread):
    def __init__(self, state, path):
        super(follow_thread, self).__init__()

        
        self._state = state
        self._path = path
        self._doFollow  = False
        
      
      
    def run(self):
        
        self._doFollow = True
        
        # get first bezier 
        bezier = self._path.bz_pt[0]
        

        idx_next = 1
        
        # set state into velocity ctr
        self._state.set_state("bezier")
        
        # init setpoints
        self._path.pub.init_sp()
        
        while self._doFollow:
            
        
    
            
            # get current position
            pc = cf.p_ros_to_numpy(self._state.driver.local_pose.pose.position)
                      
            # get t of bezier point closest to current posiition
            pd, vd, ad = bf.point_closest_to_bezier(bezier, pc)
            
            # publsih set point
            self._path.pub.pub_setpoints(cf.p_numpy_to_ros(pd))
            
            # check if pd is close to last point on bezier
            if np.linalg.norm(pd - bezier[2]) <= 0.05:

                bezier = self._path.bz_pt[idx_next]
                idx_next +=1 
                
                if idx_next == len(self._path.bz_pt):
                    self._doFollow = False
            
        
            # compute adjusted vel
            #v_adj = self._vel_mapping(pd, vd, pc)
        
        
            # send desired velocity
            self._state.set_msg(bezier)
        
        
        # go into position ctrl
        self._state.set_state("posctr")
        
        # stay at current position
        self._state.set_msg(np.append(pc,np.array([0.0,0.0,1.0,0.0])))
        
    
    
    
    # set following boolean
    def do_follow(self, arg):
        self._doFollow = arg



class thread_control():
    def __init__(self, state, path):
        
        self._state = state
        self._path = path
        self._doFollow  = False
        
        # create first thread object
        self._thread = follow_thread(self._state, self._path)
    
    
    def start_thread(self):
        
        # if alive, close current thread
        self.stop_thread()
        
        # create thread
        self._thread = follow_thread(self._state, self._path)
        
        # start thread 
        self._thread.start()
        
    def stop_thread(self):
        
        if self._thread.is_alive():
            self._thread.do_follow(False)
            self._thread.join()
    
        
        
        
        
        
        
