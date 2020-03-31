#!/usr/bin/env python
"""
Created on Tue Feb 25 08:55:41 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
import rospy, numpy as np, tf, math
from threading import Event


class TopicMapper(object):
    
    
    def __init__(self, config, topic, msg_class):
        
        self.config = config
        self.config["topic"] = topic
        
        self.x_min, self.x_max = config["limits"][:2]
        self.y_min, self.y_max = config["limits"][2:]
        
        self.x_bins = np.arange(self.x_min, self.x_max, config["resolution"])
        self.y_bins = np.arange(self.y_min, self.y_max, config["resolution"])
        
        self.nx = self.x_bins.shape[0] + 1
        self.ny = self.y_bins.shape[0] + 1
        self.shape = [self.nx, self.ny]
        
        self.init_map()

        self.tf_listener = tf.TransformListener()          
        
        self._stop_event = Event()
        
        rospy.Subscriber(topic, msg_class, self.topic_cb)
        
        
    def init_map(self):
        
        self.obs = np.zeros((self.nx, self.ny))
        self.map = np.zeros((self.nx, self.ny))  
        self.map[:] = np.nan

        self.index = [np.nan, np.nan]        
        self.position = [np.nan, np.nan]
        self.arg_at_position = np.nan
        
        if self.config["stat"] == "std":
            self.wma = np.zeros((self.nx, self.ny))
            self.wma[:] = np.nan
            
 
    def topic_cb(self, msg):
        
        if not self._stop_event.isSet():    
            
            try:
                x, y = self.get_transform()
            except: 
                rospy.logwarn("Failed to get transform between the map and baselink frames")
                return
                
            if self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max:  
                valid_arg = self.process_arg(msg)
                
                if valid_arg:
                    self.update_map(x, y)

        
    def get_transform(self):
        
        now = rospy.Time(0)
        self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(2.0))
        (trans,rot) = self.tf_listener.lookupTransform("map", "base_link", now)
        
        return trans[0], trans[1]
        
        
    def process_arg(self, msg):
        
        try:
            self.topic_arg = eval(self.config["topic_arg"])
        except Exception as e:
            rospy.logwarn("Exception while evaluating '{}': {}".format(self.config["topic_arg"], e))
            return False
            
        valid_arg = True
        arg_type = type(self.topic_arg)
        if arg_type is bool:
            self.topic_arg = int(self.topic_arg) 
        elif arg_type is not float and arg_type is not int:
            rospy.logwarn("Topic arg '{}' of {} on topic '{}' cannot be processed".format(self.topic_arg, arg_type, self.config["topic"]))
            valid_arg = False
        
        return valid_arg
        
        
    def update_map(self, x, y):

        ix = np.digitize(x, self.x_bins)
        iy = np.digitize(y, self.y_bins)     
        self.ix = ix
        self.iy = iy
        
        self.obs[ix, iy] += 1        
        N = self.obs[ix, iy]
        
        z = self.map[ix, iy]
        if np.isnan(z): z=0
        z = self.compute_stat(z, N)
            
        self.map[ix, iy] = z        
        self.index = [ix, iy]
        self.position = [x, y]
        self.arg_at_position = z
        
        
    def compute_stat(self, z, N):
        
        weighted_mean = lambda m, x: (1/N) * ((m * (N-1)) + x)
        
        
        if self.config["stat"] == "mean":
            z = weighted_mean(z, self.topic_arg)
            
        elif self.config["stat"] == "sum":
            z += self.topic_arg
            
        elif self.config["stat"] == "min":
            z = np.min([z, self.topic_arg])
            
        elif self.config["stat"] == "max":
            z = np.max([z, self.topic_arg])
            
        elif self.config["stat"] == "std":
            wm = self.wma[self.ix, self.iy]
            if np.isnan(wm): wm=0

            wm = weighted_mean(wm, self.topic_arg)
            self.wma[self.ix, self.iy] = wm
            
            z = np.sqrt(weighted_mean(z**2, (wm-self.topic_arg)**2))

        else:
            rospy.logwarn("Statistic of type '{}' not supported".format(self.config["stat"]))
            z = np.nan; 
            
        return z
        
                        
    def stop_mapping(self):
        self._stop_event.set()
        
        
    def start_mapping(self):
        self._stop_event.clear()
##########################################################################################