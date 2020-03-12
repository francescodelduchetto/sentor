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
        self.y_min, self.y_max = config["limits"][2:4]
        
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
        
        self.position = [np.nan, np.nan]
        self.index = [np.nan, np.nan]
        self.arg_at_position = np.nan
        
 
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
            rospy.logwarn("Exception while evaluating {}: {}".format(self.config["topic_arg"], e))
            return False
            
        valid_arg = True
        if type(self.topic_arg) is bool:    
            if self.topic_arg:
                self.topic_arg = 1
            else:
                self.topic_arg = 0
        elif type(self.topic_arg) is not float and type(self.topic_arg) is not int:
            rospy.logwarn("Topic arg {} of {} on topic '{}' cannot be processed".format(self.topic_arg, type(self.topic_arg), self.config["topic"]))
            valid_arg = False
        
        return valid_arg
        
        
    def update_map(self, x, y):

        ix = np.digitize(x, self.x_bins)
        iy = np.digitize(y, self.y_bins)     
        
        self.obs[ix, iy] += 1        
        N = self.obs[ix, iy]
        
        z0 = self.map[ix, iy]
        if np.isnan(z0): z0=0
            
        z = self.compute_stat(N, z0)

        self.map[ix, iy] = z        
        self.position = [x, y]
        self.index = [ix, iy]
        self.arg_at_position = z
        
        
    def compute_stat(self, N, z0):
        
        if self.config["stat"] == "mean":       
            z = (1/N) * ((z0 * (N-1)) + self.topic_arg)
            
        elif self.config["stat"] == "sum":
            z = z0 + self.topic_arg
            
        elif self.config["stat"] == "min":
            z = np.min([z0, self.topic_arg])
            
        elif self.config["stat"] == "max":
            z = np.max([z0, self.topic_arg])

        else:
            rospy.logwarn("Statistic of type '{}' not supported".format(self.config["stat"]))
            z = np.nan
            
        return z
        
                        
    def stop_mapping(self):
        self._stop_event.set()
        
        
    def start_mapping(self):
        self._stop_event.clear()
##########################################################################################