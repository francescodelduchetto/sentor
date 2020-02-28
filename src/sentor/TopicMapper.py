#!/usr/bin/env python
"""
Created on Tue Feb 25 08:55:41 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
import rospy, numpy as np, tf


class TopicMapper(object):
    
    
    def __init__(self, config, topic, msg_class):
        
        self.config = config
        
        self.x_bins = np.arange(config["limits"][0], config["limits"][1], config["resolution"]) 
        self.y_bins = np.arange(config["limits"][2], config["limits"][3], config["resolution"])
        
        self.nx = self.x_bins.shape[0] + 1
        self.ny = self.y_bins.shape[0] + 1
        
        self.obs = np.zeros((self.nx, self.ny))
        self.map = np.zeros((self.nx, self.ny))  
        self.map[:] = np.nan
        
        self.tf_listener = tf.TransformListener()
        
        rospy.Subscriber(topic, msg_class, self.topic_cb)


    def topic_cb(self, msg):
        
        try:
            x, y = self.get_transform()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get transform between map and baselink")
            return
        
        if x >= self.config["limits"][0] and x <= self.config["limits"][1] \
        and y >= self.config["limits"][2] and y <= self.config["limits"][3]:
            
            self.topic_arg = eval(self.config["topic_arg"])
            valid_arg = self.is_valid()
            
            if valid_arg:
                self.index_x = np.digitize(x, self.x_bins)
                self.index_y = np.digitize(y, self.y_bins)     
                self.update_map()
            else:
                rospy.logwarn("Topic arg of {} cannot be processed".format(type(self.topic_arg)))

        
    def get_transform(self):
        
        now = rospy.Time(0)
        self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(2.0))
        (trans,rot) = self.tf_listener.lookupTransform("map", "base_link", now)
        
        return trans[0], trans[1]
        
        
    def is_valid(self):
        
        valid_arg = True
        if type(self.topic_arg) is float:
            pass
        elif type(self.topic_arg) is int:
            pass
        elif type(self.topic_arg) is bool:    
            if self.topic_arg:
                self.topic_arg = 1
            else:
                self.topic_arg = 0           
        else:
            valid_arg = False
        
        return valid_arg
        
        
    def update_map(self):
        
        self.obs[self.index_x, self.index_y] += 1        
        N = self.obs[self.index_x, self.index_y]
        
        m = self.map[self.index_x, self.index_y]
        if np.isnan(m): m=0
        
        wm = (1/N) * ((m * (N-1)) + self.topic_arg)
        
        self.map[self.index_x, self.index_y] = wm
##########################################################################################