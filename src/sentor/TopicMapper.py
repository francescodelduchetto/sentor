#!/usr/bin/env python
"""
Created on Tue Feb 25 08:55:41 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
import rospy, numpy as np, tf, math
import rospkg, uuid, os, pickle, yaml
import matplotlib.pyplot as plt
from threading import Event


class TopicMapper(object):
    
    
    def __init__(self, config, topic, msg_class):
        
        self.config = config
        self.config["topic"] = topic

        self.nx = int(np.floor((config["limits"][1] - config["limits"][0]) / config["resolution"]))
        self.ny = int(np.floor((config["limits"][3] - config["limits"][2]) / config["resolution"]))
        
        self.x_bins = np.linspace(config["limits"][0], config["limits"][1], self.nx)
        self.y_bins = np.linspace(config["limits"][2], config["limits"][3], self.ny)
        
        self.obs = np.zeros((self.nx, self.ny))
        self.map = np.zeros((self.nx, self.ny))  
        self.map[:] = np.nan

        self.tf_listener = tf.TransformListener()
        
        package_dir = rospkg.RosPack().get_path("sentor")
        self.map_id = str(uuid.uuid4())
        self.save_dir = package_dir + "/topic_maps/" + self.map_id 
        
        self._stop_event = Event()
        
        rospy.Subscriber(topic, msg_class, self.topic_cb)
        
        gen_plts = False
        if "plt" in config.keys():
            gen_plts = config["plt"] 

        if gen_plts:
            plt_rate = 1.0
            if "plt_rate" in config.keys():
                plt_rate = config["plt_rate"]     
            
            self.fig_name = config["topic"] + " " + config["topic_arg"]
            rospy.Timer(rospy.Duration(1.0/plt_rate), self.plt_cb)


    def topic_cb(self, msg):
        
        if not self._stop_event.isSet():    
            
            try:
                x, y = self.get_transform()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to get transform between the map and baselink frames")
                return
            
            if x >= self.config["limits"][0] and x <= self.config["limits"][1] \
            and y >= self.config["limits"][2] and y <= self.config["limits"][3]:
                
                valid_arg = self.process_topic_arg(msg)
                
                if valid_arg:
                    self.update_map(x, y)

        
    def get_transform(self):
        
        now = rospy.Time(0)
        self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(2.0))
        (trans,rot) = self.tf_listener.lookupTransform("map", "base_link", now)
        
        return trans[0], trans[1]
        
        
    def process_topic_arg(self, msg):
        
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
            rospy.logwarn("Topic arg of {} on topic '{}' cannot be processed".format(type(self.topic_arg), self.config["topic"]))
            valid_arg = False
        
        return valid_arg
        
        
    def update_map(self, x, y):

        ix = np.digitize(x, self.x_bins)
        iy = np.digitize(y, self.y_bins)     
        
        self.obs[ix, iy] += 1        
        N = self.obs[ix, iy]
        
        m = self.map[ix, iy]
        if np.isnan(m): m=0
        
        wm = (1/N) * ((m * (N-1)) + self.topic_arg)
        
        self.map[ix, iy] = wm
        
        
    def write_map(self):
        
        os.mkdir(self.save_dir)
        
        pickle.dump(self.map, open(self.save_dir + "/topic_map.pkl", "wb"))
        rospy.loginfo("saving topic map '{}'".format(self.save_dir + "/topic_map.pkl"))
        
        with open(self.save_dir + "/config.yaml",'w') as f:
            yaml.dump(self.config, f, default_flow_style=False)
            
            
    def plt_cb(self, event=None):
        
        if not self._stop_event.isSet(): 
        
            masked_map = np.ma.array(self.map, mask=np.isnan(self.map))
            
            plt.pause(0.1)
            plt.figure(self.fig_name + " " + self.map_id); plt.clf()
            plt.imshow(masked_map.T, origin="lower", extent=self.config["limits"])
            plt.colorbar()
            plt.gca().set_aspect("equal", adjustable="box")
            plt.tight_layout()
        
            
    def stop_mapping(self):
        self._stop_event.set()
        
        
    def start_mapping(self):
        self._stop_event.clear()
##########################################################################################