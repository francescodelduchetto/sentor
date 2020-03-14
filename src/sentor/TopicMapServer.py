#!/usr/bin/env python
"""
Created on Mon Mar  2 10:57:11 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
import rospy, numpy as np
import os, uuid, pickle, yaml
import matplotlib.pyplot as plt

from threading import Event
from sentor.msg import TopicMap, TopicMapArray
from sentor.srv import GetTopicMaps, GetTopicMapsResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse


class TopicMapServer(object):
    

    def __init__(self, topic_monitors, map_pub_rate, map_plt_rate):
        
        self.base_dir = os.path.join(os.path.expanduser("~"), ".sentor_maps")     
        if not os.path.exists(self.base_dir):
            os.mkdir(self.base_dir)    
        
        self.topic_monitors = topic_monitors

        self._stop_event = Event()

        rospy.Service("/sentor/write_maps", Trigger, self.write_maps)               
        rospy.Service("/sentor/get_maps", GetTopicMaps, self.get_maps)               
        rospy.Service("/sentor/clear_maps", Empty, self.clear_maps)   
        rospy.Service("/sentor/stop_mapping", Empty, self.stop_mapping)   
        rospy.Service("/sentor/start_mapping", Empty, self.start_mapping)  
        
        if map_pub_rate > 0:            
            if map_pub_rate > 1:
                map_pub_rate = 1                
            self.maps_pub = rospy.Publisher('/sentor/topic_maps', TopicMapArray, queue_size=10)
            rospy.Timer(rospy.Duration(1.0/map_pub_rate), self.publish_maps)
        
        if map_plt_rate > 0:
            if map_plt_rate > 1:
                map_plt_rate = 1 
            rospy.Timer(rospy.Duration(1.0/map_plt_rate), self.plot_maps)
            
            
    def write_maps(self, req):
    
        message = "Saving maps: "
        for monitor in self.topic_monitors:
            if monitor.map is not None:

                map_dir = os.path.join(self.base_dir, str(uuid.uuid4()))
                os.mkdir(map_dir)
                
                _map = monitor.topic_mapper.map
                pickle.dump(_map, open(map_dir + "/topic_map.pkl", "wb"))
            
                with open(map_dir + "/config.yaml",'w') as f:
                    yaml.dump(monitor.map, f, default_flow_style=False)                
                    
                message = message + map_dir + " "
            
        ans = TriggerResponse()
        ans.success = True
        ans.message = message
        return ans
        
        
    def get_maps(self, req):
        
        topic_maps = TopicMapArray()
        topic_maps = self.fill_msg(topic_maps)
        
        ans = GetTopicMapsResponse()
        ans.topic_maps = topic_maps
        ans.success = True
        return ans
        
        
    def clear_maps(self, req):
        
        for monitor in self.topic_monitors:
            if monitor.map is not None:
                monitor.topic_mapper.init_map()
            
        ans = EmptyResponse()
        return ans
        
        
    def stop_mapping(self, req):
        self.stop()
        
        ans = EmptyResponse()
        return ans
        

    def start_mapping(self, req):
        self.start()
        
        ans = EmptyResponse()
        return ans
        
        
    def publish_maps(self, event=None):
        
        if not self._stop_event.isSet():
            
            topic_maps = TopicMapArray()
            topic_maps = self.fill_msg(topic_maps)
            self.maps_pub.publish(topic_maps)
            
            
    def plot_maps(self, event=None):
        
        if not self._stop_event.isSet():
            
            _id = 0
            for monitor in self.topic_monitors:
                if monitor.map is not None:     
                    fig_id = "thread " + str(_id) + ": " + monitor.topic_name + " " + monitor.map["topic_arg"] + " " + monitor.map["stat"] 
                    
                    _map = monitor.topic_mapper.map
                    masked_map = np.ma.array(_map, mask=np.isnan(_map))
                    
                    plt.pause(0.1)
                    plt.figure(fig_id); plt.clf()
                    plt.imshow(masked_map.T, interpolation="spline16", origin="lower", 
                               extent=monitor.map["limits"])
                    plt.colorbar()
                    plt.gca().set_aspect("equal", adjustable="box")
                    plt.tight_layout()
        
                _id += 1
        
        
    def fill_msg(self, topic_maps):
        
        for monitor in self.topic_monitors:
            if monitor.map is not None:
                    
                map_msg = TopicMap()
                map_msg.header.stamp = rospy.Time.now()
                map_msg.header.frame_id = "/map"
                map_msg.topic_name = monitor.topic_name
                map_msg.topic_arg = monitor.map["topic_arg"]
                map_msg.stat = monitor.map["stat"]
                map_msg.resolution = monitor.map["resolution"]
                map_msg.shape = monitor.topic_mapper.shape
                map_msg.index = monitor.topic_mapper.index
                map_msg.position = monitor.topic_mapper.position
                map_msg.arg_at_position = monitor.topic_mapper.arg_at_position
    
                topic_map = np.ndarray.tolist(np.ravel(monitor.topic_mapper.map))
                map_msg.topic_map = topic_map
            
                topic_maps.topic_maps.append(map_msg)
                
        return topic_maps
        
                
    def stop(self):

        self._stop_event.set()    
        
        for monitor in self.topic_monitors:
            if monitor.map is not None:
                monitor.topic_mapper.stop_mapping()
                
        
    def start(self):

        self._stop_event.clear()      
        
        for monitor in self.topic_monitors:
            if monitor.map is not None:
                monitor.topic_mapper.start_mapping()
##########################################################################################