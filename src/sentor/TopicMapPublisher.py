#!/usr/bin/env python
"""
Created on Mon Mar  2 10:57:11 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
import rospy
from sentor.msg import TopicMap, TopicMapArray
from numpy import ndarray, ravel
from threading import Event


class TopicMapPublisher(object):
    

    def __init__(self, topic_monitors, map_pub_rate):
        
        self.topic_monitors = topic_monitors
        self._stop_event = Event()
        
        self.maps_pub = rospy.Publisher('/sentor/topic_maps', TopicMapArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/map_pub_rate), self.map_pub_callback)
        
        
    def map_pub_callback(self, event=None):
        
        if not self._stop_event.isSet():
    
            map_msgs = TopicMapArray()
            for monitor in self.topic_monitors:
                if monitor.map is not None:
                    
                    map_msg = TopicMap()
                    map_msg.header.stamp = rospy.Time.now()
                    map_msg.header.frame_id = "/map"
                    map_msg.topic_name = monitor.topic_name
                    map_msg.topic_arg = monitor.map["topic_arg"]
                    map_msg.resolution = monitor.map["resolution"]
                    map_msg.size_x = monitor.topic_mapper.nx
                    map_msg.size_y = monitor.topic_mapper.ny
        
                    topic_map = ndarray.tolist(ravel(monitor.topic_mapper.map))
                    map_msg.data = topic_map
                
                    map_msgs.topic_maps.append(map_msg)
            self.maps_pub.publish(map_msgs)
        
        
    def stop_publishing(self):
        self._stop_event.set()


    def start_publishing(self):
        self._stop_event.clear()
##########################################################################################