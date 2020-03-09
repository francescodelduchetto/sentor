#!/usr/bin/env python
"""
Created on Fri Dec  6 08:51:15 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
from __future__ import division
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Event


class SafetyMonitor(object):
    
    
    def __init__(self, rate, auto_tagging, event_cb):
        
        self.auto_tagging = auto_tagging
        self.event_cb = event_cb
        self.topic_monitors = []
        
        self.safe_operation = True        
        self.safe_msg_sent = False
        self.unsafe_msg_sent = False
        
        self._stop_event = Event()

        self.safety_pub = rospy.Publisher('/safe_operation', Bool, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/rate), self.safety_pub_cb)
        
        rospy.Service('/sentor/set_safety_tag', SetBool, self.set_safety_tag)


    def register_monitors(self, topic_monitor):
        self.topic_monitors.append(topic_monitor)
        
        
    def safety_pub_cb(self, event=None):
        
        if not self._stop_event.isSet():
        
            if self.topic_monitors:
                threads_are_safe = [monitor.thread_is_safe for monitor in self.topic_monitors]
                
                if all(threads_are_safe) and self.auto_tagging:
                    self.safe_operation = True
                elif not all(threads_are_safe):
                    self.safe_operation = False
                    
                self.safety_pub.publish(Bool(self.safe_operation))
    
                if all(threads_are_safe) and not self.safe_msg_sent:
                    self.event_cb("SAFE OPERATION: TRUE", "info")
                    self.safe_msg_sent = True
                    self.unsafe_msg_sent = False
                    
                elif not all(threads_are_safe) and not self.unsafe_msg_sent:
                    self.event_cb("SAFE OPERATION: FALSE", "warn")
                    self.safe_msg_sent = False
                    self.unsafe_msg_sent = True
        
        
    def set_safety_tag(self, req):
        
        self.safe_operation = req.data        
        
        ans = SetBoolResponse()
        ans.success = True
        ans.message = "safe operation: {}".format(req.data)
        
        return ans
        
        
    def stop_monitor(self):
        self._stop_event.set()


    def start_monitor(self):
        self._stop_event.clear()
#####################################################################################