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
    
    
    def __init__(self, timeout, rate, auto_tagging, event_cb):
        
        if timeout > 0:
            self.timeout = timeout
        else:
            self.timeout = 0.1
        
        self.auto_tagging = auto_tagging
        self.event_cb = event_cb
        self.topic_monitors = []
        
        self.timer = None
        self.safe_operation = False        
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
                
                if self.auto_tagging and all(threads_are_safe) and self.timer is None:
                    self.timer = rospy.Timer(rospy.Duration.from_sec(self.timeout), self.timer_cb, oneshot=True)
                    
                if not all(threads_are_safe):
                    if self.timer is not None:
                        self.timer.shutdown()
                        self.timer = None

                    self.safe_operation = False                        
                    if not self.unsafe_msg_sent:
                        self.event_cb("SAFE OPERATION: FALSE", "warn")
                        self.safe_msg_sent = False
                        self.unsafe_msg_sent = True
                        
                self.safety_pub.publish(Bool(self.safe_operation))
                
                
    def timer_cb(self, event=None):
        
        self.safe_operation = True
        if not self.safe_msg_sent:
            self.event_cb("SAFE OPERATION: TRUE", "info")
            self.safe_msg_sent = True
            self.unsafe_msg_sent = False
                                       
        
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