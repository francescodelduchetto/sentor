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


class SafetyMonitor(object):
    
    
    def __init__(self, rate):
        
        self.safe_operation = True

        self.safety_pub = rospy.Publisher('/safe_operation', Bool, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/rate), self.safety_pub_cb)
        
        rospy.Service('/sentor/reset_safety_tag', SetBool, self.reset)
        
        
    def reset(self, req):
        
        self.safe_operation = req.data        
        
        ans = SetBoolResponse()
        ans.success = True
        ans.message = "safe operation: {}".format(req.data)
        
        return ans


    def safety_pub_cb(self, event=None):
        self.safety_pub.publish(Bool(self.safe_operation))
        
        
    def safety_callback(self, thread_is_safe):
        
        if not thread_is_safe:
            self.safe_operation = False
#####################################################################################