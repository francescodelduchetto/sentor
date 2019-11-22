#!/usr/bin/env python
"""
Created on Thu Nov 21 10:30:22 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
import rosservice, rospy


class Executor(object):
    
    
    def __init__(self, config):
        
        self.config = config
        self.actions = []
        
        for action in config:
            
            if action.keys()[0] == "call":
                self.init_call(action)
                    
                    
    def init_call(self, action):
        
        try:
            service_name = action["call"]["service_name"]
            service_class = rosservice.get_service_class_by_name(service_name)

            rospy.wait_for_service(service_name, timeout=5.0)
            service_client = rospy.ServiceProxy(service_name, service_class)
            
            req = service_class._request_class()
            for arg in action["call"]["service_args"]: exec(arg)

            d = {}
            d["func"] = "self.call(**kwargs)"
            d["kwargs"] = {}               
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            
            self.actions.append(d)
            
        except rospy.ROSException as e:
            rospy.logerr(e)
                    
        
    def execute(self):
        
        for action in self.actions:
            kwargs = action["kwargs"]            
            eval(action["func"])
               

    def call(self, service_client, req):
       resp = service_client(req)
       rospy.loginfo(resp)
#####################################################################################