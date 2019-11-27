#!/usr/bin/env python
"""
Created on Thu Nov 21 10:30:22 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
import rosservice, rostopic, rospy, actionlib, sys
from threading import Lock


class Executor(object):
    
    
    def __init__(self, config, lock_exec, event_cb):

        self.config = config
        self.lock_exec = lock_exec
        self.event_cb = event_cb
        
        self._lock = Lock()
        self.actions = []
        
        for action in config:
            
            if action.keys()[0] == "call":
                self.init_call(action)
                
            elif action.keys()[0] == "publish":
                self.init_publish(action)
                
            elif action.keys()[0] == "action":
                self.init_action(action)
                
            elif action.keys()[0] == "sleep":
                self.init_sleep(action)
                
            else:
                self.event_cb("action type '{}' not supported".format(action.keys()[0]), "error")
                    
                    
    def init_call(self, action):
        
        try:
            service_name = action["call"]["service_name"]
            service_class = rosservice.get_service_class_by_name(service_name)

            rospy.wait_for_service(service_name, timeout=5.0)
            service_client = rospy.ServiceProxy(service_name, service_class)
            
            req = service_class._request_class()
            for arg in action["call"]["service_args"]: exec(arg)

            d = {}
            d["action"] = action.keys()[0]
            d["string"] = "calling service '{}'.".format(service_name)
            d["func"] = "self.call(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            
            if "user_msg" in action["call"].keys():
                d["user_msg"] = action["call"]["user_msg"]
            else:
                d["user_msg"] = ""
            
            self.actions.append(d)
            
        except Exception as e:
            self.event_cb(str(e), "error")
            
            
    def init_publish(self, action):
        
        try:
            topic_name = action["publish"]["topic_name"]
            topic_latched = action["publish"]["topic_latched"]
            
            msg_class, real_topic, _ = rostopic.get_topic_class(topic_name)
            pub = rospy.Publisher(real_topic, msg_class, latch=topic_latched, 
                                  queue_size=10)
            
            msg = msg_class()
            for arg in action["publish"]["topic_data"]: exec(arg)
                
            d = {}
            d["action"] = action.keys()[0]
            d["string"] = "publishing to topic '{}'.".format(topic_name)
            d["func"] = "self.publish(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["pub"] = pub
            d["kwargs"]["msg"] = msg
            
            if "user_msg" in action["publish"].keys():
                d["user_msg"] = action["publish"]["user_msg"]
            else:
                d["user_msg"] = ""
            
            self.actions.append(d)
            
        except Exception as e:
            self.event_cb(str(e), "error")
            
            
    def init_action(self, action):
        
        try:
            namespace = action["action"]["namespace"]
            package = action["action"]["package"]
            action_prefix = action["action"]["action_prefix"]
            
            exec("from {}.msg import {} as action_spec".format(package, action_prefix + "Action"))
            exec("from {}.msg import {} as goal_class".format(package, action_prefix + "Goal"))
            
            action_client = actionlib.SimpleActionClient(namespace, action_spec)
            wait = action_client.wait_for_server(rospy.Duration(5.0))
            if not wait:
                rospy.logerr("Action server with namespace '{}' and action spec '{}' not available.".format(namespace, action_prefix + "Action"))
                return
    
            goal = goal_class()
            for arg in action["action"]["goal_args"]: exec(arg)
                
            d = {}
            d["action"] = action.keys()[0]
            d["string"] = "executing action of type '{}'.".format(action_prefix + "Action")
            d["func"] = "self.action(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["action_client"] = action_client
            d["kwargs"]["goal"] = goal
            
            if "user_msg" in action["action"].keys():
                d["user_msg"] = action["action"]["user_msg"]
            else:
                d["user_msg"] = ""
            
            self.actions.append(d)
        
        except Exception as e:
            self.event_cb(str(e), "error")
            
        
    def init_sleep(self, action):
        
        try:
            d = {}
            d["action"] = action.keys()[0]
            d["string"] = "sentor sleeping for {} seconds.".format(action["sleep"]["duration"])
            d["func"] = "self.sleep(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["duration"] = action["sleep"]["duration"]
            
            if "user_msg" in action["sleep"].keys():
                d["user_msg"] = action["sleep"]["user_msg"]
            else:
                d["user_msg"] = ""
    
            self.actions.append(d)

        except Exception as e:
            self.event_cb(str(e), "error")
        
                    
    def execute(self):
        
        if self.lock_exec:
            self._lock.acquire()
        
        for action in self.actions:
            try:
                self.event_cb("Executing '{}': ".format(action["action"]) + action["string"], "info", action["user_msg"])
                kwargs = action["kwargs"]            
                eval(action["func"])
                
            except Exception as e:
                self.event_cb(str(e), "error")
            
        if self.lock_exec:
            self._lock.release()
            

    def call(self, service_client, req):
        resp = service_client(req)
        self.event_cb(str(resp), "info")
        
        
    def publish(self, pub, msg):
        pub.publish(msg)
        
        
    def action(self, action_client, goal):
        action_client.send_goal(goal)
        
       
    def sleep(self, duration):
        rospy.sleep(duration)
#####################################################################################
