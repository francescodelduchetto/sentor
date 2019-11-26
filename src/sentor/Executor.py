#!/usr/bin/env python
"""
Created on Thu Nov 21 10:30:22 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
import rosservice, rostopic, rospy, actionlib
from threading import Lock


class Executor(object):
    
    
    def __init__(self, config, exec_once, lock_exec, bcolors):

        self.exec_once = exec_once
        self.executed = False
        self.lock_exec = lock_exec
        self._lock = Lock()
        self.bcolors = bcolors
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
                rospy.logerr("action type '{}' not supported".format(action.keys()[0]))
                    
                    
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
            d["kwargs"]["action_type"] = action.keys()[0]
            d["kwargs"]["service_name"] = service_name               
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            
            self.actions.append(d)
            
        except rospy.ROSException as e:
            rospy.logerr(e)
            
            
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
            d["func"] = "self.publish(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["action_type"] = action.keys()[0]
            d["kwargs"]["topic_name"] = topic_name
            d["kwargs"]["pub"] = pub
            d["kwargs"]["msg"] = msg
            
            self.actions.append(d)
            
        except rospy.ROSException as e:
            rospy.logerr(e)
            
            
    def init_action(self, action):
        
        namespace = action["action"]["namespace"]
        package = action["action"]["package"]
        action_prefix = action["action"]["action_prefix"]
        
        exec("from {}.msg import {} as action_spec".format(package, action_prefix + "Action"))
        exec("from {}.msg import {} as goal_class".format(package, action_prefix + "Goal"))
        
        action_client = actionlib.SimpleActionClient(namespace, action_spec)
        rospy.loginfo("Waiting for action server for action of type {} ...".format(action_prefix + "Action"))
        wait = action_client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available.")
            return
        rospy.loginfo("Connected to action server.")

        goal = goal_class()
        for arg in action["action"]["goal_args"]: exec(arg)
            
        d = {}
        d["func"] = "self.action(**kwargs)"
        d["kwargs"] = {}
        d["kwargs"]["action_type"] = action.keys()[0]
        d["kwargs"]["action_prefix"] = action["action"]["action_prefix"]
        d["kwargs"]["action_client"] = action_client
        d["kwargs"]["goal"] = goal
        
        self.actions.append(d)
            
        
    def init_sleep(self, action):
        
        d = {}
        d["func"] = "self.sleep(**kwargs)"
        d["kwargs"] = {}
        d["kwargs"]["action_type"] = action.keys()[0]
        d["kwargs"]["duration"] = action["sleep"]["duration"]

        self.actions.append(d)        
        
                    
    def execute(self):
        
        if not self.executed:        
        
            if self.lock_exec:
                self._lock.acquire()
            
            for action in self.actions:
                kwargs = action["kwargs"]            
                eval(action["func"])
                
            if self.lock_exec:
                self._lock.release()
                
            if self.exec_once:
                self.executed = True
               

    def call(self, action_type, service_name, service_client, req):
        
        self.log_info("calling service '{}'".format(service_name), action_type)
        resp = service_client(req)
        rospy.loginfo(resp)
        
        
    def publish(self, action_type, topic_name, pub, msg):
        
        self.log_info("publishing to topic '{}'".format(topic_name), action_type)
        pub.publish(msg)
        
        
    def action(self, action_type, action_prefix, action_client, goal):
        
        self.log_info("executing action of type '{}'".format(action_prefix + "Action"), action_type)
        action_client.send_goal(goal)
        
       
    def sleep(self, action_type, duration):
        
        self.log_info("sentor sleeping for {} seconds".format(duration), action_type)
        rospy.sleep(duration)
        
        
    def log_info(self, str_, action_type):
        
        msg = "Executing {}{}{}: ".format(self.bcolors.OKGREEN, action_type, self.bcolors.ENDC)
        msg = msg + str_
        rospy.loginfo(msg)
#####################################################################################