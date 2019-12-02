#!/usr/bin/env python
"""
Created on Thu Nov 21 10:30:22 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
import rospy, rosservice, rostopic, actionlib, subprocess
from threading import Lock


class Executor(object):
    
    
    def __init__(self, config, lock_exec, event_cb):

        self.config = config
        self.lock_exec = lock_exec
        self.event_cb = event_cb
        
        self._lock = Lock()
        self.actions = []
        
        for action in config:
            
            action_type = action.keys()[0]
            print "Initialising sentor action of type '{}'".format(action_type)
            
            if action_type == "call":
                self.init_call(action)
                
            elif action_type == "publish":
                self.init_publish(action)
                
            elif action_type == "action":
                self.init_action(action)
                
            elif action_type == "sleep":
                self.init_sleep(action)
                
            elif action_type == "shell":
                self.init_shell(action)
                
            else:
                rospy.logerr("Sentor action of type '{}' not supported".format(action_type))
                
        print "\n"
                    
                    
    def init_call(self, action):
        
        try:
            service_name = action["call"]["service_name"]
            service_class = rosservice.get_service_class_by_name(service_name)

            rospy.wait_for_service(service_name, timeout=5.0)
            service_client = rospy.ServiceProxy(service_name, service_class)
            
            req = service_class._request_class()
            for arg in action["call"]["service_args"]: exec(arg)

            d = {}
            d["message"] = "Calling service '{}'. ".format(service_name)
            d["user_msg"] = self.get_user_msg(action["call"])
            d["func"] = "self.call(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            
            self.actions.append(d)
            
        except Exception as e:
            rospy.logerr(e)
            
            
    def init_publish(self, action):
        
        try:
            topic_name = action["publish"]["topic_name"]
            topic_latched = action["publish"]["topic_latched"]
            
            msg_class, real_topic, _ = rostopic.get_topic_class(topic_name)
            pub = rospy.Publisher(real_topic, msg_class, latch=topic_latched, 
                                  queue_size=10)
            
            msg = msg_class()
            for arg in action["publish"]["topic_args"]: exec(arg)
                
            d = {}
            d["message"] = "Publishing to topic '{}'. ".format(topic_name)
            d["user_msg"] = self.get_user_msg(action["publish"])
            d["func"] = "self.publish(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["pub"] = pub
            d["kwargs"]["msg"] = msg
            
            self.actions.append(d)
            
        except Exception as e:
            rospy.logerr(e)
            
            
    def init_action(self, action):
        
        try:
            namespace = action["action"]["namespace"]
            package = action["action"]["package"]
            spec = action["action"]["action_spec"]
            
            exec("from {}.msg import {} as action_spec".format(package, spec))
            exec("from {}.msg import {} as goal_class".format(package, spec[:-6] + "Goal"))
            
            action_client = actionlib.SimpleActionClient(namespace, action_spec)
            wait = action_client.wait_for_server(rospy.Duration(5.0))
            if not wait:
                rospy.logerr("Action server with namespace '{}' and action spec '{}' not available.".format(namespace, spec))
                return
    
            goal = goal_class()
            for arg in action["action"]["goal_args"]: exec(arg)
                
            d = {}
            d["message"] = "Sending goal for action with spec '{}'. ".format(spec)
            d["user_msg"] = self.get_user_msg(action["action"])
            d["func"] = "self.action(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["action_client"] = action_client
            d["kwargs"]["goal"] = goal
            
            self.actions.append(d)
        
        except Exception as e:
            rospy.logerr(e)
            
        
    def init_sleep(self, action):
        
        try:
            d = {}
            d["message"] = "Sentor sleeping for {} seconds. ".format(action["sleep"]["duration"])
            d["user_msg"] = self.get_user_msg(action["sleep"])
            d["func"] = "self.sleep(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["duration"] = action["sleep"]["duration"]
            
            self.actions.append(d)

        except Exception as e:
            rospy.logerr(e)
            
            
    def init_shell(self, action):
        
        try:
            d = {}
            d["message"] = "Executing shell commands {}. ".format(action["shell"]["cmd_args"])
            d["user_msg"] = self.get_user_msg(action["shell"])
            d["func"] = "self.shell(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["cmd_args"] = action["shell"]["cmd_args"]
            
            self.actions.append(d)

        except Exception as e:
            rospy.logerr(e)
            
            
    def get_user_msg(self, action):
        
        if "user_msg" in action.keys():
            user_msg = action["user_msg"]
        else:
            user_msg = ""
        
        return user_msg
        
        
    def execute(self):
        
        if self.lock_exec:
            self._lock.acquire()
        
        for action in self.actions:
            rospy.sleep(0.1) # needed when using slackeros
            try:
                self.event_cb(action["message"] + action["user_msg"], "info")
                kwargs = action["kwargs"]            
                eval(action["func"])
                
            except Exception as e:
                self.event_cb(str(e), "error")
            
        if self.lock_exec:
            self._lock.release()
            

    def call(self, service_client, req):
        
        resp = service_client(req)
        
        if resp.success:
            self.event_cb("Service call success: {}".format(resp.success), "info")
        else:
            self.event_cb("Service call success: {}".format(resp.success), "error")
        
        
    def publish(self, pub, msg):
        pub.publish(msg)
        
        
    def action(self, action_client, goal):
        action_client.send_goal(goal, self.goal_cb)
        
       
    def sleep(self, duration):
        rospy.sleep(duration)
        
        
    def shell(self, cmd_args):
        
        process = subprocess.Popen(cmd_args,
                     stdout=subprocess.PIPE, 
                     stderr=subprocess.PIPE)
                     
        stdout, stderr = process.communicate()
        print stdout
        print stderr
        
        
    def goal_cb(self, status, result):
        
        if status == 3:
            self.event_cb("Goal achieved", "info")
        elif status == 2 or status == 6:
            self.event_cb("Goal preempted", "warn")
        else:
            self.event_cb("Goal failed", "error")
#####################################################################################