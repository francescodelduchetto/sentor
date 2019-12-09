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
        self.processes = []
        
        for process in config:
            
            process_type = process.keys()[0]
            print "Initialising process of type {}".format("\033[1m"+process_type+"\033[0m")
            
            if process_type == "call":
                self.init_call(process)
                
            elif process_type == "publish":
                self.init_publish(process)
                
            elif process_type == "action":
                self.init_action(process)
                
            elif process_type == "sleep":
                self.init_sleep(process)
                
            elif process_type == "shell":
                self.init_shell(process)
                
            elif process_type == "log":
                self.init_log(process)
                
            else:
                rospy.logerr("Process of type '{}' not supported".format(process_type))
                
        print "\n"
                    
                    
    def init_call(self, process):
        
        try:
            service_name = process["call"]["service_name"]
            service_class = rosservice.get_service_class_by_name(service_name)

            rospy.wait_for_service(service_name, timeout=5.0)
            service_client = rospy.ServiceProxy(service_name, service_class)
            
            req = service_class._request_class()
            for arg in process["call"]["service_args"]: exec(arg)

            d = {}
            d["log"] = ("Calling service '{}'".format(service_name), "info", req)
            d["func"] = "self.call(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["service_name"] = service_name
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            
            self.processes.append(d)
            
        except Exception as e:
            rospy.logerr(e)
            
            
    def init_publish(self, process):
        
        try:
            topic_name = process["publish"]["topic_name"]
            topic_latched = process["publish"]["topic_latched"]
            
            msg_class, real_topic, _ = rostopic.get_topic_class(topic_name)
            pub = rospy.Publisher(real_topic, msg_class, latch=topic_latched, 
                                  queue_size=10)
            
            msg = msg_class()
            for arg in process["publish"]["topic_args"]: exec(arg)
                
            d = {}
            d["log"] = ("Publishing to topic '{}'".format(topic_name), "info", msg)
            d["func"] = "self.publish(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["pub"] = pub
            d["kwargs"]["msg"] = msg
            
            self.processes.append(d)
            
        except Exception as e:
            rospy.logerr(e)
            
            
    def init_action(self, process):
        
        try:
            namespace = process["action"]["namespace"]
            package = process["action"]["package"]
            spec = process["action"]["action_spec"]
            
            exec("from {}.msg import {} as action_spec".format(package, spec))
            exec("from {}.msg import {} as goal_class".format(package, spec[:-6] + "Goal"))
            
            action_client = actionlib.SimpleActionClient(namespace, action_spec)
            wait = action_client.wait_for_server(rospy.Duration(5.0))
            if not wait:
                rospy.logerr("Action server with namespace '{}' and action spec '{}' not available.".format(namespace, spec))
                return
    
            goal = goal_class()
            for arg in process["action"]["goal_args"]: exec(arg)
                
            d = {}
            d["log"] = ("Sending goal for action with spec '{}'".format(spec), "info", goal)
            d["func"] = "self.action(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["spec"] = spec
            d["kwargs"]["action_client"] = action_client
            d["kwargs"]["goal"] = goal
            
            self.processes.append(d)
        
        except Exception as e:
            rospy.logerr(e)
            
        
    def init_sleep(self, process):
        
        try:
            d = {}
            d["log"] = ("Sentor sleeping for {} seconds".format(process["sleep"]["duration"]), "info", "")
            d["func"] = "self.sleep(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["duration"] = process["sleep"]["duration"]
            
            self.processes.append(d)

        except Exception as e:
            rospy.logerr(e)
            
            
    def init_shell(self, process):
        
        try:
            d = {}
            d["log"] = ("Executing shell commands {}".format(process["shell"]["cmd_args"]), "info", "")
            d["func"] = "self.shell(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["cmd_args"] = process["shell"]["cmd_args"]
            
            self.processes.append(d)

        except Exception as e:
            rospy.logerr(e)


    def init_log(self, process):
        
        try:
            d = {}
            d["log"] = (process["log"]["message"], process["log"]["level"], "")
            d["func"] = "self.log()"
            d["kwargs"] = {}
            
            self.processes.append(d)

        except Exception as e:
            rospy.logerr(e)
            
        
    def execute(self):
        
        if self.lock_exec:
            self._lock.acquire()
        
        for process in self.processes:
            rospy.sleep(0.1) # needed when using slackeros
            try:
                self.event_cb(process["log"][0], process["log"][1], process["log"][2])
                kwargs = process["kwargs"]            
                eval(process["func"])
                
            except Exception as e:
                rospy.logerr(e)
            
        if self.lock_exec:
            self._lock.release()
            

    def call(self, service_name, service_client, req):
        
        resp = service_client(req)
        
        if resp.success:
            self.event_cb("Call to service '{}' succeeded".format(service_name), "info", req)
        else:
            self.event_cb("Call to service '{}' failed".format(service_name), "warn", req)
        
        
    def publish(self, pub, msg):
        pub.publish(msg)
        
        
    def action(self, spec, action_client, goal):
        self.spec = spec
        self.goal = goal
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
        
    
    def log(self):
        pass
        
        
    def goal_cb(self, status, result):
        
        if status == 3:
            self.event_cb("Goal achieved for action with spec '{}'".format(self.spec), "info", self.goal)
        elif status == 2 or status == 6:
            self.event_cb("Goal preempted for action with spec '{}'".format(self.spec), "warn", self.goal)
        else:
            self.event_cb("Goal failed for action with spec '{}'".format(self.spec), "warn", self.goal)
#####################################################################################