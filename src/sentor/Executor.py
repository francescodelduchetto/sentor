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
        
        self.init_err_str = "Unable to initialise process of type '{}': {}"
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
                self.event_cb("Process of type '{}' not supported".format(process_type), "warn")
                
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
            d["name"] = "call"
            d["verbose"] = self.is_verbose(process["call"])
            d["def_msg"] = ("Calling service '{}'".format(service_name), "info", req)
            d["func"] = "self.call(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["service_name"] = service_name
            d["kwargs"]["service_client"] = service_client
            d["kwargs"]["req"] = req
            d["kwargs"]["verbose"] = self.is_verbose(process["call"])
            
            self.processes.append(d)
            
        except Exception as e:
            self.event_cb(self.init_err_str.format("call", str(e)), "warn")
            
            
    def init_publish(self, process):
        
        try:
            topic_name = process["publish"]["topic_name"]
            
            if "topic_latched" in process["publish"].keys():
                topic_latched = process["publish"]["topic_latched"]
            else:
                topic_latched = False
            
            msg_class, real_topic, _ = rostopic.get_topic_class(topic_name)
            pub = rospy.Publisher(real_topic, msg_class, latch=topic_latched, 
                                  queue_size=10)
            
            msg = msg_class()
            for arg in process["publish"]["topic_args"]: exec(arg)
                
            d = {}
            d["name"] = "publish"
            d["verbose"] = self.is_verbose(process["publish"])
            d["def_msg"] = ("Publishing to topic '{}'".format(topic_name), "info", msg)
            d["func"] = "self.publish(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["pub"] = pub
            d["kwargs"]["msg"] = msg
            
            self.processes.append(d)
            
        except Exception as e:
            self.event_cb(self.init_err_str.format("publish", str(e)), "warn")
            
            
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
                e = "Action server with namespace '{}' and action spec '{}' not available".format(namespace, spec)
                self.event_cb(self.init_err_str.format("action", e), "warn")
                return
    
            goal = goal_class()
            for arg in process["action"]["goal_args"]: exec(arg)
                
            d = {}
            d["name"] = "action"
            d["verbose"] = self.is_verbose(process["action"])
            d["def_msg"] = ("Sending goal for action with spec '{}'".format(spec), "info", goal)
            d["func"] = "self.action(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["spec"] = spec
            d["kwargs"]["action_client"] = action_client
            d["kwargs"]["goal"] = goal
            d["kwargs"]["verbose"] = self.is_verbose(process["action"])
            
            self.processes.append(d)
        
        except Exception as e:
            self.event_cb(self.init_err_str.format("action", str(e)), "warn")
            
        
    def init_sleep(self, process):
        
        try:
            d = {}
            d["name"] = "sleep"
            d["verbose"] = self.is_verbose(process["sleep"])
            d["def_msg"] = ("Sentor sleeping for {} seconds".format(process["sleep"]["duration"]), "info", "")
            d["func"] = "self.sleep(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["duration"] = process["sleep"]["duration"]
            
            self.processes.append(d)

        except Exception as e:
            self.event_cb(self.init_err_str.format("sleep", str(e)), "warn")            
            
            
    def init_shell(self, process):
        
        try:
            d = {}
            d["name"] = "shell"
            d["verbose"] = self.is_verbose(process["shell"])
            d["def_msg"] = ("Executing shell commands {}".format(process["shell"]["cmd_args"]), "info", "")
            d["func"] = "self.shell(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["cmd_args"] = process["shell"]["cmd_args"]
            
            self.processes.append(d)

        except Exception as e:
            self.event_cb(self.init_err_str.format("shell", str(e)), "warn")


    def init_log(self, process):
        
        try:            
            d = {}
            d["name"] = "log"
            d["verbose"] = False
            d["func"] = "self.log(**kwargs)"
            d["kwargs"] = {}
            d["kwargs"]["message"] = process["log"]["message"]
            d["kwargs"]["level"] = process["log"]["level"]
            
            if "msg_args" in process["log"].keys():
                d["kwargs"]["msg_args"] = process["log"]["msg_args"]
            else:
                d["kwargs"]["msg_args"] = None                
            
            self.processes.append(d)

        except Exception as e:
            self.event_cb(self.init_err_str.format("log", str(e)), "warn")
            
            
    def is_verbose(self, process):
        
        if "verbose" in process.keys():
            verbose = process["verbose"]
        else:
            verbose = True
            
        return verbose
            
        
    def execute(self, msg=None):
        
        if self.lock_exec:
            self._lock.acquire()
            
        self.msg = msg
        
        for process in self.processes:
            rospy.sleep(0.1) # needed when using slackeros
            try:
                if process["verbose"] and "def_msg" in process.keys():
                    self.event_cb(process["def_msg"][0], process["def_msg"][1], process["def_msg"][2])
                    
                kwargs = process["kwargs"]            
                eval(process["func"])
                
            except Exception as e:
                self.event_cb("Unable to execute process of type '{}': {}".format(process["name"], str(e)), "warn")
            
        if self.lock_exec:
            self._lock.release()
            

    def call(self, service_name, service_client, req, verbose):
        
        resp = service_client(req)
        
        if verbose and resp.success:
            self.event_cb("Call to service '{}' succeeded".format(service_name), "info", req)
        elif not resp.success:
            self.event_cb("Call to service '{}' failed".format(service_name), "warn", req)
        
        
    def publish(self, pub, msg):
        pub.publish(msg)
        
        
    def action(self, spec, action_client, goal, verbose):
        
        self.spec = spec
        self.goal = goal
        self.verbose_action = verbose
        
        action_client.send_goal(goal, self.goal_cb)
        
       
    def sleep(self, duration):
        rospy.sleep(duration)
        
        
    def shell(self, cmd_args):
        
        process = subprocess.Popen(cmd_args,
                     stdout=subprocess.PIPE, 
                     stderr=subprocess.PIPE)
                     
        stdout, stderr = process.communicate()
        print stdout
        
        if stderr:
            self.event_cb("Unable to execute shell commands {}: {}".format(cmd_args, stderr), "warn")
        
    
    def log(self, message, level, msg_args):
        
        msg = self.msg
        if msg is not None and msg_args is not None:
            args = [eval(arg) for arg in msg_args]
            args = tuple(args)
            self.event_cb("CUSTOM MSG: " + message.format(*args), level)
        else:
            self.event_cb("CUSTOM MSG: " + message, level)
         
        
    def goal_cb(self, status, result):
        
        if self.verbose_action and status == 3:
            self.event_cb("Goal achieved for action with spec '{}'".format(self.spec), "info", self.goal)
        elif status == 2 or status == 6:
            self.event_cb("Goal preempted for action with spec '{}'".format(self.spec), "warn", self.goal)
        elif status != 3:
            self.event_cb("Goal failed for action with spec '{}'".format(self.spec), "warn", self.goal)
#####################################################################################