#!/usr/bin/env python
"""
@author: Francesco Del Duchetto (FDelDuchetto@lincoln.ac.uk)
@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
from sentor.TopicMonitor import TopicMonitor
from sentor.SafetyMonitor import SafetyMonitor
from sentor.TopicMapServer import TopicMapServer
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import pprint
import signal
import rospy
import time
import yaml
import sys
import os

# TODO nice printing of frequency of the topic with curses
# TODO consider timeout

unpublished_topics_indexes = []
satisfied_filters_indexes = []

topic_monitors = []

event_pub = None

def __signal_handler(signum, frame):
    def kill_monitors():
        for topic_monitor in topic_monitors:
            topic_monitor.kill_monitor()
    def join_monitors():
        for topic_monitor in topic_monitors:
            topic_monitor.join()
    kill_monitors()
    join_monitors()
    print "stopped."
    os._exit(signal.SIGTERM)
    

def stop_monitoring(_):
    for topic_monitor in topic_monitors:
        topic_monitor.stop_monitor()
            
    safety_monitor.stop_monitor()
    
    if topic_mapping:
        topic_map_server.stop()

    rospy.logwarn("sentor_node stopped monitoring")
    ans = EmptyResponse()
    return ans
    

def start_monitoring(_):
    for topic_monitor in topic_monitors:
        topic_monitor.start_monitor()
            
    safety_monitor.start_monitor()
    
    if topic_mapping:
        topic_map_server.start()

    rospy.logwarn("sentor_node started monitoring")
    ans = EmptyResponse()
    return ans
    

def event_callback(string, type, msg=""):
    if type == "info":
        rospy.loginfo(string + '\n' + str(msg))
    elif type == "warn":
        rospy.logwarn(string + '\n' + str(msg))
    elif type == "error":
        rospy.logerr(string + '\n' + str(msg))

    if event_pub is not None:
        event_pub.publish(String("%s: %s" % (type, string)))
##########################################################################################
    

##########################################################################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    config_file = rospy.get_param("~config_file", "")

    try:
        topics = yaml.load(file(config_file, 'r'))
    except Exception as e:
        rospy.logerr("No configuration file provided: %s" % e)
        topics = []

    stop_srv = rospy.Service('/sentor/stop_monitor', Empty, stop_monitoring)
    start_srv = rospy.Service('/sentor/start_monitor', Empty, start_monitoring)

    event_pub = rospy.Publisher('/sentor/event', String, queue_size=10)

    safety_pub_rate = rospy.get_param("~safety_pub_rate", "")    
    auto_safety_tagging = rospy.get_param("~auto_safety_tagging", "")        
    safety_monitor = SafetyMonitor(safety_pub_rate, auto_safety_tagging, event_callback)   

    topic_mapping = False
    topic_monitors = []
    print "Monitoring topics:"
    for topic in topics:
        try:
            topic_name = topic["name"]
        except Exception as e:
            rospy.logerr("topic name is not specified for entry %s" % topic)
            continue

        signal_when = {}
        signal_lambdas = []
        processes = []
        timeout = 0
        default_notifications = True
        _map = None
        include = True
        if 'signal_when' in topic.keys():
            signal_when = topic['signal_when']
        if 'signal_lambdas' in topic.keys():
            signal_lambdas = topic['signal_lambdas']
        if 'execute' in topic.keys():
            processes = topic['execute']
        if 'timeout' in topic.keys():
            timeout = topic['timeout']
        if 'default_notifications' in topic.keys():
            default_notifications = topic['default_notifications']
        if 'map' in topic.keys():
            _map = topic['map']
        if 'include' in topic.keys():
            include = topic['include']
            
        if include and _map is not None:
            topic_mapping = True

        if include:
            topic_monitor = TopicMonitor(topic_name, signal_when, signal_lambdas, processes, 
                                         timeout, default_notifications, _map, event_callback)

            topic_monitors.append(topic_monitor)
            safety_monitor.register_monitors(topic_monitor)
            
    time.sleep(1)
    
    if topic_mapping:
        map_pub_rate = rospy.get_param("~map_pub_rate", "") 
        map_plt_rate = rospy.get_param("~map_plt_rate", "") 
        topic_map_server = TopicMapServer(topic_monitors, map_pub_rate, map_plt_rate)

    # start monitoring
    for topic_monitor in topic_monitors:
        topic_monitor.start()

    rospy.spin()
##########################################################################################