#!/usr/bin/env python
from sentor.TopicMonitor import TopicMonitor
from std_msgs.msg import String
from std_srvs.srv import Empty
import signal
import rospy
import time
import sys
import os

# TODO nice printing of frequency of the topic with curses

unpublished_topics_indexes = []
satisfied_filters_indexes = []

topic_monitors = []

event_pub = None

def __signal_handler(signum, frame):
    print "stopped."
    kill_monitors()
    os._exit(signal.SIGTERM)

    def kill_monitors():
        for topic_monitor in topic_monitors:
            topic_monitor.kill_monitor()
            topic_monitor.join()

def stop_monitoring(_):
    for topic_monitor in topic_monitors:
        topic_monitor.stop_monitor()
        
    rospy.logwarn("sentor_node stopped monitoring")
    return

def start_monitoring(_):
    for topic_monitor in topic_monitors:
        topic_monitor.start_monitor()

    rospy.logwarn("sentor_node started monitoring")
    return

def event_callback(msg, type):
    if type == "info":
        rospy.loginfo(msg)
    elif type == "warn":
        rospy.logwarn(msg)

    if event_pub is not None:
        event_pub.publish(String("%s: %s" % (type, msg)))


if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    topics = rospy.get_param("~topics", "")
    topics = topics.split(" ")

    stop_srv = rospy.Service('/sentor/stop_monitor', Empty, stop_monitoring)
    start_srv = rospy.Service('/sentor/start_monitor', Empty, start_monitoring)

    event_pub = rospy.Publisher('/sentor/event', String, queue_size=10)

    topic_monitors = []
    print "Monitoring topics:"
    for topic in topics:
        topic_name, expressions = topic.split(".")[0], ".".join(topic.split(".")[1:])

        topic_monitor = TopicMonitor(topic_name, expressions, event_callback)

        # start monitoring
        topic_monitor.start()

        topic_monitors.append(topic_monitor)


    while not rospy.is_shutdown():
        time.sleep(1)


    stop_monitor(Empty())



        # # sys.stdout.write('\r')
        # # sys.stdout.flush()
        # line_to_print = ""
        # for n, rth in enumerate(rths):
        #     res = rth.get_hz()
        #     if res is None:
        #         if n not in unpublished_topics_indexes:
        #             rospy.logwarn("Topic %s is not published anymore" % rth.topic_name)
        #             unpublished_topics_indexes.append(n)
        #     else:
        #         line_to_print += rth.topic_name + ": "
        #         (hz, min_delta, max_delta, std_dev, window_size) = res
        #         line_to_print += "%.1f  " % hz
        #
        #         if n in unpublished_topics_indexes:
        #             unpublished_topics_indexes.remove(n)
        #
        # # sys.stdout.write(line_to_print)
        # # sys.stdout.flush()
        #
        # for n, rtf in enumerate(rtfs):
        #     if rtf.is_filter_satisfied():
        #         if n not in satisfied_filters_indexes:
        #             rospy.logwarn("Expression %s on topic %s is satisfied" % (rtf.expression, rtf.topic_name))
        #             satisfied_filters_indexes.append(n)
        #     else:
        #         if n in satisfied_filters_indexes:
        #             satisfied_filters_indexes.remove(n)
        #
        # rate.sleep()
