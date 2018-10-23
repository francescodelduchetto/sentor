#!/usr/bin/env python
from sentor.ROSTopicHz import ROSTopicHz
from sentor.ROSTopicFilter import ROSTopicFilter
import rostopic
import signal
import rospy
import time
import math
import sys
import os

# TODO nice printing of frequency of the topic with curses

unpublished_topics_indexes = []
satisfied_filters_indexes = []

def __signal_handler(signum, frame):
    print "stopped."
    os._exit(signal.SIGTERM)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    topics = rospy.get_param("~topics", "")
    topics = topics.split(" ")

    rths = []
    rtfs = []
    print "Monitoring topics:"
    for topic in topics:
        topic_name, expression = topic.split(".")[0], ".".join(topic.split(".")[1:])
        print "\t-", topic_name,
        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(topic_name, blocking=False)
            topic_type, _, _ = rostopic.get_topic_type(topic_name, blocking=False)
        except ROSTopicException as e:
            print " ",
            rospy.logwarn("Topic %s type cannot be determined, or ROS master cannot be contacted" % topic_name)
        else:
            print "\t", topic_type,
            if real_topic is None:
                print " "
                rospy.logwarn("Topic %s is not published" % topic_name)
                continue

        if expression == "":
            print "\t = monitoring hz"
            hz = ROSTopicHz(topic_name, 1000)
            rths.append(hz)

            rospy.Subscriber(real_topic, msg_class, hz.callback_hz)

        # if there is something else then we also have a filter on the message
        else:
            lambdas = ROSTopicFilter.get_lambdas(expression)

            print "\n\t  = monitoring expressions:",
            for (expression, parameter, lambda_filter) in lambdas:

                if parameter != "":
                    print expression, ",", #, parameter, lambda_filter
                    filter = ROSTopicFilter(topic_name, expression, parameter, lambda_filter)
                    rtfs.append( filter )

                    rospy.Subscriber(real_topic, msg_class, filter.callback_filter)

            print ""


    time.sleep(1)

    rate = rospy.Rate(1) #1Hz
    line_to_print = ""
    while not rospy.is_shutdown():
        # sys.stdout.write('\r')
        # sys.stdout.flush()
        line_to_print = ""
        for n, rth in enumerate(rths):
            res = rth.get_hz()
            if res is None:
                if n not in unpublished_topics_indexes:
                    rospy.logwarn("Topic %s is not published anymore" % rth.topic_name)
                    unpublished_topics_indexes.append(n)
            else:
                line_to_print += rth.topic_name + ": "
                (hz, min_delta, max_delta, std_dev, window_size) = res
                line_to_print += "%.1f  " % hz

                if n in unpublished_topics_indexes:
                    unpublished_topics_indexes.remove(n)

        # sys.stdout.write(line_to_print)
        # sys.stdout.flush()

        for n, rtf in enumerate(rtfs):
            if rtf.is_filter_satisfied():
                if n not in satisfied_filters_indexes:
                    rospy.logwarn("Expression %s on topic %s is satisfied" % (rtf.expression, rtf.topic_name))
                    satisfied_filters_indexes.append(n)
            else:
                if n in satisfied_filters_indexes:
                    satisfied_filters_indexes.remove(n)

        rate.sleep()
