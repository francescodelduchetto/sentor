#!/usr/bin/env python
from sentor.ROSTopicHz import ROSTopicHz
import rostopic
import signal
import rospy
import time
import math
import sys
import os


unpublished_topics_indexes = []

def __signal_handler(signum, frame):
    print "stopped."
    os._exit(signal.SIGTERM)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    topics = rospy.get_param("~topics", "")
    topics = topics.split()

    rts = []
    print "Monitoring topics:"
    for n, topic in enumerate(topics):
        print "\t-", topic,
        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
            topic_type, _, _ = rostopic.get_topic_type(topic, blocking=False)
        except ROSTopicException as e:
            print " "
            rospy.logwarn("Topic %s type cannot be determined, or ROS master cannot be contacted" % topic)
        else:
            print real_topic, "\t", topic_type
            if real_topic is None:
                rospy.logwarn("Topic %s is not published" % topic)
                continue


        rts.append( ROSTopicHz(1000) )

        rospy.Subscriber(real_topic, msg_class, rts[n].callback_hz)

    time.sleep(1)

    rate = rospy.Rate(1) #1Hz
    line_to_print = ""
    while not rospy.is_shutdown():
        sys.stdout.write('\r')
        sys.stdout.flush()
        # print "lines", lines, len(line_to_print), int(columns), tabs
        line_to_print = ""
        for n, rt in enumerate(rts):
            res = rt.get_hz()
            if res is None:
                if n not in unpublished_topics_indexes:
                    rospy.logwarn("Topic %s is not published anymore" % topics[n])
                    unpublished_topics_indexes.append(n)
            else:
                line_to_print += topics[n] + ": "
                (hz, min_delta, max_delta, std_dev, window_size) = res
                line_to_print += "%.1f  " % hz

                if n in unpublished_topics_indexes:
                    unpublished_topics_indexes.remove(n)
        sys.stdout.write(line_to_print)
        sys.stdout.flush()

        rate.sleep()
