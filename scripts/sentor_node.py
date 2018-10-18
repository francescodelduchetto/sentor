#!/usr/bin/env python
from os import _exit
from sentor.ROSTopicHz import ROSTopicHz
import rostopic
import signal
import rospy
import time

def __signal_handler(signum, frame):
    print "stopped."
    _exit(signal.SIGTERM)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    topics = rospy.get_param("~topics", "")
    topics = topics.split()

    rts = []
    print "Monitoring topics:"
    for n, topic in enumerate(topics):
        print "\t-", topic,
        msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
        topic_type, _, _ = rostopic.get_topic_type(topic, blocking=False)
        if real_topic is None:
            rospy.logwarn("Topic %s is not published" % topic)
            continue

        print real_topic, "\t", topic_type

        rts.append( ROSTopicHz(1000) )

        rospy.Subscriber(real_topic, msg_class, rts[n].callback_hz)

    time.sleep(1)

    rate = rospy.Rate(1) #5Hz
    while not rospy.is_shutdown():
        for n, rt in enumerate(rts):
            print topics[n],

            res = rt.get_hz()
            if res is None:
                rospy.logwarn("Topic %s is not published anymore" % topics[n])
            else:
                (hz, min_delta, max_delta, std_dev, window_size) = res
                print "average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%\
                    (hz, min_delta, max_delta, std_dev, window_size)

        rate.sleep()
