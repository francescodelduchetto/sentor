#!/usr/bin/env python
"""
@author: Francesco Del Duchetto (FDelDuchetto@lincoln.ac.uk)

"""
#####################################################################################
import rospy


class ROSTopicPub(object):

    def __init__(self, topic_name):

        self.topic_name = topic_name
        self.pub_callbacks = []

    def callback_pub(self, msg):

        for func in self.pub_callbacks:
            func("'published'")

    def register_published_cb(self, func):

        self.pub_callbacks.append(func)
#####################################################################################
