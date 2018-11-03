"""
Modified from https://github.com/strawlab/ros_comm/blob/master/tools/rostopic/src/rostopic.py
"""
import rospy
import threading
import math

class ROSTopicHz(object):
    """
    ROSTopicHz receives messages for a topic and computes frequency stats
    """
    def __init__(self, topic_name, window_size, filter_expr=None):
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.times =[]
        self.filter_expr = filter_expr
        self.topic_name = topic_name

        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def callback_hz(self, m):
        """
        ros sub callback
        @param m: Message instance
        @type  m: roslib.message.Message
        """
        # #694: ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            curr_rostime = rospy.get_rostime()

            # time reset
            if curr_rostime.is_zero():
                if len(self.times) > 0:
                    print("time has reset, resetting counters")
                    self.times = []
                return

            curr = curr_rostime.to_sec()
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.times = []
            else:
                self.times.append(curr - self.msg_tn)
                self.msg_tn = curr

            #only keep statistics for the last 10000 messages so as not to run out of memory
            if len(self.times) > self.window_size - 1:
                self.times.pop(0)

    def print_hz(self):
        """
        print the average publishing rate to screen
        """
        if not self.times:
            return
        elif self.msg_tn == self.last_printed_tn:
            print("no new messages")
            return
        with self.lock:
            #frequency

            # kwc: In the past, the rate decayed when a publisher
            # dies.  Now, we use the last received message to perform
            # the calculation.  This change was made because we now
            # report a count and keep track of last_printed_tn.  This
            # makes it easier for users to see when a publisher dies,
            # so the decay is no longer necessary.

            n = len(self.times)
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)

            # min and max
            max_delta = max(self.times)
            min_delta = min(self.times)

            self.last_printed_tn = self.msg_tn
        print("average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(rate, min_delta, max_delta, std_dev, n+1))

    def get_hz(self):
        """
        return the average publishing rate to screen
        """
        if not self.times:
            return
        elif self.msg_tn == self.last_printed_tn:
            # print("no new messages")
            return None
        with self.lock:
            #frequency

            # kwc: In the past, the rate decayed when a publisher
            # dies.  Now, we use the last received message to perform
            # the calculation.  This change was made because we now
            # report a count and keep track of last_printed_tn.  This
            # makes it easier for users to see when a publisher dies,
            # so the decay is no longer necessary.

            n = len(self.times)
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)

            # min and max
            max_delta = max(self.times)
            min_delta = min(self.times)

            self.last_printed_tn = self.msg_tn
        return rate
