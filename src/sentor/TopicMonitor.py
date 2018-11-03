from sentor.ROSTopicHz import ROSTopicHz
from sentor.ROSTopicFilter import ROSTopicFilter

from threading import Thread, Event
import rostopic
import rospy
import time

class TopicMonitor(Thread):

    def __init__(self, topic_name, expressions, event_callback):
        Thread.__init__(self)

        self.topic_name = topic_name
        self.expressions = expressions
        self.event_callback = event_callback
        self.satisfied_expressions = []
        self.unsatisfied_expressions = []
        self.is_instantiated = False
        self.is_instantiated = self._instantiate_monitors()
        self.is_topic_published = True # initially assume it is
        self.is_latch = False
        self.published_filters_list = []

        self._stop_event = Event()
        self._killed_event = Event()


    def _instantiate_monitors(self):
        if self.is_instantiated: return True

        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(self.topic_name, blocking=False)
            topic_type, _, _ = rostopic.get_topic_type(self.topic_name, blocking=False)
        except rostopic.ROSTopicException as e:
            self.event_callback("Topic %s type cannot be determined, or ROS master cannot be contacted" % self.topic_name)
            return False
        else:
            if real_topic is None:
                self.event_callback("Topic %s is not published" % self.topic_name, "warn")
                return False

            self.hz_monitor = self._instantiate_hz_monitor(real_topic, self.topic_name, msg_class)

            print "TopicMonitor for %s initialized" % self.topic_name

            # if there is something else then we have a filter on the message
            if self.expressions != "":
                print "\t  = monitoring expressions:",

                self.lambdas = ROSTopicFilter.get_lambdas(self.expressions)

                self.lambda_monitor_list = []
                for (expression, parameter, lambda_filter) in self.lambdas:
                    if parameter != "":
                        print expression, " ",
                        lambda_monitor = self._instantiate_lambda_monitor(real_topic, expression, parameter, msg_class, lambda_filter)

                        # register cb that notifies when the lambda function is True
                        lambda_monitor.register_satisfied_cb(self.lambda_satisfied_cb)
                        lambda_monitor.register_unsatisfied_cb(self.lambda_unsatisfied_cb)

                        self.lambda_monitor_list.append(lambda_monitor)
                print ""
            self.is_instantiated = True

            return True

    def _instantiate_hz_monitor(self, real_topic, topic_name, msg_class):
        hz = ROSTopicHz(topic_name, 1000)

        rospy.Subscriber(real_topic, msg_class, hz.callback_hz)

        return hz

    def _instantiate_lambda_monitor(self, real_topic, expression, parameter, msg_class, lambda_filter):
        filter = ROSTopicFilter(self.topic_name, expression, parameter, lambda_filter)

        rospy.Subscriber(real_topic, msg_class, filter.callback_filter)

        return filter

    def run(self):
        # if the topic was not published initially then no monitor is running
        # but, maybe now it is published
        if not self.is_instantiated:
            if not self._instantiate_monitors():
                return
            else:
                self.is_instantiated = True

        while not self._killed_event.isSet():
            while not self._stop_event.isSet():
                # check it is still published
                rate = self.hz_monitor.get_hz()

                # if the publishing rate is less than 1Hz we assume it's a latch message
                if rate is not None:
                    self.is_latch = (rate < 1.0)

                if rate is None and self.is_topic_published and not self.is_latch:
                    self.event_callback("Topic %s is not published anymore" % self.topic_name, "warn")
                    self.is_topic_published = False

                if rate is not None and not self.is_latch and not self.is_topic_published:
                    self.is_topic_published = True

                for _ in range(len(self.satisfied_expressions)):
                    expr = self.satisfied_expressions.pop()
                    if not expr in self.published_filters_list:
                        self.event_callback("Expression %s on topic %s satisfied" % (expr, self.topic_name), "warn")
                        self.published_filters_list.append(expr)

                for _ in range(len(self.unsatisfied_expressions)):
                    expr = self.unsatisfied_expressions.pop()
                    if expr in self.published_filters_list:
                        self.published_filters_list.remove(expr)

                time.sleep(1)
            time.sleep(1)

    def stop_monitor(self):
        self._stop_event.set()

    def start_monitor(self):
        self._stop_event.clear()

    def kill_monitor(self):
        self._kill_event.set()

    def lambda_satisfied_cb(self, msg):
        if not self._stop_event.isSet():
            self.satisfied_expressions.append(msg)

    def lambda_unsatisfied_cb(self, msg):
        if not self._stop_event.isSet():
            self.unsatisfied_expressions.append(msg)
