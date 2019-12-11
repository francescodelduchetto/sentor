from sentor.ROSTopicHz import ROSTopicHz
from sentor.ROSTopicFilter import ROSTopicFilter
from sentor.ROSTopicPub import ROSTopicPub
from sentor.Executor import Executor

from threading import Thread, Event, Lock
import rostopic
import rospy
import time

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class TopicMonitor(Thread):


    def __init__(self, topic_name, signal_when, safety_critical, signal_lambdas, 
                 processes, lock_exec, timeout, event_callback, safety_callback):
        Thread.__init__(self)

        self.topic_name = topic_name
        self.signal_when = signal_when
        self.safety_critical = safety_critical
        self.signal_lambdas = signal_lambdas
        self.processes = processes
        if timeout > 0:
            self.timeout = timeout
        else:
            self.timeout = 0.1
        self.event_callback = event_callback
        self.safety_callback = safety_callback
        self.satisfied_expressions = []
        self.sat_expressions_timer = {}
        self.pub_monitor = None
        self.hz_monitor = None
        self.is_topic_published = True # initially assume it is
        self.is_latch = False
        self.published_filters_list = []
        self.unsatisfied_expressions = []
        self.is_instantiated = False
        self.is_instantiated = self._instantiate_monitors()

        if processes:
            self.executor = Executor(processes, lock_exec, event_callback)

        self._stop_event = Event()
        self._killed_event = Event()

        self._lock = Lock()

    def _instantiate_monitors(self):
        if self.is_instantiated: return True

        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(self.topic_name, blocking=False)
            topic_type, _, _ = rostopic.get_topic_type(self.topic_name, blocking=False)
        except rostopic.ROSTopicException as e:
            self.event_callback("Topic %s type cannot be determined, or ROS master cannot be contacted" % self.topic_name)
            return False

        if real_topic is None:
            self.event_callback("Topic %s is not published" % self.topic_name, "warn")
            return False

        if self.signal_when.lower() == 'published':
            print "Signaling 'published' for "+ bcolors.OKBLUE + self.topic_name + bcolors.ENDC +" initialized"
            # signal every msg published
            self.pub_monitor = self._instantiate_pub_monitor(real_topic, self.topic_name, msg_class)

            self.pub_monitor.register_published_cb(self.published_cb)

        elif self.signal_when.lower() == 'not published':
            print "Signaling 'not published' for "+ bcolors.BOLD + str(self.timeout) + " seconds" + bcolors.ENDC +" for " + bcolors.OKBLUE + self.topic_name + bcolors.ENDC +" initialized"
            # signal when it is not published
            self.hz_monitor = self._instantiate_hz_monitor(real_topic, self.topic_name, msg_class)

        # if there is something else then we have a filter on the message
        if len(self.signal_lambdas):
            print "Signaling expressions for "+ bcolors.OKBLUE + self.topic_name + bcolors.ENDC + " ("+ bcolors.BOLD+"timeout: %s seconds" %  self.timeout + bcolors.ENDC +"):"

            self.lambda_monitor_list = []
            for signal_lambda in self.signal_lambdas:
                
                lambda_fn_str = signal_lambda["expression"]
                
                if "safety_critical" in signal_lambda.keys(): 
                    safety_critical_lambda = signal_lambda["safety_critical"]
                else:
                    safety_critical_lambda = False
                
                if lambda_fn_str != "":
                    print "\t" + bcolors.OKGREEN + lambda_fn_str + bcolors.ENDC
                    lambda_monitor = self._instantiate_lambda_monitor(real_topic, msg_class, lambda_fn_str, safety_critical_lambda)

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

    def _instantiate_pub_monitor(self, real_topic, topic_name, msg_class):
        pub = ROSTopicPub(topic_name)

        rospy.Subscriber(real_topic, msg_class, pub.callback_pub)

        return pub

    def _instantiate_lambda_monitor(self, real_topic, msg_class, lambda_fn_str, safety_critical_lambda):
        filter = ROSTopicFilter(self.topic_name, lambda_fn_str, safety_critical_lambda)

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
                
        def cb(_):
            if self.safety_critical:
                self.safety_callback(False)
                self.event_callback("SAFETY CRITICAL: Topic %s is not published anymore" % self.topic_name, "warn")
            else:
                self.event_callback("Topic %s is not published anymore" % self.topic_name, "warn")
            self.execute()

        timer = None
        while not self._killed_event.isSet():
            while not self._stop_event.isSet():
                if self.hz_monitor is not None:
                    # check it is still published (None if not)
                    rate = self.hz_monitor.get_hz()

                    # # if the publishing rate is less than 1Hz we assume it's a latch message
                    # if rate is not None:
                    #     self.is_latch = (rate < 0.5)


                    if rate is None and self.is_topic_published: #and not self.is_latch:
                        self.is_topic_published = False

                        timer = rospy.Timer(rospy.Duration.from_sec(self.timeout), cb, oneshot=True)
                        # self.event_callback("Topic %s is not published anymore" % self.topic_name, "warn")

                    if rate is not None:# and not self.is_topic_published:# and not self.is_latch:
                        self.is_topic_published = True

                        if timer is not None:
                            timer.shutdown()
                            timer = None

                # self._lock.acquire()
                # while len(self.satisfied_expressions) > 0:
                #     expr = self.satisfied_expressions.pop()
                #     if not expr in self.published_filters_list:
                #         self.event_callback("Expression %s on topic %s satisfied" % (expr, self.topic_name), "warn")
                #         self.published_filters_list.append(expr)
                #         #print "+", expr
                #     #else:
                #         #print "=", expr
                #
                # while len(self.unsatisfied_expressions) > 0:
                #     expr = self.unsatisfied_expressions.pop()
                #     if expr in self.published_filters_list:
                #         self.published_filters_list.remove(expr)
                #         #print "-", expr
                # self._lock.release()

                time.sleep(0.3)
            time.sleep(1)

    def lambda_satisfied_cb(self, expr, msg, safety_critical_lambda):
        if not self._stop_event.isSet():            
            if not expr in self.sat_expressions_timer.keys():
                # self.satisfied_expressions.append(expr)
                def cb(_):
                    if safety_critical_lambda:
                        self.safety_callback(False)  
                        self.event_callback("SAFETY CRITICAL: Expression '%s' for %s seconds on topic %s satisfied" % (expr, self.timeout, self.topic_name), "warn", msg)
                    else:
                        self.event_callback("Expression '%s' for %s seconds on topic %s satisfied" % (expr, self.timeout, self.topic_name), "warn", msg)
                    if len(self.sat_expressions_timer.keys()) == len(self.signal_lambdas):
                        self.execute(msg)
                
                self._lock.acquire()
                self.sat_expressions_timer.update({expr: rospy.Timer(rospy.Duration.from_sec(self.timeout), cb, oneshot=True)})
                self._lock.release()
            #print "sat", msg

    def lambda_unsatisfied_cb(self, expr):
        if not self._stop_event.isSet():
            if expr in self.sat_expressions_timer.keys():
                self._lock.acquire()
                self.sat_expressions_timer[expr].shutdown()
                self.sat_expressions_timer.pop(expr)
                self._lock.release()
            # if not msg in self.unsatisfied_expressions and msg in self.satisfied_expressions:
            #     self.unsatisfied_expressions.append(msg)
            #print "unsat", msg

    def published_cb(self, msg):
        if not self._stop_event.isSet():
            if self.safety_critical:
                self.safety_callback(False)
                self.event_callback("SAFETY CRITICAL: Topic %s is published " % (self.topic_name), "warn")
            else:
                self.event_callback("Topic %s is published " % (self.topic_name), "warn")
            self.execute(msg)
            # self._lock.acquire()
            # if not msg in self.satisfied_expressions:
            #     self.satisfied_expressions.append(msg)
            #     self.sat_expressions_time.update({msg: rospy.Time.now()})
            #     if msg in self.published_filters_list:
            #         self.published_filters_list.remove(msg)
            # self._lock.release()
            
    def execute(self, msg=None):
        if self.processes:
            rospy.sleep(0.1) # needed when using slackeros
            self.executor.execute(msg)
            
    def stop_monitor(self):
        self._stop_event.set()

    def start_monitor(self):
        self._stop_event.clear()

    def kill_monitor(self):
        self.stop_monitor()
        self._killed_event.set()
