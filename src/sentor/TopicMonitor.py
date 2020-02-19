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


    def __init__(self, topic_name, signal_when, safety_critical, signal_lambdas, processes, 
                 lock_exec, repeat_exec, timeout, lambdas_when_published, default_notifications,
                 event_callback):
        Thread.__init__(self)

        self.topic_name = topic_name
        self.signal_when = signal_when
        self.safety_critical = safety_critical
        self.signal_lambdas = signal_lambdas
        self.processes = processes
        self.repeat_exec = repeat_exec
        if timeout > 0:
            self.timeout = timeout
        else:
            self.timeout = 0.1
        self.lambdas_when_published = lambdas_when_published
        self.default_notifications = default_notifications
        self.event_callback = event_callback
        self.satisfied_expressions = []
        self.sat_expressions_timer = {}
        self.sat_expr_crit_timer = {}
        self.sat_expr_repeat_timer = {}
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
        
        self.signal_when_is_safe = True
        self.lambdas_are_safe = True
        self.thread_is_safe = True
        rospy.Timer(rospy.Duration(0.1), self.safety_cb)

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
            if self.signal_when.lower() == 'not published' and self.safety_critical:
                self.signal_when_is_safe = False
            return False
            
        self.hz_monitor = self._instantiate_hz_monitor(real_topic, self.topic_name, msg_class)

        if self.signal_when.lower() == 'published':
            print "Signaling 'published' for "+ bcolors.OKBLUE + self.topic_name + bcolors.ENDC +" initialized"
            # signal every msg published
            self.pub_monitor = self._instantiate_pub_monitor(real_topic, self.topic_name, msg_class)

            self.pub_monitor.register_published_cb(self.published_cb)
            
            if self.safety_critical:
                self.signal_when_is_safe = False

        elif self.signal_when.lower() == 'not published':
            print "Signaling 'not published' for "+ bcolors.BOLD + str(self.timeout) + " seconds" + bcolors.ENDC +" for " + bcolors.OKBLUE + self.topic_name + bcolors.ENDC +" initialized"
            # signal when it is not published

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
            if self.signal_when.lower() == 'not published':
                if self.safety_critical:
                    self.signal_when_is_safe = False
                if self.default_notifications and self.safety_critical:
                    self.event_callback("SAFETY CRITICAL: Topic %s is not published anymore" % self.topic_name, "warn")
                elif self.default_notifications:
                    self.event_callback("Topic %s is not published anymore" % self.topic_name, "warn")
                if not self.repeat_exec:
                    self.execute()

        def repeat_cb(_):
            if self.signal_when.lower() == 'not published':
                self.execute()

        timer = None
        timer_repeat = None
        while not self._killed_event.isSet():
            while not self._stop_event.isSet():
                # check it is still published (None if not)
                if self.hz_monitor is not None:
                    rate = self.hz_monitor.get_hz()
    
                    # # if the publishing rate is less than 1Hz we assume it's a latch message
                    # if rate is not None:
                    #     self.is_latch = (rate < 0.5)
    
    
                    if rate is None and self.is_topic_published: #and not self.is_latch:
                        self.is_topic_published = False
    
                        timer = rospy.Timer(rospy.Duration.from_sec(self.timeout), cb, oneshot=True)
                        
                        if self.repeat_exec:
                            timer_repeat = rospy.Timer(rospy.Duration.from_sec(self.timeout), repeat_cb, oneshot=False)
                        # self.event_callback("Topic %s is not published anymore" % self.topic_name, "warn")
    
                    if rate is not None:# and not self.is_topic_published:# and not self.is_latch:
                        self.is_topic_published = True
                        
                        if self.safety_critical:
                            self.signal_when_is_safe = True
    
                        if timer is not None:
                            timer.shutdown()
                            timer = None
                        
                        if self.repeat_exec:
                            if timer_repeat is not None:
                                timer_repeat.shutdown()
                                timer_repeat = None
    
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
            if safety_critical_lambda:
                if not expr in self.sat_expr_crit_timer.keys():
                    def crit_cb(_):
                        process_lambda, self.sat_expr_crit_timer = ProcessLambda(self.sat_expr_crit_timer)
                        if process_lambda:
                            self.lambdas_are_safe = False
                            if self.default_notifications:
                                self.event_callback("SAFETY CRITICAL: Expression '%s' for %s seconds on topic %s satisfied" % (expr, self.timeout, self.topic_name), "warn", msg)
                    
                    self._lock.acquire()
                    self.sat_expr_crit_timer.update({expr: rospy.Timer(rospy.Duration.from_sec(self.timeout), crit_cb, oneshot=True)})
                    self._lock.release()            
            
            if not expr in self.sat_expressions_timer.keys():
                def cb(_):
                    process_lambda, self.sat_expressions_timer = ProcessLambda(self.sat_expressions_timer)
                    if process_lambda:
                        if self.default_notifications and not safety_critical_lambda:
                            self.event_callback("Expression '%s' for %s seconds on topic %s satisfied" % (expr, self.timeout, self.topic_name), "warn", msg)
                        if not self.repeat_exec and len(self.sat_expressions_timer.keys()) == len(self.signal_lambdas):
                                self.execute(msg)
                
                self._lock.acquire()
                self.sat_expressions_timer.update({expr: rospy.Timer(rospy.Duration.from_sec(self.timeout), cb, oneshot=True)})
                self._lock.release()
            
            if self.repeat_exec:
                if not expr in self.sat_expr_repeat_timer.keys():
                    def repeat_cb(_):
                        process_lambda, self.sat_expr_repeat_timer = ProcessLambda(self.sat_expr_repeat_timer)
                        if process_lambda:                        
                            if len(self.sat_expr_repeat_timer.keys()) == len(self.signal_lambdas):
                                self.execute(msg)
                                for expr in self.sat_expr_repeat_timer.keys():
                                    self.sat_expr_repeat_timer = self.kill_timer(self.sat_expr_repeat_timer, expr) 
                        
                    self._lock.acquire()
                    self.sat_expr_repeat_timer.update({expr: rospy.Timer(rospy.Duration.from_sec(self.timeout), repeat_cb, oneshot=True)})
                    self._lock.release()  
                    
        def ProcessLambda(timer_dict):
            if self.lambdas_when_published and not self.is_topic_published:
                process_lambda = False
                for expr in timer_dict.keys():
                    timer_dict = self.kill_timer(timer_dict, expr) 
            else:
                process_lambda = True
            return process_lambda, timer_dict

    def lambda_unsatisfied_cb(self, expr):
        if not self._stop_event.isSet():
            if expr in self.sat_expr_crit_timer.keys():
                self.sat_expr_crit_timer = self.kill_timer(self.sat_expr_crit_timer, expr) 
            
            if expr in self.sat_expressions_timer.keys():
                self.sat_expressions_timer = self.kill_timer(self.sat_expressions_timer, expr) 
                
            if expr in self.sat_expr_repeat_timer.keys():
                self.sat_expr_repeat_timer = self.kill_timer(self.sat_expr_repeat_timer, expr) 
                
            if not self.sat_expr_crit_timer:
                self.lambdas_are_safe = True
            # if not msg in self.unsatisfied_expressions and msg in self.satisfied_expressions:
            #     self.unsatisfied_expressions.append(msg)
            #print "unsat", msg
                
    def kill_timer(self, timer_dict, expr):
        self._lock.acquire()
        timer_dict[expr].shutdown()
        timer_dict.pop(expr)
        self._lock.release()
        return timer_dict

    def published_cb(self, msg):
        if not self._stop_event.isSet():
            if self.safety_critical:
                self.signal_when_is_safe = False
            if self.default_notifications and self.safety_critical:
                self.event_callback("SAFETY CRITICAL: Topic %s is published " % (self.topic_name), "warn")
            elif self.default_notifications:
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
            
    def safety_cb(self, event=None):
        if self.signal_when_is_safe and self.lambdas_are_safe:
            self.thread_is_safe = True
        else:
            self.thread_is_safe = False
            
    def stop_monitor(self):
        self._stop_event.set()

    def start_monitor(self):
        self._stop_event.clear()

    def kill_monitor(self):
        self.stop_monitor()
        self._killed_event.set()
