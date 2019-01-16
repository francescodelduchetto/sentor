import rospy


class ROSTopicFilter(object):

    def __init__(self, topic_name, lambda_fn_str):
        self.topic_name = topic_name
        self.lambda_fn_str = lambda_fn_str
        self.lambda_fn = None
        try:
            self.lambda_fn = eval(self.lambda_fn_str)
        except Exception as e:
            rospy.logerr("Error evaluating lambda function %s : %s" % (self.lambda_fn_str, e))

        self.filter_satisfied = False
        self.unread_satisfied = False
        self.value_read = False
        self.sat_callbacks = []
        self.unsat_callbacks = []

    def callback_filter(self, msg):
        if self.lambda_fn is None:
            return

        try:
            self.filter_satisfied = self.lambda_fn(msg)
        except Exception as e:
            rospy.logwarn("Exception while evaluating %s: %s" % (self.lambda_fn_str, e))

        # if the last value was read: set value_read to False
        if self.value_read:
            self.value_read = False
        # else if filter_satisfied
        elif self.filter_satisfied:
            self.unread_satisfied = True
            # notify the listeners

        if self.filter_satisfied:
            for func in self.sat_callbacks:
                func(self.lambda_fn_str)
        else:
            for func in self.unsat_callbacks:
                func(self.lambda_fn_str)


        # if not self.filter_satisfied and not self.value_read:
        #     self.filter_satisfied = self.lambda_fn(value)

        # print value, self.filter_satisfied, self.value_read


    def is_filter_satisfied(self):
        self.value_read = True

        if self.unread_satisfied:
            self.unread_satisfied = False
            return True

        return self.filter_satisfied

    def register_satisfied_cb(self, func):

        self.sat_callbacks.append(func)

    def register_unsatisfied_cb(self, func):

        self.unsat_callbacks.append(func)
