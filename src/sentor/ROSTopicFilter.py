import rospy


class ROSTopicFilter(object):

    def __init__(self, topic_name, expression, parameter, lambda_filter):
        self.topic_name = topic_name
        self.expression = expression
        self.parameter = parameter
        self.lambda_filter = lambda_filter
        self.filter_satisfied = False
        self.unread_satisfied = False
        self.value_read = False
        self.sat_callbacks = []
        self.unsat_callbacks = []

    @staticmethod
    def get_lambdas(expression):

        expression_list = expression.split(",")

        lambdas = []
        for expression in expression_list:
            if "<=" in expression:
                parameter, value = "".join(expression.split("<=")[0]), "".join(expression.split("<=")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) <= float(value)) )
            elif ">=" in expression:
                parameter, value = "".join(expression.split(">=")[0]), "".join(expression.split(">=")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) >= float(value)) )
            elif "==" in expression:
                parameter, value = "".join(expression.split("==")[0]), "".join(expression.split("==")[-1])
                try:
                    float(value)
                except ValueError:
                    lambdas.append( (expression, parameter, lambda x, value=value : str(x) == value) )
                else:
                    lambdas.append( (expression, parameter, lambda x, value=value : float(x) == float(value)) )
            elif "!=" in expression:
                parameter, value = "".join(expression.split("!=")[0]), "".join(expression.split("!=")[-1])
                try:
                    float(value)
                except ValueError:
                    lambdas.append( (expression, parameter, lambda x, value=value : str(x) != value) )
                else:
                    lambdas.append( (expression, parameter, lambda x, value=value : float(x) != float(value)) )
            elif "<" in expression:
                parameter, value = "".join(expression.split("<")[0]), "".join(expression.split("<")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) < float(value)) )
            elif ">" in expression:
                parameter, value = "".join(expression.split(">")[0]), "".join(expression.split(">")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) > float(value)) )
            elif "=" in expression:
                parameter, value = "".join(expression.split("=")[0]), "".join(expression.split("=")[-1])
                try:
                    float(value)
                except ValueError:
                    lambdas.append( (expression, parameter, lambda x, value=value : str(x) == value) )
                else:
                    lambdas.append( (expression, parameter, lambda x, value=value : float(x) == float(value)) )
        return lambdas

    def callback_filter(self, msg):
        value = msg
        for part in self.parameter.split("."):
            value = value.__getattribute__(part)

        self.filter_satisfied = self.lambda_filter(value)

        # if the last value was read: set value_read to False
        if self.value_read:
            self.value_read = False
        # else if filter_satisfied
        elif self.filter_satisfied:
            self.unread_satisfied = True
            # notify the listeners

        if self.filter_satisfied:
            for func in self.sat_callbacks:
                func(self.expression)
        else:
            for func in self.unsat_callbacks:
                func(self.expression)


        # if not self.filter_satisfied and not self.value_read:
        #     self.filter_satisfied = self.lambda_filter(value)

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
