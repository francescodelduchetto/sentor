import rospy


class ROSTopicFilter(object):

    def __init__(self, topic_name, expression, parameter, lambda_filter):
        self.topic_name = topic_name
        self.expression = expression
        self.parameter = parameter
        self.lambda_filter = lambda_filter
        self.filter_satisfied = False
        self.value_read = False

    @staticmethod
    def get_lambdas(expression):

        expression_list = expression.split(",")

        lambdas = []
        for expression in expression_list:
            if "<=" in expression:
                parameter, value = "".join(expression.split("<=")[0]), "".join(expression.split("<=")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) <= float(value)) )
            if ">=" in expression:
                parameter, value = "".join(expression.split(">=")[0]), "".join(expression.split(">=")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) >= float(value)) )
            if "==" in expression:
                parameter, value = "".join(expression.split("==")[0]), "".join(expression.split("==")[-1])
                try:
                    float(value)
                except ValueError:
                    lambdas.append( (expression, parameter, lambda x, value=value : str(x) == value) )
                else:
                    lambdas.append( (expression, parameter, lambda x, value=value : float(x) == float(value)) )
            if "!=" in expression:
                parameter, value = "".join(expression.split("!=")[0]), "".join(expression.split("!=")[-1])
                try:
                    float(value)
                except ValueError:
                    lambdas.append( (expression, parameter, lambda x, value=value : str(x) != value) )
                else:
                    lambdas.append( (expression, parameter, lambda x, value=value : float(x) != float(value)) )
            if "<" in expression:
                parameter, value = "".join(expression.split("<")[0]), "".join(expression.split("<")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) < float(value)) )
            if ">" in expression:
                parameter, value = "".join(expression.split(">")[0]), "".join(expression.split(">")[-1])
                lambdas.append( (expression, parameter, lambda x, value=value : float(x) > float(value)) )
            if "=" in expression:
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

        if self.value_read:
            self.filter_satisfied = self.lambda_filter(value)
            self.value_read = False

        if not self.filter_satisfied and not self.value_read:
            self.filter_satisfied = self.lambda_filter(value)

    def is_filter_satisfied(self):
        self.value_read = True
        return self.filter_satisfied
