
> # ROS messages monitoring node

Continuously monitor topic messages. Send warnings and execute other processes when certain conditions on the messages are satisfied. 

## Launch

Example launch command:

```sh
roslaunch sentor sentor.launch config_file:=$(rospack find sentor)/config/execute.yaml
```

## Config

The config file contains the list of topics to be monitored and the definition, for each, of when we want to be alerted. It can also contain, for each monitored topic, an optional list of processes to be executed after sending the alert.

```yaml
- name: "/topological_navigation/feedback"
#  signal_when : 'not published'
#  safety_critical: False
  signal_lambdas:
  - expression: "lambda msg: msg.feedback.route == 'WayPoint1'"
    safety_critical: True
  execute:
  - log:
      message: "Navigating away from {}"
      level: warn
      msg_args:
      - "msg.feedback.route"
  - action:
      verbose: True
      namespace: "/topological_navigation"
      package: "topological_navigation"
      action_spec: "GotoNodeAction"
      goal_args:
      -  "goal.target = 'WayPoint45'"
  - shell:
      verbose: True
      cmd_args:
      - "cowsay"
      - "moo"
  timeout: 0.0
  lambdas_when_published: False
  lock_exec: False
  repeat_exec: False
  default_notifications: True
  include: True                  


- name: "/topological_navigation/feedback"
#  signal_when : 'not published'
#  safety_critical: False
  signal_lambdas:
  - expression: "lambda msg: msg.feedback.route == 'WayPoint45'"
    safety_critical: False
  execute:
  - call:
      verbose: True
      service_name: "/sentor/set_safety_tag"
      service_args:
      -  "req.data = True"
  - log:
      message: "Teleporting the robot"
      level: info
  - call:
      verbose: True
      service_name: "/gazebo/set_model_state"
      service_args:
      -  "req.model_state.model_name = 'thorvald_ii'"
      -  "req.model_state.pose.orientation.w = 1.0"
  - sleep:
      verbose: True
      duration: 3.0
  - log:
      message: "Relocalising the robot"
      level: info
  - publish:
      verbose: True
      topic_name: "/initialpose"
      topic_latched: False
      topic_args:
      -  "msg.header.frame_id = '/map'"
      -  "msg.pose.pose.orientation.w = 1.0"
  timeout: 0.0
  lambdas_when_published: False
  lock_exec: False
  repeat_exec: False
  default_notifications: True
  include: True   


- name : '/row_detector/path_error'
  signal_lambdas :
  - expression: "lambda msg : math.isnan(msg.y)"
  include: False 
```
Top-level arguments:
- `name`: is the name of the topic to monitor
- `signal_when`: optional, can be either 'not published' or 'published'. Respectively, it will send a warning when the topic is not published or when it is.
- `safety_critical`: optional (default=False), tag the `signal_when` condition as 'safety critical' (or not). See section 'Safety critical conditions'.
- `signal_lambdas`: optional, it's a list of (pythonic) lambda expressions such that when they are satisfied a warning is sent. You can use the python package `math` in your lambda expressions. See 'Child arguments of `signal_lambdas`' below.
- `execute`: optional, a list of processes to execute if `signal_when` is satisfied, or if all lambda expressions are satisfied. They will be executed in sequence. See 'Child arguments of `execute`' below. 
- `timeout`: optional (default=0.1), amount of time (in seconds) for which the signal has to be satisfied before sending the warning/executing processes.
- `lambdas_when_published`: optional (default=False), setting this to 'True' will ensure that lambda expressions can be satisfied (for `timeout` seconds) only if the topic is currently being published. 
- `lock_exec`: optional (default=False), lock out other threads while this one is executing its sequence of processes.
- `repeat_exec`: optional (default=False), default behaviour is to execute processes once after conditions (`signal_when` or lambdas) have been satisfied for `timeout` seconds.  They will not execute again until a change occurs (i.e. conditions become unsatisfied, then satisfied again). If `repeat_exec` is set to 'True' then processes will be executed every `timeout` seconds whilst the conditions are satisfied.
- `default_notifications`:  optional (default=True), setting this to 'False' will turn off the default warnings given when conditions (`signal_when` or lambdas) are satisfied.
- `include`:  optional (default=True), include this monitor (or not).

Child arguments of `signal_lambdas`:
- `expression`: the lambda expression.
- `safety_critical`: optional (default=False), tag this lambda expression as 'safety critical' (or not). See section 'Safety critical conditions'.

Child arguments of `execute`:
- `call`: optional, call a rosservice.
- `publish`: optional, publish to a rostopic.
- `action`: optional, send a goal for an actionlib action.
- `sleep`: optional, put the sentor node to sleep.
- `shell`: optional, execute a shell command.
- `log`:  optional, ros log a message.

Child arguments of `call`:
- `verbose`: optional (default=True), setting this argument to 'False' will limit notifications from this process to the notification of errors only. 
- `service_name`: the name of the service you are calling.
- `service_args`: a list of service arguments specified in the service request class. Each arg must be prefixed by `req.`

Child arguments of `publish`:
- `verbose`: optional (default=True), setting this argument to 'False' will limit notifications from this process to the notification of errors only. 
- `topic_name`: the name of the topic you are publishing to. 
- `topic_latched`: optional (default=False), boolean specifying whether you are latching the topic (or not).
- `topic_args`: a list of topic arguments specified in the topic's message class. Each arg must be prefixed by `msg.`

Child arguments of `action`:
- `verbose`: optional (default=True), setting this argument to 'False' will limit notifications from this process to the notification of errors only. 
- `namespace`: the namespace of the action.
- `package`: the ros package from which the action specification is retrieved. Specifically the action specification is retrieved from `package.msg`. 
- `action_spec`: the action specification.
- `goal_args`: a list of goal arguments specified in the action spec's goal class. Each arg must be prefixed by `goal.`

Child arguments of `sleep`:
- `verbose`:  optional (default=True), setting this argument to 'False' will limit notifications from this process to the notification of errors only. 
- `duration`: sleep the sentor node for `duration` seconds.

Child arguments of `shell`:
- `verbose`: optional (default=True), setting this argument to 'False' will limit notifications from this process to the notification of errors only. 
- `cmd_args`: a list of shell command components.

Child arguments of `log`:
- `message`: the message that you are logging.
- `level`: the log level (can be 'info', 'warn' or 'error').
- `msg_args`: optional, you can supply message data from the topic you are monitoring in your logs. See the first monitor in the example config.

## Safety critical conditions
A topic monitor's `signal_when` condition, and each of its lambda expressions, can be tagged as *safety critical*. If any safety critical condition in any topic monitor is satisfied then the boolean message from the topic `safe operation` will be set to 'False'. 

By setting the arg `auto_safety_tagging` (see `sentor.launch`) to 'True' sentor will automatically set `safe operation` to True when all safety critical condition across all monitors are unsatisfied.  If `auto_safety_tagging` is set to 'False' then the service `/sentor/set_safety_tag` must be called.

## Using sentor with this example config
You will need the RASberry repo (<a href="https://github.com/LCAS/RASberry">get it here</a>) and all its dependencies. Also install `cowsay`. Create a file `.rasberryrc` in your home directory and put the following inside it:

`export ROBOT_NAME="thorvald_023"`<br />
`export SCENARIO_NAME="sim_riseholme-uv_poly_act"`  

Then issue this command:
```sh
cd $(rospack find rasberry_bringup)/tmule && tmule -c rasberry-simple_robot_corner_hokuyos.yaml -W 3 launch
```
A Thorvald robot will appear in a gazebo environment and a topological map will be displayed in rviz. Then launch sentor as per the example launch command given above. Then send the robot to the `WayPoint1` node in the topological map. 

You should view gazebo, rviz, the terminal in which you have launched the sentor node and another terminal in which you have echoed the topic `/safe_operation`. You will see that sentor executes the processes for each monitor in the example config. 
