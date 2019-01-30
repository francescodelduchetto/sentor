# ROS messages monitoring node

Continuously monitor topic messages. Sends warnings when certain conditions on the messages are satisfied. 

## Launch

Example launch command:

```sh
roslaunch sentor sentor.launch config_file:=config/rob_lindsey.yaml
```

## Config

The config file contains the list of topics to be monitored and the definition, for each, of when we want to be alerted.

```yaml
- name : '/virtual_bumper_event'
  signal_lambdas :
    - 'lambda msg : msg.freeRunStarted == True'

- name : '/diagnostics_toplevel_state'
  signal_when : 'not published'
  signal_lambdas :
    - 'lambda msg : msg.level == 1'
    - 'lambda msg : msg.level == 2'
  timeout : 3

- name : '/interface/buttonPressedWhileDisabled'
  signal_when : 'published'
```
- `name`: is the name of the topic to monitor
- `signal_when`: optional, can be either 'not published' or 'published'. Respectively, it will send a warning when the topic is not published or when it is.
- `signal_lambdas`: optional, it's a list of (pythonic) lambda expressions such that when they are satisfied a warning is sent
- `timeout`: optional (default=0.1), amount of time (in seconds) for which the signal has to be satisfied before sending the warning.
