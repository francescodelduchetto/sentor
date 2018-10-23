# ROS sensor monitoring node

Continuously monitor that sensor messages are published on certain topics. If not published, sends warnings. But in general works with any topic, not only for sensor messages.

## launch

Example launch command to monitor the topics `/scan` and `/image_color`:

`roslaunch sentor sentor_node.py topics:="/scan /image_color"`

We can also monitor that monitor that the topic messages fulfill a certain expression (and get a warning when that happen):

```
roslaunch sentor sentor_node.py topics:="/scan /image_color /battery_state /battery_state.lifePercent<5,lifePercent==1"
```

with the above command we get a warning when the topics `/scan`, `/image_color` and `/battery_state` are not published and when the parameter `lifePercent` value of topic messages in `/battery_state` becomes less than 5 and when it becomes equal to 1. The comma separates different expressions on the same topic message. Valid operators are `==`, `<=`, `>=`, `!=`, `<`, `>`.
