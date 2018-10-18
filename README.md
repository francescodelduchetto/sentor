# ROS sensor monitoring node

Continuously monitor that sensor messages are published on certain topics. If not sends warnings. But in general works with any topic, not only for sensor messages.

## launch

Example launch command to monitor the topics `/scan` and `/image_color`:

`roslaunch sentor sentor_node.py topics:="/scan /image_color"`
