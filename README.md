# ROS sensor monitoring node

Continuously monitor that sensor messages are published on certain topics. If not sends warnings/errors on the `/monitor` topic.

## launch

Example launch command to monitor the topics `/scan` and `/image_color`:

`roslaunch sentor sentor_node.py topics:="/scan /image_color"`
