# rosaria
ROS package for communication with Pioneer robots



## Install

First, clone the package to your `catkin_ws`

To install all the dependencies of this package (including `libaria`), use this command:

```bash
$ rosdep install --from-paths src --ignore-src -r -y
```


Then, compile the package

```bash
$ catkin build
```



## Improvements over the original rosaria package


- Inclusion of the topic `cmd_vel_raw`, which receives a `Twist` message and converts it to right and left speeds. These velocities are sent to the robot. The original `cmd_vel`. sends the linear and angular speeds to the robot, which are internally converted to right and left speeds.

- Inclusion of topics `vel_r` and `vel_l`, which provides the linear speeds measured by the encoders. The topics `omega_r` and `omega_l` provide the correspondent angular speeds.

- Python script to teleop the robot


## Extra info

The rosaria package provides the RosAria ROS node, which uses ARIA to communicate with a mobile robot platform from Adept MobileRobots (formerly MobileRobots Inc, formerly ActivMedia Robotics).  The  RosAria node is implemented in the RosAriaNode class.

More information is available at <http://wiki.ros.org/rosaria> and <http://wiki.ros.org/Robots/Pioneer>.