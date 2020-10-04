# ROS1 sample packages
This project is a set of sample ROS1 packages.

## Setup
```
$ mkdir -p /path/to/project
$ cd /path/to/project
$ mkdir src
$ catkin_make
$ cd src
$ git clone https://github.com/sykwer/ros_samples.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## Sample0
![sample0](https://github.com/sykwer/ros_samples/blob/master/images/sample0.png?raw=true)

You can watch relayed messages with timestamp history via `/ros_sample0/destination_output` topic.
```
$ roslaunch ros_sample0 ros_sample0.launch
$ rostopic echo /ros_sample0/destination_output
```

## Sample1
![sample1](https://github.com/sykwer/ros_samples/blob/master/images/sample1.png?raw=true)

You can watch relayed messages with timestamp history via `/ros_sample1/destination_output` topic.
```
$ roslaunch ros_sample1 ros_sample1.launch
$ rostopic echo /ros_sample1/destination_output
```
