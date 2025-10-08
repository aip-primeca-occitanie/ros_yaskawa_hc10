# ROS for Yaskawa HC10

The folders in this repo :
- motoman, motoman_drive, motoman_hc_10_support, motoman_msgs, motoman_resources are taken directly from the motoman official repo, branch noetic-devel : https://github.com/ros-industrial/motoman
- motoman_hc_10_moveit_config (originally created with moveit setup assistant, and since modified) : contains all the custom files


## How to use: with the real robot

0. In every terminal, before doing any ROS command, you have to do:

```sh
source ~/catkin_ws/devel/setup.bash
```

1. Connect to the robot:

For the left-side Yaskawa (HC10DT, white covers):

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=false left_robot:=true robot_ip:=10.100.12.134 controller:=yrc1000 
```

For the right-side Yaskawa (HC10, blue covers):

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=false left_robot:=false robot_ip:=10.100.12.133 controller:=yrc1000 
```

You should see a Rviz window with the robot in its current position.
The grey model represents the position of the physical robot.
The orange model is the Rviz vizualization.

2. Enable the motors :

In another terminal, for the left-side robot:

```sh
source ~/catkin_ws/devel/setup.bash
rosservice call /yaskawa_LEFT/robot_enable
```

Or for the right-side robot:

```sh
source ~/catkin_ws/devel/setup.bash
rosservice call /yaskawa_RIGHT/robot_enable
```

3. Execute a trajectory

You can plan and execute a trajectory directly from Rviz, or you can use a trajectory file that is already created. To do this, in another terminal:


```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config execute_trajectory.launch left_robot:=false
```

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config execute_trajectory.launch left_robot:=true
```

By default, the trajectory file read is `/home/yaska/catkin_ws/src/ros_yaskawa_hc10/motoman_hc10_moveit_config/trajectories/trajectory.csv`.
You can change this file to yours, or specify another filepath like so :

```sh
roslaunch motoman_hc10_moveit_config execute_trajectory.launch left_robot:=false filename:=/path/to/your/trajectory/file.csv
```


## How to use: in simulation

0. In every terminal, before doing any ROS command, you have to do:

```sh
source ~/catkin_ws/devel/setup.bash
```

1. Run the simulation

For the left-side Yaskawa (HC10DT, white covers):

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=true left_robot:=true
```

For the right-side Yaskawa (HC10, blue covers):

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=true left_robot:=false
```

2. Execute a trajectory

This step is identical in simulation and on the real robot.
