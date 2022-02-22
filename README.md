# ros_yaskawa_hc10
Pilotage d'un robot collaboratif HC10 /Control of an HC10 collaborative robot

*Auteure: Andrea Melissa Pérez Peña - Github ID: andreaPP9*

## Contents
This is a fork made from the motoman_driver repository. Only the packages related to the motoman HC10 robot were kept since it is the model being used at the AIP.
You can find all of them in the `-devel-AP` branch.

This project aims to control a Yaskawa HC10 robot through ROS. A `motoman_hc10_moveit_config` package was created for the planning and execution of trajectories. Also, a solution to exploit the robot's collabotive functionalities was developped. This solution involves reading sensor data through I/O access.

## How to build
This repository has been successfully built on Ubuntu Bionic/ROS Melodic. It depends on the `industrial_core` repository, so you may need to install the binaries and dependencies associated using:
```bash
sudo apt install ros-melodic-industrial-core ros-melodic-moveit 
```
We assume a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has already been created. If not, please follow the instructions for creating one.

Follow the next steps in order to build the `-devel-AP` branch on a **ROS Melodic** system:

```bash
# change to the src folder of the Catkin workspace
cd ~/catkin_ws/src

# retrieve the devel-AP branch
git clone -b devel-AP https://github.com/aip-primeca-occitanie/ros_yaskawa_hc10.git

# change to the root of the Catkin workspace
cd ~/catkin_ws/

# build the workspace
catkin_make
```
## Activating the workspace
If you are working with multiple workspaces activate the new one with :
```bash
source ~/catkin_ws/devel/setup.bash
```
If you are working with one workspace only, you can use:
```bash
echo "source  ~/catkin_ws/devel/setup.bash"  >>  ~/.bashrc 
source ~/.bashrc 
```

## Planning and Execution with MoveIt!
### To simulate the robot:
```bash
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch 
```
### To connect to the real robot:
```bash
# By default, sim:=True. If needed you can also modify the robot_ip address.
roslaunch motoman_hc10_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.113 controller:=yrc1000
```
Make sure the robot is able to receive commands by setting the **Teach pendant** to `Remote Mode` and running in another terminal:
```bash
#Activate your workspace in every new terminal
source ~/catkin_ws/devel/setup.bash
# Enabling ROS commands
rosservice call /robot_enable
```
Now, you can start controlling your robot with Moveit!

**Note:** In order to ***disable*** ROS commands you must run:

```bash
# Disabling ROS commands
rosservice call /robot_disable
```

### Example 
The `execute_trajectroy.py` script in the `motoman_hc10_moveit_config` package, plans and executes trajectories to predefined configurations (defined in the srdf file which is placed on the `config` folder). For the robot to move you must uncomment the required lines in the script and then run in another terminal:

```bash
source ~/catkin_ws/devel/setup.bash
# The function '.go' is blocking, trajectory execution will only stop after reaching the target
rosrun motoman_hc10_moveit_config execute_trajectory.py
```
**Note:** Make sure to always check the planned path before executing any trajectory and add any new obstacles to the `add_interferences.py` script if need be.

## Reading the sensor registers

The ROS-Industrial Simple Message protocol does not have an ID (yet) for sending and receiving effort data. Even so, we can acces this information by reading some of the controller registers. For this purpose, we use the Motoman specific IDs [REP-I0004](https://github.com/ros-industrial/rep/blob/master/rep-I0004.rst). Please, check the documentation on this repository to understand how this can be done from a ROS perspective as well as a controller perspective.

By default, some registers are already being read. You can disable this option by commenting the call to the `io_relay` node in the `robot_interface_streaming.launch` file of the `motoman_driver` package.

Otherwise, you can see the data being published with the **rqt_console**. Check the `/joint_effort` , `/joint_vitesse`, `/joint_position` topics.

***Note:*** *The addresses being read using ROS must match the addresses defined on the controller side. These addresses may change depending on the robot controller. So, refer also to the Yaskawa Motoman documentation on IO addressing and configuration.*
