#! /usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory',anonymous=True)
current_joint_position = [0,0,0,0,0,0]


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    global current_joint_position
    current_joint_position = data.position
rospy.Subscriber("joint_states", JointState, callback)
pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=10)

def is_current_init(initial_position):
    global current_joint_position
    for i in range(len(initial_position)):
        if ((initial_position[i]-current_joint_position[i])>0.01):
            return False
    return True
    

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("DN2P1") # "manipulator"
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
                                               
                                               
                                               
trajectory = JointTrajectory()
trajectory.joint_names = ["yaska1_joint_1_s","yaska1_joint_2_l","yaska1_joint_3_u","yaska1_joint_4_r","yaska1_joint_5_b","yaska1_joint_6_t"]


#TODO: clean this file
#TODO: get trajectory file path from user input
traj_file_path = '../trajectories/trajectory.csv'

with open(traj_file_path) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 1
    Ts = 0.1
    for row in csv_reader:
        point = JointTrajectoryPoint()
        point.positions = list(map(float,row))
        point.velocities = [0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(Ts*line_count)
        trajectory.points.append(point)
        #trajectory.append(list(map(float,row)))
        #print(row)
        line_count += 1
            
initial_position = trajectory.points[0].positions
init_ok = is_current_init(initial_position) # is the first point of the trajectory (almost) equal tu current point?
print (initial_position)
if (init_ok == False):
    print("First going to initial position via moveit")
    group.set_joint_value_target(initial_position)
    group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
    group.stop()
    rospy.sleep(3)

point_current = JointTrajectoryPoint()
point_current.positions = list(current_joint_position)
point_current.velocities = [0,0,0,0,0,0]
point_current.time_from_start = rospy.Duration(0)
trajectory.points.insert(0, point_current)

#print (len(trajectory))
trajectory.header.stamp = rospy.Time.now()
trajectory.header.seq = 1
pub.publish(trajectory)
print (trajectory)


#Planning and executing with set_joint_value_target

"""for n in range(len(trajectory)):
    if not rospy.is_shutdown():
        group.set_joint_value_target(trajectory[n])
        print("New target has been set")
        #plan2 = group.plan()
        print("Plannig done, now executing \n")
        group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
        group.stop()

moveit_commander.roscpp_shutdown()
"""






