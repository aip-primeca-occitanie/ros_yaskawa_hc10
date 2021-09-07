#! /usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory',anonymous=True)

#Misc variables
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

#Group states taken from the srdf file
group_state = group.get_named_targets()

#Planning and executing with set_joint_value_target
print "Number of group states in srdf file: %i \n" % len(group_state)
for n in range(0,len(group_state)): #Home configuration (i.e 0 position) is a singularity 
    if not rospy.is_shutdown():
        print "group state %i: %s" %(n,group_state[n])
        print "Joint Values %s" %group.get_named_target_values(group_state[n])
        group.set_joint_value_target(group.get_named_target_values(group_state[n]))
        print "New target has been set"
        plan2 = group.plan()
        rospy.sleep(1)
#If you want to move the group to the specified target uncomment the lines below
#        print "Plannig done, now executing \n"
#        group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
#        group.stop()
#        rospy.sleep(1)

        
"""
#Planning and executing with set_named_target
print "Number of group states in srdf file: %i \n" % len(group_state)
for n in range(1,len(group_state)):
    print "group state %i: %s" %(n,group_state[n])
    print "Joint Values %s" %group.get_named_target_values(group_state[n])
    group.set_named_target(group_state[n])
    print "New target has been set"
    plan2 = group.plan()
    rospy.sleep(1)
    print "Plannig done, now executing \n"
    group.go(wait=True)
    group.stop()
    rospy.sleep(1)

#Example 1: Other ways to move your group
#You must take into account the number of joints
joint_values = group.get_current_joint_values()
#Note that the values must be in radians
joint_values[0] = 0
joint_values[1] = 0
joint_values[2] = 0
joint_values[3] = 0
joint_values[4] = -1.57
joint_values[5] = 0
group.set_joint_value_target(joint_values)
plan2 = group.plan()
#group.go(wait=True)

#Example 2: Other ways to move your group
group.set_joint_value_target(group.get_named_target_values(group_state[2]))
plan2 = group.plan()
#group.go(wait=True)

group.set_joint_value_target(group.get_named_target_values(group_state[3]))
plan2 = group.plan()
#group.go(wait=True)

group.set_joint_value_target(group.get_named_target_values(group_state[4]))
plan2 = group.plan()
#group.go(wait=True)

"""
moveit_commander.roscpp_shutdown()







