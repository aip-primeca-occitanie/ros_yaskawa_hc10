#! /usr/bin/env python
from os import wait
import sys
import argparse
import rospy
import moveit_commander
import moveit_msgs.msg
import csv
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryExecutor():
    def __init__(self, argv):
        self.current_joint_position = [0,0,0,0,0,0]
        moveit_commander.roscpp_initialize(argv)
        rospy.init_node('execute_trajectory',anonymous=True)

        rospy.Subscriber("joint_states", JointState, self.callback)
        self.pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=10)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("DN2P1") # "manipulator"
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        self.joint_names = ["yaska1_joint_1_s","yaska1_joint_2_l","yaska1_joint_3_u","yaska1_joint_4_r","yaska1_joint_5_b","yaska1_joint_6_t"]

    def callback(self, data):
        """Update the current joint position."""
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
        self.current_joint_position = data.position

    def is_current_init(self, initial_position):
        for i in range(len(initial_position)):
            if ((initial_position[i]-self.current_joint_position[i])>0.01):
                return False
        return True
    
    def read_trajectory(self, filename: str, Ts: float = 0.1) -> JointTrajectory:
        """Read a trajectory file and return a JointTrajectory."""
        print("Reading trajectory : ", filename)
        # create a new trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        # read the points in the file and add them to the JointTrajectory
        with open(filename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 1
            for row in csv_reader:
                point = JointTrajectoryPoint()
                point.positions = list(map(float,row))
                point.velocities = [0,0,0,0,0,0]
                point.time_from_start = rospy.Duration(Ts*line_count)
                trajectory.points.append(point)
                line_count += 1
        return trajectory

    def execute_trajectory(self, filename: str = None):
        """Read a trajectory file and execute it."""

        if filename is None:
            filename = '../trajectories/trajectory.csv'

        trajectory = self.read_trajectory(filename)

        # compare initial position to current position
        initial_position = trajectory.points[0].positions
        init_ok = self.is_current_init(initial_position) # is the first point of the trajectory (almost) equal tu current point?
        print ("Initial position : ", initial_position)

        # if necessary, move to initial position
        if (init_ok == False):
            print("First, going to initial position via moveit")
            self.group.set_joint_value_target(initial_position)
            self.group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
            self.group.stop()
            rospy.sleep(3)
        
        # adding current position at the start of the trajectory
        point_current = JointTrajectoryPoint()
        point_current.positions = list(self.current_joint_position)
        point_current.velocities = [0,0,0,0,0,0]
        point_current.time_from_start = rospy.Duration(0)
        trajectory.points.insert(0, point_current)

        # populate header
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.seq = 1

        # publish trajectory to execute the movement
        self.pub.publish(trajectory)

        print (trajectory)


    def plan_and_execute(self, filename: str = None):
        """NOT TESTED! Planning and executing with set_joint_value_target."""
        if filename is None:
            filename = '../trajectories/trajectory.csv'

        trajectory = self.read_trajectory(filename)
        for n in range(len(trajectory)):
            if not rospy.is_shutdown():
                group.set_joint_value_target(trajectory[n])
                print("New target has been set")
                #plan2 = group.plan()
                print("Plannig done, now executing \n")
                group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
                group.stop()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":

    # parse command line arguments to get path file
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename")
    known_args, remaining_args = parser.parse_known_args()

    trajExecutor = TrajectoryExecutor(remaining_args)
    trajExecutor.execute_trajectory(filename=known_args.filename)
