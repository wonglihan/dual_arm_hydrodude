#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.02
joint_goal[1] = 0.1923
joint_goal[2] = 0.7686
joint_goal[3] = 0.0869
joint_goal[4] = 0.6827
group.go(joint_goal, wait=True)

joint_goal[0] = -0.1
joint_goal[1] = 0.1923
joint_goal[2] = 0.7686
joint_goal[3] = 0.0869
joint_goal[4] = 0.6827
group.go(joint_goal, wait=True)

joint_goal[0] = -0.02
joint_goal[1] = 0.1923
joint_goal[2] = 0.7686
joint_goal[3] = 0.0869
joint_goal[4] = 0.6827
group.go(joint_goal, wait=True)
#home
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 1.571
joint_goal[3] = -2.356
joint_goal[4] = 0.785
group.go(joint_goal, wait=True)
group.stop()

moveit_commander.roscpp_shutdown()

