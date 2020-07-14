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

planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
print ""
#harvesting trajectory for left arm
joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.1
joint_goal[1] = 0.2671
joint_goal[2] = 1.5657
joint_goal[3] = -0.4708
joint_goal[4] = -1.0949
group.go(joint_goal, wait=True)

joint_goal[0] = -0.1
joint_goal[1] = 0.2083
joint_goal[2] = 1.2197
joint_goal[3] = 0.0929
joint_goal[4] = -1.3126
group.go(joint_goal, wait=True)

joint_goal[0] = -0.05
joint_goal[1] = 0.2030
joint_goal[2] = 1.5265
joint_goal[3] = -2.3317
joint_goal[4] = 0.8052
group.go(joint_goal, wait=True)

joint_goal[0] = 0
joint_goal[1] = 0.0004
joint_goal[2] = 1.2677
joint_goal[3] = -2.4956
joint_goal[4] = 1.2278
group.go(joint_goal, wait=True)

joint_goal[0] = 0
joint_goal[1] = 0.0004
joint_goal[2] = 0.5244
joint_goal[3] = -0.8947
joint_goal[4] = 0.3704
group.go(joint_goal, wait=True)

joint_goal[0] = 0
joint_goal[1] = 0.0004
joint_goal[2] = 1.2677
joint_goal[3] = -2.4956
joint_goal[4] = 1.2278
group.go(joint_goal, wait=True)
#home
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 1.571
joint_goal[3] = -2.356
joint_goal[4] = 0.785
group.go(joint_goal, wait=True)
group.stop()

#planting trajectory for right arm
group = moveit_commander.MoveGroupCommander("right_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.0004
joint_goal[1] = 1.2677
joint_goal[2] = -2.4956
joint_goal[3] = 1.2278
group.go(joint_goal, wait=True)
#extending into storage
joint_goal[0] = -0.0004
joint_goal[1] = 0.5244
joint_goal[2] = -0.8947
joint_goal[3] = 0.3704
group.go(joint_goal, wait=True)

joint_goal[0] = -0.0004
joint_goal[1] = 1.2677
joint_goal[2] = -2.4956
joint_goal[3] = 1.2278
group.go(joint_goal, wait=True)
#middle
joint_goal[0] = -0.2030
joint_goal[1] = 1.5265
joint_goal[2] = -2.3317
joint_goal[3] = 0.8052
group.go(joint_goal, wait=True)
#above planting hole
joint_goal[0] = -0.75
joint_goal[1] = 1.221
joint_goal[2] = -0.7407
joint_goal[3] = -0.4804
group.go(joint_goal, wait=True)
group.stop()

#switch to left arm controller to move prisma joint
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.1
joint_goal[1] = 0
joint_goal[2] = 1.571
joint_goal[3] = -2.356
joint_goal[4] = 0.785
group.go(joint_goal, wait=True)

joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 1.571
joint_goal[3] = -2.356
joint_goal[4] = 0.785
group.go(joint_goal, wait=True)
group.stop()

#switch back to right arm and home
group = moveit_commander.MoveGroupCommander("right_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 1.571
joint_goal[2] = -2.356
joint_goal[3] = 0.785
group.go(joint_goal, wait=True)
group.stop()

moveit_commander.roscpp_shutdown()
