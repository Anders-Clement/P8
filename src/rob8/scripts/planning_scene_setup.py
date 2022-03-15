#!/usr/bin/env python3

import sys

import rospy
import moveit_commander
import geometry_msgs

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("planning_scene_setup", anonymous=True)

    # robot = moveit_commander.RobotCommander()
    # group_name = "panda_arm"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = -0.0002
    box_name = "floor"
    scene.add_box(box_name, box_pose, size=(100, 100, 0.0001))
    rospy.sleep(2)