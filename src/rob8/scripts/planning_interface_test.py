#! /bin/python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    old_robot_state = copy.deepcopy(robot.get_current_state())

    plan = group.plan()

    new_state = copy.deepcopy(old_robot_state)
    print(type(new_state.joint_state.position))
    new_state.joint_state.position = (0, 0, 0, 0, 0, 0)

    group.set_start_state(new_state)
    group.set_joint_value_target(old_robot_state.joint_state.position)
    plan2 = group.plan()

    print(plan)
    print("============= HERE =============")
    print(plan2)
    print("============= HERE2 =============")

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory.trajectory.append(plan2)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    
