#! /bin/python3

from msilib.schema import Class
import sys
import copy

from tomlkit import string
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int8MultiArray
from rob8.msg import boxes
from moveit_commander.conversions import pose_to_list

class RosPlanner:
    def __init__(self) -> None:
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.boxes = []
        self.plans = []
        self.vis_paths = []

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

        self.boxes_subscriber = rospy.Subscriber("/boxes", boxes, self.incoming_box)
        self.vis_mode_subscriber = rospy.Subscriber("/vis_num", Int8MultiArray, self.change_mode)

    def run(self):
        rospy.spin()
        
        # while not rospy.is_shutdown():
        #     old_robot_state = copy.deepcopy(self.robot.get_current_state())

        #     plan = self.group.plan()

        #     new_state = copy.deepcopy(old_robot_state)
        #     print(type(new_state.joint_state.position))
        #     new_state.joint_state.position = (0, 0, 0, 0, 0, 0)

        #     self.group.set_start_state(new_state)
        #     self.group.set_joint_value_target(old_robot_state.joint_state.position)
        #     plan2 = self.group.plan()

        #     print(plan)
        #     print("============= HERE =============")
        #     print(plan2)
        #     print("============= HERE2 =============")

        #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #     display_trajectory.trajectory_start = self.robot.get_current_state()
        #     display_trajectory.trajectory.append(plan)
        #     display_trajectory.trajectory.append(plan2)
        #     # Publish
        #     self.display_trajectory_publisher.publish(display_trajectory)

    def incoming_box(self, msg):
        if (msg.boxid > len(self.boxes)):
            self.boxes.append((msg.boxid, msg.pose, msg.type))
        else:
            self.boxes[msg.boxid] = (msg.boxid, msg.pose, msg.type)

        self.calculate_trajectory(msg.boxid)

    def change_mode(self, msg):
        if len(msg.data) == 0:
            self.vis_paths = []
        elif len(msg.data) == 1:
            if msg.data[0] == 0:
                self.vis_paths = [0, 1]
            elif msg.data[0] == len(self.boxes):
                self.vis_paths = [self.boxes - 1, self.boxes]
            else:
                self.vis_paths = [msg.data[0] - 1, msg.data[0], msg.data[0] + 1]
        else:
            #Implement multiple paths, ie 0-1 and 3-4 and such
            pass

    def calculate_trajectory(self, id):
        if len(self.boxes) == 1:
            old_robot_state = copy.deepcopy(self.robot.get_current_state())

            self.group.set_start_state(old_robot_state)
            self.group.set_pose_target(self.boxes[0][1])

            plan2 = self.group.plan()
            self.plans = [copy.deepcopy(plan2)]

        elif len(self.boxes) > 1:
            old_robot_state = copy.deepcopy(self.robot.get_current_state())

            self.group.set_start_state(old_robot_state)
            self.group.set_pose_target(self.boxes[0][1])

            plan2 = self.group.plan()
            self.plans = []

            for i in range(1, len(self.boxes)):
                old_robot_state = plan2.joint_trajectory.points[-1].positions

                self.group.set_start_state(old_robot_state)
                self.group.set_pose_target(self.boxes[i][1])

                plan = self.group.plan()
                self.plans.append(copy.deepcopy(plan))

            self.group.set_start_state(self.plans[-1].joint_trajectory.points[-1].positions)
            self.group.set_pose_target(self.boxes[0][1])

            plan2 = self.group.plan()
            self.plans = [copy.deepcopy(plan2)]

        self.update_visualizer()


    def update_visualizer(self):
        if len(self.vis_paths) == 0:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.plans[0].joint_trajectory.points[-1].positions

            for plan in self.plans:
                display_trajectory.trajectory.append(plan)

            self.display_trajectory_publisher.publish(display_trajectory)
        else:
            #Implement fragmented path showing
            pass


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_path_visualizer', anonymous=True)
    planner = RosPlanner()
    planner.run()

    
