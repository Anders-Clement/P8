#! /bin/python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from rob8.msg import boxes
from moveit_commander.conversions import pose_to_list

class display_object:
    def __init__(self, pose, scale, type, move_group, robot, previouse_pose) -> None:
        self.goto_plan = None
        self.plans = []
        self.pose = pose #This pose in cartesian space
        self.type = type
        self.scale = scale
        self.previouse_pose = previouse_pose #Previosue pose is in joint_state
        #Moveit commander parameters
        self.move_group_ref = move_group
        self.robot_ref = robot

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y + self.scale[1] / 2
        new_pose.position.z = self.pose.position.z + self.scale[2] + 0.05
        self.pose_above_box = new_pose

        self.calculate_plans()

    def calculate_plans(self):
        #Position above the virtual box
        old_robot_state = copy.deepcopy(self.previouse_pose)
        self.move_group_ref.set_start_state(old_robot_state)
        self.move_group_ref.set_pose_target(self.pose_above_box)
        plan2 = self.move_group_ref.plan()
        self.goto_plan = plan2[1]

    def update_previouse_pose(self, pose):
        self.previouse_pose = pose
        self.calculate_plans()

    def update_pose(self, pose, scale):
        self.pose = pose
        self.scale = scale

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y + self.scale[1] / 2
        new_pose.position.z = self.pose.position.z + self.scale[2] + 0.05
        self.pose_above_box = new_pose

        self.calculate_plans()

    def update_type(self, type):
        self.type = type

    def close_gripper(self, close_true):
        pass

    def calculate_action_plan(self, object_pose):
        if self.type == "pickup":
            #Pickup cube
            self.move_group_ref.set_start_state(self.robot_ref.get_current_state())
            new_pose = copy.deepcopy(object_pose)
            new_pose.position.z = self.pose.position.z + 0.2
            self.move_group_ref.set_pose_target(new_pose)
            plan2 = self.move_group_ref.plan()
            self.plans.append(plan2[1])

            self.plans.append((False))

            self.move_group_ref.set_start_state(plan2[1].joint_trajectory.points[-1].positions)
            new_pose = copy.deepcopy(object_pose)
            new_pose.position.z = self.pose.position.z + 0.1
            self.move_group_ref.set_pose_target(new_pose)
            plan3 = self.move_group_ref.plan()
            self.plans.append(plan3[1])

            self.plans.append((True))

            self.move_group_ref.set_start_state(plan3[1].joint_trajectory.points[-1].positions)
            self.move_group_ref.set_pose_target(self.pose_above_box)
            plan4 = self.move_group_ref.plan()
            self.plans.append(plan4[1])

        elif self.type == "place":
            self.plans.append((False))

    def execute(self, target_object):
        self.move_group_ref.execute(plan_msg=self.goto_plan, wait=True)

        for plan in self.plans:
            if len(plan) > 1:
                self.calculate_action_plan(target_object)
                self.move_group_ref.execute(plan_msg=plan, wait=True)
            elif len(plan) == 1:
                self.move_group_ref.stop()
                self.close_gripper(plan[0])

    def publish_cube(self, vis_pub, id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time()
        marker.ns = ""
        marker.id = id
        marker.type = 1
        marker.action = 0

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y + self.scale[1] / 2

        marker.pose = new_pose
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        vis_pub.publish( marker )

class RosPlanner:
    def __init__(self) -> None:
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.boxes = []
        self.vis_paths = []
        self.pickup_box_pose = None

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

        self.boxes_subscriber = rospy.Subscriber("/boxes", boxes, self.incoming_box)
        self.vis_mode_subscriber = rospy.Subscriber("/vis_num", Int8MultiArray, self.change_mode)
        self.physical_box = rospy.Subscriber("/physicalbox", PoseStamped, self.incoming_box_pose)
        self.vis_pub = rospy.Publisher( "visualization_marker", Marker, queue_size=10)

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
        if (msg.boxid >= len(self.boxes)):
            previouse_pose = self.robot.get_current_state() #Joint space

            if msg.boxid != 0:
                previouse_pose.joint_state.position = self.boxes[msg.boxid - 1].goto_plan.joint_trajectory.points[-1].positions #Joint space

            self.boxes.append(display_object(msg.pose, (msg.scalex, msg.scaley, msg.scalez), msg.type, self.group, self.robot, previouse_pose))
        else:
            self.boxes[msg.boxid].update_pose(msg.pose, (msg.scalex, msg.scaley, msg.scalez)) #Cartesian space
            if len(self.boxes) > msg.boxid + 1:
                self.boxes[msg.boxid + 1].update_previouse_pose(self.boxes[msg.boxid].goto_plan.points[-1].positions)

        if self.boxes[msg.boxid].type != msg.type:
            self.boxes[msg.boxid].update_type(msg.type)

        self.update_visualizer()

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

    def update_visualizer(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()

        for id, box in enumerate(self.boxes):
            display_trajectory.trajectory.append(box.goto_plan)
            box.publish_cube(self.vis_pub, id )

        self.display_trajectory_publisher.publish(display_trajectory)


    def incoming_box_pose(self, msg):
        self.pickup_box_pose = msg.pose

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_path_visualizer')
    planner = RosPlanner()
    planner.run()

    
