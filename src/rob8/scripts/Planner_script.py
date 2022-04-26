#! /bin/python3

import enum
from logging import exception
import sys
import copy
import threading
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rob8.msg import Boxes, ExecutepathAction, ExecutepathActionFeedback, ExecutepathActionGoal, \
    ExecutepathActionResult, BoxResponse
from rob8.msg import ExecutespecificAction, ExecutespecificActionGoal, ExecutespecificFeedback, ExecutespecificResult
import actionlib
from moveit_commander.conversions import pose_to_list
from threading import Thread
import moveit_msgs.msg
from std_msgs.msg import Float64, Int16
from ur_msgs.srv import SetIO, SetIORequest, SetIOResponse

import tf2_ros
import tf2_geometry_msgs


class display_object:
    def __init__(self, pose, scale, type_s, move_group, robot, previouse_pose) -> None:
        self.goto_plan = None
        self.last_box_plan = None

        self.plans = []
        self.pose = pose  # This pose in cartesian space
        self.type = type_s
        self.scale = scale
        self.previouse_pose = previouse_pose  # Previosue pose is in joint_state
        # Moveit commander parameters
        self.move_group_ref = move_group
        self.robot_ref = robot
        self.z_offset = 0.3

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x  # + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y  # + self.scale[1] / 2
        new_pose.position.z = self.pose.position.z + self.scale[2] / 2
        if self.type[1] == 'i' or self.type[1] == 'l':
            new_pose.position.z += self.z_offset
        new_pose.orientation.x = 1
        new_pose.orientation.y = 0
        new_pose.orientation.z = 0
        new_pose.orientation.w = 0
        self.pose_above_box = new_pose

    def calculate_plans(self):
        # Position above the virtual box
        old_robot_state = copy.deepcopy(self.previouse_pose)
        self.move_group_ref.set_start_state(old_robot_state)
        self.move_group_ref.set_pose_target(self.pose_above_box)

        print("Velocity scaling set to 1")
        self.move_group_ref.set_max_acceleration_scaling_factor(0.1)
        self.move_group_ref.set_max_velocity_scaling_factor(0.1)

        # start_time = time.time()
        plan2 = self.move_group_ref.plan()
        # print(time.time() - start_time)

        self.goto_plan = None
        if plan2[0]:
            self.goto_plan = plan2[1]

    def update_previouse_pose(self, pose):
        self.previouse_pose = pose

    def update_pose(self, pose, scale):
        self.pose = pose
        self.scale = scale

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x  # + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y  # + self.scale[1] / 2
        new_pose.position.z = self.pose.position.z + self.scale[2] / 2
        if self.type[1] == 'i' or self.type[1] == 'l':
            new_pose.position.z += self.z_offset
        new_pose.orientation.x = 1
        new_pose.orientation.y = 0
        new_pose.orientation.z = 0
        new_pose.orientation.w = 0
        self.pose_above_box = new_pose

        self.goto_plan = None
        self.last_box_plan = None

    def update_type(self, type):
        self.type = type

    def close_gripper(self, close_true):
        time.sleep(1)
        return
        rospy.wait_for_service("/ur_hardware_interface/set_io", timeout=10)
        set_io_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        msg = SetIORequest()
        msg.fun = 1
        msg.pin = 16
        msg.state = int(close_true)
        set_io_srv(msg)
        time.sleep(1)

    def calculate_action_plan(self, object_pose, publisher):
        if self.type == "pickup" or self.type[1] == 'i':
            if object_pose is None:
                return

            # Pickup cube
            starting_pose = self.robot_ref.get_current_state()  # Joint space, beginning pose

            self.move_group_ref.set_start_state_to_current_state()
            new_pose2 = copy.deepcopy(object_pose)
            new_pose2.position.z = new_pose2.position.z + self.z_offset
            waypoints = [copy.deepcopy(new_pose2)]
            (plan1, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            starting_pose.joint_state.name = self.goto_plan.joint_trajectory.joint_names
            starting_pose.joint_state.position = plan1.joint_trajectory.points[-1].positions
            self.move_group_ref.set_start_state(starting_pose)
            waypoints = []
            new_pose = copy.deepcopy(object_pose)
            new_pose.position.z = new_pose.position.z + self.z_offset - 0.05
            waypoints.append(copy.deepcopy(new_pose))
            (plan2, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            starting_pose.joint_state.name = self.goto_plan.joint_trajectory.joint_names
            starting_pose.joint_state.position = plan2.joint_trajectory.points[-1].positions
            self.move_group_ref.set_start_state(starting_pose)
            waypoints = [copy.deepcopy(new_pose2)]
            (plan3, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            starting_pose.joint_state.name = self.goto_plan.joint_trajectory.joint_names
            starting_pose.joint_state.position = plan3.joint_trajectory.points[-1].positions
            self.move_group_ref.set_start_state(starting_pose)
            waypoints = [self.pose_above_box]
            (plan4, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot_ref.get_current_state()
            display_trajectory.trajectory.append(plan1)
            display_trajectory.trajectory.append(plan2)
            display_trajectory.trajectory.append(plan3)
            display_trajectory.trajectory.append(plan4)
            publisher.publish(display_trajectory)

            print("Go above detected box")
            self.move_group_ref.execute(plan1, wait=True)
            self.close_gripper(False)
            print("Go straight down to the box")
            self.move_group_ref.execute(plan2, wait=True)
            self.close_gripper(True)
            print("Go up")
            self.move_group_ref.execute(plan3, wait=True)
            print("Goto start position")
            self.move_group_ref.execute(plan4, wait=True)

        elif self.type == "place":
            self.move_group_ref.set_start_state_to_current_state()
            waypoints = []
            new_pose = copy.deepcopy(self.pose_above_box)
            new_pose.position.z = new_pose.position.z - (self.scale.z / 2.0)
            waypoints.append(copy.deepcopy(new_pose))
            (plan1, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            self.move_group_ref.execute(plan1, wait=True)

            self.move_group_ref.set_start_state_to_current_state()
            waypoints = []
            new_pose = copy.deepcopy(self.pose_above_box)
            waypoints.append(copy.deepcopy(new_pose))
            (plan4, fraction) = self.move_group_ref.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            self.close_gripper(False)

            self.move_group_ref.execute(plan4, wait=True)

    def execute(self, target_object, publisher):
        print("Executing goto plan")
        self.move_group_ref.execute(plan_msg=self.goto_plan, wait=True)
        print("Executing action plan")
        self.calculate_action_plan(target_object, publisher)

        if self.last_box_plan is not None:
            self.move_group_ref.execute(plan_msg=self.last_box_plan, wait=True)

    def last_box(self, first_box):
        previouse_pose = self.robot_ref.get_current_state()  # Joint space
        if self.goto_plan is None:
            return
        if len(self.goto_plan.joint_trajectory.points) == 0:
            self.last_box_plan = None
            return

        previouse_pose.joint_state.name = self.goto_plan.joint_trajectory.joint_names
        previouse_pose.joint_state.position = self.goto_plan.joint_trajectory.points[-1].positions

        self.move_group_ref.set_start_state(previouse_pose)
        self.move_group_ref.set_joint_value_target(first_box.joint_state.position[:6])

        plan3 = self.move_group_ref.plan()
        self.last_box_plan = None
        if plan3[0]:
            self.last_box_plan = plan3[1]

    def publish_cube(self, vis_pub, id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time()
        marker.ns = ""
        marker.id = id
        marker.type = 1
        marker.action = 0
        marker.lifetime = rospy.Duration(30)

        new_pose = copy.deepcopy(self.pose)
        # new_pose.position.x = self.pose.position.x + self.scale[0] / 2
        # new_pose.position.y = self.pose.position.y + self.scale[1] / 2

        marker.pose = new_pose
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker
        # vis_pub.publish( marker )


class RosPlanner:
    def __init__(self) -> None:
        # Moveit commander interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id("RRTConnect")
        self.group.set_planning_time(5)
        self.group.set_num_planning_attempts(1000)
        self.group.allow_replanning(False)
        # Variables for keeping track of boxes
        self.boxes = []
        self.vis_paths = []
        self.pickup_box_pose = None
        # Action server variables
        self.executing_boxid = 0
        self.executing_finished = False
        self.executing_thread = None
        self.feedback_msg = ExecutepathActionFeedback()
        self.result_msg = ExecutepathActionResult()
        self.result_msg2 = ExecutespecificResult()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.replan = False
        replan_seconds = 5.0
        self.replan_time = rospy.Rate(1.0 / replan_seconds)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        self.boxes_subscriber = rospy.Subscriber("/boxes", Boxes, self.incoming_box, queue_size=10)
        self.vis_mode_subscriber = rospy.Subscriber("/vis_num", Int8MultiArray, self.change_mode)
        self.physical_box = rospy.Subscriber("/detection", PoseStamped, self.incoming_box_pose, queue_size=1)
        self.vis_pub = rospy.Publisher("visualization_markers", MarkerArray, queue_size=10)
        self.box_res_pub = rospy.Publisher("/box_planner_status", BoxResponse, queue_size=10)
        self.scene_change_sub = rospy.Subscriber("/scene/change", Int16, self.scene_change, queue_size=2)

        self.action_server = actionlib.SimpleActionServer("planner_executer", ExecutepathAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

        self.action_server2 = actionlib.SimpleActionServer("planner_give_executer", ExecutespecificAction,
                                                           execute_cb=self.execute_cb2, auto_start=False)
        self.action_server2.start()

        self.starting_pose = self.robot.get_current_state()  # Joint space, beginning pose

        # construct a message
        joint_constraint2 = moveit_msgs.msg.JointConstraint()
        joint_constraint2.joint_name = self.group.get_joints()[4]
        print(joint_constraint2.joint_name)
        joint_constraint2.position = -3.14 / 2.0
        joint_constraint2.tolerance_above = 3.14 / 2.0
        joint_constraint2.tolerance_below = 3.14 / 2.0
        joint_constraint2.weight = 1

        # construct a message
        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = self.group.get_joints()[1]
        print(joint_constraint.joint_name)
        joint_constraint.position = 3.14 / 4.0
        joint_constraint.tolerance_above = 3.14 / 2.0
        joint_constraint.tolerance_below = 3.14 / 2.0
        joint_constraint.weight = 1.0

        # construct a message
        joint_constraint3 = moveit_msgs.msg.JointConstraint()
        joint_constraint3.joint_name = self.group.get_joints()[2]
        print(joint_constraint3.joint_name)
        joint_constraint3.position = -3.14 / 2.0
        joint_constraint3.tolerance_above = 3.14 / 3.0
        joint_constraint3.tolerance_below = 3.14 / 3.0
        joint_constraint3.weight = 1.0

        joint_constraint5 = moveit_msgs.msg.JointConstraint()
        joint_constraint5.joint_name = self.group.get_joints()[5]
        print(joint_constraint5.joint_name)
        joint_constraint5.position = 0
        joint_constraint5.tolerance_above = 3.14
        joint_constraint5.tolerance_below = 3.14
        joint_constraint5.weight = 1.0

        joint_constraint_list = [joint_constraint, joint_constraint2, joint_constraint3, joint_constraint5]

        constraint_list = moveit_msgs.msg.Constraints()
        constraint_list.name = 'middle_of_travel'
        constraint_list.joint_constraints = joint_constraint_list

        self.group.set_path_constraints(constraint_list)

    def scene_change(self, msg):
        self.boxes = []
        rospy.logwarn("Clearing box buffer")

    def execute_cb2(self, goal):
        my_rate = rospy.Rate(10)
        boxid = int(goal.action_string.split(':')[0])
        print(boxid)
        if int(goal.action_string.split(':')[1]) == 0:
            self.group.execute(plan_msg=self.boxes[boxid].goto_plan, wait=True)

        elif int(goal.action_string.split(':')[1]) == 2:
            self.group.execute(plan_msg=self.boxes[boxid].last_box_plan, wait=True)

        else:
            if self.pickup_box_pose is not None:
                self.boxes[boxid].calculate_action_plan(self.pickup_box_pose, self.display_trajectory_publisher)

        self.result_msg2.succededTrue = False
        self.action_server2.set_succeeded(self.result_msg2)

    def execute_cb(self, goal):
        self.executing_finished = False
        my_rate = rospy.Rate(10)
        for box in self.boxes:
            if box.goto_plan is None:
                self.result_msg.result.succededTrue = False
                self.action_server.set_succeeded(self.result_msg.result)
                return

        if self.boxes[-1].last_box_plan is None:
            self.result_msg.result.succededTrue = False
            self.action_server.set_succeeded(self.result_msg.result)
            return

        while not rospy.is_shutdown():
            if goal.runpath:
                if self.executing_thread is None:
                    self.executing_thread = threading.Thread(target=self.execute_plan, daemon=True)
                    self.executing_thread.start()
                    # print("Creating moveit commander Thread")
                elif not self.executing_thread.is_alive():
                    self.result_msg.result.succededTrue = False
                    self.action_server.set_succeeded(self.result_msg.result)
                    break

                if self.executing_finished:
                    self.executing_thread = None
                    self.result_msg.result.succededTrue = True
                    self.action_server.set_succeeded(self.result_msg.result)
                    break
                    # print("Succeded")
                else:
                    self.feedback_msg.feedback.currentPath = self.executing_boxid
                    self.action_server.publish_feedback(self.feedback_msg.feedback)
                    # print("Feedbacking")
            else:
                print("Havent implemented replanning yet")

            my_rate.sleep()

        self.executing_thread = None

    def execute_plan(self):
        self.executing_finished = False
        object_counter = 0
        while not rospy.is_shutdown():
            if object_counter >= len(self.boxes):
                # print(len(self.boxes))
                print("Exiting after box {}".format(object_counter))
                break
            self.executing_boxid = object_counter
            print("Executing box: {}".format(object_counter))

            self.boxes[object_counter].execute(self.pickup_box_pose, self.display_trajectory_publisher)
            object_counter += 1

        self.executing_finished = True

    def run(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.replan_time.sleep()
            if self.replan:
                for x in self.boxes:
                    x.goto_plan = None
                    self.replan = False
                    self.last_box_plan = False

            if len(self.boxes) <= 0:
                self.update_visualizer()
                print("Missing boxes")
                continue

            self.group.clear_pose_targets()

            try:

                # print("Planning")
                for id, box in enumerate(self.boxes):
                    if id != 0:
                        if self.boxes[id - 1].goto_plan is not None:
                            if len(self.boxes[id - 1].goto_plan.joint_trajectory.points) == 0:
                                continue
                            previouse_pose = self.robot.get_current_state()
                            previouse_pose.joint_state.name = self.boxes[id - 1].goto_plan.joint_trajectory.joint_names
                            previouse_pose.joint_state.position = self.boxes[id - 1].goto_plan.joint_trajectory.points[
                                -1].positions
                            box.update_previouse_pose(previouse_pose)
                            if box.goto_plan is None:
                                box.calculate_plans()
                    else:
                        if box.goto_plan is None:
                            box.calculate_plans()

                    if self.boxes[-1].last_box_plan is None:
                        self.boxes[-1].last_box(self.starting_pose)
            except Exception as e:
                print(e)

            self.update_visualizer()

    def incoming_box(self, msg):
        if msg.boxid + 1 < len(self.boxes):
            self.boxes[msg.boxid + 1].goto_Plan = None

        if msg.type.data == "delete":
            self.boxes.pop(msg.boxid)
            self.replan = True
            print("Got delete")
            # if len(self.boxes) > msg.boxid + 1:
            #     previouse_pose.joint_state.position = self.boxes[msg.boxid - 1].goto_plan.joint_trajectory.points[-1].positions
            #     self.boxes[msg.boxid].update_previouse_pose(previouse_pose)
            return

        if msg.boxid == len(self.boxes):
            # if msg.boxid != 0:
            #     previouse_pose.joint_state.position = self.boxes[msg.boxid - 1].goto_plan.joint_trajectory.points[-1].positions #Joint space

            self.boxes.append(
                display_object(msg.pose, (msg.scalex, msg.scaley, msg.scalez), msg.type.data, self.group, self.robot,
                               self.starting_pose))

            if msg.boxid > 0:
                self.boxes[msg.boxid - 1].last_box_plan = None

            print("Created box {}".format(msg.boxid))
            self.replan = True
        elif msg.boxid > len(self.boxes) - 1:
            print("Oliver fucked the order up")
            # Implement message buffer for out of order boxes, ie oliver fucked up the order
            return
        else:
            self.boxes[msg.boxid].update_pose(msg.pose, (msg.scalex, msg.scaley, msg.scalez))  # Cartesian space
            self.boxes[msg.boxid].goto_plan = None
            if len(self.boxes) > msg.boxid + 1:
                self.boxes[msg.boxid + 1].goto_plan = None
            # self.replan = True
            # if len(self.boxes) > msg.boxid + 1:
            #     previouse_pose.joint_state.position = self.boxes[msg.boxid].goto_plan.joint_trajectory.points[-1].positions
            #     self.boxes[msg.boxid + 1].update_previouse_pose(previouse_pose)

        if self.boxes[msg.boxid].type != msg.type.data:
            self.boxes[msg.boxid].update_type(msg.type.data)

        # if len(self.boxes) > 1:
        #     self.boxes[-2].last_box_plan = None

        # self.boxes[-1].last_box(self.starting_pose)

        print("Tracking boxes: {}".format(len(self.boxes)))
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
            # Implement multiple paths, ie 0-1 and 3-4 and such
            pass

    def update_visualizer(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.clear()

        marker_array = MarkerArray()

        for id, box in enumerate(self.boxes):
            marker_array.markers.append(box.publish_cube(self.vis_pub, id))

        self.vis_pub.publish(marker_array)

        try:
            for id, box in enumerate(self.boxes):
                if box.goto_plan is not None:
                    #print("Adding {}".format(id))
                    display_trajectory.trajectory.append(box.goto_plan)

            if self.boxes[-1].last_box_plan is not None:
                #print("Adding last plan {}".format(len(self.boxes)))
                display_trajectory.trajectory.append(self.boxes[-1].last_box_plan)

        except Exception as e:
            print(e)
            rospy.logwarn("Missing a plan, cannot visualise")

        self.display_trajectory_publisher.publish(display_trajectory)

    def check_scale(self, x, boxx, scalex):
        if boxx + scalex / 2.0 > x > boxx - scalex / 2.0:
            return True
        False

    def boundry_check(self, incoming_pose):
        return_bool = False
        for box in self.boxes:
            # Check x
            if self.check_scale(incoming_pose.position.x, box.pose.position.x, box.scale.x):
                if self.check_scale(incoming_pose.position.y, box.pose.position.y, box.scale.y):
                    return_bool = True

        return return_bool

    def incoming_box_pose(self, msg):
        # print(msg)
        transform = self.tf_buffer.lookup_transform("base_link",
                                                    # source frame:
                                                    msg.header.frame_id,
                                                    # get the tf at the time the pose was valid
                                                    msg.header.stamp,
                                                    # wait for at most 1 second for transform, otherwise throw
                                                    rospy.Duration(1.0))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
        if self.boundry_check(pose_transformed.pose):
            self.pickup_box_pose = pose_transformed.pose

            self.pickup_box_pose.orientation.x = 1
            self.pickup_box_pose.orientation.y = 0
            self.pickup_box_pose.orientation.z = 0
            self.pickup_box_pose.orientation.w = 0
        else:
            self.pickup_box_pose = None


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('trajectory_planner')
    planner = RosPlanner()
    planner.run()
