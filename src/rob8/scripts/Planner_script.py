#! /bin/python3

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
from visualization_msgs.msg import Marker
from rob8.msg import Boxes, ExecutepathAction, ExecutepathActionFeedback, ExecutepathActionGoal, ExecutepathActionResult
import actionlib
from moveit_commander.conversions import pose_to_list

class display_object:
    def __init__(self, pose, scale, type_s, move_group, robot, previouse_pose) -> None:
        self.goto_plan = None
        self.plans = []
        self.pose = pose #This pose in cartesian space
        self.type = type_s
        self.scale = scale
        self.previouse_pose = previouse_pose #Previosue pose is in joint_state
        #Moveit commander parameters
        self.move_group_ref = move_group
        self.robot_ref = robot
        self.last_box_plan = None

        new_pose = copy.deepcopy(self.pose)
        new_pose.position.x = self.pose.position.x + self.scale[0] / 2
        new_pose.position.y = self.pose.position.y + self.scale[1] / 2
        new_pose.position.z = self.pose.position.z + self.scale[2] + 0.05
        new_pose.orientation.x = 1
        new_pose.orientation.y = 0
        new_pose.orientation.z = 0
        new_pose.orientation.w = 0
        self.pose_above_box = new_pose

        self.calculate_plans()

    def calculate_plans(self):
        #Position above the virtual box
        old_robot_state = copy.deepcopy(self.previouse_pose)
        self.move_group_ref.set_start_state(old_robot_state)
        self.move_group_ref.set_pose_target(self.pose_above_box)

        print("Velocity scaling set to 1")
        self.move_group_ref.set_max_acceleration_scaling_factor(1)
        self.move_group_ref.set_max_velocity_scaling_factor(1)

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
        new_pose.orientation.x = 1
        new_pose.orientation.y = 0
        new_pose.orientation.z = 0
        new_pose.orientation.w = 0
        self.pose_above_box = new_pose

        self.calculate_plans()

    def update_type(self, type):
        self.type = type

    def close_gripper(self, close_true):
        pass

    def calculate_action_plan(self, object_pose):
        if self.type == "pickup":
            #Pickup cube
            new_pose = copy.deepcopy(object_pose)
            new_pose.position.z = self.pose.position.z + 0.2
            self.move_group_ref.set_pose_target(new_pose)
            self.move_group_ref.go()

            self.close_gripper(False)

            new_pose = copy.deepcopy(object_pose)
            new_pose.position.z = self.pose.position.z + 0.1
            self.move_group_ref.set_pose_target(new_pose)
            self.move_group_ref.go()

            self.close_gripper(True)

            self.move_group_ref.go(self.goto_plan.joint_trajectory.points[-1].positions, wait=True)

        elif self.type == "place":
            self.plans.append((False))

    def execute(self, target_object):
        print("Executing goto plan")
        self.move_group_ref.execute(plan_msg=self.goto_plan, wait=True)

        print("Executing action plan")
        self.calculate_action_plan(target_object)

        if self.last_box_plan is not None:
            self.move_group_ref.execute(plan_msg=self.last_box_plan, wait=True)

    def last_box(self, first_box):
        previouse_pose = self.robot_ref.get_current_state() #Joint space
        previouse_pose.joint_state.position = self.goto_plan.joint_trajectory.points[-1].positions

        self.move_group_ref.set_start_state(previouse_pose)
        self.move_group_ref.set_joint_value_target(first_box)
        
        plan3 = self.move_group_ref.plan()
        self.last_box_plan = plan3[1]

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
        #Moveit commander interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        #Variables for keeping track of boxes
        self.boxes = []
        self.vis_paths = []
        self.pickup_box_pose = None
        #Action server variables
        self.executing_boxid = 0
        self.executing_finished = False
        self.executing_thread = None
        self.feedback_msg = ExecutepathActionFeedback()
        self.result_msg = ExecutepathActionResult()

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

        self.boxes_subscriber = rospy.Subscriber("/boxes", Boxes, self.incoming_box)
        self.vis_mode_subscriber = rospy.Subscriber("/vis_num", Int8MultiArray, self.change_mode)
        self.physical_box = rospy.Subscriber("/physicalbox", PoseStamped, self.incoming_box_pose, queue_size=1)
        self.vis_pub = rospy.Publisher( "visualization_marker", Marker, queue_size=10)

        self.action_server = actionlib.SimpleActionServer("planner_executer", ExecutepathAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

    def execute_cb(self, goal):
        self.executing_finished = False
        my_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if goal.runpath == True:
                if self.executing_thread is None:
                    self.executing_thread = threading.Thread(target=self.execute_plan)
                    self.executing_thread.start()
                    #print("Creating moveit commander Thread")

                if self.executing_finished == True:
                    self.executing_thread = None
                    self.result_msg.result.succededTrue = True
                    self.action_server.set_succeeded(self.result_msg.result)
                    break
                    #print("Succeded")
                else:
                    self.feedback_msg.feedback.currentPath = self.executing_boxid
                    self.action_server.publish_feedback(self.feedback_msg.feedback)
                    #print("Feedbacking")
            else:
                print("Havent implemented replanning yet")

            my_rate.sleep()

        self.executing_thread = None

    def execute_plan(self):
        self.executing_finished = False
        object_counter = 0
        while not rospy.is_shutdown():
            if object_counter >= len(self.boxes):
                #print(len(self.boxes))
                print("Exiting after box {}".format(object_counter))
                break
            self.executing_boxid = object_counter
            print("Executing box: {}".format(object_counter))

            #=========== Testing Target =====
            target = self.boxes[object_counter].pose_above_box
            target.position.z = 0.0
            print("Using made up target")

            self.boxes[object_counter].execute(target)
            object_counter += 1

        self.executing_finished = True
            
    def run(self):
        rospy.spin()

    def incoming_box(self, msg):
        previouse_pose = self.robot.get_current_state() #Joint space
        if (msg.boxid == len(self.boxes)):
            if msg.boxid != 0:
                previouse_pose.joint_state.position = self.boxes[msg.boxid - 1].goto_plan.joint_trajectory.points[-1].positions #Joint space
            
            self.boxes.append(display_object(msg.pose, (msg.scalex, msg.scaley, msg.scalez), msg.type.data, self.group, self.robot, previouse_pose))
        elif (msg.boxid > len(self.boxes)):
            print("Oliver fucked the order up")
            #Implement message buffer for out of order boxes, ie oliver fucked up the order
            return
        else:
            self.boxes[msg.boxid].update_pose(msg.pose, (msg.scalex, msg.scaley, msg.scalez)) #Cartesian space
            if len(self.boxes) > msg.boxid + 1:
                previouse_pose.joint_state.position = self.boxes[msg.boxid].goto_plan.joint_trajectory.points[-1].positions
                self.boxes[msg.boxid + 1].update_previouse_pose(previouse_pose)

        if self.boxes[msg.boxid].type != msg.type.data:
            self.boxes[msg.boxid].update_type(msg.type.data)

        if len(self.boxes) > 1:
            self.boxes[-2].last_box_plan = None
        self.boxes[-1].last_box(self.boxes[0].goto_plan.joint_trajectory.points[0].positions)

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

    
