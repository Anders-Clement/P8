#!/usr/bin/env python3

import rospy
import rob8.msg
import actionlib
import time

def talker():
    pub = rospy.Publisher('/boxes', rob8.msg.Boxes, queue_size=10)
    rospy.init_node('talker', anonymous=True)
   
    new_box = rob8.msg.Boxes()
    new_box.boxid = 0
    new_box.pose.position.x = 0.7
    new_box.pose.position.y = 0.5
    new_box.pose.position.z = 0.2
    new_box.scalex = 0.2
    new_box.scaley = 0.2
    new_box.scalez = 0.1
    new_box.pose.orientation.w = 1
    new_box.type.data = "pickup"
    rospy.loginfo(new_box)
    pub.publish(new_box)

    new_box = rob8.msg.Boxes()
    new_box.boxid = 1
    new_box.pose.position.x = -0.2
    new_box.pose.position.y = 0.7
    new_box.pose.position.z = 0.2
    new_box.scalex = 0.2
    new_box.scaley = 0.2
    new_box.scalez = 0.1
    new_box.pose.orientation.w = 1
    new_box.type.data = "place"
    rospy.loginfo(new_box)
    pub.publish(new_box)

    # time.sleep(30)

    # client = actionlib.SimpleActionClient('planner_give_executer', rob8.msg.ExecutespecificAction)
    # client.wait_for_server()
    # goal = rob8.msg.ExecutespecificActionGoal()
    # goal.goal.action_string = "0:0"
    # print(goal)
    # client.send_goal(goal.goal)
    # client.wait_for_result()
    # print(client.get_result())


    # time.sleep(10)


    # client = actionlib.SimpleActionClient('planner_give_executer', rob8.msg.ExecutespecificAction)
    # client.wait_for_server()
    # goal = rob8.msg.ExecutespecificActionGoal()
    # goal.goal.action_string = "0:1"
    # print(goal)
    # client.send_goal(goal.goal)
    # client.wait_for_result()
    # print(client.get_result())

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass