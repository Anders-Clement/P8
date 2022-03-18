#!/usr/bin/env python3

import rospy
import rob8.msg
import actionlib

def talker():
    pub = rospy.Publisher('/boxes', rob8.msg.boxes, queue_size=10)
    rospy.init_node('talker', anonymous=True)
   
    new_box = rob8.msg.boxes()
    new_box.boxid = 0
    new_box.pose.position.x = 0.5
    new_box.pose.position.y = 0.5
    new_box.pose.position.z = 0.0
    new_box.scalex = 0.2
    new_box.scaley = 0.2
    new_box.scalez = 0.1
    new_box.pose.orientation.w = 1
    new_box.type = "place"
    rospy.loginfo(new_box)
    pub.publish(new_box)

    new_box = rob8.msg.boxes()
    new_box.boxid = 1
    new_box.pose.position.x = -0.5
    new_box.pose.position.y = -0.5
    new_box.pose.position.z = 0.0
    new_box.scalex = 0.2
    new_box.scaley = 0.2
    new_box.scalez = 0.1
    new_box.pose.orientation.w = 1
    new_box.type = "place"
    rospy.loginfo(new_box)
    pub.publish(new_box)

    client = actionlib.SimpleActionClient('planner_executer', rob8.msg.ExecutepathAction)
    client.wait_for_server()
    goal = rob8.msg.ExecutepathActionGoal()
    goal.goal.runpath = True
    print(goal)
    client.send_goal(goal.goal)
    client.wait_for_result()
    print(client.get_result())

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass