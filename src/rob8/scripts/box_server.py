#!/usr/bin/env python3

import rospy
import rob8.msg
import actionlib
import time

def talker():
    pub = rospy.Publisher('/boxes', rob8.msg.Boxes, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    client = actionlib.SimpleActionClient('planner_executer', rob8.msg.ExecutepathAction)
    client.wait_for_server()
    goal = rob8.msg.ExecutepathActionGoal()
    goal.goal.runpath = True
    print(goal)
    client.send_goal(goal.goal)
    client.wait_for_result()
    print(client.get_result())

    # listofitems = ['0:0', '1:0', '1:2']
    # for x in listofitems:
    #     client = actionlib.SimpleActionClient('planner_give_executer', rob8.msg.ExecutespecificAction)
    #     client.wait_for_server()
    #     goal = rob8.msg.ExecutespecificActionGoal()
    #     goal.goal.action_string = x
    #     print(goal)
    #     client.send_goal(goal.goal)
    #     client.wait_for_result()
    #     print(client.get_result())

    #     time.sleep(25)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass