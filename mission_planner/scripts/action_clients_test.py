#! /usr/bin/env python

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

# import actionlib_tutorials.msg
import mission_planner.msg

def feedback_cb(feedback):
    print("feedback_cb called!")

def crosswalk_stop_client():
    client = actionlib.SimpleActionClient("crosswalk_stop", mission_planner.msg.MissionPlannerAction)

    client.wait_for_server()
    goal = mission_planner.msg.MissionPlannerGoal()
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    print("get result!!");

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        print("Initialize client node")
        rospy.init_node('vision_action_client')

        print("Wait for crosswalk_stop")
        result = crosswalk_stop_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
