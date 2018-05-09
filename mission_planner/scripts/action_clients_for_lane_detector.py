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

def lane_detector_client():
    client = actionlib.SimpleActionClient("lane_detector", mission_planner.msg.MissionPlannerAction)

    client.wait_for_server()
    goal = mission_planner.msg.MissionPlannerGoal()
    #goal.mission = 0;
    goal.mission = 1;
    client.send_goal(goal, feedback_cb=feedback_cb)
    print("send goal!");
    client.wait_for_result()
    print("get result!!");

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        print("Initialize client node")
        rospy.init_node('vision_action_client')

        print("Wait for lane_detector")
        result = lane_detector_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
