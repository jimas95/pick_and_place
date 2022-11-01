#! /usr/bin/env python

from __future__ import print_function

import rospy
import sys
import actionlib
import pick_and_place.msg

def action_client(mode):

    if(mode):
        rospy.loginfo("starting build_wall action")
        client = actionlib.SimpleActionClient('build_wall', pick_and_place.msg.pickAndPlaceAction)
    else:
        rospy.loginfo("starting simple_pick_and_place action")
        client = actionlib.SimpleActionClient('simple_pick_and_place', pick_and_place.msg.pickAndPlaceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = pick_and_place.msg.pickAndPlaceGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        rospy.init_node('action_client')

        if(rospy.has_param('action_mode')):
            mode = rospy.get_param('action_mode')
        else:
            mode = False

        result = action_client(mode)
        print("Result: " + str(result.result))
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

