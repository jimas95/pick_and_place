#! /usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
import pick_and_place.msg

def action_client():
    # Creates the SimpleActionClient, passing the type of the action
    print("hello")
    client = actionlib.SimpleActionClient('simple_pick_and_place', pick_and_place.msg.pick_and_placeAction)
    print("hello")

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("hello")
    # Creates a goal to send to the action server.
    goal = pick_and_place.msg.pick_and_placeGoal(order=20)

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
        result = action_client()
        print("Result:", ', '.join(result.result))

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)