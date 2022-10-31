#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty,EmptyResponse
from pick_and_place.srv import go_to,go_toResponse

import random

try:
    from math import pi, tau, dist, fabs, cos, sin
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sin, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def handle_service(req):
    rospy.loginfo('called!')
    return EmptyResponse()

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # create services 
        rospy.Service('show_build' , Empty, self.add_all_bricks)
        rospy.Service('reset_scene', Empty, self.srv_remove_all_bricks)
        rospy.Service('add_brick'  , Empty, self.srv_add_random_brick)
        rospy.Service('add_brick'  , Empty, self.srv_simple_pick_and_place)
        rospy.Service('go_to'      , go_to, self.srv_go_to)


        # fetch a group of brick data parameters from rosparam server
        self.yaml_data = rospy.get_param('brick_data')
        print(self.yaml_data)
        self.angle_range = self.yaml_data['angle']
        self.radious = self.yaml_data['radious']
        self.brick_size = (self.yaml_data['brick_size']['width'],self.yaml_data['brick_size']['length'],self.yaml_data['brick_size']['height'])
        

        # Misc variables
        self.box_name = "brick_01"
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.brick_id = 0
        self.brick_pose= geometry_msgs.msg.PoseStamped()

        # reset scene
        self.detach_box()
        self.remove_all_bricks()

    def srv_simple_pick_and_place(self,req):
        self.simple_pick_and_place()
        return EmptyResponse()

    # service of setting a pose goal. look srv msg go_to for info, under the srv folder 
    def srv_go_to(self,req):
        q = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = req.x
        pose_goal.position.y = req.y
        pose_goal.position.z = req.z
        result = self.go_to(pose_goal)
        return go_toResponse(result)

    # set arm to default home position
    def go_home(self):
        q = quaternion_from_euler(pi, 0, -pi/4)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = -0.5
        pose_goal.position.y =  0.0
        pose_goal.position.z =  0.5
        self.go_to(pose_goal)

    # main function for setting end effector to pose_goal pose 
    def go_to(self,pose_goal):
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        if(success):
            rospy.loginfo("success of go to")
        else:
            rospy.logerr("go to failed to reach target")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    # main function for grabing the last spawned brick
    def grab_brick(self):
        # create grabing pose 
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = self.brick_pose.pose.orientation.x
        pose_goal.orientation.y = self.brick_pose.pose.orientation.y
        pose_goal.orientation.z = self.brick_pose.pose.orientation.z
        pose_goal.orientation.w = self.brick_pose.pose.orientation.w
        pose_goal.position.x    = self.brick_pose.pose.position.x
        pose_goal.position.y    = self.brick_pose.pose.position.y
        pose_goal.position.z    = self.brick_pose.pose.position.z

        print("go_home")
        self.go_home()

        # aling with cube
        print("pree_grab")
        pose_goal.position.z = pose_goal.position.z + 0.2 + self.yaml_data['brick_size']['height']/2
        self.go_to(pose_goal)

        # go to cube
        print("pose_grab")
        pose_goal.position.z = pose_goal.position.z - 0.2 + self.yaml_data['brick_size']['height']
        self.go_to(pose_goal)

        # attach 
        print("attach_box")
        self.box_name = "brick_0" + str(self.brick_id)
        self.attach_box()

        print("go_home")
        self.go_home()

    # place brick at given pose 
    def place_brick(self,pose):

        print("go_home")
        self.go_home()

        # aling with cube
        print("pree_place")
        pose.position.z = pose.position.z + 0.2 
        self.go_to(pose)

        # go to cube
        print("pose_place")
        pose.position.z = pose.position.z - 0.15
        self.go_to(pose)

        # attach 
        print("detach_box")
        self.detach_box()

        print("go_home")
        self.go_home()

    #  simple pick and place implementation for 2 bricks 
    def simple_pick_and_place(self):
        
        # pick
        self.add_random_brick()
        self.grab_brick()

        # place 
        self.add_random_brick()

        # create place pose 
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = self.brick_pose.pose.orientation.x
        pose_goal.orientation.y = self.brick_pose.pose.orientation.y
        pose_goal.orientation.z = self.brick_pose.pose.orientation.z
        pose_goal.orientation.w = self.brick_pose.pose.orientation.w
        pose_goal.position.x    = self.brick_pose.pose.position.x
        pose_goal.position.y    = self.brick_pose.pose.position.y
        pose_goal.position.z    = self.brick_pose.pose.position.z + self.yaml_data['brick_size']['height']*2
        self.place_brick(pose_goal)


    # wait, Ensuring Collision Updates Are Received
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=2):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False



    # service for addind a brick at random position
    def srv_add_random_brick(self,req):
        self.add_random_brick()
        return EmptyResponse()

    # add a brick in a random position, but in the dedicated pick up place
    def add_random_brick(self):
        angle_rad = random.random()*2*pi
        q = quaternion_from_euler(pi, 0, angle_rad)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]
        box_pose.pose.position.x = self.yaml_data['pick_place']['position']['x'] + (random.random()-0.5)*self.yaml_data['pick_place']['size']
        box_pose.pose.position.y = self.yaml_data['pick_place']['position']['y'] + (random.random()-0.5)*self.yaml_data['pick_place']['size']
        box_pose.pose.position.z = self.yaml_data['pick_place']['position']['z'] + (random.random())* 0.2
        self.brick_id = self.brick_id + 1
        box_name = "brick_0" + str(self.brick_id)
        self.scene.add_box(box_name, box_pose, size=self.brick_size)
        
        # this is a bug, we add pi/4 to fix orientation of end effector 
        q = quaternion_from_euler(pi, 0, angle_rad+pi/4)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]
        self.brick_pose = box_pose



    # convert degrees to radian and return the angle 
    def to_rad(self,angle_degrees):
        return angle_degrees*pi/180.0


    # convert from polar coordinate system(r meters, angle degrees) to x,y,rad
    def polar_coords(self,angle_degrees):
        angle_rad = self.to_rad(angle_degrees)
        x = self.radious*cos(angle_rad)
        y = self.radious*sin(angle_rad)
        return [x,y,angle_rad]

    # service for reseting the scene. Removes all brick items from moveit scene
    def srv_remove_all_bricks(self,req):
        self.remove_all_bricks()
        return EmptyResponse()

    # removes all bricks from the scene
    def remove_all_bricks(self):
        items = self.scene.get_known_object_names()
        for item in items:
            # if item contains brick in its name remove it 
            if("brick_" in item):
                self.box_name = item
                self.remove_box()


    # add all boxes to see the final build product
    def add_all_bricks(self, req):
        thikness = self.yaml_data['thikness']
        step = int(self.angle_range/thikness)
        scene = self.scene

        for theta in range(0,self.angle_range,step):

            x,y,angle_rad = self.polar_coords(theta)
            q = quaternion_from_euler(0, 0, angle_rad)

            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "world"
            box_pose.pose.orientation.x = q[0]
            box_pose.pose.orientation.y = q[1]
            box_pose.pose.orientation.z = q[2]
            box_pose.pose.orientation.w = q[3]

            
            box_pose.pose.position.x = x
            box_pose.pose.position.y = y
            box_pose.pose.position.z = 0.1
            
            box_name = "brick_0" + str(theta)
            scene.add_box(box_name, box_pose, size=self.brick_size)

        return EmptyResponse()


    def attach_box(self, timeout=2):
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.box_name)
        print("removing --> " + self.box_name)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


    
def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface PICK AND PLACE BRICKS")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
    
        tutorial = MoveGroupPythonInterfaceTutorial()


        input("---")
        tutorial.go_home()
        # tutorial.add_random_brick()
        # tutorial.grab_brick()
        tutorial.simple_pick_and_place()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

