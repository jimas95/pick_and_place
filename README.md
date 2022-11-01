# Pick And Place Challenge

# Background
Use MoveIt to execute a pick & place operation that stacks two blocks on top of each other. Leverage the Panda arm environment provided in the MoveIt install.

## Milestone #1

Launch panda MoveIt environment

Run `setup` task
Run `install dependencies` task
Run `build` task
`roslaunch panda_moveit_config demo.launch rviz_tutorial:=true`

https://ros-planning.github.io/moveit_tutorials/

## Milestone #2

Create a node that exposes a service to randomly add two boxes to the planning scene

## Milestone #3

Write a service that when called, plans and executes a motion that stacks on box on the other

### Bonus: Use an action instead of a service

### Acknowledgments

VSCode devcontainer environment setup inspired Allison Thackston's work found here:
- https://github.com/athackst/vscode_ros2_workspace
- https://www.allisonthackston.com/articles/vscode-docker-ros2.html


## Milestone Tasks 
- [x] Done --> Milestone #1
- [x] Done --> Milestone #2
- [x] Done --> Milestone #3
- [x] Done --> Milestone Extra



## Brick Stacking
For the (Milestone #2) brick stack, the idea is first we spawn the first brick we pick it up. That ensures there will be no conflict with the second brick. We spawn the second brick and then place the first one on top of it.

## Build Wall
I took this assignment a step further and added the capability of building walls. This is configured from the yaml file. The idea is since we can now easily pick and place a brick, let's do something with it. So if this mode is activated the robot will start to pick bricks and build a wall around him.
![wall.png](https://github.com/jimas95/pick_and_place/blob/main/gifs/wall.png)



## Approach
The main approach here is that we exploit moveit both for planning and the PlanningSceneInterface in order to avoid obstacles. The whole implementation is based on the tutorial demo 'panda_moveit_config demo.launch'. A main function called go_to is responsible for moving the eef to the desired position and orientation. After this, additional functions for pick and place use the go_to function to perform simple manipulations. 

### Dedicated Pick-Up Space
The brick_data.yaml file defines a dedicated space (a cube of reconfigurable size(default 0.5)) where the bricks will be spawned. Inside this space, the bricks can be spawned anywhere and with random positions and orientations but always within this imaginary cube. 

### Pick And Place 
The brick position and size are considered already known. Given those, we set the planer to first align with the orientation of the brick and right above it (on z axis) this is considered the pre-grasping position. After this first move, we move to the grasping position which is the position of the brick. When we reach the desired position we attach the eef to it and go back to pre-grasping position. A similar approach is being implemented for placing the brick in the desired position.

### Build A Wall
An extra action/functionality has been implemented where the robot will start picking up bricks and start building a circular wall around it. The size,position and layers of the wall are set by the yaml file. 
![build_wall](https://github.com/jimas95/pick_and_place/blob/main/gifs/build_wall.gif)


## How To Install
```
mkdir -p ~/ws/src
cd ~/ws/src
wstool init .
wstool merge -t . https://github.com/jimas95/pick_and_place/blob/main/ros.repos
wstool update -t .
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
./build.sh
```

## How To Launch
It is recommended to launch the panda robot separate from the pick and place node. Hence 
```
cd ws/
source devel/setup.bash
roslaunch pick_and_place start_brick_layer.launch start_builder:=False start_action:=False
rosrun pick_and_place brick_builder.py
roslaunch pick_and_place start_action.launch mode:=False

#otherwise you can try to launch all(sometimes does not connect with move it commander)
roslaunch pick_and_place start_brick_layer.launch

#for Build wall mode run 
roslaunch pick_and_place start_action.launch mode:=False
```
## Services
Multiple Services do exist 
```
1. rospy.Service('show_build' , Empty, self.add_all_bricks)
2. rospy.Service('reset_scene', Empty, self.srv_remove_all_bricks)
3. rospy.Service('add_brick'  , Empty, self.srv_add_random_brick)
4. rospy.Service('pickPlace'  , Empty, self.srv_simple_pick_and_place)
5. rospy.Service('go_to'      , go_to, self.srv_go_to)
```

1. It shows the final wall that we will attempt to build 
![build_wall](https://github.com/jimas95/pick_and_place/blob/main/gifs/build_wall.gif)

2. will reset the scene in moveit, by removing all the bricks from it
![clear_scene](https://github.com/jimas95/pick_and_place/blob/main/gifs/clear_scene.gif)

3. add a brick in the dedicated space but in a random position and orientation
![add_brick](https://github.com/jimas95/pick_and_place/blob/main/gifs/add_brick.gif)

4. perform a simple pick and place brick stacking task (Milestone #2)
![clear_scene](https://github.com/jimas95/pick_and_place/blob/main/gifs/brick_stack.gif)

5. I used it, mainly as a helper function, it assigns the robots eef to plan and execute a pose goal 



## Actions
There are two different actions that have been implemented.
1. stacks one brick on top of the other 
2. the robot will start to pick up bricks and build a wall around him 

```
roslaunch pick_and_place start_action.launch mode:=False
roslaunch pick_and_place start_action.launch mode:=True

```