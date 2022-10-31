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

### Acknowledgements

VSCode devcontainer environment setup inspired Allison Thackston's work found here:
- https://github.com/athackst/vscode_ros2_workspace
- https://www.allisonthackston.com/articles/vscode-docker-ros2.html


- [x] Milestone #1
- [x] Milestone #2
- [x] Milestone #3
- [x] Milestone Extra


## Install
## HOW TO LAUNCH
It is recomeneded to launch the franka robot seperate from the pick and place node. Hence 
```
cd ws/
source devel/setup.bash
roslaunch pick_and_place start_brick_layer.launch
rosrun pick_and_place brick_builder.py
rosrun pick_and_place action_client.py

otherwise you can try (but does not connect every time)
roslaunch pick_and_place start_brick_layer.launch launch_all:=True

```
## SERVICES
Multiple serveces dp exist 
```
1.        rospy.Service('show_build' , Empty, self.add_all_bricks)
2.        rospy.Service('reset_scene', Empty, self.srv_remove_all_bricks)
3.        rospy.Service('add_brick'  , Empty, self.srv_add_random_brick)
4.        rospy.Service('pickPlace'  , Empty, self.srv_simple_pick_and_place)
5.        rospy.Service('go_to'      , go_to, self.srv_go_to)
```

1. It shows the final wall that we will attempt to build 
![build_wall](https://github.com/jimas95/pick_and_place/blob/main/gifs/build_wall.gif)

2. will reset the scene in moveit, by removing all the bricks from it
![clear_scene](https://github.com/jimas95/pick_and_place/blob/main/gifs/clear_scene.gif)

3. add a brick in the dedicated space but in random position and orientation
![add_brick](https://github.com/jimas95/pick_and_place/blob/main/gifs/add_brick.gif)
4. perform a simple pick and place brick stacking task (Milestone #2)
![clear_scene](https://github.com/jimas95/pick_and_place/blob/main/gifs/brick_stack.gif)
5. I used it, mainly as helper function, it assigns the robots eef to plan and execute a pose goal 

## BRICK STACKING
For the (Milestone #2) brick stack, the idea is first we spawn the first brick we pick it up. That ensures there will be no conflict with the second brick. We spawn the second brick and then place the first one on top of it.

## BUILD WALL
I took this asigmented a step further and added the capability of building walls. This is configured from the yaml file. The idea is since we can now easyly pick and place a brick, lets do something with it. So if this mode is activated the robot will start to pick bricks and build a wall around him.
![wall.png](https://github.com/jimas95/pick_and_place/blob/main/gifs/wall.png)


## ACTIONS
There are two different actions that have implemented.
1. stacks one brick on top of the other 
2. the robot will start to pick up bricks and build a wall around him 

```
rosrun pick_and_place action_client.py 
rosrun pick_and_place action_client.py

```

## Approach
The main aproach here is that we exploit moveit both for planing and the PlanningSceneInterface in order to avoid obstacles. The whole implementation is based on the tutorial demo 'panda_moveit_config demo.launch'   

dedicated space for pick 
pre grab pose 
go to servise for debug 
build wall 