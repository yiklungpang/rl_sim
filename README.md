# Reinforcement Learning Simulation Environment
This is a catkin workspace containing the simulation environment for robot reinforcement learning.

## Prerequisites
You will need the following software installed on your machine:
- ROS Kinetic
- Gazebo 8
- RViz
- MoveIt!

## Getting Started
First clone the catkin workspace

`git clone https://github.com/yiklungpang/rl_sim.git`

Once the repository is downloaded, run
```
cd rl_sim
catkin_make
source devel/setup.bash
```
## Demo
This is a simple example for moving the UR5 arm and taking RGB images

You can start the process by running:

`roslaunch rl_demo demo.launch`

Wait 10 seconds after the environment is launched for the initial config to be loaded, then start the movement by runnning:

`rosrun rl_demo simple_movement.py`

The UR5 arm will move close to the blue cube and close the gripper. The RGB images from the camera will be saved in:

`rl_sim/src/rl_demo/image`
