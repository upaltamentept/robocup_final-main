# Robocup-Storing Groceries

This project is for the Robocup competation Storing Groceries (2022 5.9).

The code is tested on Pal robot Tiago. The code was written by Yueyang Zhang and Zhen Chen.

A video-demo is added [here](https://drive.google.com/file/d/1JAiL-L5672wRGSGQ0zPKZHxpijRqsEVa/view?usp=sharing) for your reference.

## Pre-work

Please put the folder map into your map path, and upload the map to tiago.

Please put the folder tiago into your tiago workspace. Allow all merge and replace requires

Please put other folder into your darknet workspace.

## Launch the whole project

### change map in robot

```bash
rosservice call /pal_map_manager/change_map "input: robocup_zhen" 
```

### rviz for navigation

```bash
rosrun rviz rviz -d `rospack find tiago_2dnav`/config/rviz/navigation.rviz
```

### highly suggest tuck arm before running

```bash
rosrun tiago_gazebo tuck_arm.py 
```

### localization

```bash
roslaunch tiago_localization tiago_localization.launch
```

### head look down

```bash
rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["head_1_joint", "head_2_joint"], points: [{positions: [-0.0,-0.6], time_from_start: [2,0]}]}'
```

### Launch the recogition  

```bash
roslaunch task_manager demo.launch
```

### Launch grasping and placing

```bash
roslaunch pick_place pick_place.launch 
```

### Launch state machine after gripper open

```bash
roslaunch task_manager task_manager.launch  
```
