# inail2arm_ros_control

## Setup to use ros_control (and moveit) with xbot2!

Clone this repo and https://github.com/ADVRHumanoids/ros_xbot_hw_interface

- switch on the robot / launch gazebo (eg `roslaunch inail2arm_gazebo inail2arm_world_haria.launch`)
- run xbot2 (eg `xbot2-core`)
- put robot in homing, better safe than sorry (eg `xbot2-gui`)
- launch ros -> xbot hw interface and the controllers (`roslaunch inail2arm_ros_control inail2arm_control.launch`). check the args for configurations
- launch moveit movegroup: `roslaunch inail2arm_moveit_config move_group.launch`

To check if everything is ok, you can add in rviz the motionPlanning plugin and play by planning and executing a trajectory.

Further istructions soon 
