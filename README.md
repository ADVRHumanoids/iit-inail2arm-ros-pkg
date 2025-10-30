# iit-inail2arm-ros2-pkg

tested on

- ubuntu 24.04
- ros2 jazzy

## Pre-request

### Xbot2 ros2 (with cartesio)

follow install the xbot2 framework part on [this website](https://advrhumanoids.github.io/xbot2/master/quickstart.html#install-the-xbot2-framework:~:text=dev%20qtdeclarative5%2Ddev-,Install%20the%20xbot2%20framework,-%EF%83%81), but change the http address prefixe with:

```bash
http://xbot.cloud/xbot2-nightly/
```

### iit-dagana-ros-pkg (optional)

```bash
git clone -b ros2 https://github.com/ADVRHumanoids/iit-inail2arm-ros-pkg.git
```

## how to use

### Xbot Control Joint Control

```bash
source ~/<ros2_ws>/install/setup.bash
ros2 launch inail2arm_gazebo inail2arm_gazebo.launch.py
xobt2-core --hw sim --simtime
ros2 run inail2arm_gazebo send_xbot_command
```

### Xbot Gripper Control

```bash
source ~/<ros2_ws>/install/setup.bash
ros2 launch inail2arm_gazebo inail2arm_gazebo.launch.py
xobt2-core --hw sim --simtime
ros2 run inail2arm_gazebo send_xbot_gripper_command
```

### [Cartesio](https://advrhumanoids.github.io/CartesianInterface/) End-effector Pose Control

```bash
source ~/<ros2_ws>/install/setup.bash
ros2 launch inail2arm_gazebo inail2arm_gazebo.launch.py
xobt2-core --hw sim --simtime
ros2 launch inail2arm_cartesio inail2arm_cartesio.launch
rros2 run inail2arm_gazebo send_cartesio_command
```
