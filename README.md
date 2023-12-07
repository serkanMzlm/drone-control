## Usage of PX4 in Offboard Mode
The purpose of this project is to use the PX4 Autopilot software in Offboard mode with ROS2.
It allows the control of a PX4 drone in a simulation environment using data from a joystick.

### Installation

1. [PX4 Autoplot](https://docs.px4.io/main/en/dev_setup/building_px4.html)

2. **drone-control** packages installation:
```
git clone git@github.com:serkanMzlm/drone-control.git
cd drone-control
git submodule init
git submodule update
```

### Build
- Firstly, the 'px4_msgs' package needs to be built and included in the system. This is necessary because other packages need the messages provided by 'px4_msgs' during their build process
```
colcon build --packages-select px4_msgs
. install/setup.bash
```
- After building and including the 'px4_msgs' package in the system, all files can be built.
```
colcon build 
. install/setup.bash
```
**Caution:** Messages within the 'px4_msgs' package in PX4 Autopilot software may change in future versions. Therefore, errors may occur during ROS2 execution. To address these errors, copy the 'px4_msgs' files from PX4 Autopilot to the 'px4_msgs' folder.
`PX4-Autopilot/msg -> drone-control/px4_msgs`
### Run
- Terminal 1:
```
make px4_sitl gz_x500 
```

- Terminal 2:
```
ros2 launch drone_sim startup.launch.py
```
- Terminal 3:
```
ros2 run controller  controller_node
```
