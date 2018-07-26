# roswan
ROS on SWAN

### Setup guide
#### 1. Setup swan
1. [Flashing Ubuntu 16.04 to Beaglebone](./doc/setup_ubuntu.md) (on Ubuntu 16.04).
2. [Setup beaglebone](./doc/setup_beaglebone.md) (wifi, gpio, miscellaneous).
3. [Install ROS Kinetic](./doc/setup_ros.md).
4. [Install ROSwan](./doc/setup_roswan.md).
5. [Additional tools](./doc/swan_addition.md) (optional).
#### 2. Setup remote computer
6. Install ROS Kinetic. Please refer to ROS [wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu).
7. [Install ROSwan](./doc/setup_roswan.md).
###  Running Naviagtion
Change environment variables in `.bashrc`
```
export THRUSTER=trex_driver
export IMU=os5000
export GPS=gps
export CONTROLLER=diff_controller
export MAP=pandan
```
Connect to Angsa wifi and launch minimal configuration for navigation:
```
roslaunch swan_bringup minimal.launch
```
Run loiter (in a new terminal): 
```
rosrun r2c2 captain_loiter_test
```
Run missions (in a new terminal):
```
rosrun r2c2 captain_mission_test
```
The missions points are specified in [mission_points.json](../roswan/r2c2/params/mission_points.json)

After [setting up the remote computer](./doc/setup_roswan.md), you can run `rviz` or `rqt` to visualize the swan. The rviz and rqt configurations are stored inside **visualization tools** folder.
###  File structure explained
- **swan_bringup:** All the launch files and parameters for operation excluding parameters for move_base.
- **swan_driver:** Drivers for the swan including [compass](./swan_driver/os5000_compass//README.md) and thruster.
- **swan_controller:** There are 3 packages inside this folder.
  - [**swan_controller:**](./swan_controller/swan_controller/README.md) A controller node that handles PID and differential drive.
  - [**swan_localization:**](./swan_controller/swan_localization/README.md)  Estimate odometry base on command and dynnamic property or compass and gps.
  - **swan_sim** Some nodes that were used for testing.
- **swan_navigation:** Parameters for the nav stack and move_base. Nav stack tuning guide: http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide.
- **r2c2:** Command and control . It is not fully written yet currently. The ones inside this package are written only for field trial testing.
- **bag:** bag files from field trial.
- **visual_tools:** rqt and rviz plugins (Only for remote computer).
- [**swan_msgs:**](./swan_msgs/README.md) custom msgs. (Used only for PID tuning currently.)

*For more details please refer to the README inside each package*