# swan_controller
## 1. Overview
This package is the controller for swan. The controller handles the differential drive, heading or turn pid control.
## 2. Nodes
### 2.1 swan_controller_node
#### 2.1.1 Subscribed Topics
#####    imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
Imu message from the compass. Only the orientation part is used.
##### cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Controller node will receive command message when enable_key is set to true. It can be either directly from testing node or from the nav stack.
##### joy ([sensor_msgs/Joy](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Joy.html))
Controller node will receive command message from the joy stick when enable_joy is set to true. Not used yet. 
#### 2.1.2 Published Topics
##### pid_diagnostic ([swan_msgs/PID_diagnostic](../../swan_msgs/README.md))
Publish diagnostic message from the pid controller when the mode is set "DEBUG".
##### l_motor ([std_msgs/Float64](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float64.html))
Power to the left motor. range from 0.0 to 1.0.
##### r_motor ([std_msgs/Float64](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float64.html))
Power to the left motor. range from 0.0 to 1.0.
#### 2.1.3 Parameters
##### ~frequency (int, default: 20)
The frequency the controller is running.
##### ~mode (std::string, default: STANDARD)
There are two modes available: DEBUG and STANDARD. 

In DEBUG mode enable_joy, enable_key and enable_pid is available to configure. PID_diagnostic message is publishing for debugging. kp, ki and kd is available for dynamic reconfigure.

In STANDARD mode enable_joy, enable_key and enable_pid is set to their default values. kp, ki and kd can be only changed by ros parameters.
##### ~enable_joy (bool, default: false)
Whether or not the node subscribes to joy stick messages.
##### ~enable_key (bool, default: true)
Whether or not the node subscribe to command messages.
##### ~enable_pid (bool, default: true)
Whether or pid is in use.
##### ~type (std::string, default: TURN)
There are two types: TURN and HEADING. This determines whether the pid input is turn or heading
##### ~pwr_min (double, default: 0.4)
The minimal power send to the motor. Range from 0 to 1.
##### ~pwr_max (double, default: 0.9)
The maximal power send to the motor. Range from 0 to 1.
##### ~speed_min (double, default: 0.0)
The minimal speed accepted by the controller.
##### ~speed_max (double, default: 2.0)
The maximal speed accepted by the controller. The speed will be linearly translated to power based on the power and speed constraint set.
##### ~gain_min (double, default: 0.05)
Any gain calculated from the pid controller that is smaller than this will be considered to be zero.
##### ~kp (double, default: 1.0)
##### ~ki (double, default: 0.0)
##### ~kd (double, default: 0.0)
##### ~max_no_cmd_time(double, default: 0.2)
For the failsafe inside the controller. When there is no command for more than the **max_cmd_time**, the controller will send stop to the thrusters for **stop_cmd_duration** long.
##### ~stop_cmd_duration(double, default: 0.2)
Please refer to the max_no_cmd_time.