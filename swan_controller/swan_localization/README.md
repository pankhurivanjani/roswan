# swan_localization
## 1. Overview
Estimate odometry base on command and dynnamic property or compass and gps.
## 2. Nodes
### 2.1 sensor_odom_estimator
#### 2.1.1 Subscribed Topics
#####    imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
Imu message from the compass. Only the orientation part is used.
##### fix ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
GPS position fix. This will be published with whatever positional and status data was available even if the device doesn't have a valid fix. Invalid fields may contain NaNs.

#### 2.1.2 Published Topics
##### odom ([nav_msgs/Odometry](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html))
Odometry message translated from the fix.

#### 2.1.3 Parameters
##### ~frame_id (std::string, default: odom)
##### ~child_frame_id (std::string, default: base_link)
##### ~publish_transform (bool, default: true)
Whether or not publish the tf messages.
##### ~use_imu_orientation (bool, default: true)
Whether or not to use imu message as the orientation for the odometry message.
##### ~frequency (int, default: 20)
The frequency that this node runs.
##### ~rot_covariance (double, default: 99999)
Rotational covariance for the odometry message.
##### ~use_map_frame (bool, default: false)
Whether or not to transform the odometry with reference to the centre of the map.
##### ~x (double, default: 0.0)
The UTM easting for the center of the map.
##### ~y (double, default: 0.0)
The UTM northing for the center of the map.




### 2.2 dynamic_odom_estimator
#### 2.2.1 Subscribed Topics
##### cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
Command from the nav stack. The position estimation will be based on the message from here.
##### initialpose ([nav_msgs/Odometry](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html))
Initial position from the rviz when sim_mode parameter is set to true.
##### initialodom ([nav_msgs/Odometry](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html))
Initial position from GPS when sim_mode parameter is set to false.
#### 2.2.2 Published Topics
##### odom ([nav_msgs/Odometry](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html))
Odometry message estimated based on the command.
#### 2.2.3 Parameters
##### ~frequency (int, default: 20)
The frequency the node is running.
##### ~sim_mode (bool, default: true)
When it is true, the node will wait for the rviz to give a pose estimate as initial pose. When it is false it will wait for a gps odometry as initial pose.
##### ~holonomic (bool, default: false)
##### ~max_no_cmd_time (double, default: 0.5)
Take the failsafe mechanism in the controller node into consideration.
##### ~stop_cmd_duration (double, default: 0.2)
Please refer to max_no_cmd_time.

