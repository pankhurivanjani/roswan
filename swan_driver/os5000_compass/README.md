# os5000_compass
## 1. Overview
This package is the driver for the os5000 compass.
## 2. Nodes
### 2.1 os5000_compass
#### 2.1.1 Publish Topics
#####     os5000([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
Imu message from the compass. Only the orientation part is used.
#### 2.1.2 Parameters
##### ~devicePath (std::string, default: /dev/compass)
The os5000 path.
##### ~updateFrequency (int, default: 40)
The sampling frequency for the os5000 compass.
##### ~frame_id(std::string, default: compass)
