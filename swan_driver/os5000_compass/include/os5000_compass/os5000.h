#ifndef _DRIVER_LINUX_OS5000_H
#define _DRIVER_LINUX_OS5000_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <cstring>
#include <cmath>
#include <string>

class CompassDriverLinuxOS5000
{
protected:
    float heading, pitch, roll, yaw, temperature;
    double t;
    bool isCalib;
    std::string devicePath;
    speed_t baudRate;
    int openSerialPort(int baud);
    void sendCmd(int len, const char* buf);
    boost::mutex mtx;
    boost::thread* serial_thread;

    // ROS nodes and params
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    ros::Publisher compass_pub;

public:
    CompassDriverLinuxOS5000();
    ~CompassDriverLinuxOS5000();
    int parseString(char* string, char* end);
    void run();
    void timerCallback(const ros::TimerEvent& event);

};
#endif
