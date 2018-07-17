#include <razor_imu/razor_imu.h>

namespace RazorIMUConstant{
static const int OS5000_SERIAL_DELAY = 25000;
static const double OPEN_FILE_TIMEOUT = 10.0;
static int fd;
static char sbuf[80], rbuf[80];
static const int BUF_SIZE = 80; // OS5000_STRING_LENGTH * NUM_STRINGS_PER_READ
static int badY, badP, badR;
static double lastFileOpen;
static int numOpenTries;
static double accel_factor = 9.806 / 256.0; // sensor report accel as 256.0 = 1G (9.8 m/s^2)
}

using namespace RazorIMUConstant;

RazorIMU::RazorIMU()
    :isCalib(false), baudRate(B19200), nh(), pnh("~")
{
    badY = badR = badP = 1;
    pnh.param("devicePath", devicePath, std::string("/dev/ttyACM0"));
    pnh.param("updateFrequency", updateDelay, int(30));
    pnh.param("frame_id", frame_id, std::string("compass"));
    numOpenTries = 0;
    sbuf[0] = 0;

    if(!openSerialPort(baudRate)){
        ROS_INFO("Serial port %s opened at baud rate %d", devicePath.c_str(), (int)baudRate);
        lastFileOpen = ros::Time::now().toSec();
    }
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / (double)updateDelay), &RazorIMU::timerCallback, this);
    compass_pub = nh.advertise<sensor_msgs::Imu>("os5000", 10);
    serial_thread = new boost::thread(boost::bind(&RazorIMU::run, this));

    // Initialize the compass message to publish
    compass_msg.header.frame_id = frame_id;
    // Orientation covariance estimation:
    // Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
    // Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
    // Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
    // cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
    // i.e. variance in yaw: 0.0025
    // Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
    // static roll/pitch error of 0.8%, owing to gravity orientation sensing
    // error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
    // so set all covariances the same.
    compass_msg.orientation_covariance = boost::assign::list_of	(0.0025)(0)(0)
    															(0)(0.0025)(0)
																(0)(0)(0.0025);
    // Angular velocity covariance estimation:
    // Observed gyro noise: 4 counts => 0.28 degrees/sec
    // nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
    // Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
    compass_msg.linear_acceleration_covariance = boost::assign::list_of	(0.02)(0)(0)
    																	(0)(0.02)(0)
																		(0)(0)(0.02);
    // linear acceleration covariance estimation:
    // observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
    // nonliniarity spec: 0.5% of full scale => 0.2m/s^2
    // Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
    compass_msg.angular_velocity_covariance = boost::assign::list_of(0.04)(0)(0)
    																(0)(0.04)(0)
																	(0)(0)(0.04);
    if(nh.ok())
        ros::spin();
    serial_thread->join();
}

RazorIMU::~RazorIMU(){
    close(fd);
    delete serial_thread;
    fd = -1;
}


void RazorIMU::timerCallback(const ros::TimerEvent& event){
    double now = ros::Time::now().toSec();
    if (fd < 0) {
        if (lastFileOpen-now <= 0 && numOpenTries < 3) {
        ROS_WARN("device path not opened, trying to open the port once again");
        ++numOpenTries;
        openSerialPort(B19200);
        lastFileOpen = now + OPEN_FILE_TIMEOUT;
        }
    }

    // copy any incoming data
    rbuf[0] = 0;
    mtx.lock();
    if(sbuf[0] != 0)
    {
        strncpy(rbuf,sbuf,sizeof(sbuf));
        sbuf[0] = 0;
    }
    mtx.unlock();
    // process incoming data
    if(rbuf[0]){
        parseString(rbuf, &(rbuf[BUF_SIZE-1]));
        sendImuMsg(yaw, pitch, roll);
    }
}

void RazorIMU::sendImuMsg(float _yaw, float _pitch, float _roll){
    compass_msg.header.stamp = ros::Time::now();
    tf2::Quaternion qt;
    qt.setRPY(_roll, _pitch, _yaw);
    tf2::convert(qt, compass_msg.orientation);
    compass_msg.linear_acceleration.x = ax;
    compass_msg.linear_acceleration.y = ay;
    compass_msg.linear_acceleration.z = az;
    compass_msg.angular_velocity.x = wx;
    compass_msg.angular_velocity.y = wy;
    compass_msg.angular_velocity.z = wz;


    compass_pub.publish(compass_msg);
}

int RazorIMU::openSerialPort(int baud){
    char updateDelayStr[6] = {0};
    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1){
        ROS_WARN("Unable to open %s", devicePath.c_str());
        return -1;
    }
    fcntl(fd, F_SETFL, 0); // block serial read
    termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ISIG);
    options.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tcsetattr(fd,TCSAFLUSH,&options);
    tcflush(fd,TCIOFLUSH);

    // Set Data format to OPHR strings. 1 = NMEA, 2 = OPHR
    sendCmd(4, "#o0\r");
    usleep(OS5000_SERIAL_DELAY);
    tcflush(fd, TCIFLUSH);

//    // Set update delay in updates per seconds
//    snprintf(updateDelayStr, 6, "\eR%02d\r", updateDelay);
//    sendCmd(5, updateDelayStr);
//    usleep(OS5000_SERIAL_DELAY);
//    tcflush(fd, TCIFLUSH);
//
//    // Set OPHR data fields 15 = Azimuth,Pitch,Roll,Temp; 31 = Azimuth,Pitch,Roll,Temp,Depth
//    sendCmd(5, "\eX15\r");
//    usleep(OS5000_SERIAL_DELAY);
//    tcflush(fd, TCIFLUSH);
//    usleep(OS5000_SERIAL_DELAY);
    return 0;
}


void RazorIMU::sendCmd(int len, const char* buf){
    for(int i = 0; i < len; i++){
        write(fd, &buf[i], 1);

    }
    usleep(OS5000_SERIAL_DELAY);
}



int RazorIMU::parseString(char* string, char* end){
    ROS_DEBUG("parsing %s\n", string);

    // Ensure that we are starting with form the beginning of OHPR string
    
    char* start = strchr(string, '#');

    if (start == NULL || strlen(start) < 8 || strncmp(start, "#YPRAG=", 7)){
        ROS_WARN("Warning: non-OHPR string\n");
        return 0;
    }

    if(isCalib){
        ROS_INFO("calibration mode: %s", string);
        return 0;
    }

    
    /*
    The default mode setup for OS5000 is OHPR mode with 4 data
    fields (Azimuth,Pitch,Roll,Temp). The string format is
    $OHPR,Azimuth,Pitch,Roll,Temp*CC where CC is the checksum.
    */

    char* p = start;

    // data fields
    float heading1 = 0, pitch1 = 0, roll1 = 0, dh = 0;

    // Advance to the first data
    p += 7;

    // Iterate over the 4 values to be read (heading, pitch, roll, temp)
    for(int f = 0; f < 9; f++){
        // Parse the various data fields
        switch(f){
            // yaw
            case 0:
                heading1 = strtof(p, NULL);
                dh = fabs(heading1 - heading);
                if(dh > 180.0) 
                    dh = fabs(dh - 360);
                if(!badY && dh > 25.0)
                    badY = 1;
                else{
                    heading = heading1;
                    yaw = - heading * M_PI / 180;
                    while(yaw < -M_PI)
                        yaw += 2 * M_PI;
                    while(yaw >= M_PI)
                        yaw -= 2 * M_PI;
                    badY = 0;
                }
                break;

            // pitch
            case 1:
                pitch1 = strtof(p, NULL) * M_PI / 180;
                if(!badP && fabs(pitch1 - pitch) > 0.5)
                    badP = 1;
                else{
                    pitch = -pitch1;
                    badP = 0;
                }
                break;

            // roll
            case 2:
                roll1 = (180 + strtof(p, NULL)) * M_PI / 180;
                if(!badR && fabs(roll1 - roll) > 0.5)
                    badR = 1;
                else{
                    roll = roll1;
                    badR = 0;
                }
                break;

            // ax
            case 3:
            	ax = strtof(p, NULL) * accel_factor;
            	break;

            // ay
            case 4:
            	ay = strtof(p, NULL) * accel_factor;
            	break;

            // az
            case 5:
            	az = strtof(p, NULL) * accel_factor;
            	break;

            // wx
            case 6:
            	wx = strtof(p, NULL);
            	break;

			// wy
            case 7:
            	wy = - strtof(p, NULL);
            	break;

			// wz
            case 8:
            	wz = - strtof(p, NULL);
            	break;


        }

        // Move the read pointer till the next comma
        while (*p && *p != ',' && p != end) p++;
        if ((!*p || p == end) && f != 8){
            // if the string ended prematurely, then ignore it.
            ROS_WARN("Ended prematurely %d - %d\n", f, *p);
            return -1;
        }
        // move one char forward to the next data-field
        p++;
    }
    ROS_DEBUG("yaw: %.2f, pitch: %.2f, roll: %.2f", yaw * 180 / M_PI, pitch * 180 / M_PI, roll * 180 / M_PI);

}


void RazorIMU::run(){
    int n;
    size_t bufSize = BUF_SIZE;
    char* tbuf;
    FILE* file = fdopen(fd, "r");
    while(nh.ok()){
    	sendCmd(3, "#f\r");
        n = getline(&tbuf, &bufSize, file);
        ROS_DEBUG("r : %d - %s\n", n, tbuf);
        for(int i = 0; i < n; i++){
            mtx.lock();
            sbuf[i] = tbuf[i];
            mtx.unlock();
        }

        // Null terminate for safety;
        sbuf[sizeof(sbuf)-1]  = '\0';
        usleep(5000);
    }
    free(file);
    free(tbuf);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "os5000");
    RazorIMU os5000;
}
