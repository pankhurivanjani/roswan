#include <os5000_compass/os5000.h>

static const int OS5000_SERIAL_DELAY = 100000;
static const double OPEN_FILE_TIMEOUT = 10.0;
static int updateDelay = 40;
static int fd;
static char sbuf[34], rbuf[34];
static const int BUF_SIZE = 34; // OS5000_STRING_LENGTH * NUM_STRINGS_PER_READ
static int badY, badP, badR;
static double lastFileOpen;
static int numOpenTries;

CompassDriverLinuxOS5000::CompassDriverLinuxOS5000()
    :isCalib(false), baudRate(B19200), nh(), pnh("~")
{
    badY = badR = badP = 0;
    pnh.param("devicePath", devicePath, std::string("/dev/compass"));
    pnh.param("updateFrequency", updateDelay, int(40));
    pnh.param("frame_id", frame_id, std::string("compass"));
    numOpenTries = 0;
    sbuf[0] = 0;

    if(!openSerialPort(baudRate)){
        ROS_INFO("Serial port %s opened at baud rate %d", devicePath.c_str(), (int)baudRate);
        lastFileOpen = ros::Time::now().toSec();
    }
    ros::Timer timer = nh.createTimer(ros::Duration(1 / updateDelay), &CompassDriverLinuxOS5000::timerCallback, this);
    compass_pub = nh.advertise<sensor_msgs::Imu>("os5000_compass", 10);
    serial_thread = new boost::thread(boost::bind(&CompassDriverLinuxOS5000::run, this));
    if(nh.ok())
        ros::spin();
    serial_thread->join();
}

CompassDriverLinuxOS5000::~CompassDriverLinuxOS5000(){
    close(fd);
    delete serial_thread;
    fd = -1;
}


void CompassDriverLinuxOS5000::timerCallback(const ros::TimerEvent& event){
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

void CompassDriverLinuxOS5000::sendImuMsg(float _yaw, float _pitch, float _roll){
    sensor_msgs::Imu compass_msg;
    compass_msg.header.stamp = ros::Time::now();
    compass_msg.header.frame_id = frame_id;
    tf2::Quaternion qt;
    qt.setRPY(_roll, _pitch, _yaw);
    tf2::convert(qt, compass_msg.orientation);
    compass_pub.publish(compass_msg.orientation);
}

int CompassDriverLinuxOS5000::openSerialPort(int baud){
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
    sendCmd(4, "\e*2\r");
    usleep(OS5000_SERIAL_DELAY);
    tcflush(fd, TCIFLUSH);

    // Set update delay in updates per seconds
    snprintf(updateDelayStr, 6, "\eR%02d\r", updateDelay);
    sendCmd(5, updateDelayStr);
    usleep(OS5000_SERIAL_DELAY);
    tcflush(fd, TCIFLUSH);

    // Set OPHR data fields 15 = Azimuth,Pitch,Roll,Temp; 31 = Azimuth,Pitch,Roll,Temp,Depth
    sendCmd(5, "\eX15\r");
    usleep(OS5000_SERIAL_DELAY);
    tcflush(fd, TCIFLUSH);
    usleep(OS5000_SERIAL_DELAY);
    return 0;
}


void CompassDriverLinuxOS5000::sendCmd(int len, const char* buf){
    for(int i = 0; i < len; i++){
        write(fd, &buf[i], 1);
        usleep(OS5000_SERIAL_DELAY);
    }
}



int CompassDriverLinuxOS5000::parseString(char* string, char* end){
    ROS_DEBUG("parsing %s\n", string);

    // Ensure that we are starting with form the beginning of OHPR string
    
    char* start = strchr(string, '$');
    if (strlen(start) < 7 || strncmp(start, "$OHPR,", 6)){
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
    float heading1 = 0, pitch1 = 0, roll1 = 0, temp1 = 0, dh = 0;

    // Advance to the first data
    p += 6;

    // Iterate over the 4 values to be read (heading, pitch, roll, temp)
    for(int f = 0; f < 4; f++){
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
                    yaw = (90 - heading) * M_PI / 180;
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
                    pitch = pitch1;
                    badP = 0;
                }
                break;

            // roll
            case 2:
                roll1 = strtof(p, NULL) * M_PI / 180;
                if(!badR && fabs(roll1 - roll) > 0.5)
                    badR = 1;
                else{
                    roll = roll1;
                    badR = 0;
                }
                break;

            // temp
            case 3:
                temp1 = strtof(p, NULL);
                if(temp1 > -45 && temp1 < 85)
                    temperature = temp1;
                break;
        }

        // Move the read pointer till the next comma
        while (*p && *p != ',' && p != end) p++;
        if ((!*p || p == end) && f != 3){
            // if the string ended prematurely, then ignore it.
            ROS_WARN("Ended prematurely %d - %d\n", f, *p);
            return -1;
        }
    }
}


void CompassDriverLinuxOS5000::run(){
    int n;
    size_t bufSize = BUF_SIZE;
    char* tbuf;
    FILE* file = fdopen(fd, "r");
    while(nh.ok()){
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
    CompassDriverLinuxOS5000 os5000;
}
