class Base_Odom
{
protected:
    ros::NodeHandle n;
    ros::Subscriber compass_sub;
    ros::Publisher odom_pub;
    ros::Rate r;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;

    std::string frame_id, child_frame_id;

    bool send_transform;

    double vx, vy, vth ;
    double x, y, z, last_th, th, dt, t;

//    const double deg_to_rad(const double deg){
//        return M_PI / 180 * deg;
//    }
//
//    const double rad_to_deg(const double rad){
//        return rad * 180 / M_PI;
//    }
//
//    const double steer_to_rad(const int steer){
//        double rad = deg_to_rad((double)steer * steer_ratio);
//        return rad;
//    }
//
//    const double counts_to_m(const int counts){
//        double m = (double)counts * encoder_ratio;
//    }


public:
    Base_Odom(double _forward_speed);
    void update_yaw(const sensor_msgs::Imu::ConstPtr& msg);
    void update_position();
    void pub_tf();
    void pub_odom();

};

