#include <swan_localization/utm_tf_node.h>
class static_UTM_publisher : public UTM_tf_node
{
public:
    static_UTM_publisher(int a){
        init();
    }
    void init(){
        int frequency;
        double latitude, longitude;
        ros::param::param<std::string>("~frame_id", frame_id, "odom");
        ros::param::param<std::string>("~child_frame_id", child_frame_id, "base_link");
        ros::param::param<int>("~frequency", frequency, 20);
        ros::param::param<double>("~latitude", latitude, 0);
        ros::param::param<double>("~longitude", longitude, 0);
        ros::param::param<double>("~altitude", altitude, 0);
        ros::Rate r(frequency);
        
        gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf2::convert(q, fix_trans.transform.rotation);
        while(n.ok()){
            ros::spinOnce();
            pub_tf();
            r.sleep();
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "static_utm_publisher");
    static_UTM_publisher static_utm_publisher(1);
}
