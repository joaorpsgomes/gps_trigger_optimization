
#include <math.h>
#include "ros/ros.h"
//// MSG ////
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "rtabmap_ros/Info.h"
#include "gps_optimize_tool/GPS_optimize.h"




long double distance(long double lat1, long double long1,
                     long double lat2, long double long2);

long double toRadians(const long double degree);



//////////////////////// Callbacks ////////////////////////



class GPS_handler{

    private:
        ros::Publisher pub_;
        ros::Subscriber sub_gps_;
        ros::Subscriber sub_loop_closure_id_;
        long double dist_;
        double x_;
        double y_;
        double z_;


    public:
        GPS_handler(ros::NodeHandle *n_);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void rtabmap_infoCallback(const rtabmap_ros::Info::ConstPtr& msg);

};