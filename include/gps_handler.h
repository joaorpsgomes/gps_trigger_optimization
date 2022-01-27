
#include <math.h>
#include "ros/ros.h"
//// MSG ////
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "rtabmap_ros/Info.h"
#include "gps_optimize_tool/GPS_optimize.h"




long double distance_simple(long double lat1, long double long1,
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
		long double threshold_; // in meters


	public:
		GPS_handler(ros::NodeHandle *n_);
		void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void rtabmap_infoCallback(const rtabmap_ros::Info::ConstPtr& msg);

};

class GPS2plane{
	private:
		double r_ox_;
		double r_oy_;
		double r_oz_;
		double e_xx_;
		double e_xy_;
		double e_xz_;
		double e_yx_;
		double e_yy_;
		double e_yz_;
		vector<double> x_;
		vector<double> y_;

	public:
		GPS2plane(double latitude, double longitude, double altitude);
		void gps2cart(double *x, double *y, double *z, double latitude, double longitude, double altitude);
		vector<double> get_x_(){return x_;}
		vector<double> get_y_(){return y_;}
		void set_point(double x, double y);
};