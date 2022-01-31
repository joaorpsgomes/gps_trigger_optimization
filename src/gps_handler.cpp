#include "../include/gps_handler.h"




long double distance_simple(long double lat1, long double long1,
                    	    long double lat2, long double long2)
{
	lat1 = toRadians(lat1);
	long1 = toRadians(long1);
	lat2 = toRadians(lat2);
	long2 = toRadians(long2);

	long double dlong = long2 - long1;
	long double dlat = lat2 - lat1;
	
	long double ans = pow(sin(dlat / 2), 2) +
			      cos(lat1) * cos(lat2) *
			      pow(sin(dlong / 2), 2);
	
	ans = 2 * asin(sqrt(ans));
	
	long double R = 6371000;
	
	ans = ans * R;
	
	return ans; // in km
}

long double toRadians(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}










//////////////////////// Callbacks ////////////////////////





void GPS_handler::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

	static long double lat_prev=0;
	static long double long_prev=0;
	double x,y;
	static double x_prev=0,y_prev=0;

	long double lat=static_cast<double>(msg->latitude);
	long double longi=static_cast<double>(msg->longitude);
	long double alt=static_cast<double>(msg->altitude);

	if(start_==false){	//First gps_callback
		gps2plane_=GPS2plane((double)lat,(double)longi,(double)alt);

		start_=true;
	}
	
	if(lat_prev==0 && long_prev==0){
		dist_=0;

	}else{
		gps2plane_.gps2planeCoordinates(&x,&y,lat,longi,alt);
		dist_+=sqrt(pow(x-x_prev,2)+pow(y-y_prev,2));
		x_prev=x;
		y_prev=y;
	}
	
	std::cout << "Dist: " << dist_ << "\n" <<std::flush;
}

void GPS_handler::rtabmap_infoCallback(const rtabmap_ros::Info::ConstPtr& msg)
{
	static int count=0;
	static int time_after_optimization=msg->header.stamp.sec;
	gps_optimize_tool::GPS_optimize msg_p;

	if(msg->loopClosureId==0){   // Loop closure not detected
		
		if(dist_>20.0){

			///// Fill header /////
			
			msg_p.header.seq=count;
			msg_p.header.stamp=ros::Time::now();
			msg_p.header.frame_id=msg->header.frame_id;

			///// Fill loopClosureId /////

			msg_p.loopClosureId=msg->loopClosureId;

			///// Fill gps_pose /////

			msg_p.gps_pose.position.x=x_;
			msg_p.gps_pose.position.y=y_;
			msg_p.gps_pose.position.z=z_;


			///// Publish message /////
			pub_.publish(msg_p);
			count++;
			dist_=0;
		}
	}else{                        // Loop closure detected
		std::cout<<"Loop closure heard : " << msg->loopClosureId<< "\n"<<std::flush;
		time_after_optimization=msg->header.stamp.sec;
		dist_=0;
	}  

  
}


GPS_handler::GPS_handler(ros::NodeHandle *n_){ 
	double aux;
	ros::param::get("/threshold", aux);
	threshold_=(long double)aux;
	sub_gps_ = n_->subscribe("/ublox_gps/fix", 1000, &GPS_handler::gpsCallback,this);
	sub_loop_closure_id_ = n_->subscribe("/rtabmap/info", 1000, &GPS_handler::rtabmap_infoCallback,this);
	pub_= n_->advertise<gps_optimize_tool::GPS_optimize>("/gps_optimize_pose",10);
	start_=false;

	x_=0;
	y_=0;
	z_=0;
}

