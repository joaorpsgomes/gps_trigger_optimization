#include "../include/gps_handler.h"




long double distance(long double lat1, long double long1,
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

  long double lat=static_cast<double>(msg->latitude);
  long double longi=static_cast<double>(msg->longitude);

  if(lat_prev==0 && long_prev==0){
    dist_=0;
  }else{
    dist_+=distance(lat_prev,long_prev,lat,longi);
  }
  //std::cout << "GPS: " << lat << " " << longi << " " << msg->altitude << " \n" << std::flush;
  //std::cout << "Prev: " << lat_prev << " "<< long_prev << "\nDist: " << dist*1000 << std::flush;  
  
  std::cout << "Dist: " << dist_ << "\n" <<std::flush;
  lat_prev=lat;
  long_prev=longi;
}

void GPS_handler::rtabmap_infoCallback(const rtabmap_ros::Info::ConstPtr& msg)
{
  static int count=0;
  static int time_after_optimization=msg->header.stamp.sec;
  gps_optimize_tool::GPS_optimize msg_p;

  if(msg->loopClosureId==0){   // Loop closure not detected
    std::cout<<"Time after last optimization: " << msg->header.stamp.sec-time_after_optimization<<"\n"<<std::flush;
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
    //std::cout << "Object created" << std::endl;

    x_=0;
    y_=0;
    z_=0;
}

