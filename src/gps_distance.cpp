#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"

long double distance(long double lat1, long double long1,
                     long double lat2, long double long2);

long double toRadians(const long double degree);




void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
static long double dist=0;
static long double lat_prev=0;
static long double long_prev=0;

  long double lat=static_cast<double>(msg->latitude);
  long double longi=static_cast<double>(msg->longitude);

  if(lat_prev==0 && long_prev==0){
    dist=0;
  }else{
    dist+=distance(lat_prev,long_prev,lat,longi);
  }
  std::cout << "GPS: " << lat << " " << longi << " " << msg->altitude << " \n" << std::flush;
  std::cout << "Prev: " << lat_prev << " "<< long_prev << "\nDist: " << dist*1000 << std::flush;  
  //printf("GPS: %g %g %g\n",lat, longi, msg->altitude);
  //printf("Prev:%g %g \n Dist: %g\n",lat_prev, long_prev, dist);

  lat_prev=lat;
  long_prev=longi;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_distance");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ublox_gps/fix", 1000, chatterCallback);
  ros::spin();
  return 0;
}



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
 
    long double R = 6371;
     
    ans = ans * R;
 
    return ans; // in km
}

long double toRadians(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}
 