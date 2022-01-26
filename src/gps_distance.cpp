#include "ros/ros.h"
#include <iostream>


//// MSG ////
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "rtabmap_ros/Info.h"


#include "../include/gps_handler.h"





int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_distance");
  ros::NodeHandle nh;
  GPS_handler gps_h=GPS_handler(&nh);
  ros::spin();
  return 0;
}




 