#include "GpsAccuracyStatusManager.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_accuracy_manager");
  ros::NodeHandle nh;

  gps_accuracy_manager::GpsAccuracyStatusManager gam(nh);

  ros::spin();
}