#include "MockGpsAccuracyPublisher.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_accuracy_publisher");
  ros::NodeHandle nh;

  gps_accuracy_manager::GpsAccuracyPublisher gps_accuracy_publisher(nh);

  ros::Rate rate(1);  // Publish at 1 Hz
  while (ros::ok())
  {
    gps_accuracy_publisher.PublishGpsAccuracy();

    ros::spinOnce();
    rate.sleep();
  }
}