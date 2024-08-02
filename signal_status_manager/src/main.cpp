#include "SignalStatusManager.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signal_status_manager");
  ros::NodeHandle nh;

  signal_status_manager::SignalStatusManager ssm(nh);

  ros::spin();
}