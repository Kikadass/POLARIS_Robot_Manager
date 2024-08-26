#include "RobotStatusManager.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_status_manager");
  ros::NodeHandle nh;
  robot_status_manager::RobotStatusManager rsm(nh);

  ros::Rate rate(1);  // Publish at 1 Hz
  while (ros::ok())
  {
    rsm.Update();

    ros::spinOnce();
    rate.sleep();
  }
}