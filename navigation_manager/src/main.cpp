#include "NavigationStatusManager.h"

#include <robot_manager_msgs/NavigationStatus.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_manager");
  ros::NodeHandle nh;
  navigation_manager::NavigationStatusManager nsm(nh);

  ros::Rate rate(1);  // Publish at 1 Hz
  while (ros::ok())
  {
    nsm.PublishNavigationStatus();

    ros::spinOnce();
    rate.sleep();
  }
}
