#include "battery_manager/BatteryStatus.h"
#include "BatteryStatusManager.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_manager");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<battery_manager::BatteryStatus>("battery_status", 1000);
  battery_manager::BatteryStatusManager bsm(pub);

  ros::Subscriber sub =
      nh.subscribe("battery_percentage", 1000, &battery_manager::BatteryStatusManager::BatteryPercentageCB, &bsm);

  ros::spin();
}