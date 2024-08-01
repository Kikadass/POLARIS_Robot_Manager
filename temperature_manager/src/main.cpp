#include "temperature_manager/TemperatureStatus.h"
#include "TemperatureStatusManager.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temperature_manager");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<temperature_manager::TemperatureStatus>("temperature_status", 1000);
  temperature_manager::TemperatureStatusManager tsm(pub);

  ros::Subscriber sub =
      nh.subscribe("temperature", 1000, &temperature_manager::TemperatureStatusManager::TemperatureCB, &tsm);

  ros::spin();
}