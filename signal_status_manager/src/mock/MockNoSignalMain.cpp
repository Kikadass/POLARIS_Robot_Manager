/**
 * @file MockNoSignalMain.cpp
 * @author Enrique Piera Serra (enriquepiera8@hotmail.com)
 * @brief This is a small main file to trigger the SignalStrengthPublisher using only NO_SIGNAL
 * @version 0.1
 * @date 2024-08-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "MockSignalStrengthPublisher.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signal_strength_publisher");
  ros::NodeHandle nh;
  const bool noSignal = true;
  signal_status_manager::SignalStrengthPublisher signal_strength_publisher(nh, noSignal);

  ros::Rate rate(1);  // Publish at 1 Hz
  while (ros::ok())
  {
    signal_strength_publisher.PublishSignalStrength();

    ros::spinOnce();
    rate.sleep();
  }
}