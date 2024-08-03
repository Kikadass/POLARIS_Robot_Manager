#include "MockGpsAccuracyPublisher.h"

#include <ros/ros.h>
#include <std_msgs/UInt64.h>

#include <cstdlib>
#include <ctime>

namespace gps_accuracy_manager
{
  GpsAccuracyPublisher::GpsAccuracyPublisher(ros::NodeHandle& nh)
      : m_pub(nh.advertise<std_msgs::UInt64>("gps_accuracy", 1000))
      , m_timer(nh.createTimer(ros::Duration(10.0), &GpsAccuracyPublisher::ChangeAccuracy, this, false, true))
      , m_currentAccuracy(50u)
  {
    m_timer.start();
    ROS_INFO("Start timer.");

    srand(static_cast<unsigned int>(time(0)));  // Seed for randomness
  }

  void GpsAccuracyPublisher::ChangeAccuracy(const ros::TimerEvent& event)
  {
    // Generate a random number between 5 and 20
    const int newDuration = std::rand() % (20 - 5 + 1) + 5;

    // Generate a random number between 50 and 300
    m_currentAccuracy = std::rand() % (300 - 50 + 1) + 50;

    m_timer.stop();
    m_timer.setPeriod(ros::Duration(newDuration));
    m_timer.start();

    ROS_INFO("Change timer to (s): %u", newDuration);
    ROS_INFO("Change accuracy to (mm): %u", m_currentAccuracy);

    PublishGpsAccuracy();
  }

  void GpsAccuracyPublisher::PublishGpsAccuracy()
  {
    std_msgs::UInt64 msg;
    msg.data = m_currentAccuracy;
    m_pub.publish(msg);

    ROS_INFO("Published gps accuracy: %d", m_currentAccuracy);
  }
}  // namespace gps_accuracy_manager
