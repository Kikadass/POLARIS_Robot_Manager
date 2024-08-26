#include "MockSignalStrengthPublisher.h"

#include <robot_manager_msgs/SignalStrength.h>
#include <ros/ros.h>

#include <cstdlib>
#include <ctime>

namespace signal_status_manager
{
  SignalStrengthPublisher::SignalStrengthPublisher(ros::NodeHandle& nh, const bool noSignal)
      : m_pub(nh.advertise<robot_manager_msgs::SignalStrength>("signal_strength", 1000))
      , m_stableStrengthTimer(
            nh.createTimer(ros::Duration(30.0), &SignalStrengthPublisher::StartUnstableConnection, this, true, true))
      , m_unstableStrengthTimer(nh.createTimer(
            ros::Duration(noSignal ? 15.0 : 25.0), &SignalStrengthPublisher::StopUnstableConnection, this, true, false))
      , m_stable(true)  // Start in STABLE with acceptable temporary drops
      , m_noSignal(noSignal)
  {
    m_stableStrengthTimer.start();
    ROS_INFO("Start timer.");

    srand(static_cast<unsigned int>(time(0)));  // Seed for randomness
  }

  void SignalStrengthPublisher::StartUnstableConnection(const ros::TimerEvent& event)
  {
    ROS_INFO("Start unstable connection.");
    m_stable = false;
    m_stableStrengthTimer.stop();
    m_unstableStrengthTimer.start();
  }

  void SignalStrengthPublisher::StopUnstableConnection(const ros::TimerEvent& event)
  {
    ROS_INFO("Start stable connection.");
    m_stable = true;
    m_stableStrengthTimer.start();
    m_unstableStrengthTimer.stop();
  }

  void SignalStrengthPublisher::PublishSignalStrength()
  {
    robot_manager_msgs::SignalStrength msg;

    if (m_stable && (rand() % 10 != 0))
    {
      msg.strength = robot_manager_msgs::SignalStrength::STABLE;
    }
    else
    {
      msg.strength = (m_noSignal || (rand() % 2 == 0)) ? robot_manager_msgs::SignalStrength::NO_SIGNAL
                                                       : robot_manager_msgs::SignalStrength::LOW;
    }

    m_pub.publish(msg);
    ROS_INFO("Published signal strength: %d", msg.strength);
  }
}  // namespace signal_status_manager
