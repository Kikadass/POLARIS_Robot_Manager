#include "SignalStatusManager.h"

#include <robot_manager_msgs/SignalStatus.h>
#include <robot_manager_msgs/SignalStrength.h>

namespace signal_status_manager
{
  /**
   * @brief Construct a new Signal Status Manager:: Signal Status Manager object
   *
   * This also initializes two timers that only run once (oneshot = true),
   * and are stopped initially (autostart = false).
   *
   * @param nh NodeHandle to create publishers, subscribers and timers from.
   */
  SignalStatusManager::SignalStatusManager(ros::NodeHandle& nh)
      : m_pub(nh.advertise<robot_manager_msgs::SignalStatus>("signal_status", 1000))
      , m_sub(nh.subscribe("signal_strength", 1000, &SignalStatusManager::SignalStrengthCB, this))
      , m_lowSignalTimer(nh.createTimer(ros::Duration(20.0), &SignalStatusManager::TimerCB, this, true, false))
      , m_noSignalTimer(nh.createTimer(ros::Duration(10.0), &SignalStatusManager::TimerCB, this, true, false))
      , m_status(robot_manager_msgs::SignalStatus::UNKNOWN)
  {
  }

  void SignalStatusManager::SignalStrengthCB(const robot_manager_msgs::SignalStrength::ConstPtr& msg)
  {
    const unsigned int signalStrength = msg->strength;

    ROS_DEBUG("SignalStrength received: %d", signalStrength);

    // If status is error, only listen to SignalStrength::STABLE to get out of the error
    if ((m_status != robot_manager_msgs::SignalStatus::ERROR)
        || (signalStrength == robot_manager_msgs::SignalStrength::STABLE))
    {
      switch (signalStrength)
      {
        case robot_manager_msgs::SignalStrength::NO_SIGNAL:
        {
          ROS_WARN("No Signal");
          m_status = robot_manager_msgs::SignalStatus::UNSTABLE;

          if (!m_noSignalTimer.hasStarted()) m_noSignalTimer.start();
        }
        break;
        case robot_manager_msgs::SignalStrength::STABLE:
        {
          if (m_status == robot_manager_msgs::SignalStatus::ERROR)
            ROS_INFO("Signal ERROR: Cleared. Back to stable connection.");
          m_status = robot_manager_msgs::SignalStatus::STABLE;
          StopTimers();
        }
        break;
        case robot_manager_msgs::SignalStrength::LOW:
        {
          ROS_WARN("Low Signal");
          m_status = robot_manager_msgs::SignalStatus::UNSTABLE;

          if (!m_lowSignalTimer.hasStarted()) m_lowSignalTimer.start();
          if (m_noSignalTimer.hasStarted()) m_noSignalTimer.stop();
        }
        break;
        default:
          ROS_ERROR("Unexpected signal: %d", signalStrength);
          break;
      }
    }

    PublishStatus();
  }

  void SignalStatusManager::TimerCB(const ros::TimerEvent& event)
  {
    ROS_ERROR("Signal ERROR: Too long in unstable or no signal.");

    m_status = robot_manager_msgs::SignalStatus::ERROR;
    PublishStatus();
    StopTimers();
  }

  void SignalStatusManager::PublishStatus() const
  {
    robot_manager_msgs::SignalStatus ts;
    ts.status = m_status;
    m_pub.publish(ts);
  }

  void SignalStatusManager::StopTimers()
  {
    m_noSignalTimer.stop();
    m_lowSignalTimer.stop();
  }

}  // namespace signal_status_manager
