#include "SignalStatusManager.h"

#include "signal_status_manager/SignalStatus.h"
#include "signal_status_manager/SignalStrength.h"

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
      : m_pub(nh.advertise<signal_status_manager::SignalStatus>("signal_status", 1000))
      , m_sub(nh.subscribe("signal_strength", 1000, &SignalStatusManager::SignalStrengthCB, this))
      , m_lowSignalTimer(nh.createTimer(ros::Duration(20.0), &SignalStatusManager::TimerCB, this, true, false))
      , m_noSignalTimer(nh.createTimer(ros::Duration(10.0), &SignalStatusManager::TimerCB, this, true, false))
      , m_status(SignalStatus::UNKNOWN)
  {
  }

  void SignalStatusManager::SignalStrengthCB(const signal_status_manager::SignalStrength::ConstPtr& msg)
  {
    const unsigned int signalStrength = msg->strength;

    ROS_DEBUG("SignalStrength received: %d", signalStrength);

    // If status is error, only listen to SignalStrength::STABLE to get out of the error
    if ((m_status != SignalStatus::ERROR) || (signalStrength == SignalStrength::STABLE))
    {
      switch (signalStrength)
      {
        case SignalStrength::NO_SIGNAL:
        {
          ROS_WARN("No Signal");
          m_status = SignalStatus::UNSTABLE;

          if (!m_noSignalTimer.hasStarted()) m_noSignalTimer.start();
        }
        break;
        case SignalStrength::STABLE:
        {
          if (m_status == SignalStatus::ERROR) ROS_INFO("Signal ERROR: Cleared. Back to stable connection.");
          m_status = SignalStatus::STABLE;
          StopTimers();
        }
        break;
        case SignalStrength::LOW:
        {
          ROS_WARN("Low Signal");
          m_status = SignalStatus::UNSTABLE;

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

    m_status = SignalStatus::ERROR;
    PublishStatus();
    StopTimers();
  }

  void SignalStatusManager::PublishStatus() const
  {
    SignalStatus ts;
    ts.status = m_status;
    m_pub.publish(ts);
  }

  void SignalStatusManager::StopTimers()
  {
    m_noSignalTimer.stop();
    m_lowSignalTimer.stop();
  }

}  // namespace signal_status_manager
