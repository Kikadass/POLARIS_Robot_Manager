#include "RobotStatusManager.h"

#include "robot_status_manager/RobotStatus.h"

#include <functional>
#include <memory>

namespace robot_status_manager
{
  RobotStatusManager::RobotStatusManager(ros::NodeHandle& nh)
      : m_pub(nh.advertise<RobotStatus>("robot_status", 1000))
      , m_batteryStatusSub(nh.subscribe<battery_manager::BatteryStatus>(
            "battery_status", 1000, &RobotStatusManager::CheckForErrorsCB<battery_manager::BatteryStatus>, this))
      , m_emergencyButtonSub(
            nh.subscribe("emergency_button_pressed", 1000, &RobotStatusManager::EmergencyButtonPressedCB, this))
      , m_gpsAccuracyStatusSub(nh.subscribe<gps_accuracy_manager::GpsAccuracyStatus>(
            "gps_accuracy_status",
            1000,
            &RobotStatusManager::CheckForErrorsCB<gps_accuracy_manager::GpsAccuracyStatus>,
            this))
      , m_signalStatusSub(nh.subscribe<signal_status_manager::SignalStatus>(
            "signal_status", 1000, &RobotStatusManager::CheckForErrorsCB<signal_status_manager::SignalStatus>, this))
      , m_temperatureStatusSub(nh.subscribe<temperature_manager::TemperatureStatus>(
            "temperature_status",
            1000,
            &RobotStatusManager::CheckForErrorsCB<temperature_manager::TemperatureStatus>,
            this))
      , m_status(RobotStatus::IDLE)
  {
  }

  void RobotStatusManager::PublishRobotStatus() const
  {
    RobotStatus msg;
    msg.status = (m_errors.empty()) ? RobotStatus::IDLE : RobotStatus::ERROR;
    m_pub.publish(msg);

    ROS_INFO("Published robot status: %d", msg.status);
  }

  void RobotStatusManager::EmergencyButtonPressedCB(const std_msgs::Bool::ConstPtr& msg)
  {
    // If the button is pressed, add the error. Otherwise, remove the error
    if (msg->data)
    {
      m_errors.insert(ERROR_TYPE::EMERGENCY_BUTTON);
    }
    else
    {
      m_errors.erase(ERROR_TYPE::EMERGENCY_BUTTON);
    }
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const battery_manager::BatteryStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::BATTERY;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const gps_accuracy_manager::GpsAccuracyStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::GPS_ACCURACY;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const signal_status_manager::SignalStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::SIGNAL;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const temperature_manager::TemperatureStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::TEMPERATURE;
  }

}  // namespace robot_status_manager
