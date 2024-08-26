#include "RobotStatusManager.h"

#include <robot_manager_msgs/RobotStatus.h>

#include <functional>
#include <memory>
#include <unordered_map>

namespace robot_status_manager
{
  RobotStatusManager::RobotStatusManager(ros::NodeHandle& nh)
      : m_pub(nh.advertise<robot_manager_msgs::RobotStatus>("robot_status", 1000))
      , m_batteryStatusSub(nh.subscribe<robot_manager_msgs::BatteryStatus>(
            "battery_status", 1000, &RobotStatusManager::CheckForErrorsCB<robot_manager_msgs::BatteryStatus>, this))
      , m_emergencyButtonSub(
            nh.subscribe("emergency_button_pressed", 1000, &RobotStatusManager::EmergencyButtonPressedCB, this))
      , m_gpsAccuracyStatusSub(nh.subscribe<robot_manager_msgs::GpsAccuracyStatus>(
            "gps_accuracy_status",
            1000,
            &RobotStatusManager::CheckForErrorsCB<robot_manager_msgs::GpsAccuracyStatus>,
            this))
      , m_signalStatusSub(nh.subscribe<robot_manager_msgs::SignalStatus>(
            "signal_status", 1000, &RobotStatusManager::CheckForErrorsCB<robot_manager_msgs::SignalStatus>, this))
      , m_temperatureStatusSub(nh.subscribe<robot_manager_msgs::TemperatureStatus>(
            "temperature_status",
            1000,
            &RobotStatusManager::CheckForErrorsCB<robot_manager_msgs::TemperatureStatus>,
            this))
      , m_navigationStatusSub(nh.subscribe("navigation_status", 1000, &RobotStatusManager::NavigationStatusCB, this))
      , m_status(robot_manager_msgs::RobotStatus::IDLE)
  {
  }

  void RobotStatusManager::Update()
  {
    if (!m_errors.empty())
    {
      m_status = robot_manager_msgs::RobotStatus::ERROR;
    }
    else if (m_status == robot_manager_msgs::RobotStatus::ERROR)
    {
      m_status = robot_manager_msgs::RobotStatus::IDLE;
    }

    PublishRobotStatus();
  }

  void RobotStatusManager::PublishRobotStatus() const
  {
    robot_manager_msgs::RobotStatus msg;
    msg.status = m_status;
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

  void RobotStatusManager::NavigationStatusCB(const robot_manager_msgs::NavigationStatus::ConstPtr& msg)
  {
    if (msg->status == robot_manager_msgs::NavigationStatus::ERROR)
    {
      m_errors.insert(ERROR_TYPE::NAVIGATION);
    }
    else
    {
      if (m_errors.count(ERROR_TYPE::NAVIGATION) > 0)
      {
        m_errors.erase(ERROR_TYPE::NAVIGATION);
      }

      using namespace robot_manager_msgs;
      const std::unordered_map<unsigned int, unsigned int> statusMap = {
          {NavigationStatus::RUNNING, RobotStatus::RUNNING},
          {NavigationStatus::IDLE, RobotStatus::IDLE},
          {NavigationStatus::ERROR, RobotStatus::ERROR}};

      SetStatus(statusMap.at(msg->status));
    }
  }

  void RobotStatusManager::SetStatus(const unsigned int robotStatus)
  {
    // Do not change status if there are ERRORS.
    if (m_errors.empty()) m_status = robotStatus;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const robot_manager_msgs::BatteryStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::BATTERY;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const robot_manager_msgs::GpsAccuracyStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::GPS_ACCURACY;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const robot_manager_msgs::SignalStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::SIGNAL;
  }

  ERROR_TYPE RobotStatusManager::determineErrorType(const robot_manager_msgs::TemperatureStatus::ConstPtr& msg) const
  {
    return ERROR_TYPE::TEMPERATURE;
  }

}  // namespace robot_status_manager
