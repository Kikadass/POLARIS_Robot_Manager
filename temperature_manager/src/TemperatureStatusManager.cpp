#include "TemperatureStatusManager.h"

#include <robot_manager_msgs/TemperatureStatus.h>

namespace temperature_manager
{
  TemperatureStatusManager::TemperatureStatusManager(const ros::Publisher& pub)
      : m_pub(pub)
      , m_status(robot_manager_msgs::TemperatureStatus::UNKNOWN)
  {
  }

  void TemperatureStatusManager::TemperatureCB(const std_msgs::Float32::ConstPtr& msg)
  {
    const float celsius = msg->data;

    ROS_DEBUG("Temperature received (C): %f", celsius);

    if (celsius > 55.0f)
    {
      ROS_WARN("High temperature (C): %f", celsius);
      m_status = robot_manager_msgs::TemperatureStatus::ERROR;
    }
    else if (celsius < -10.0f)  // As per Polaris Gem e2 specs
    {
      ROS_WARN("Low temperature (C): %f", celsius);
      m_status = robot_manager_msgs::TemperatureStatus::ERROR;
    }
    else
    {
      m_status = robot_manager_msgs::TemperatureStatus::NORMAL;
    }

    robot_manager_msgs::TemperatureStatus ts;
    ts.status = m_status;
    m_pub.publish(ts);
  }

}  // namespace temperature_manager
