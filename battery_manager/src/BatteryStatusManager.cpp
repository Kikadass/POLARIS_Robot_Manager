#include "BatteryStatusManager.h"

#include "battery_manager/BatteryStatus.h"

namespace battery_manager
{
  BatteryStatusManager::BatteryStatusManager(const ros::Publisher& pub)
      : m_pub(pub)
      , m_status(BatteryStatus::UNKNOWN)
  {
  }

  void BatteryStatusManager::BatteryPercentageCB(const std_msgs::Float32::ConstPtr& msg)
  {
    const float percentage = msg->data;

    ROS_DEBUG("Battery percentage received: %f", percentage);

    if (percentage > 100.0f || percentage < 0.0f)
    {
      ROS_ERROR("Invalid battery percentage: %f", percentage);
      m_status = BatteryStatus::ERROR;
    }
    else if (percentage > 50.0f)
    {
      m_status = BatteryStatus::NORMAL;
    }
    else
    {
      m_status = BatteryStatus::ERROR;
      ROS_WARN("Battery is low.");
    }

    BatteryStatus bs;
    bs.status = m_status;
    m_pub.publish(bs);
  }

}  // namespace battery_manager
