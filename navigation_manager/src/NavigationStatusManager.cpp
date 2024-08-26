#include "NavigationStatusManager.h"

#include "navigation_manager/NavigationStatus.h"

namespace navigation_manager
{
  NavigationStatusManager::NavigationStatusManager(ros::NodeHandle& nh)
      : m_pub(nh.advertise<NavigationStatus>("navigation_status", 1000))
      , m_sub(nh.subscribe("robot_status", 1000, &NavigationStatusManager::RobotStatusCB, this))
      , m_status(NavigationStatus::IDLE)
  {
  }

  void NavigationStatusManager::PublishNavigationStatus() const
  {
    NavigationStatus msg;
    msg.status = m_status;
    m_pub.publish(msg);

    ROS_INFO("Published navigation status: %d", msg.status);
  }

  void NavigationStatusManager::RobotStatusCB(const robot_manager_msgs::RobotStatus::ConstPtr& msg)
  {
    ROS_DEBUG("Robot status received: %u", msg->status);

    switch (msg->status)
    {
      case robot_manager_msgs::RobotStatus::IDLE:
        m_status = NavigationStatus::RUNNING;
        break;
      case robot_manager_msgs::RobotStatus::RUNNING:
        break;
      case robot_manager_msgs::RobotStatus::ERROR:
        m_status = NavigationStatus::IDLE;
        break;
      default:
        ROS_WARN("Unexpected robot status: %u", msg->status);
        break;
    }

    NavigationStatus ns;
    ns.status = m_status;
    m_pub.publish(ns);
    ROS_INFO("Changed navigation status: %u", m_status);
  }

}  // namespace navigation_manager
