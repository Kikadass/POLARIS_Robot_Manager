#include "NavigationStatusManager.h"

#include <robot_manager_msgs/NavigationStatus.h>

namespace navigation_manager
{
  NavigationStatusManager::NavigationStatusManager(ros::NodeHandle& nh)
      : m_pub(nh.advertise<robot_manager_msgs::NavigationStatus>("navigation_status", 1000))
      , m_sub(nh.subscribe("robot_status", 1000, &NavigationStatusManager::RobotStatusCB, this))
      , m_status(robot_manager_msgs::NavigationStatus::IDLE)
  {
  }

  void NavigationStatusManager::PublishNavigationStatus() const
  {
    robot_manager_msgs::NavigationStatus msg;
    msg.status = m_status;
    m_pub.publish(msg);

    ROS_INFO("Published navigation status: %d", msg.status);
  }

  void NavigationStatusManager::RobotStatusCB(const robot_manager_msgs::RobotStatus::ConstPtr& msg)
  {
    ROS_DEBUG("Robot status received: %u", msg->status);
    bool changed = false;

    // For readability
    using namespace robot_manager_msgs;

    switch (msg->status)
    {
      case RobotStatus::IDLE:
        changed = (m_status != NavigationStatus::RUNNING);
        m_status = NavigationStatus::RUNNING;
        break;
      case RobotStatus::RUNNING:
        break;
      case RobotStatus::ERROR:
        changed = (m_status != NavigationStatus::IDLE);
        m_status = NavigationStatus::IDLE;
        break;
      default:
        ROS_WARN("Unexpected robot status: %u", msg->status);
        break;
    }

    // Only publish when there has been a change.
    if (changed)
    {
      NavigationStatus ns;
      ns.status = m_status;
      m_pub.publish(ns);

      ROS_INFO("Changed navigation status to: %u", m_status);
    }
  }

}  // namespace navigation_manager
