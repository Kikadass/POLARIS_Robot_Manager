#include "GpsAccuracyStatusManager.h"

#include "gps_accuracy_manager/GpsAccuracyStatus.h"

namespace gps_accuracy_manager
{
  GpsAccuracyStatusManager::GpsAccuracyStatusManager(ros::NodeHandle& nh)
      : m_highAccuracy(100u)
      , m_lowAccuracy(200u)
      , m_pub(nh.advertise<GpsAccuracyStatus>("gps_accuracy_status", 1000))
      , m_sub(nh.subscribe("gps_accuracy", 1000, &GpsAccuracyStatusManager::GpsAccuracyCB, this))
      , m_lowAccuracyTimer(nh.createTimer(ros::Duration(15.0), &GpsAccuracyStatusManager::TimerCB, this, true, false))
      , m_status(GpsAccuracyStatus::UNKNOWN)
  {
  }

  void GpsAccuracyStatusManager::GpsAccuracyCB(const std_msgs::UInt64::ConstPtr& msg)
  {
    const unsigned int accuracy = msg->data;

    ROS_DEBUG("GpsAccuracy received (mm): %u", accuracy);

    if ((m_status == GpsAccuracyStatus::ERROR) && (accuracy <= m_lowAccuracy))
    {
      ROS_INFO("GPS Accuracy ERROR: Cleared with accuracy (mm): %u", accuracy);
    }

    if (accuracy <= m_highAccuracy)
    {
      m_status = GpsAccuracyStatus::HIGH;
      m_lowAccuracyTimer.stop();
    }
    else if (accuracy <= m_lowAccuracy)
    {
      m_status = GpsAccuracyStatus::NORMAL;
      m_lowAccuracyTimer.stop();
    }
    else if (m_status == GpsAccuracyStatus::ERROR)
    {
      ROS_ERROR("ERROR: LOW GPS Accuracy (mm): %u", accuracy);
      m_lowAccuracyTimer.stop();
    }
    else
    {
      ROS_WARN("LOW GPS Accuracy (mm): %u", accuracy);
      m_status = GpsAccuracyStatus::LOW;

      if (!m_lowAccuracyTimer.hasStarted()) m_lowAccuracyTimer.start();
    }

    PublishStatus();
  }

  void GpsAccuracyStatusManager::TimerCB(const ros::TimerEvent& event)
  {
    ROS_ERROR("GPS Accuracy ERROR: Too long with low accuracy.");

    m_status = GpsAccuracyStatus::ERROR;
    m_lowAccuracyTimer.stop();
    PublishStatus();
  }

  void GpsAccuracyStatusManager::PublishStatus() const
  {
    GpsAccuracyStatus statusMsg;
    statusMsg.status = m_status;
    m_pub.publish(statusMsg);
  }
}  // namespace gps_accuracy_manager
