#include "battery_manager/BatteryStatus.h"
#include "gps_accuracy_manager/GpsAccuracyStatus.h"
#include "signal_status_manager/SignalStatus.h"
#include "temperature_manager/TemperatureStatus.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <vector>

namespace robot_status_manager
{
  enum class ERROR_TYPE
  {
    BATTERY,
    EMERGENCY_BUTTON,
    GPS_ACCURACY,
    SIGNAL,
    TEMPERATURE
  };

  class RobotStatusManager
  {
   public:
    RobotStatusManager(ros::NodeHandle& nh);

    void PublishRobotStatus() const;

   private:
    template <typename MessageType>
    void CheckForErrorsCB(const typename MessageType::ConstPtr& msg)
    {
      ERROR_TYPE type = determineErrorType(msg);

      if (msg->status == MessageType::ERROR)
      {
        m_errors.insert(type);
      }
      else
      {
        m_errors.erase(type);
      }
    }

    // void BatteryStatusCB(const battery_manager::BatteryStatus::ConstPtr& msg);
    // void GpsAccuracyStatusCB(const gps_accuracy_manager::GpsAccuracyStatus::ConstPtr& msg);
    // void SignalStatusCB(const signal_status_manager::SignalStatus::ConstPtr& msg);
    // void TemperatureStatusCB(const temperature_manager::TemperatureStatus::ConstPtr& msg);

    void EmergencyButtonPressedCB(const std_msgs::Bool::ConstPtr& msg);

    ERROR_TYPE determineErrorType(const battery_manager::BatteryStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const gps_accuracy_manager::GpsAccuracyStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const signal_status_manager::SignalStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const temperature_manager::TemperatureStatus::ConstPtr& msg) const;

   private:
    ros::Publisher m_pub;
    ros::Subscriber m_batteryStatusSub;
    ros::Subscriber m_emergencyButtonSub;
    ros::Subscriber m_gpsAccuracyStatusSub;
    ros::Subscriber m_signalStatusSub;
    ros::Subscriber m_temperatureStatusSub;

    std::set<ERROR_TYPE> m_errors;

    unsigned int m_status;
  };
}  // namespace robot_status_manager
