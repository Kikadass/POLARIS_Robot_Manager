#include <robot_manager_msgs/BatteryStatus.h>
#include <robot_manager_msgs/GpsAccuracyStatus.h>
#include <robot_manager_msgs/NavigationStatus.h>
#include <robot_manager_msgs/SignalStatus.h>
#include <robot_manager_msgs/TemperatureStatus.h>
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
    TEMPERATURE,
    NAVIGATION
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

    void EmergencyButtonPressedCB(const std_msgs::Bool::ConstPtr& msg);
    void NavigationStatusCB(const robot_manager_msgs::NavigationStatus::ConstPtr& msg);

    void SetStatus(const unsigned int robotStatus);

    ERROR_TYPE determineErrorType(const robot_manager_msgs::BatteryStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const robot_manager_msgs::GpsAccuracyStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const robot_manager_msgs::SignalStatus::ConstPtr& msg) const;
    ERROR_TYPE determineErrorType(const robot_manager_msgs::TemperatureStatus::ConstPtr& msg) const;

   private:
    ros::Publisher m_pub;
    ros::Subscriber m_batteryStatusSub;
    ros::Subscriber m_emergencyButtonSub;
    ros::Subscriber m_gpsAccuracyStatusSub;
    ros::Subscriber m_signalStatusSub;
    ros::Subscriber m_temperatureStatusSub;
    ros::Subscriber m_navigationStatusSub;

    std::set<ERROR_TYPE> m_errors;

    unsigned int m_status;
  };
}  // namespace robot_status_manager
