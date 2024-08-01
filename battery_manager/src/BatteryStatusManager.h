#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace battery_manager
{
  class BatteryStatusManager
  {
   public:
    BatteryStatusManager(const ros::Publisher& pub);

    void BatteryPercentageCB(const std_msgs::Float32::ConstPtr& msg);

   private:
    ros::Publisher m_pub;
    unsigned int m_status;
  };
}  // namespace battery_manager
