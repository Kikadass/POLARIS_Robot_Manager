#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace temperature_manager
{
  class TemperatureStatusManager
  {
   public:
    TemperatureStatusManager(const ros::Publisher& pub);

    void TemperatureCB(const std_msgs::Float32::ConstPtr& msg);

   private:
    ros::Publisher m_pub;
    unsigned int m_status;
  };
}  // namespace temperature_manager
