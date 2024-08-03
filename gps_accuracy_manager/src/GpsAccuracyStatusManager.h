#include <ros/ros.h>
#include <std_msgs/UInt64.h>

namespace gps_accuracy_manager
{
  class GpsAccuracyStatusManager
  {
   public:
    GpsAccuracyStatusManager(ros::NodeHandle& nh);

   private:
    void GpsAccuracyCB(const std_msgs::UInt64::ConstPtr& msg);

    void TimerCB(const ros::TimerEvent& event);

    void PublishStatus() const;

   private:
    const unsigned int m_highAccuracy;
    const unsigned int m_lowAccuracy;

    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    ros::Timer m_lowAccuracyTimer;

    unsigned int m_status;
  };
}  // namespace gps_accuracy_manager
