#include <robot_manager_msgs/SignalStrength.h>
#include <ros/ros.h>

namespace signal_status_manager
{
  class SignalStatusManager
  {
   public:
    SignalStatusManager(ros::NodeHandle& nh);

   private:
    void SignalStrengthCB(const robot_manager_msgs::SignalStrength::ConstPtr& msg);

    void TimerCB(const ros::TimerEvent& event);

    void PublishStatus() const;

    void StopTimers();

   private:
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    ros::Timer m_lowSignalTimer;
    ros::Timer m_noSignalTimer;

    unsigned int m_status;
  };
}  // namespace signal_status_manager
