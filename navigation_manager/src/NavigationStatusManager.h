#include <robot_manager_msgs/RobotStatus.h>
#include <ros/ros.h>

namespace navigation_manager
{
  class NavigationStatusManager
  {
   public:
    NavigationStatusManager(ros::NodeHandle& nh);

    void PublishNavigationStatus() const;

   private:
    void RobotStatusCB(const robot_manager_msgs::RobotStatus::ConstPtr& msg);

   private:
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    unsigned int m_status;
  };
}  // namespace navigation_manager
