#include <ros/ros.h>
#include <signal_status_manager/SignalStrength.h>

#include <cstdlib>
#include <ctime>

namespace signal_status_manager
{
  class SignalStrengthPublisher
  {
   public:
    SignalStrengthPublisher(ros::NodeHandle& nh)
        : m_pub(nh.advertise<signal_status_manager::SignalStrength>("signal_strength", 1000))
        , m_stableStrengthTimer(
              nh.createTimer(ros::Duration(30.0), &SignalStrengthPublisher::StartUnstableConnection, this, true, true))
        , m_lowStrengthTimer(
              nh.createTimer(ros::Duration(20.0), &SignalStrengthPublisher::StopUnstableConnection, this, true, false))
        , m_stable(true)  // Start in STABLE with acceptable temporary drops
    {
      m_stableStrengthTimer.start();
      ROS_INFO("Start timer.");

      srand(static_cast<unsigned int>(time(0)));  // Seed for randomness
    }

    void StartUnstableConnection(const ros::TimerEvent& event)
    {
      ROS_INFO("Start unstable connection.");
      m_stable = false;
      m_stableStrengthTimer.stop();
      m_lowStrengthTimer.start();
    }

    void StopUnstableConnection(const ros::TimerEvent& event)
    {
      ROS_INFO("Start stable connection.");
      m_stable = true;
      m_stableStrengthTimer.start();
      m_lowStrengthTimer.stop();
    }

    void publishSignal()
    {
      ros::Rate rate(1);  // Publish at 1 Hz
      while (ros::ok())
      {
        SignalStrength msg;

        if (m_stable && (rand() % 10 != 0))
        {
          msg.strength = SignalStrength::STABLE;
        }
        else
        {
          msg.strength = (rand() % 2 == 0) ? SignalStrength::LOW : SignalStrength::NO_SIGNAL;
        }

        m_pub.publish(msg);
        ROS_INFO("Published signal strength: %d", msg.strength);
        ros::spinOnce();
        rate.sleep();
      }
    }

   private:
    ros::Publisher m_pub;

    ros::Timer m_stableStrengthTimer;
    ros::Timer m_lowStrengthTimer;

    bool m_stable;
  };
}  // namespace signal_status_manager

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signal_strength_publisher");
  ros::NodeHandle nh;
  signal_status_manager::SignalStrengthPublisher signal_strength_publisher(nh);
  signal_strength_publisher.publishSignal();
  return 0;
}