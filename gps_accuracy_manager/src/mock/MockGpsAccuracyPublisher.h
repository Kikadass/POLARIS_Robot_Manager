#include <ros/ros.h>

namespace gps_accuracy_manager
{
  /**
   * @brief This class will mock the gps accuracy by publishing on the "gps_accuracy" topic
   *
   * @details
   * It runs through a random set timer between 5 and 20 seconds publishing the same random accuracy during that time.
   * Once the timer is over, it will change the timer duration and accuracy value.
   *
   */
  class GpsAccuracyPublisher
  {
   public:
    /**
     * @brief Construct a new Signal Strength Publisher object
     *
     * @param nh NodeHandle to create timers and publisher.
     * @param noSignal Whether to us only NO_SIGNAL or also use LOW.
     */
    GpsAccuracyPublisher(ros::NodeHandle& nh);

    void PublishGpsAccuracy();

   private:
    void ChangeAccuracy(const ros::TimerEvent& event);

    ros::Publisher m_pub;

    ros::Timer m_timer;

    unsigned int m_currentAccuracy;
  };
}  // namespace gps_accuracy_manager
