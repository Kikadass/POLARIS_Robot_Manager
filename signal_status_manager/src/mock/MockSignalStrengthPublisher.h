/**
 * @file MockSignalStrengthPublisher.h
 * @author Enrique Piera Serra (enriquepiera8@hotmail.com)
 * @version 0.1
 * @date 2024-08-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <ros/ros.h>

namespace signal_status_manager
{
  /**
   * @brief This class will mock the signal strength by publishing on the "signal_strength" topic
   *
   * This mock can test that:
   * - occasional drops of connection are acceptable
   * - ERROR should get triggered on the unstable phase
   * - When the connection goes back to STABLE after ERROR, the SignalStatusManager should leave the ERROR state.
   *
   * @details
   * First it will publish a stable connection for a few seconds, while occasionally dropping to LOW or NO_SIGNAL, but
   * coming back to STABLE. After some time it will go fully unstable, publishing only NO_SIGNAL or a mix of NO_SIGNAL
   * and LOW signals (depending on what the user chose)
   *
   * If noSignal was chosen, it will be in NO_SIGNAL for 15s. As the SignalStatusManager only needs 10s to trigger an
   * ERROR state.
   *
   * If noSignal is not chosen, it will use both NO_SIGNAL and LOW signals for 25s, as the
   * SignalStatusManager would need at least 20s to declare it ERROR state.
   *
   * After the unstable state, it will go back to Stable and repeat the sequence again.
   *
   */
  class SignalStrengthPublisher
  {
   public:
    /**
     * @brief Construct a new Signal Strength Publisher object
     *
     * @param nh NodeHandle to create timers and publisher.
     * @param noSignal Whether to us only NO_SIGNAL or also use LOW.
     */
    SignalStrengthPublisher(ros::NodeHandle& nh, const bool noSignal = false);

    void PublishSignalStrength();

   private:
    void StartUnstableConnection(const ros::TimerEvent& event);

    void StopUnstableConnection(const ros::TimerEvent& event);

    ros::Publisher m_pub;

    ros::Timer m_stableStrengthTimer;
    ros::Timer m_unstableStrengthTimer;

    bool m_stable;
    const bool m_noSignal;
  };
}  // namespace signal_status_manager
