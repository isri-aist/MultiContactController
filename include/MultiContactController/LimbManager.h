#pragma once

#include <deque>
#include <unordered_map>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/ImpedanceGains.h>

#include <TrajColl/CubicInterpolator.h>
#include <TrajColl/CubicSpline.h>

#include <MultiContactController/ContactTypes.h>
#include <MultiContactController/LimbTypes.h>
#include <MultiContactController/RobotUtils.h>

namespace MCC
{
class MultiContactController;
class SwingTraj;

/** \brief Limb manager.

    Limb manager generates a swing limb trajectory from a specified contact command sequence.
*/
class LimbManager
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "LimbManager";

    //! Limb task gains
    TaskGain taskGain = TaskGain(sva::MotionVecd(Eigen::Vector6d::Constant(1000)));

    //! Default swing trajectory type
    std::string defaultSwingTrajType = "CubicSplineSimple";

    //! Policy for determining the start pose of the swing trajectory
    std::string swingStartPolicy = "IK";

    //! Whether to overwrite landing pose so that the relative pose from support limb to swing limb is retained
    bool overwriteLandingPose = false;

    //! Whether to stop swing trajectory for touch down limb
    bool stopSwingTrajForTouchDownLimb = true;

    //! Whether to keep limb pose of touch down limb during support phase
    bool keepPoseForTouchDownLimb = false;

    //! Whether to enable wrench distribution for touch down limb
    bool enableWrenchDistForTouchDownLimb = true;

    //! Thresholds for touch down detection
    //! @{
    double touchDownRemainingDuration = 0.2; // [sec]
    double touchDownPosError = 0.05; // [m]
    double touchDownForceZ = 50; // [N]
    //! @}

    //! Impedance gains for limb tasks
    std::unordered_map<std::string, mc_tasks::force::ImpedanceGains> impGains = {
        {"SingleContact", mc_tasks::force::ImpedanceGains::Default()},
        {"MultiContact", mc_tasks::force::ImpedanceGains::Default()},
        {"Swing", mc_tasks::force::ImpedanceGains::Default()}};

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param limb limb
      \param mcRtcConfig mc_rtc configuration
  */
  LimbManager(MultiContactController * ctlPtr, const Limb & limb, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.
      \param constraintConfig mc_rtc configuration for contact constraint

      This method should be called once when controller is reset.
  */
  void reset(const mc_rtc::Configuration & constraintConfig);

  /** \brief Update.

      This method should be called once every control cycle.
  */
  void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Append a target contact command to the queue.
      \param newContactCommand contact command to append
      \return whether newContactCommand is appended
  */
  bool appendContactCommand(const ContactCommand & newContactCommand);

  /** \brief Access contact command queue. */
  inline const std::deque<ContactCommand> & contactCommandQueue() const noexcept
  {
    return commandQueue_;
  }

  /** \brief Access contact state list. */
  inline const std::map<double, std::shared_ptr<ContactState>> & contactStateList() const noexcept
  {
    return contactStateList_;
  }

  /** \brief Get contact state at the specified time. */
  std::shared_ptr<ContactState> getContactState(double t) const;

  /** \brief Get whether the limb is contacting now. */
  bool isContact() const;

  /** \brief Get whether the limb is contacting at the specified time.
      \param t time
   */
  bool isContact(double t) const;

  /** \brief Get contact weight at the specified time.
      \param t time
      \param weightTransitDuration duration of weight transition [sec]
   */
  double getContactWeight(double t, double weightTransitDuration = 0.1) const;

protected:
  /** \brief Const accessor to the controller. */
  inline const MultiContactController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline MultiContactController & ctl()
  {
    return *ctlPtr_;
  }

  /** \brief Get the remaining duration for next touch down.

      Returns zero in double stance phase. */
  double touchDownRemainingDuration() const;

  /** \brief Detect touch down.
      \return true if touch down is detected during swing
  */
  bool detectTouchDown() const;

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Limb
  Limb limb_;

  //! Limb task
  std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask> limbTask_;

  //! Contact command queue
  std::deque<ContactCommand> commandQueue_;

  //! ContactCommand currently executing
  const ContactCommand * executingCommand_ = nullptr;

  //! Contact state list (sorted by time)
  std::map<double, std::shared_ptr<ContactState>> contactStateList_;

  //! Target limb pose represented in world frame
  sva::PTransformd targetPose_;

  //! Target limb velocity represented in world frame
  sva::MotionVecd targetVel_;

  //! Target limb acceleration represented in world frame
  sva::MotionVecd targetAccel_;

  //! Limb task gain
  TaskGain taskGain_;

  //! Limb swing trajectory
  std::shared_ptr<SwingTraj> swingTraj_ = nullptr;

  //! Whether touch down is detected during swing
  bool touchDown_ = false;

  //! Type of impedance gains
  std::string impGainType_ = "Uninitialized";

  //! Whether to require updating impedance gains for limb tasks
  bool requireImpGainUpdate_ = true;
};
} // namespace MCC
