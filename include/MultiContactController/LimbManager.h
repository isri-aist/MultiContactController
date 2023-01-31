#pragma once

#include <map>
#include <unordered_map>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/ImpedanceGains.h>

#include <TrajColl/CubicInterpolator.h>
#include <TrajColl/CubicSpline.h>

#include <MultiContactController/CommandTypes.h>
#include <MultiContactController/LimbTypes.h>
#include <MultiContactController/RobotUtils.h>

namespace mc_tasks
{
namespace force
{
struct FirstOrderImpedanceTask;
}
} // namespace mc_tasks

namespace MCC
{
class MultiContactController;
class SwingTraj;

/** \brief Limb manager.

    Limb manager generates a swing limb trajectory from a specified swing command sequence.
*/
class LimbManager
{
  // Allow access to LimbManager config_ from LimbManagerSet
  friend class LimbManagerSet;

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
    std::string swingStartPolicy = "ControlRobot";

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

  /** \brief Append a step command to the command list.
      \param stepCommand step command to append
      \return whether stepCommand is appended
  */
  bool appendStepCommand(const StepCommand & stepCommand);

  /** \brief Access swing command list. */
  inline const std::map<double, std::shared_ptr<SwingCommand>> & swingCommandList() const noexcept
  {
    return swingCommandList_;
  }

  /** \brief Access contact command list. */
  inline const std::map<double, std::shared_ptr<ContactCommand>> & contactCommandList() const noexcept
  {
    return contactCommandList_;
  }

  /** \brief Access gripper command list. */
  inline const std::map<double, std::shared_ptr<GripperCommand>> & gripperCommandList() const noexcept
  {
    return gripperCommandList_;
  }

  /** \brief Get target limb pose at the specified time.
      \param t time

      \note Returns the end pose of the swing even while the limb is swinging.
   */
  sva::PTransformd getLimbPose(double t) const;

  /** \brief Get contact command at the specified time.
      \param t time

      If nullptr is returned, the limb is not contacting at the specified time, otherwise it is contacting.
   */
  std::shared_ptr<ContactCommand> getContactCommand(double t) const;

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

      Returns zero if the limb is not swinging . */
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

  //! Swing command list (map of start time and swing command)
  std::map<double, std::shared_ptr<SwingCommand>> swingCommandList_;

  //! Current swing command
  std::shared_ptr<SwingCommand> currentSwingCommand_ = nullptr;

  //! Previous swing command pose
  std::shared_ptr<SwingCommand> prevSwingCommand_ = nullptr;

  //! Contact command list (map of start time and contact command)
  std::map<double, std::shared_ptr<ContactCommand>> contactCommandList_;

  //! Current contact command
  std::shared_ptr<ContactCommand> currentContactCommand_ = nullptr;

  //! Previous contact command
  std::shared_ptr<ContactCommand> prevContactCommand_ = nullptr;

  //! Gripper command list (map of start time and gripper command)
  std::map<double, std::shared_ptr<GripperCommand>> gripperCommandList_;

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

  //! Phase
  std::string phase_ = "Uninitialized";

  //! Type of impedance gains
  std::string impGainType_ = "Uninitialized";

  //! Whether to require updating impedance gains for limb tasks
  bool requireImpGainUpdate_ = true;
};
} // namespace MCC
