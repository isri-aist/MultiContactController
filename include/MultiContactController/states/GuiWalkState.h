#pragma once

#include <MultiContactController/LimbTypes.h>
#include <MultiContactController/State.h>

namespace MCC
{
/** \brief FSM state to send walking command from GUI. */
struct GuiWalkState : State
{
public:
  // Left foot limb
  static inline const Limb leftFoot = Limb("LeftFoot");

  // Right foot limb
  static inline const Limb rightFoot = Limb("RightFoot");

public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Send command to walk to the relative target pose.
      \param targetTrans relative target pose of foot midpose (x [m], y [m], theta [rad])
      \param lastFootstepNum number of last footstep
      \return whether command is successfully sent
   */
  bool sendWalkCommand(const Eigen::Vector3d & targetTrans, int lastFootstepNum) const;

  /** \brief Clamp a foot midpose transformation with limit
      \param deltaTrans foot midpose transformation
      \param foot foot
  */
  Eigen::Vector3d clampDeltaTrans(const Eigen::Vector3d & deltaTrans, const Limb & foot) const;

  /** \brief Make a step command.
      \param foot foot
      \param footMidpose middle pose of both feet
      \param startTime time to start the command
  */
  StepCommand makeStepCommand(const Limb & foot, const sva::PTransformd & footMidpose, double startTime) const;

protected:
  //! Entry keys of GUI form
  const std::unordered_map<std::string, std::string> walkConfigKeys_ = {{"x", "goal x [m]"},
                                                                        {"y", "goal y [m]"},
                                                                        {"theta", "goal theta [deg]"},
                                                                        {"last", "number of last footstep"}};

  //! Duration of one footstep [sec]
  double footstepDuration_ = 1.0;

  //! Duration ratio of double support phase
  double doubleSupportRatio_ = 0.2;

  //! Limit of foot midpose transformation for one footstep (x [m], y [m], theta [rad])
  Eigen::Vector3d deltaTransLimit_ = Eigen::Vector3d(0.15, 0.1, mc_rtc::constants::toRad(12.5));

  //! Transformation from foot midpose to each foot pose
  std::unordered_map<Limb, sva::PTransformd> midToFootTranss_ = {
      {leftFoot, sva::PTransformd(Eigen::Vector3d(0, 0.105, 0))},
      {rightFoot, sva::PTransformd(Eigen::Vector3d(0, -0.105, 0))}};
};
} // namespace MCC
