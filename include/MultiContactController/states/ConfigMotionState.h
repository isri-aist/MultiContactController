#pragma once

#include <map>

#include <MultiContactController/State.h>

namespace MCC
{
/** \brief FSM state to send motion from configuration. */
struct ConfigMotionState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Collision configuration list
  std::multimap<double, mc_rtc::Configuration> collisionConfigList_;

  //! Task configuration list
  std::multimap<double, mc_rtc::Configuration> taskConfigList_;

  //! Option to select whether this state should wait for finishing swing motion or not
  bool exitWhenLimbSwingFinished_ = false;

  //! Option to select whether this state should wait for finishing reference CoM trajectory or not
  bool exitWhenCentroidalManagerFinished_ = false;

  //! Option to select whether this state should wait for finishing reference postures or not
  bool exitWhenPostureManagerFinished_ = false;
};
} // namespace MCC
