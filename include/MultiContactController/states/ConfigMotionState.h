#pragma once

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
};
} // namespace MCC
