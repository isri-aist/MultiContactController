#pragma once

#include <MultiContactController/State.h>

namespace MCC
{
/** \brief FSM state to initialize. */
struct InterruptState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Check whether state is completed. */
  bool complete() const;

protected:
  //! Phase
  int phase_ = 0;

  //! BaseTime for automatic start
  double baseTime_ = 0.0;
};
} // namespace MCC
