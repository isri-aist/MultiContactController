#pragma once

#include <mc_control/fsm/State.h>

namespace MCC
{
class MultiContactController;

/** \brief FSM State with utility functions. */
struct State : mc_control::fsm::State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & _ctl) override;

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

protected:
  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;
};
} // namespace MCC
