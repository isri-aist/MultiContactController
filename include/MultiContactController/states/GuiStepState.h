#pragma once

#include <MultiContactController/State.h>

namespace MCC
{
/** \brief FSM state to send step command from GUI. */
struct GuiStepState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Send step command.
      \param config mc_rtc configuration from GUI form
      \return whether command is successfully sent
   */
  bool sendStepCommand(const mc_rtc::Configuration & config) const;

protected:
  //! Entry keys of GUI form
  const std::unordered_map<std::string, std::string> stepConfigKeys_ = {{"limb", "limb"},
                                                                        {"type", "type"},
                                                                        {"startTime", "start time from now [sec]"},
                                                                        {"duration", "swing duration [sec]"},
                                                                        {"xyz", "xyz position [m]"},
                                                                        {"rpy", "rpy orientation [deg]"},
                                                                        {"baseFrame", "base frame"},
                                                                        {"constraintType", "contact constraint type"}};
};
} // namespace MCC
