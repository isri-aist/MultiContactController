#include <mc_rtc/gui/Button.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/InterruptState.h>

using namespace MCC;

void InterruptState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  baseTime_ = 0.0;
  if(config_.has("configs") && config_("configs").has("baseTime"))
  {
    if(config_("configs")("baseTime") == "Relative")
    {
      baseTime_ = ctl().t();
    }
    else
    {
      baseTime_ = static_cast<double>(config_("configs")("baseTime"));
    }
  }

  // Stop updating references
  ctl().enableManagerUpdate_ = false;

  // Setup GUI
  ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("Start", [this]() { phase_ = 1; }));

  output("OK");
}

bool InterruptState::run(mc_control::fsm::Controller &)
{
  if(phase_ == 0)
  {
    // Auto start
    if(config_.has("configs") && config_("configs").has("autoStartTime")
       && ctl().t() - baseTime_ > static_cast<double>(config_("configs")("autoStartTime")))
    {
      phase_ = 1;
    }
  }
  if(phase_ == 1)
  {
    phase_ = 2;

    // Clean up GUI
    ctl().gui()->removeElement({ctl().name()}, "Start");

    // Start updating references
    ctl().enableManagerUpdate_ = true;
  }

  return complete();
}

void InterruptState::teardown(mc_control::fsm::Controller &) {}

bool InterruptState::complete() const
{
  return phase_ == 2;
}

EXPORT_SINGLE_STATE("MCC::Interrupt", InterruptState)
