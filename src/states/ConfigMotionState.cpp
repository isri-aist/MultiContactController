#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/ConfigMotionState.h>

using namespace MCC;

void ConfigMotionState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Send step command
  if(config_.has("configs") && config_("configs").has("stepCommandList"))
  {
    double baseTime = 0.0;
    if(config_("configs").has("baseTime"))
    {
      if(config_("configs")("baseTime") == "Relative")
      {
        baseTime = ctl().t();
      }
      else
      {
        baseTime = static_cast<double>(config_("configs")("baseTime"));
      }
    }

    for(const auto & stepCommandConfig : config_("configs")("stepCommandList"))
    {
      StepCommand stepCommand = StepCommand(stepCommandConfig);
      if(baseTime != 0.0)
      {
        stepCommand.setBaseTime(baseTime);
      }
      Limb limb = Limb(stepCommandConfig("limb"));
      ctl().limbManagerSet_->at(limb)->appendStepCommand(stepCommand);
    }
  }

  output("OK");
}

bool ConfigMotionState::run(mc_control::fsm::Controller &)
{
  return !ctl().limbManagerSet_->contactCommandStacked();
}

void ConfigMotionState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("MCC::ConfigMotion", ConfigMotionState)
