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
    for(const auto & stepCommandConfig : config_("configs")("stepCommandList"))
    {
      StepCommand stepCommand = StepCommand(stepCommandConfig);
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
