#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/ConfigMotionState.h>

using namespace MCC;

void ConfigMotionState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Send contact command
  if(config_.has("configs") && config_("configs").has("contactList"))
  {
    for(const auto & commandConfig : config_("configs")("contactList"))
    {
      ContactCommand command = ContactCommand(commandConfig);
      Limb limb = Limb(commandConfig("limb"));
      ctl().limbManagerSet_->at(limb)->appendContactCommand(command);
    }
  }

  output("OK");
}

bool ConfigMotionState::run(mc_control::fsm::Controller &)
{
  return !ctl().limbManagerSet_->contactStateStacked();
}

void ConfigMotionState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("MCC::ConfigMotion", ConfigMotionState)
