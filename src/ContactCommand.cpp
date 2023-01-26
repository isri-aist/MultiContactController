#include <mc_rtc/logging.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/ContactCommand.h>

using namespace MCC;

ContactCommand::ContactCommand(const mc_rtc::Configuration & mcRtcConfig)
{
  if(mcRtcConfig("type") == "Add")
  {
    // Automatically set constraint pose if it is not set
    // Make deep copy to avoid changing the original configuration
    // See https://github.com/jrl-umi3218/mc_rtc/issues/195
    mc_rtc::Configuration constraintConfig;
    constraintConfig.load(mcRtcConfig("constraint"));
    if(!constraintConfig.has("pose"))
    {
      constraintConfig.add("pose", mcRtcConfig("surfacePose"));
    }

    type = Type::Add;
    startTime = mcRtcConfig("startTime");
    endTime = mcRtcConfig("endTime");
    removeTime = mcRtcConfig("removeTime", std::as_const(startTime));
    addTime = mcRtcConfig("addTime", std::as_const(endTime));
    surfacePose = mcRtcConfig("surfacePose");
    constraint = ContactConstraint::makeSharedFromConfig(constraintConfig);
    mcRtcConfig("swingTrajConfig", swingTrajConfig);
  }
  else if(mcRtcConfig("type") == "Remove")
  {
    type = Type::Remove;
    startTime = mcRtcConfig("startTime");
    endTime = mcRtcConfig("endTime");
    removeTime = mcRtcConfig("removeTime", std::as_const(startTime));
    mcRtcConfig("swingTrajConfig", swingTrajConfig);
  }
  else
  {
    mc_rtc::log::error_and_throw("[ContactCommand] Invalid type: {}.", mcRtcConfig("type"));
  }
}
