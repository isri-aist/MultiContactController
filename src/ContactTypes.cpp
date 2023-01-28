#include <mc_rtc/logging.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/ContactTypes.h>

using namespace MCC;

ContactCommand::ContactCommand(const mc_rtc::Configuration & mcRtcConfig)
{
  if(mcRtcConfig("type") == "Add")
  {
    // Make deep copy. See https://github.com/jrl-umi3218/mc_rtc/issues/195
    mc_rtc::Configuration constraintConfig;
    constraintConfig.load(mcRtcConfig("constraint"));
    if(!constraintConfig.has("name"))
    {
      constraintConfig.add("name", mcRtcConfig("limb"));
    }
    if(!constraintConfig.has("verticesName"))
    {
      constraintConfig.add("verticesName", mcRtcConfig("limb"));
    }
    if(!constraintConfig.has("pose"))
    {
      constraintConfig.add("pose", mcRtcConfig("pose"));
    }

    type = Type::Add;
    startTime = mcRtcConfig("startTime");
    endTime = mcRtcConfig("endTime");
    removeTime = mcRtcConfig("removeTime", std::as_const(startTime));
    addTime = mcRtcConfig("addTime", std::as_const(endTime));
    pose = mcRtcConfig("pose");
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
