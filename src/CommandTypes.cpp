#include <mc_rtc/logging.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/CommandTypes.h>

using namespace MCC;

SwingCommand::SwingCommand(const mc_rtc::Configuration & mcRtcConfig)
{
  type = strToType.at(mcRtcConfig("type"));
  if(type == Type::Add)
  {
    startTime = mcRtcConfig("startTime");
    endTime = mcRtcConfig("endTime");
    pose = mcRtcConfig("pose");
    mcRtcConfig("config", config);
  }
  else // if(type == Type::Remove)
  {
    startTime = mcRtcConfig("startTime");
    endTime = mcRtcConfig("endTime");
    mcRtcConfig("config", config);
  }
}

ContactCommand::ContactCommand(const mc_rtc::Configuration & mcRtcConfig)
{
  time = mcRtcConfig("time");
  constraint = ContactConstraint::makeSharedFromConfig(mcRtcConfig("constraint"));
}

StepCommand::StepCommand(const mc_rtc::Configuration & _mcRtcConfig)
{
  // Parse the simple configuration format
  mc_rtc::Configuration mcRtcConfig;
  if(_mcRtcConfig.has("swingCommand") || _mcRtcConfig.has("contactCommandList"))
  {
    mcRtcConfig = _mcRtcConfig;
  }
  else
  {
    const auto & type = SwingCommand::strToType.at(_mcRtcConfig("type"));
    mcRtcConfig.add("limb", _mcRtcConfig("limb"));
    if(type == SwingCommand::Type::Add)
    {
      mcRtcConfig.add("pose", _mcRtcConfig("pose"));
    }
    else
    {
      if(_mcRtcConfig.has("pose"))
      {
        mc_rtc::log::error("[StepCommand] pose entry is not used in the remove type command.");
      }
      if(_mcRtcConfig.has("constraint"))
      {
        mc_rtc::log::error("[StepCommand] constraint entry is not used in the remove type command.");
      }
    }

    // Add SwingCommand configuration
    {
      mc_rtc::Configuration swingCommandConfig;
      swingCommandConfig.add("type", _mcRtcConfig("type"));
      swingCommandConfig.add("startTime", _mcRtcConfig("startTime"));
      swingCommandConfig.add("endTime", _mcRtcConfig("endTime"));
      if(_mcRtcConfig.has("swingConfig"))
      {
        swingCommandConfig.add("config", _mcRtcConfig("swingConfig"));
      }
      mcRtcConfig.add("swingCommand", swingCommandConfig);
    }

    // Add ContactCommand configuration
    mcRtcConfig.array("contactCommandList");
    {
      mc_rtc::Configuration removeContactCommandConfig;
      removeContactCommandConfig.add("time", _mcRtcConfig("startTime"));
      removeContactCommandConfig.add_null("constraint");
      mcRtcConfig("contactCommandList").push(removeContactCommandConfig);
    }
    if(type == SwingCommand::Type::Add)
    {
      mc_rtc::Configuration addContactCommandConfig;
      addContactCommandConfig.add("time", _mcRtcConfig("endTime"));
      addContactCommandConfig.add("constraint", _mcRtcConfig("constraint"));
      mcRtcConfig("contactCommandList").push(addContactCommandConfig);
    }
  }

  // Set SwingCommand
  if(mcRtcConfig.has("swingCommand"))
  {
    mc_rtc::Configuration swingCommandConfig;
    swingCommandConfig.load(mcRtcConfig("swingCommand"));
    const auto & type = SwingCommand::strToType.at(swingCommandConfig("type"));
    if(type == SwingCommand::Type::Add)
    {
      if(!swingCommandConfig.has("pose"))
      {
        swingCommandConfig.add("pose", mcRtcConfig("pose"));
      }
    }
    else
    {
      if(swingCommandConfig.has("pose"))
      {
        mc_rtc::log::error("[StepCommand] pose entry is not used in the remove type command.");
      }
    }
    swingCommand = std::make_shared<SwingCommand>(swingCommandConfig);
  }

  // Set ContactCommand
  if(mcRtcConfig.has("contactCommandList"))
  {
    for(const auto & _contactCommandConfig : mcRtcConfig("contactCommandList"))
    {
      if(_contactCommandConfig("constraint").empty())
      {
        contactCommandList.emplace(_contactCommandConfig("time"), nullptr);
      }
      else
      {
        // Make deep copy. See https://github.com/jrl-umi3218/mc_rtc/issues/195
        mc_rtc::Configuration contactCommandConfig;
        contactCommandConfig.load(_contactCommandConfig);

        mc_rtc::Configuration constraintConfig = contactCommandConfig("constraint");
        if(!constraintConfig.has("name"))
        {
          constraintConfig.add("name", mcRtcConfig("limb"));
        }
        if(!constraintConfig.has("verticesName"))
        {
          constraintConfig.add("verticesName", mcRtcConfig("limb"));
        }
        if(!constraintConfig.has("pose") && mcRtcConfig.has("pose"))
        {
          constraintConfig.add("pose", mcRtcConfig("pose"));
        }

        contactCommandList.emplace(contactCommandConfig("time"),
                                   std::make_shared<ContactCommand>(contactCommandConfig));
      }
    }
  }

  // Set GripperCommand
  if(mcRtcConfig.has("gripperCommandList"))
  {
    for(const auto & gripperCommandConfig : mcRtcConfig("gripperCommandList"))
    {
      gripperCommandList.emplace(gripperCommandConfig("time"), GripperCommand(gripperCommandConfig));
    }
  }
}
