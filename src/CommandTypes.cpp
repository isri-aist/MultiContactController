#include <mc_rtc/logging.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/CommandTypes.h>

using namespace MCC;

SwingCommand::SwingCommand(const mc_rtc::Configuration & mcRtcConfig)
{
  type = strToType.at(mcRtcConfig("type"));
  startTime = mcRtcConfig("startTime");
  endTime = mcRtcConfig("endTime");
  if(type == Type::Add)
  {
    pose = mcRtcConfig("pose");
  }
  mcRtcConfig("config", config);
}

void SwingCommand::setBaseTime(double baseTime)
{
  startTime += baseTime;
  endTime += baseTime;
}

ContactCommand::ContactCommand(double _time, const std::shared_ptr<ContactConstraint> & _constraint)
: time(_time), constraint(_constraint)
{
  assert(constraint);
}

ContactCommand::ContactCommand(const mc_rtc::Configuration & mcRtcConfig)
: ContactCommand(mcRtcConfig("time"), ContactConstraint::makeSharedFromConfig(mcRtcConfig("constraint")))
{
}

void ContactCommand::setBaseTime(double baseTime)
{
  time += baseTime;
}

void GripperCommand::setBaseTime(double baseTime)
{
  time += baseTime;
}

StepCommand::StepCommand(const mc_rtc::Configuration & _mcRtcConfig)
{
  // Parse according to simple/full description format
  mc_rtc::Configuration mcRtcConfig;
  if(_mcRtcConfig.has("swingCommand") || _mcRtcConfig.has("contactCommandList")) // full description format
  {
    mcRtcConfig = _mcRtcConfig;
  }
  else // simple description format
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
    swingCommandConfig.load(mcRtcConfig("swingCommand")); // deep copy

    // "pose" entry is automatically set
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
        mc_rtc::Configuration contactCommandConfig;
        contactCommandConfig.load(_contactCommandConfig); // deep copy

        mc_rtc::Configuration constraintConfig = contactCommandConfig("constraint");
        // "name", "verticesName", and "pose" entries are automatically set
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
      gripperCommandList.emplace(gripperCommandConfig("time"), std::make_shared<GripperCommand>(gripperCommandConfig));
    }
  }
}

void StepCommand::setBaseTime(double baseTime)
{
  if(swingCommand)
  {
    swingCommand->setBaseTime(baseTime);
  }

  {
    std::map<double, std::shared_ptr<ContactCommand>> newContactCommandList;
    for(auto & contactCommandKV : contactCommandList)
    {
      auto & contactCommand = contactCommandKV.second;
      if(contactCommand)
      {
        contactCommand->setBaseTime(baseTime);
      }
      newContactCommandList.emplace(contactCommandKV.first + baseTime, contactCommand);
    }
    contactCommandList = newContactCommandList;
  }

  {
    std::map<double, std::shared_ptr<GripperCommand>> newGripperCommandList;
    for(auto & gripperCommandKV : gripperCommandList)
    {
      auto & gripperCommand = gripperCommandKV.second;
      if(gripperCommand)
      {
        gripperCommand->setBaseTime(baseTime);
      }
      newGripperCommandList.emplace(gripperCommandKV.first + baseTime, gripperCommand);
    }
    gripperCommandList = newGripperCommandList;
  }
}
