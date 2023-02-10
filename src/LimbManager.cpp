#include <limits>

#include <mc_tasks/FirstOrderImpedanceTask.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/swing/SwingTrajCubicSplineSimple.h>

using namespace MCC;

void LimbManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  if(mcRtcConfig.has("taskGain"))
  {
    taskGain = TaskGain(mcRtcConfig("taskGain"));
  }
  mcRtcConfig("defaultSwingTrajType", defaultSwingTrajType);
  mcRtcConfig("swingStartPolicy", swingStartPolicy);
  mcRtcConfig("overwriteLandingPose", overwriteLandingPose);
  mcRtcConfig("stopSwingTrajForTouchDownLimb", stopSwingTrajForTouchDownLimb);
  mcRtcConfig("keepPoseForTouchDownLimb", keepPoseForTouchDownLimb);
  mcRtcConfig("enableWrenchDistForTouchDownLimb", enableWrenchDistForTouchDownLimb);
  mcRtcConfig("weightTransitDuration", weightTransitDuration);
  mcRtcConfig("touchDownRemainingDuration", touchDownRemainingDuration);
  mcRtcConfig("touchDownPosError", touchDownPosError);
  mcRtcConfig("touchDownForceZ", touchDownForceZ);
  if(mcRtcConfig.has("impedanceGains"))
  {
    mcRtcConfig("impedanceGains")("SingleContact", impGains.at("SingleContact"));
    mcRtcConfig("impedanceGains")("MultiContact", impGains.at("MultiContact"));
    mcRtcConfig("impedanceGains")("Swing", impGains.at("Swing"));
  }
}

LimbManager::LimbManager(MultiContactController * ctlPtr, const Limb & limb, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr), limb_(limb)
{
  config_.load(mcRtcConfig);
}

void LimbManager::reset(const mc_rtc::Configuration & _constraintConfig)
{
  swingCommandList_.clear();

  currentSwingCommand_.reset();
  prevSwingCommand_.reset();

  gripperCommandList_.clear();

  targetPose_ = limbTask()->frame().position();
  targetVel_ = sva::MotionVecd::Zero();
  targetAccel_ = sva::MotionVecd::Zero();
  taskGain_ = config_.taskGain;

  swingTraj_.reset();

  touchDown_ = false;

  // Following variables will be updated in the update method
  {
    phase_ = "Uninitialized";

    impGainType_ = "Uninitialized";

    requireImpGainUpdate_ = false;
  }

  contactCommandList_.clear();
  if(_constraintConfig.empty())
  {
    ctl().solver().removeTask(limbTask());
    currentContactCommand_ = nullptr;
  }
  else
  {
    limbTask()->reset();
    ctl().solver().addTask(limbTask());
    limbTask()->setGains(taskGain_.stiffness, taskGain_.damping);

    // Make deep copy. See https://github.com/jrl-umi3218/mc_rtc/issues/195
    mc_rtc::Configuration constraintConfig;
    constraintConfig.load(_constraintConfig);
    if(!constraintConfig.has("name"))
    {
      constraintConfig.add("name", std::to_string(limb_));
    }
    if(!constraintConfig.has("verticesName"))
    {
      constraintConfig.add("verticesName", std::to_string(limb_));
    }
    if(!constraintConfig.has("pose"))
    {
      constraintConfig.add("pose", targetPose_);
    }
    currentContactCommand_ =
        std::make_shared<ContactCommand>(ctl().t(), ContactConstraint::makeSharedFromConfig(constraintConfig));
  }
  contactCommandList_.emplace(ctl().t(), currentContactCommand_);
}

void LimbManager::update()
{
  // Disable hold mode by default
  limbTask()->hold(false);

  // Remove old contact command
  {
    auto it = contactCommandList_.upper_bound(ctl().t());
    if(it != contactCommandList_.begin())
    {
      it--;
      bool currentContact = static_cast<bool>(it->second);

      // Always keep up to one previous command
      if(it != contactCommandList_.begin())
      {
        it--;
      }

      // Find the most recent command with contact and hold up to it
      if(!currentContact)
      {
        while(!it->second && it != contactCommandList_.begin())
        {
          it--;
        }
      }

      // Erase all elements in the range from begin to it (including begin but not including it)
      contactCommandList_.erase(contactCommandList_.begin(), it);
    }
  }

  // Finalize completed swing command
  while(!swingCommandList_.empty() && (swingCommandList_.begin()->second->endTime < ctl().t()))
  {
    const auto & completedSwingCommand = swingCommandList_.begin()->second;

    if(completedSwingCommand->type == SwingCommand::Type::Add)
    {
      if(!(config_.keepPoseForTouchDownLimb && touchDown_))
      {
        targetPose_ = swingTraj_->endPose_;
      }
      targetVel_ = sva::MotionVecd::Zero();
      targetAccel_ = sva::MotionVecd::Zero();

      taskGain_ = config_.taskGain;

      touchDown_ = false;
    }
    else // if(completedSwingCommand->type == SwingCommand::Type::Remove)
    {
      ctl().solver().removeTask(limbTask());
    }

    // Update variables
    swingTraj_.reset();
    currentSwingCommand_.reset();
    prevSwingCommand_ = completedSwingCommand;

    // Remove completed swing command
    swingCommandList_.erase(swingCommandList_.begin());
  }

  // Process swing command
  if(!swingCommandList_.empty() && (swingCommandList_.begin()->first <= ctl().t()))
  {
    if(currentSwingCommand_)
    {
      // Check if currentSwingCommand_ is consistent
      assert(currentSwingCommand_ == swingCommandList_.begin()->second);
    }
    else
    {
      // Set currentSwingCommand_
      currentSwingCommand_ = swingCommandList_.begin()->second;

      // Enable hold mode to prevent IK target pose from jumping
      // https://github.com/jrl-umi3218/mc_rtc/pull/143
      if(currentContactCommand_)
      {
        limbTask()->hold(true);
      }

      // Add limb task
      if(currentSwingCommand_->type == SwingCommand::Type::Add)
      {
        limbTask()->reset();
        ctl().solver().addTask(limbTask());
      }

      // Set swingTraj_
      {
        sva::PTransformd swingStartPose;
        if(config_.swingStartPolicy == "ControlRobot")
        {
          swingStartPose = limbTask()->frame().position(); // control robot pose (i.e., IK result)
        }
        else if(config_.swingStartPolicy == "Target")
        {
          swingStartPose = limbTask()->targetPose(); // target pose
        }
        else if(config_.swingStartPolicy == "Compliance")
        {
          swingStartPose = limbTask()->compliancePose(); // compliance pose, which is modified by impedance
        }
        else
        {
          mc_rtc::log::error_and_throw("[LimbManager({})] swingStartPolicy must be \"ControlRobot\", \"Target\", or "
                                       "\"Compliance\", but \"{}\" is specified.",
                                       std::to_string(limb_), config_.swingStartPolicy);
        }
        sva::PTransformd swingEndPose = currentSwingCommand_->pose;
        if(config_.overwriteLandingPose && prevSwingCommand_ && prevSwingCommand_->type == SwingCommand::Type::Add)
        {
          sva::PTransformd swingRelPose = currentSwingCommand_->pose * prevSwingCommand_->pose.inv();
          swingEndPose = swingRelPose * swingStartPose;
        }

        std::string swingTrajType =
            currentSwingCommand_->config("type", static_cast<std::string>(config_.defaultSwingTrajType));
        if(swingTrajType == "CubicSplineSimple")
        {
          swingTraj_ = std::make_shared<SwingTrajCubicSplineSimple>(
              currentSwingCommand_->type, static_cast<bool>(currentContactCommand_), swingStartPose, swingEndPose,
              currentSwingCommand_->startTime, currentSwingCommand_->endTime, config_.taskGain,
              currentSwingCommand_->config);
        }
        else
        {
          mc_rtc::log::error_and_throw("[LimbManager({})] Invalid swingTrajType: {}.", std::to_string(limb_),
                                       swingTrajType);
        }
      }

      touchDown_ = false;
    }

    // Update touchDown_
    if(currentSwingCommand_->type == SwingCommand::Type::Add && !touchDown_ && detectTouchDown())
    {
      touchDown_ = true;

      if(config_.stopSwingTrajForTouchDownLimb)
      {
        swingTraj_->touchDown(ctl().t());
      }
    }

    // Update target
    {
      targetPose_ = swingTraj_->pose(ctl().t());
      targetVel_ = swingTraj_->vel(ctl().t());
      targetAccel_ = swingTraj_->accel(ctl().t());
      taskGain_ = swingTraj_->taskGain(ctl().t());
    }
  }

  // Process gripper command
  while(!gripperCommandList_.empty() && gripperCommandList_.begin()->first <= ctl().t())
  {
    auto it = gripperCommandList_.begin();
    const auto & gripperCommand = it->second;

    // Send gripper command
    ctl().robot().gripper(gripperCommand->name).configure(gripperCommand->config);

    // Remove processed gripper command
    gripperCommandList_.erase(it);
  }

  // Update currentContactCommand_ (this should be after setting swingTraj_)
  currentContactCommand_ = getContactCommand(ctl().t());

  // Update phase_
  if(currentSwingCommand_)
  {
    phase_ = "Swing (" + swingTraj_->type() + ")" + (touchDown_ ? " [TouchDown]" : "");
  }
  else if(currentContactCommand_)
  {
    phase_ = "Contact (" + currentContactCommand_->constraint->type() + ")";
  }
  else
  {
    phase_ = "Free";
  }

  // Set target of limb task
  {
    limbTask()->targetPose(targetPose_);
    // ImpedanceTask::targetVel receive the velocity represented in the world frame
    limbTask()->targetVel(targetVel_);
    // ImpedanceTask::targetAccel receive the acceleration represented in the world frame
    limbTask()->targetAccel(targetAccel_);
    limbTask()->setGains(taskGain_.stiffness, taskGain_.damping);
  }

  // Update impGainType_ and requireImpGainUpdate_
  {
    std::string newImpGainType;
    if(currentContactCommand_)
    {
      if(ctl().limbManagerSet_->contactList(ctl().t()).size() == 1)
      {
        newImpGainType = "SingleContact";
      }
      else
      {
        newImpGainType = "MultiContact";
      }
    }
    else
    {
      newImpGainType = "Swing";
    }

    if(impGainType_ != newImpGainType)
    {
      impGainType_ = newImpGainType;
      requireImpGainUpdate_ = true;
    }
  }

  // Set impedance gains of limb task
  if(requireImpGainUpdate_)
  {
    requireImpGainUpdate_ = false;

    limbTask()->gains() = config_.impGains.at(impGainType_);
  }

  // Update contact visualization
  {
    ctl().gui()->removeCategory({ctl().name(), config_.name, std::to_string(limb_), "ContactMarker"});

    int contactIdx = 0;
    for(const auto & contactCommandKV : contactCommandList_)
    {
      if(!contactCommandKV.second)
      {
        continue;
      }
      // Skip current contact as it is visualized in CentroidalManager
      if(contactCommandKV.second == currentContactCommand_)
      {
        continue;
      }

      contactCommandKV.second->constraint->addToGUI(
          *ctl().gui(),
          {ctl().name(), config_.name, std::to_string(limb_), "ContactMarker",
           contactCommandKV.second->constraint->name_ + "_" + std::to_string(contactIdx)},
          0.0, 0.0);

      contactIdx++;
    }
  }
}

void LimbManager::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void LimbManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({ctl().name(), config_.name, std::to_string(limb_), "Status"},
                 mc_rtc::gui::Label("frame", [this]() { return limbTask()->frame().name(); }),
                 mc_rtc::gui::Label("swingCommandListSize", [this]() { return swingCommandList_.size(); }),
                 mc_rtc::gui::Label("contactCommandListSize", [this]() { return contactCommandList_.size(); }),
                 mc_rtc::gui::Label("gripperCommandListSize", [this]() { return gripperCommandList_.size(); }),
                 mc_rtc::gui::Label("phase", [this]() -> const std::string & { return phase_; }),
                 mc_rtc::gui::Label("impGainType", [this]() -> const std::string & { return impGainType_; }));
}

void LimbManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name, std::to_string(limb_)});
}

void LimbManager::addToLogger(mc_rtc::Logger & logger)
{
  std::string name = config_.name + "_" + std::to_string(limb_);

  logger.addLogEntry(name + "_swingCommandListSize", this, [this]() { return swingCommandList_.size(); });
  logger.addLogEntry(name + "_contactCommandListSize", this, [this]() { return contactCommandList_.size(); });
  logger.addLogEntry(name + "_gripperCommandListSize", this, [this]() { return gripperCommandList_.size(); });
  logger.addLogEntry(name + "_isContact", this, [this]() { return static_cast<bool>(currentContactCommand_); });
  MC_RTC_LOG_HELPER(name + "_phase", phase_);
  MC_RTC_LOG_HELPER(name + "_impGainType", impGainType_);
  logger.addLogEntry(name + "_contactWeight", this, [this]() { return getContactWeight(ctl().t()); });

  constexpr bool enableDebugLog = false;
  if constexpr(enableDebugLog)
  {
    logger.addLogEntry(name + "_closestContactTimes", this, [this]() { return getClosestContactTimes(ctl().t()); });
  }
}

void LimbManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

bool LimbManager::appendStepCommand(const StepCommand & stepCommand)
{
  // Check time of swing command
  if(stepCommand.swingCommand)
  {
    double swingCommandStartTime = stepCommand.swingCommand->startTime;
    if(swingCommandStartTime < ctl().t())
    {
      mc_rtc::log::error("[LimbManager({})] Ignore a new step command with swing command with past time: {} < {}",
                         std::to_string(limb_), swingCommandStartTime, ctl().t());
      return false;
    }
    if(!swingCommandList_.empty())
    {
      double lastSwingCommandTime = swingCommandList_.rbegin()->second->endTime;
      if(swingCommandStartTime < lastSwingCommandTime)
      {
        mc_rtc::log::error("[LimbManager({})] Ignore a new step command with swing command earlier than the last swing "
                           "command: {} < {}",
                           std::to_string(limb_), swingCommandStartTime, lastSwingCommandTime);
        return false;
      }
    }
  }

  // Check time of contact command
  if(!stepCommand.contactCommandList.empty())
  {
    double contactCommandTime = stepCommand.contactCommandList.begin()->first;
    if(contactCommandTime < ctl().t())
    {
      mc_rtc::log::error("[LimbManager({})] Ignore a new step command with contact command with past time: {} < {}",
                         std::to_string(limb_), contactCommandTime, ctl().t());
      return false;
    }
    if(!contactCommandList_.empty())
    {
      double lastContactCommandTime = contactCommandList_.rbegin()->first;
      if(contactCommandTime < lastContactCommandTime)
      {
        mc_rtc::log::error("[LimbManager({})] Ignore a new step command with contact command earlier than the last "
                           "contact command: {} < {}",
                           std::to_string(limb_), contactCommandTime, lastContactCommandTime);
        return false;
      }
    }
  }

  // Check time of gripper command
  if(!stepCommand.gripperCommandList.empty())
  {
    double gripperCommandTime = stepCommand.gripperCommandList.begin()->first;
    if(gripperCommandTime < ctl().t())
    {
      mc_rtc::log::error("[LimbManager({})] Ignore a new step command with gripper command with past time: {} < {}",
                         std::to_string(limb_), gripperCommandTime, ctl().t());
      return false;
    }
    if(!gripperCommandList_.empty())
    {
      double lastGripperCommandTime = gripperCommandList_.rbegin()->first;
      if(gripperCommandTime < lastGripperCommandTime)
      {
        mc_rtc::log::error("[LimbManager({})] Ignore a new step command with gripper command earlier than the last "
                           "gripper command: {} < {}",
                           std::to_string(limb_), gripperCommandTime, lastGripperCommandTime);
        return false;
      }
    }
  }

  // Append swing command
  if(stepCommand.swingCommand)
  {
    swingCommandList_.emplace(stepCommand.swingCommand->startTime, stepCommand.swingCommand);
  }

  // Append contact command
  if(!stepCommand.contactCommandList.empty())
  {
    contactCommandList_.insert(stepCommand.contactCommandList.begin(), stepCommand.contactCommandList.end());
  }

  // Append gripper command
  if(!stepCommand.gripperCommandList.empty())
  {
    gripperCommandList_.insert(stepCommand.gripperCommandList.begin(), stepCommand.gripperCommandList.end());
  }

  return true;
}

sva::PTransformd LimbManager::getLimbPose(double t) const
{
  auto it = swingCommandList_.upper_bound(t);
  if(it == swingCommandList_.begin())
  {
    // If there is no swing command before the specified time, get pose based on some assumptions
    if(swingTraj_)
    {
      // Assume that the start pose of the current swing trajectory is kept
      return swingTraj_->startPose_;
    }
    else if(prevSwingCommand_ && prevSwingCommand_->type == SwingCommand::Type::Add)
    {
      // Assume that the pose of the previous swing command is kept
      return prevSwingCommand_->pose;
    }
    else
    {
      // Assume that the current target pose is kept
      return targetPose_;
    }
  }
  else
  {
    it--;
    if(it->second == currentSwingCommand_)
    {
      // If the swing command at the specified time is same as currentSwingCommand_, return the end pose of swingTraj_
      // (reflecting the override)
      return swingTraj_->endPose_;
    }
    else
    {
      return it->second->pose;
    }
  }
}

std::shared_ptr<ContactCommand> LimbManager::getContactCommand(double t) const
{
  auto it = contactCommandList_.upper_bound(t);
  if(it == contactCommandList_.begin())
  {
    mc_rtc::log::error_and_throw(
        "[LimbManager({})] Past time is specified in getContactCommand. specified time: {}, current time: {}",
        std::to_string(limb_), t, ctl().t());
  }
  it--;

  auto currentIt = contactCommandList_.upper_bound(ctl().t());
  if(currentIt != contactCommandList_.begin())
  {
    currentIt--;
    // If the current command without contact is found and touch down is detected, return the next contact
    // clang-format off
    if(it == currentIt
       && !it->second
       && std::next(it) != contactCommandList_.end()
       && std::next(it)->second
       && config_.enableWrenchDistForTouchDownLimb
       && touchDown_
       )
    // clang-format on
    {
      return std::next(it)->second;
    }
  }

  return it->second;
}

double LimbManager::getContactWeight(double t) const
{
  auto nextIt = contactCommandList_.upper_bound(t);
  if(nextIt == contactCommandList_.begin())
  {
    mc_rtc::log::error_and_throw(
        "[LimbManager({})] Past time is specified in getContactWeight. specified time: {}, current time: {}",
        std::to_string(limb_), t, ctl().t());
  }
  auto currentIt = std::prev(nextIt);

  // Check time consistency
  assert(currentIt->first <= t);
  if(nextIt != contactCommandList_.end())
  {
    assert(t <= nextIt->first);
  }

  if(currentIt->second)
  {
    constexpr double minWeight = 1e-8;

    // Check whether it is the beginning of contact
    if(currentIt != contactCommandList_.begin())
    {
      auto prevIt = std::prev(currentIt);
      if(!prevIt->second && (t - currentIt->first < config_.weightTransitDuration))
      {
        return mc_filter::utils::clamp((t - currentIt->first) / config_.weightTransitDuration, minWeight, 1.0);
      }
    }

    // Check whether it is the end of contact
    if(nextIt != contactCommandList_.end())
    {
      if(!nextIt->second && (nextIt->first - t < config_.weightTransitDuration))
      {
        return mc_filter::utils::clamp((nextIt->first - t) / config_.weightTransitDuration, minWeight, 1.0);
      }
    }

    // If contacted at the specified time, return 1
    return 1;
  }
  else
  {
    // If not contacted at the specified time, return 0
    return 0;
  }
}

std::array<double, 2> LimbManager::getClosestContactTimes(double t) const
{
  auto it = contactCommandList_.upper_bound(t);
  if(it == contactCommandList_.begin())
  {
    mc_rtc::log::error_and_throw(
        "[LimbManager({})] Past time is specified in getClosestContactTimes. specified time: {}, current time: {}",
        std::to_string(limb_), t, ctl().t());
  }
  it--;

  if(it->second)
  {
    return std::array<double, 2>{t, t};
  }
  else
  {
    std::array<double, 2> closestContactTimes = {std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN()};
    using ConstReverseIterator = std::map<double, std::shared_ptr<ContactCommand>>::const_reverse_iterator;
    for(auto backwardIt = ConstReverseIterator(it); backwardIt != contactCommandList_.rend(); backwardIt++)
    {
      constexpr double epsDuration = 1e-10;
      if(backwardIt->second)
      {
        closestContactTimes[0] = std::prev(backwardIt)->first - epsDuration;
        break;
      }
    }
    for(auto forwardIt = it; forwardIt != contactCommandList_.end(); forwardIt++)
    {
      if(forwardIt->second)
      {
        closestContactTimes[1] = forwardIt->first;
        break;
      }
    }
    return closestContactTimes;
  }
}

const std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask> & LimbManager::limbTask() const
{
  return ctl().limbTasks_.at(limb_);
}

bool LimbManager::detectTouchDown() const
{
  if(!currentSwingCommand_)
  {
    mc_rtc::log::error_and_throw("[LimbManager({})] detectTouchDown is called, but executingSwingCommand is empty.",
                                 std::to_string(limb_));
  }

  // False if the remaining duration does not meet the threshold
  if(swingTraj_->endTime_ - ctl().t() > config_.touchDownRemainingDuration)
  {
    return false;
  }

  // False if the position error does not meet the threshold
  if((swingTraj_->endPose_.translation() - swingTraj_->pose(ctl().t()).translation()).norm()
     > config_.touchDownPosError)
  {
    return false;
  }

  // Return false if the normal force does not meet the threshold
  double fz = limbTask()->measuredWrench().force().z();
  if(fz < config_.touchDownForceZ)
  {
    return false;
  }

  return true;
}
