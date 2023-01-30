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
: ctlPtr_(ctlPtr), limb_(limb), limbTask_(ctlPtr->limbTasks_.at(limb_))
{
  config_.load(mcRtcConfig);
}

void LimbManager::reset(const mc_rtc::Configuration & _constraintConfig)
{
  swingCommandQueue_.clear();

  executingSwingCommand_.reset();
  prevSwingCommandPose_.reset();

  targetPose_ = limbTask_->surfacePose();
  targetVel_ = sva::MotionVecd::Zero();
  targetAccel_ = sva::MotionVecd::Zero();
  taskGain_ = config_.taskGain;

  swingTraj_.reset();

  touchDown_ = false;

  impGainType_ = "Uninitialized"; // will be updated in the update method

  requireImpGainUpdate_ = false;

  contactCommandList_.clear();
  if(_constraintConfig.empty())
  {
    ctl().solver().removeTask(limbTask_);
    currentContact_ = nullptr;
  }
  else
  {
    limbTask_->reset();
    ctl().solver().addTask(limbTask_);
    limbTask_->setGains(taskGain_.stiffness, taskGain_.damping);

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
    currentContact_ = std::make_shared<ContactCommand>(ctl().t(), limbTask_->surfacePose(),
                                                       ContactConstraint::makeSharedFromConfig(constraintConfig));
  }
  prevContact_ = currentContact_;
  contactCommandList_.emplace(ctl().t(), currentContact_);
}

void LimbManager::update()
{
  // Disable hold mode by default
  limbTask_->hold(false);

  // Remove old contact command
  {
    auto it = contactCommandList_.upper_bound(ctl().t());
    if(it != contactCommandList_.begin())
    {
      it--;
      if(it != contactCommandList_.begin())
      {
        prevContact_ = std::prev(it)->second;
        contactCommandList_.erase(contactCommandList_.begin(), it);
      }
    }
  }

  // Remove old swing command
  while(!swingCommandQueue_.empty() && swingCommandQueue_.front()->endTime < ctl().t())
  {
    const auto & completedSwingCommand = swingCommandQueue_.front();

    if(completedSwingCommand->type == SwingCommand::Type::Add)
    {
      // Update target
      if(!(config_.keepPoseForTouchDownLimb && touchDown_))
      {
        targetPose_ = swingTraj_->endPose_;
        targetVel_ = sva::MotionVecd::Zero();
        targetAccel_ = sva::MotionVecd::Zero();
      }

      prevSwingCommandPose_ = std::make_shared<sva::PTransformd>(completedSwingCommand->pose);

      taskGain_ = config_.taskGain;

      touchDown_ = false;
    }
    else // if(completedSwingCommand->type == SwingCommand::Type::Remove)
    {
      ctl().solver().removeTask(limbTask_);
    }

    // Clear swingTraj_
    swingTraj_.reset();

    // Clear executingSwingCommand_
    executingSwingCommand_.reset();

    // Remove swing command from queue
    swingCommandQueue_.pop_front();
  }

  if(!swingCommandQueue_.empty() && (swingCommandQueue_.front()->startTime <= ctl().t()))
  {
    if(executingSwingCommand_)
    {
      // Check if executingSwingCommand_ is consistent
      if(executingSwingCommand_ != swingCommandQueue_.front())
      {
        mc_rtc::log::error_and_throw("[LimbManager({})] Swing command is not consistent.", std::to_string(limb_));
      }
    }
    else
    {
      // Set executingSwingCommand_
      executingSwingCommand_ = swingCommandQueue_.front();

      // Enable hold mode to prevent IK target pose from jumping
      // https://github.com/jrl-umi3218/mc_rtc/pull/143
      if(currentContact_)
      {
        limbTask_->hold(true);
      }

      // Add limb task
      if(executingSwingCommand_->type == SwingCommand::Type::Add)
      {
        limbTask_->reset();
        ctl().solver().addTask(limbTask_);
      }

      // Set swingTraj_
      {
        sva::PTransformd swingStartPose;
        if(config_.swingStartPolicy == "ControlRobot")
        {
          swingStartPose = limbTask_->surfacePose(); // control robot pose (i.e., IK result)
        }
        else if(config_.swingStartPolicy == "Target")
        {
          swingStartPose = limbTask_->targetPose(); // target pose
        }
        else if(config_.swingStartPolicy == "Compliance")
        {
          swingStartPose = limbTask_->compliancePose(); // compliance pose, which is modified by impedance
        }
        else
        {
          mc_rtc::log::error_and_throw(
              "[LimbManager({})] swingStartPolicy must be ControlRobot, Target, or Compliance, but {} is specified.",
              std::to_string(limb_), config_.swingStartPolicy);
        }
        sva::PTransformd swingEndPose = executingSwingCommand_->pose;
        if(config_.overwriteLandingPose && prevSwingCommandPose_)
        {
          sva::PTransformd swingRelPose = executingSwingCommand_->pose * prevSwingCommandPose_->inv();
          swingEndPose = swingRelPose * swingStartPose;
        }

        std::string swingTrajType =
            executingSwingCommand_->config("type", static_cast<std::string>(config_.defaultSwingTrajType));
        if(swingTrajType == "CubicSplineSimple")
        {
          swingTraj_ = std::make_shared<SwingTrajCubicSplineSimple>(
              executingSwingCommand_->type, static_cast<bool>(currentContact_), swingStartPose, swingEndPose,
              executingSwingCommand_->startTime, executingSwingCommand_->endTime, config_.taskGain,
              executingSwingCommand_->config);
        }
        else
        {
          mc_rtc::log::error_and_throw("[LimbManager({})] Invalid swingTrajType: {}.", std::to_string(limb_),
                                       swingTrajType);
        }
      }

      // Remove contact during adding/removing contact
      touchDown_ = false;
    }

    // Update touchDown_
    if(executingSwingCommand_->type == SwingCommand::Type::Add && !touchDown_ && detectTouchDown())
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

  // Update currentContact_ (this should be after setting swingTraj_)
  currentContact_ = getContactCommand(ctl().t());

  // Set target of limb task
  {
    limbTask_->targetPose(targetPose_);
    // ImpedanceTask::targetVel receive the velocity represented in the world frame
    limbTask_->targetVel(targetVel_);
    // ImpedanceTask::targetAccel receive the acceleration represented in the world frame
    limbTask_->targetAccel(targetAccel_);
    limbTask_->setGains(taskGain_.stiffness, taskGain_.damping);
  }

  // Update impGainType_ and requireImpGainUpdate_
  {
    std::string newImpGainType;
    if(currentContact_)
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

    limbTask_->gains() = config_.impGains.at(impGainType_);
  }
}

void LimbManager::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void LimbManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({ctl().name(), config_.name, std::to_string(limb_)},
                 mc_rtc::gui::Label("surface", [this]() { return limbTask_->surface(); }),
                 mc_rtc::gui::Label("swingCommandQueueSize", [this]() { return swingCommandQueue_.size(); }),
                 mc_rtc::gui::Label("contactCommandListSize", [this]() { return contactCommandList_.size(); }),
                 mc_rtc::gui::Label("phase",
                                    [this]() -> std::string {
                                      if(executingSwingCommand_)
                                      {
                                        return touchDown_ ? "Swing (TouchDown)" : "Swing";
                                      }
                                      else if(currentContact_)
                                      {
                                        return "Contact (" + currentContact_->constraint->type() + ")";
                                      }
                                      else
                                      {
                                        return "Free";
                                      }
                                    }),
                 mc_rtc::gui::Label("impGainType", [this]() { return impGainType_; }));
}

void LimbManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name, std::to_string(limb_)});
}

void LimbManager::addToLogger(mc_rtc::Logger & logger)
{
  std::string name = config_.name + "_" + std::to_string(limb_);

  logger.addLogEntry(name + "_swingCommandQueueSize", this, [this]() { return swingCommandQueue_.size(); });
  logger.addLogEntry(name + "_contactCommandListSize", this, [this]() { return contactCommandList_.size(); });
  logger.addLogEntry(name + "_contactWeight", this, [this]() { return getContactWeight(ctl().t()); });
  MC_RTC_LOG_HELPER(name + "_impGainType", impGainType_);
  logger.addLogEntry(name + "_phase", this, [this]() -> std::string {
    if(executingSwingCommand_)
    {
      return touchDown_ ? "Swing (TouchDown)" : "Swing";
    }
    else if(currentContact_)
    {
      return "Contact (" + currentContact_->constraint->type() + ")";
    }
    else
    {
      return "Free";
    }
  });
}

void LimbManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

bool LimbManager::appendStepCommand(const StepCommand & stepCommand)
{
  // Check time of swing command
  if(stepCommand.swingCommand->startTime < ctl().t())
  {
    mc_rtc::log::error("[LimbManager({})] Ignore a new step command with swing command with past time: {} < {}",
                       std::to_string(limb_), stepCommand.swingCommand->startTime, ctl().t());
    return false;
  }
  if(!swingCommandQueue_.empty())
  {
    const auto & lastSwingCommand = swingCommandQueue_.back();
    if(stepCommand.swingCommand->startTime < lastSwingCommand->endTime)
    {
      mc_rtc::log::error(
          "[LimbManager({})] Ignore a new step command with swing command earlier than the last swing command: {} < {}",
          std::to_string(limb_), stepCommand.swingCommand->startTime, lastSwingCommand->endTime);
      return false;
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
      const auto & lastContactCommandTime = contactCommandList_.rbegin()->first;
      if(contactCommandTime < lastContactCommandTime)
      {
        mc_rtc::log::error("[LimbManager({})] Ignore a new step command with contact command earlier than the last "
                           "contact command: {} < {}",
                           std::to_string(limb_), contactCommandTime, lastContactCommandTime);
        return false;
      }
    }
  }

  // Append swing command
  swingCommandQueue_.push_back(stepCommand.swingCommand);

  // Append contact command
  contactCommandList_.insert(stepCommand.contactCommandList.begin(), stepCommand.contactCommandList.end());

  return true;
}

std::shared_ptr<ContactCommand> LimbManager::getContactCommand(double t) const
{
  auto it = contactCommandList_.upper_bound(t);
  if(it == contactCommandList_.begin())
  {
    // Past time is given
    return nullptr;
  }
  else
  {
    it--;
    if(config_.enableWrenchDistForTouchDownLimb && touchDown_ && !it->second && std::next(it)->second)
    {
      return std::next(it)->second;
    }
    return it->second;
  }
}
bool LimbManager::isContact(double t) const
{
  return getContactCommand(t) != nullptr;
}

double LimbManager::getContactWeight(double t, double weightTransitDuration) const
{
  // Past time is given
  auto nextIt = contactCommandList_.upper_bound(t);
  if(nextIt == contactCommandList_.begin())
  {
    return 0;
  }

  // Is not contacting
  auto currentIt = std::prev(nextIt);
  if(!currentIt->second)
  {
    return 0;
  }

  // Check time consistency
  assert(currentIt->first <= t);
  if(nextIt != contactCommandList_.end())
  {
    assert(t <= nextIt->first);
  }

  const std::shared_ptr<ContactCommand> & prevContact =
      (currentIt == contactCommandList_.begin() ? prevContact_ : std::prev(currentIt)->second);
  if(!prevContact && t - currentIt->first < weightTransitDuration)
  {
    return mc_filter::utils::clamp((t - currentIt->first) / weightTransitDuration, 1e-8, 1.0);
  }
  else if(nextIt != contactCommandList_.end() && !nextIt->second && nextIt->first - t < weightTransitDuration)
  {
    return mc_filter::utils::clamp((nextIt->first - t) / weightTransitDuration, 1e-8, 1.0);
  }
  else
  {
    return 1;
  }
}

double LimbManager::touchDownRemainingDuration() const
{
  if(executingSwingCommand_)
  {
    return executingSwingCommand_->endTime - ctl().t();
  }
  else
  {
    return 0;
  }
}

bool LimbManager::detectTouchDown() const
{
  if(!executingSwingCommand_)
  {
    mc_rtc::log::error_and_throw("[LimbManager({})] detectTouchDown is called, but executingSwingCommand is empty.",
                                 std::to_string(limb_));
  }

  // False if the remaining duration does not meet the threshold
  if(touchDownRemainingDuration() > config_.touchDownRemainingDuration)
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
  double fz = limbTask_->measuredWrench().force().z();
  if(fz < config_.touchDownForceZ)
  {
    return false;
  }

  return true;
}
