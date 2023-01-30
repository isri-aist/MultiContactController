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
  contactCommandQueue_.clear();

  executingContactCommand_ = nullptr;
  prevContactCommandPose_.reset();

  targetPose_ = limbTask_->surfacePose();
  targetVel_ = sva::MotionVecd::Zero();
  targetAccel_ = sva::MotionVecd::Zero();
  taskGain_ = config_.taskGain;

  swingTraj_.reset();

  isContact_ = false;

  touchDown_ = false;

  impGainType_ = "Uninitialized"; // will be updated in the update method

  requireImpGainUpdate_ = false;

  contactStateList_.clear();
  if(_constraintConfig.empty())
  {
    ctl().solver().removeTask(limbTask_);
    contactStateList_.emplace(ctl().t(), nullptr);
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
    contactStateList_.emplace(
        ctl().t(), std::make_shared<ContactState>(limbTask_->surfacePose(),
                                                  ContactConstraint::makeSharedFromConfig(constraintConfig)));
  }
}

void LimbManager::update()
{
  // Disable hold mode by default
  limbTask_->hold(false);

  // Remove old contact state
  {
    auto it = contactStateList_.upper_bound(ctl().t());
    if(it != contactStateList_.begin())
    {
      it--;
      if(it != contactStateList_.begin())
      {
        contactStateList_.erase(contactStateList_.begin(), it);
      }
    }
  }

  // Complete executing contact command
  while(!contactCommandQueue_.empty() && contactCommandQueue_.front().endTime < ctl().t())
  {
    const auto & completedContactCommand = contactCommandQueue_.front();

    if(completedContactCommand.type == ContactCommand::Type::Add)
    {
      // Update target
      if(!(config_.keepPoseForTouchDownLimb && touchDown_))
      {
        targetPose_ = swingTraj_->endPose_;
        targetVel_ = sva::MotionVecd::Zero();
        targetAccel_ = sva::MotionVecd::Zero();
      }

      prevContactCommandPose_ = std::make_shared<sva::PTransformd>(completedContactCommand.pose);

      taskGain_ = config_.taskGain;

      touchDown_ = false;
    }
    else // if(completedContactCommand.type == ContactCommand::Type::Remove)
    {
      ctl().solver().removeTask(limbTask_);
    }

    // Clear swingTraj_
    swingTraj_.reset();

    // Clear executingContactCommand_
    executingContactCommand_ = nullptr;

    // Remove contact command from queue
    contactCommandQueue_.pop_front();
  }

  if(!contactCommandQueue_.empty() && (contactCommandQueue_.front().startTime <= ctl().t()))
  {
    if(executingContactCommand_)
    {
      // Check if executingContactCommand_ is consistent
      if(executingContactCommand_ != &(contactCommandQueue_.front()))
      {
        mc_rtc::log::error_and_throw("[LimbManager] Contact command is not consistent.");
      }
    }
    else
    {
      // Set executingContactCommand_
      executingContactCommand_ = &(contactCommandQueue_.front());

      // Enable hold mode to prevent IK target pose from jumping
      // https://github.com/jrl-umi3218/mc_rtc/pull/143
      if(isContact_)
      {
        limbTask_->hold(true);
      }

      // Add limb task
      if(executingContactCommand_->type == ContactCommand::Type::Add)
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
              "[LimbEndManager({})] swingStartPolicy must be ControlRobot, Target, or Compliance, but {} is specified.",
              std::to_string(limb_), config_.swingStartPolicy);
        }
        sva::PTransformd swingEndPose = executingContactCommand_->pose;
        if(config_.overwriteLandingPose && prevContactCommandPose_)
        {
          sva::PTransformd swingRelPose = executingContactCommand_->pose * prevContactCommandPose_->inv();
          swingEndPose = swingRelPose * swingStartPose;
        }

        std::string swingTrajType =
            executingContactCommand_->swingTrajConfig("type", static_cast<std::string>(config_.defaultSwingTrajType));
        if(swingTrajType == "CubicSplineSimple")
        {
          swingTraj_ = std::make_shared<SwingTrajCubicSplineSimple>(
              executingContactCommand_->type, isContact_, swingStartPose, swingEndPose,
              executingContactCommand_->startTime, executingContactCommand_->endTime, config_.taskGain,
              executingContactCommand_->swingTrajConfig);
        }
        else
        {
          mc_rtc::log::error_and_throw("[LimbManager] Invalid swingTrajType: {}.", swingTrajType);
        }
      }

      // Remove contact during adding/removing contact
      touchDown_ = false;
    }

    // Update touchDown_
    if(executingContactCommand_->type == ContactCommand::Type::Add && !touchDown_ && detectTouchDown())
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

  // Update isContact_
  isContact_ = isContact(ctl().t());

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
    if(isContact_)
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
                 mc_rtc::gui::Label("contactCommandQueueSize", [this]() { return contactCommandQueue_.size(); }),
                 mc_rtc::gui::Label("contactStateListSize", [this]() { return contactStateList_.size(); }),
                 mc_rtc::gui::Label("phase",
                                    [this]() -> std::string {
                                      if(executingContactCommand_)
                                      {
                                        return touchDown_ ? "Swing (TouchDown)" : "Swing";
                                      }
                                      else if(isContact_)
                                      {
                                        return "Contact (" + getContactState(ctl().t())->constraint->type() + ")";
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

  logger.addLogEntry(name + "_contactCommandQueueSize", this, [this]() { return contactCommandQueue_.size(); });
  logger.addLogEntry(name + "_contactStateListSize", this, [this]() { return contactStateList_.size(); });
  logger.addLogEntry(name + "_contactWeight", this, [this]() { return getContactWeight(ctl().t()); });
  MC_RTC_LOG_HELPER(name + "_impGainType", impGainType_);
  logger.addLogEntry(name + "_phase", this, [this]() -> std::string {
    if(executingContactCommand_)
    {
      return touchDown_ ? "Swing (TouchDown)" : "Swing";
    }
    else if(isContact_)
    {
      return "Contact (" + getContactState(ctl().t())->constraint->type() + ")";
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

bool LimbManager::appendContactCommand(const ContactCommand & newContactCommand)
{
  // Check time of new contactCommand
  if(newContactCommand.startTime < ctl().t())
  {
    mc_rtc::log::error("[LimbManager({})] Ignore a new contact command with past time: {} < {}", std::to_string(limb_),
                       newContactCommand.startTime, ctl().t());
    return false;
  }
  if(!contactCommandQueue_.empty())
  {
    const auto & lastContactCommand = contactCommandQueue_.back();
    if(newContactCommand.startTime < lastContactCommand.endTime)
    {
      mc_rtc::log::error(
          "[LimbManager({})] Ignore a new contact command earlier than the last contact command: {} < {}",
          std::to_string(limb_), newContactCommand.startTime, lastContactCommand.endTime);
      return false;
    }
  }

  // Push to the queue
  contactCommandQueue_.push_back(newContactCommand);

  // Insert contact state
  contactStateList_.emplace(newContactCommand.removeTime, nullptr);
  if(newContactCommand.type == ContactCommand::Type::Add)
  {
    contactStateList_.emplace(newContactCommand.addTime,
                              std::make_shared<ContactState>(newContactCommand.pose, newContactCommand.constraint));
  }

  return true;
}

std::shared_ptr<ContactState> LimbManager::getContactState(double t) const
{
  auto it = contactStateList_.upper_bound(t);
  if(it == contactStateList_.begin())
  {
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
  return getContactState(t) != nullptr;
}

double LimbManager::getContactWeight(double t, double weightTransitDuration) const
{
  auto endIt = contactStateList_.upper_bound(t);
  if(endIt == contactStateList_.begin())
  {
    return 0;
  }

  // Is not contacting
  auto startIt = endIt;
  startIt--;
  if(!startIt->second)
  {
    return 0;
  }

  // t is between startIt->first and endIt->first
  assert(t - startIt->first >= 0);
  if(endIt != contactStateList_.end())
  {
    assert(!endIt->second);
    assert(endIt->first - t >= 0);
  }

  if(t - startIt->first < weightTransitDuration)
  {
    return mc_filter::utils::clamp((t - startIt->first) / weightTransitDuration, 1e-8, 1.0);
  }
  else if(endIt != contactStateList_.end() && endIt->first - t < weightTransitDuration)
  {
    return mc_filter::utils::clamp((endIt->first - t) / weightTransitDuration, 1e-8, 1.0);
  }
  else
  {
    return 1;
  }
}

double LimbManager::touchDownRemainingDuration() const
{
  if(executingContactCommand_)
  {
    return executingContactCommand_->endTime - ctl().t();
  }
  else
  {
    return 0;
  }
}

bool LimbManager::detectTouchDown() const
{
  if(!executingContactCommand_)
  {
    mc_rtc::log::error_and_throw("[LimbManager({})] detectTouchDown is called, but executingContactCommand is empty.",
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
