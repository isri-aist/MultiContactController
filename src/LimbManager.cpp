#include <algorithm>
#include <limits>

#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/IntegerInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Polygon.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/LimbManager.h>
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

void LimbManager::reset(const mc_rtc::Configuration & constraintConfig)
{
  commandQueue_.clear();

  executingCommand_ = nullptr;
  prevCommandPose_.reset();

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
  if(constraintConfig.empty())
  {
    ctl().solver().removeTask(limbTask_);
  }
  else
  {
    limbTask_->reset();
    ctl().solver().addTask(limbTask_);
    limbTask_->setGains(taskGain_.stiffness, taskGain_.damping);

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

  // Update isContact_
  isContact_ = isContact(ctl().t());

  // Complete executing command
  while(!commandQueue_.empty() && commandQueue_.front().endTime < ctl().t())
  {
    const auto & completedCommand = commandQueue_.front();

    if(completedCommand.type == ContactCommand::Type::Add)
    {
      // Update target
      if(!(config_.keepPoseForTouchDownLimb && touchDown_))
      {
        targetPose_ = swingTraj_->endPose_;
        targetVel_ = sva::MotionVecd::Zero();
        targetAccel_ = sva::MotionVecd::Zero();
      }

      prevCommandPose_ = std::make_shared<sva::PTransformd>(completedCommand.pose);

      taskGain_ = config_.taskGain;

      touchDown_ = false;
    }
    else // if(completedCommand.type == ContactCommand::Type::Remove)
    {
      ctl().solver().removeTask(limbTask_);
    }

    // Clear swingTraj_
    swingTraj_.reset();

    // Clear executingCommand_
    executingCommand_ = nullptr;

    // Remove command from command queue
    commandQueue_.pop_front();
  }

  if(!commandQueue_.empty() && (commandQueue_.front().startTime <= ctl().t()))
  {
    if(executingCommand_)
    {
      // Check if executingCommand_ is consistent
      if(executingCommand_ != &(commandQueue_.front()))
      {
        mc_rtc::log::error_and_throw("[LimbManager] Contact command is not consistent.");
      }
    }
    else
    {
      // Set executingCommand_
      executingCommand_ = &(commandQueue_.front());

      // Enable hold mode to prevent IK target pose from jumping
      // https://github.com/jrl-umi3218/mc_rtc/pull/143
      if(isContact_)
      {
        limbTask_->hold(true);
      }

      // Add limb task
      if(executingCommand_->type == ContactCommand::Type::Add)
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
        sva::PTransformd swingEndPose = executingCommand_->pose;
        if(config_.overwriteLandingPose && prevCommandPose_)
        {
          sva::PTransformd swingRelPose = executingCommand_->pose * prevCommandPose_->inv();
          swingEndPose = swingRelPose * swingStartPose;
        }

        std::string swingTrajType =
            executingCommand_->swingTrajConfig("type", static_cast<std::string>(config_.defaultSwingTrajType));
        if(swingTrajType == "CubicSplineSimple")
        {
          swingTraj_ = std::make_shared<SwingTrajCubicSplineSimple>(
              executingCommand_->type, isContact_, swingStartPose, swingEndPose, executingCommand_->startTime,
              executingCommand_->endTime, config_.taskGain, executingCommand_->swingTrajConfig);
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
    if(executingCommand_->type == ContactCommand::Type::Add && !touchDown_ && detectTouchDown())
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
      if(true) // \todo (ctl().limbManagerSet_->contactList().size() == 1)
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

void LimbManager::addToGUI(mc_rtc::gui::StateBuilder & gui) {}

void LimbManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui) {}

void LimbManager::addToLogger(mc_rtc::Logger & logger)
{
  std::string name = config_.name + "_" + std::to_string(limb_);

  logger.addLogEntry(name + "_contactCommandQueueSize", this, [this]() { return commandQueue_.size(); });
  logger.addLogEntry(name + "_contactStateListSize", this, [this]() { return contactStateList_.size(); });
  logger.addLogEntry(name + "_isContact", this, [this]() { return isContact_; });
  logger.addLogEntry(name + "_contactWeight", this, [this]() { return getContactWeight(ctl().t()); });
  MC_RTC_LOG_HELPER(name + "_impGainType", impGainType_);
  logger.addLogEntry(name + "_phase", this, [this]() -> std::string {
    if(executingCommand_)
    {
      return touchDown_ ? "Swing (TouchDown)" : "Swing";
    }
    else if(isContact_)
    {
      return "Contact";
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

bool LimbManager::appendContactCommand(const ContactCommand & command)
{
  // Check time of new contactCommand
  if(command.startTime < ctl().t())
  {
    mc_rtc::log::error("[LimbManager({})] Ignore a new contact command with past time: {} < {}", std::to_string(limb_),
                       command.startTime, ctl().t());
    return false;
  }
  if(!commandQueue_.empty())
  {
    const auto & lastCommand = commandQueue_.back();
    if(command.startTime < lastCommand.endTime)
    {
      mc_rtc::log::error("[LimbManager({})] Ignore a new command earlier than the last command: {} < {}",
                         std::to_string(limb_), command.startTime, lastCommand.endTime);
      return false;
    }
  }

  // Push to the queue
  commandQueue_.push_back(command);

  // Insert contact state
  contactStateList_.emplace(command.removeTime, nullptr);
  if(command.type == ContactCommand::Type::Add)
  {
    contactStateList_.emplace(command.addTime, std::make_shared<ContactState>(command.pose, command.constraint));
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
bool LimbManager::isContact() const
{
  return isContact_;
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
  if(executingCommand_)
  {
    return executingCommand_->endTime - ctl().t();
  }
  else
  {
    return 0;
  }
}

bool LimbManager::detectTouchDown() const
{
  if(!executingCommand_)
  {
    mc_rtc::log::error_and_throw("[LimbManager({})] detectTouchDown is called, but executingCommand is empty.",
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
