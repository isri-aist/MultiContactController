#include <cmath>
#include <limits>
#include <utility>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/PostureManager.h>
#include <MultiContactController/states/ConfigMotionState.h>

using namespace MCC;

void ConfigMotionState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Set baseTime
  double baseTime = std::numeric_limits<double>::quiet_NaN();
  if(config_.has("configs") && config_("configs").has("baseTime"))
  {
    if(config_("configs")("baseTime") == "Relative")
    {
      baseTime = ctl().t();
    }
    else
    {
      baseTime = static_cast<double>(config_("configs")("baseTime"));
    }
  }

  // Send step command
  if(config_.has("configs") && config_("configs").has("stepCommandList"))
  {
    for(const auto & stepCommandConfig : config_("configs")("stepCommandList"))
    {
      StepCommand stepCommand = StepCommand(stepCommandConfig);
      if(!std::isnan(baseTime))
      {
        stepCommand.setBaseTime(baseTime);
      }
      Limb limb = Limb(stepCommandConfig("limb"));
      ctl().limbManagerSet_->at(limb)->appendStepCommand(stepCommand);
    }
  }

  // Send nominal centroidal pose
  if(config_.has("configs") && config_("configs").has("nominalCentroidalPoseList"))
  {
    for(const auto & nominalCentroidalPoseConfig : config_("configs")("nominalCentroidalPoseList"))
    {
      double time = nominalCentroidalPoseConfig("time");
      if(!std::isnan(baseTime))
      {
        time += baseTime;
      }
      ctl().centroidalManager_->appendNominalCentroidalPose(
          time, static_cast<sva::PTransformd>(nominalCentroidalPoseConfig("pose")));
    }
  }

  // Send nominal posture
  if(config_.has("configs") && config_("configs").has("nominalPostureList"))
  {
    for(const auto & nominalPostureConfig : config_("configs")("nominalPostureList"))
    {
      double time = nominalPostureConfig("time");
      if(!std::isnan(baseTime))
      {
        time += baseTime;
      }
      PostureManager::PostureMap refPosture = nominalPostureConfig("target");
      ctl().postureManager_->appendNominalPosture(time, refPosture);
    }
  }

  // Set collision configuration list
  collisionConfigList_.clear();
  if(config_.has("configs") && config_("configs").has("collisionConfigList"))
  {
    for(const auto & _collisionConfig : config_("configs")("collisionConfigList"))
    {
      mc_rtc::Configuration collisionConfig;
      collisionConfig.load(_collisionConfig); // deep copy
      if(!std::isnan(baseTime))
      {
        collisionConfig.add("time", static_cast<double>(collisionConfig("time")) + baseTime);
      }
      collisionConfigList_.emplace(static_cast<double>(collisionConfig("time")), collisionConfig);
    }
  }

  // Set task configuration list
  taskConfigList_.clear();
  if(config_.has("configs") && config_("configs").has("taskConfigList"))
  {
    for(const auto & _taskConfig : config_("configs")("taskConfigList"))
    {
      mc_rtc::Configuration taskConfig;
      taskConfig.load(_taskConfig); // deep copy
      if(!std::isnan(baseTime))
      {
        taskConfig.add("time", static_cast<double>(taskConfig("time")) + baseTime);
      }
      taskConfigList_.emplace(static_cast<double>(taskConfig("time")), taskConfig);
    }
  }

  // Set option to wait for finishing swing motion
  if(config_.has("configs") && config_("configs").has("exitWhenLimbSwingFinished"))
  {
    exitWhenLimbSwingFinished_ = static_cast<bool>(config_("configs")("exitWhenLimbSwingFinished"));
  }

  output("OK");
}

bool ConfigMotionState::run(mc_control::fsm::Controller &)
{
  // Process collision configuration
  {
    auto it = collisionConfigList_.begin();
    while(it != collisionConfigList_.end())
    {
      if(it->first <= ctl().t())
      {
        const auto & collisionConfig = it->second;
        std::string r1 = collisionConfig("r1");
        std::string r2 = collisionConfig("r2", std::as_const(r1));
        if(collisionConfig("type") == "Add")
        {
          ctl().addCollisions(r1, r2, static_cast<std::vector<mc_rbdyn::Collision>>(collisionConfig("collisions")));
        }
        else // if(collisionConfig("type") == "Remove")
        {
          if(collisionConfig.has("collisions"))
          {
            ctl().removeCollisions(r1, r2,
                                   static_cast<std::vector<mc_rbdyn::Collision>>(collisionConfig("collisions")));
          }
          else
          {
            ctl().removeCollisions(r1, r2);
          }
        }
        it = collisionConfigList_.erase(it);
      }
      else
      {
        break;
      }
    }
  }

  // Process task configuration
  {
    auto it = taskConfigList_.begin();
    while(it != taskConfigList_.end())
    {
      if(it->first <= ctl().t())
      {
        const auto & taskConfig = it->second;
        bool taskFound = false;
        for(const auto & task : ctl().solver().tasks())
        {
          if(task->name() == static_cast<std::string>(taskConfig("name")))
          {
            taskFound = true;
            task->load(ctl().solver(), taskConfig);
            break;
          }
        }
        if(!taskFound)
        {
          mc_rtc::log::error("[ConfigMotionState] Task named \"{}\" not found.",
                             static_cast<std::string>(taskConfig("name")));
        }
        it = taskConfigList_.erase(it);
      }
      else
      {
        break;
      }
    }
  }

  return !ctl().limbManagerSet_->contactCommandStacked() && taskConfigList_.empty() && collisionConfigList_.empty()
         && (!exitWhenLimbSwingFinished_ || !ctl().limbManagerSet_->isExecutingLimbSwing());
}

void ConfigMotionState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("MCC::ConfigMotion", ConfigMotionState)
