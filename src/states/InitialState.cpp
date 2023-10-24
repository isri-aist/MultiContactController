#include <mc_rtc/gui/Button.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/OrientationTask.h>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/PostureManager.h>
#include <MultiContactController/states/InitialState.h>

using namespace MCC;

void InitialState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  // Setup GUI
  ctl().gui()->addElement({ctl().name()}, mc_rtc::gui::Button("Start", [this]() { phase_ = 1; }));

  output("OK");
}

bool InitialState::run(mc_control::fsm::Controller &)
{
  if(phase_ == 0)
  {
    // Auto start
    if(config_.has("configs") && config_("configs").has("autoStartTime")
       && ctl().t() > static_cast<double>(config_("configs")("autoStartTime")))
    {
      phase_ = 1;
    }
  }
  if(phase_ == 1)
  {
    phase_ = 2;

    // Clean up GUI
    ctl().gui()->removeElement({ctl().name()}, "Start");

    // Reset and add tasks
    ctl().comTask_->reset();
    ctl().solver().addTask(ctl().comTask_);
    ctl().baseOriTask_->reset();
    ctl().solver().addTask(ctl().baseOriTask_);
    ctl().momentumTask_->reset();
    ctl().momentumTask_->momentum(sva::ForceVecd::Zero());
    ctl().solver().addTask(ctl().momentumTask_);
    // limb tasks are added in LimbManager

    // Setup task stiffness interpolation
    comTaskStiffness_ = ctl().comTask_->dimStiffness();
    baseOriTaskStiffness_ = ctl().baseOriTask_->dimStiffness();
    momentumTaskStiffness_ = ctl().momentumTask_->dimStiffness();
    // Do not interpolate the stiffness of the limb tasks because the current pose is retained as the target pose
    constexpr double stiffnessInterpDuration = 1.0; // [sec]
    stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
        std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});

    // Initialize base pose of the robot
    if(config_.has("configs") && config_("configs").has("basePose"))
    {
      sva::PTransformd configBasePose = static_cast<sva::PTransformd>(config_("configs")("basePose"));
      ctl().robot().posW(configBasePose);
      ctl().realRobot().posW(configBasePose);
    }
    else if(ctl().datastore().has("MCC::LastBasePose"))
    {
      sva::PTransformd lastBasePose = ctl().datastore().get<sva::PTransformd>("MCC::LastBasePose");
      ctl().robot().posW(lastBasePose);
      ctl().realRobot().posW(lastBasePose);
    }

    // Reset managers
    mc_rtc::Configuration initialContactsConfig;
    if(config_.has("configs") && config_("configs").has("initialContacts"))
    {
      initialContactsConfig = config_("configs")("initialContacts");
    }
    ctl().limbManagerSet_->reset(initialContactsConfig);

    if(config_.has("configs"))
    {
      // Overwrite nominalCetnroidalPose if config_("configs")("nominalCentroidalPose") exists
      ctl().centroidalManager_->reset(config_("configs"));
    }
    else
    {
      ctl().centroidalManager_->reset();
    }

    if(config_.has("configs") && config_("configs").has("nominalPosture"))
    {
      mc_rtc::Configuration nominalPostureConfig = config_("configs")("nominalPosture");
      PostureManager::PostureMap initPosture = nominalPostureConfig("target");
      ctl().postureManager_->reset(initPosture);
    }
    else
    {
      ctl().postureManager_->reset();
    }

    ctl().enableManagerUpdate_ = true;

    // Setup collisions
    if(config_.has("configs") && config_("configs").has("collisionConfigList"))
    {
      for(const auto & collisionConfig : config_("configs")("collisionConfigList"))
      {
        std::string r1 = collisionConfig("r1");
        std::string r2 = collisionConfig("r2", std::as_const(r1));
        if(collisionConfig("type") == "Add")
        {
          ctl().addCollisions(r1, r2, static_cast<std::vector<mc_rbdyn::Collision>>(collisionConfig("collisions")));
        }
        else
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
      }
    }

    // Setup anchor frame
    ctl().centroidalManager_->setAnchorFrame();

    // Send commands to gripper
    if(config_.has("configs") && config_("configs").has("gripperCommands"))
    {
      for(const auto & gripperCommandConfig : config_("configs")("gripperCommands"))
      {
        ctl().robot().gripper(gripperCommandConfig("name")).configure(gripperCommandConfig);
      }
    }

    // Add GUI of managers
    ctl().limbManagerSet_->addToGUI(*ctl().gui());
    ctl().centroidalManager_->addToGUI(*ctl().gui());
  }
  else if(phase_ == 2)
  {
    phase_ = 3;

    // Add logger of managers
    // Considering the possibility that logger entries assume that variables are set in the manager's update method,
    // it is safe to call the update method once and then add the logger
    ctl().limbManagerSet_->addToLogger(ctl().logger());
    ctl().centroidalManager_->addToLogger(ctl().logger());
    ctl().postureManager_->addToLogger(ctl().logger());
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() <= stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      ctl().comTask_->stiffness(stiffnessRatio * comTaskStiffness_);
      ctl().baseOriTask_->stiffness(stiffnessRatio * baseOriTaskStiffness_);
      ctl().momentumTask_->stiffness(stiffnessRatio * momentumTaskStiffness_);
    }
    else
    {
      stiffnessRatioFunc_.reset();
    }
  }

  return complete();
}

void InitialState::teardown(mc_control::fsm::Controller &) {}

bool InitialState::complete() const
{
  if(phase_ != 3)
  {
    return false;
  }

  if(stiffnessRatioFunc_)
  {
    return false;
  }

  if(config_.has("configs") && config_("configs").has("gripperCommands"))
  {
    for(const auto & gripperCommandConfig : config_("configs")("gripperCommands"))
    {
      if(!ctl().robot().gripper(gripperCommandConfig("name")).complete())
      {
        return false;
      }
    }
  }

  return true;
}

EXPORT_SINGLE_STATE("MCC::Initial", InitialState)
