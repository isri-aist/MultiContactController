#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/OrientationTask.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/PostureManager.h>
#include <MultiContactController/centroidal/CentroidalManagerDDP.h>
#include <MultiContactController/centroidal/CentroidalManagerPC.h>
#include <MultiContactController/centroidal/CentroidalManagerSRB.h>

using namespace MCC;

MultiContactController::MultiContactController(mc_rbdyn::RobotModulePtr rm,
                                               double dt,
                                               const mc_rtc::Configuration & _config)
: mc_control::fsm::Controller(rm, dt, _config)
{
  // Get the robot-specific configuration
  auto rconfig = config()("robots")(robot().module().name);
  if(rconfig.empty())
  {
    mc_rtc::log::error_and_throw("[MultiContactController] {} section is empty, please provide a configuration",
                                 robot().module().name);
  }
  // Load the robot's configuration into the controller's configuration
  config().load(rconfig);
  // Load extra-overwrites
  auto overwriteConfigList = config()("OverwriteConfigList", mc_rtc::Configuration());
  auto overwriteConfigKeys = config()("OverwriteConfigKeys", std::vector<std::string>{});
  for(const auto & overwriteConfigKey : overwriteConfigKeys)
  {
    if(!overwriteConfigList.has(overwriteConfigKey))
    {
      mc_rtc::log::error_and_throw("[MultiContactController] {} in OverwriteConfigKeys but not in OverwriteConfigList",
                                   overwriteConfigKey);
    }
    config().load(overwriteConfigList(overwriteConfigKey));
  }

  config()("controllerName", name_);

  // Setup tasks
  if(config().has("CoMTask"))
  {
    comTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::CoMTask>(solver(), config()("CoMTask"));
    comTask_->name("CoMTask");
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] CoMTask configuration is missing.");
  }
  if(config().has("BaseOrientationTask"))
  {
    baseOriTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::OrientationTask>(solver(), config()("BaseOrientationTask"));
    baseOriTask_->name("BaseOriTask");
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] BaseOrientationTask configuration is missing.");
  }
  if(config().has("MomentumTask"))
  {
    momentumTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::MomentumTask>(solver(), config()("MomentumTask"));
    momentumTask_->name("MomentumTask");
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] MomentumTask configuration is missing.");
  }
  if(config().has("LimbTaskList"))
  {
    for(const auto & limbTaskConfig : config()("LimbTaskList"))
    {
      Limb limb = Limb(limbTaskConfig("limb"));
      limbTasks_.emplace(
          limb, mc_tasks::MetaTaskLoader::load<mc_tasks::force::FirstOrderImpedanceTask>(solver(), limbTaskConfig));
      limbTasks_.at(limb)->name("LimbTask_" + std::to_string(limb));
    }
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] LimbTaskList configuration is missing.");
  }

  // Setup managers
  if(config().has("LimbManagerSet"))
  {
    limbManagerSet_ = std::make_shared<LimbManagerSet>(this, config()("LimbManagerSet"));
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] LimbManagerSet configuration is missing.");
  }
  if(config().has("CentroidalManager"))
  {
    std::string centroidalManagerMethod = config()("CentroidalManager")("method", std::string(""));
    if(centroidalManagerMethod == "DDP")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerDDP>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "PC")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerPC>(this, config()("CentroidalManager"));
    }
    else if(centroidalManagerMethod == "SRB")
    {
      centroidalManager_ = std::make_shared<CentroidalManagerSRB>(this, config()("CentroidalManager"));
    }
    else
    {
      mc_rtc::log::error_and_throw("[MultiContactController] Invalid centroidalManagerMethod: {}.",
                                   centroidalManagerMethod);
    }
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] CentroidalManager configuration is missing.");
  }
  if(config().has("PostureManager"))
  {
    postureManager_ = std::make_shared<PostureManager>(this, config()("PostureManager"));
  }
  else
  {
    mc_rtc::log::warning("[MultiContactController] PostureManager configuration is missing.");
    postureManager_ = std::make_shared<PostureManager>(this); // config is not mandatory
  }

  // Load other configurations
  if(config().has("Contacts"))
  {
    const auto & contactsConfig = config()("Contacts");
    ForceColl::SurfaceContact::loadVerticesMap(contactsConfig("Surface", mc_rtc::Configuration{}));
    ForceColl::GraspContact::loadVerticesMap(contactsConfig("Grasp", mc_rtc::Configuration{}));
  }
  if(config_.has("basePose"))
  {
    // Initialize basePose from yaml only once
    auto configPose = config_("basePose").operator sva::PTransformd();
    auto & ds = datastore();
    if(!ds.has("MCC::ResetBasePose"))
    {
      ds.make<sva::PTransformd>("MCC::ResetBasePose", configPose);
    }
    else
    {
      ds.assign<sva::PTransformd>("MCC::ResetBasePose", configPose);
    }
  }
  if(config().has("saveLastBasePose"))
  {
    saveLastBasePose_ = config()("saveLastBasePose");
  }

  // Setup anchor
  setDefaultAnchor();

  mc_rtc::log::success("[MultiContactController] Constructed.");
}

void MultiContactController::reset(const mc_control::ControllerResetData & resetData)
{
  mc_control::fsm::Controller::reset(resetData);

  if(datastore().has("MCC::ResetBasePose"))
  {
    const auto & pose = datastore().get<sva::PTransformd>("MCC::ResetBasePose");
    robot().posW(pose);
    realRobot().posW(pose);
    mc_rtc::log::info("[MultiContactController] update basePose:\ntrans={}\nrot=\n{}",
                      pose.translation().transpose(), pose.rotation());
  }

  enableManagerUpdate_ = false;

  // Print message to set priority
  long tid = static_cast<long>(syscall(SYS_gettid));
  mc_rtc::log::info("[MultiContactController] TID is {}. Run the following command to set high priority:\n  sudo "
                    "renice -n -20 -p {}",
                    tid, tid);
  mc_rtc::log::info("[MultiContactController] You can check the current priority by the following command:\n  ps -p "
                    "`pgrep choreonoid` -o pid,tid,args,ni,pri,wchan m");

  mc_rtc::log::success("[MultiContactController] Reset.");
}

bool MultiContactController::run()
{
  t_ += dt();

  if(enableManagerUpdate_)
  {
    // Update managers
    limbManagerSet_->update();
    centroidalManager_->update();
    postureManager_->update();
  }

  return mc_control::fsm::Controller::run();
}

void MultiContactController::stop()
{
  // Clean up tasks
  solver().removeTask(comTask_);
  solver().removeTask(baseOriTask_);
  solver().removeTask(momentumTask_);
  for(const auto & limbTaskKV : limbTasks_)
  {
    solver().removeTask(limbTaskKV.second);
  }

  // Clean up managers
  limbManagerSet_->stop();
  centroidalManager_->stop();
  postureManager_->stop();

  // Clean up anchor
  setDefaultAnchor();

  // Save last base pose to keep base pose after changing controllers
  if(saveLastBasePose_)
  {
    auto & ds = datastore();
    if(!ds.has("MCC::ResetBasePose"))
    {
      ds.make<sva::PTransformd>("MCC::ResetBasePose", robot().posW());
    }
    else
    {
      ds.assign<sva::PTransformd>("MCC::ResetBasePose", robot().posW());
    }
  }

  mc_control::fsm::Controller::stop();
}

void MultiContactController::setDefaultAnchor()
{
  std::string anchorName = "KinematicAnchorFrame::" + robot().name();
  if(datastore().has(anchorName))
  {
    datastore().remove(anchorName);
  }
  datastore().make_call(anchorName, [](const mc_rbdyn::Robot & robot) { return robot.posW(); });
}
