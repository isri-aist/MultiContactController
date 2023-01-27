#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/ConfigUtils.h>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/swing/SwingTrajCubicSplineSimple.h>

using namespace MCC;

MultiContactController::MultiContactController(mc_rbdyn::RobotModulePtr rm,
                                               double dt,
                                               const mc_rtc::Configuration & _config)
: mc_control::fsm::Controller(rm, dt, BWC::overwriteConfig(_config, rm->name))
{
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

  // Load other configurations
  if(config().has("SwingTraj"))
  {
    SwingTrajCubicSplineSimple::loadDefaultConfig(config()("SwingTraj")("CubicSplineSimple", mc_rtc::Configuration{}));
  }

  // Setup anchor
  setDefaultAnchor();

  mc_rtc::log::success("[MultiContactController] Constructed.");
}

void MultiContactController::reset(const mc_control::ControllerResetData & resetData)
{
  mc_control::fsm::Controller::reset(resetData);

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
  }

  return mc_control::fsm::Controller::run();
}

void MultiContactController::stop()
{
  // Clean up tasks
  solver().removeTask(comTask_);
  solver().removeTask(baseOriTask_);
  for(const auto & limbTaskKV : limbTasks_)
  {
    solver().removeTask(limbTaskKV.second);
  }

  // Clean up managers
  limbManagerSet_->stop();
  limbManagerSet_.reset();

  // Clean up anchor
  setDefaultAnchor();

  mc_control::fsm::Controller::stop();
}

void MultiContactController::setDefaultAnchor()
{
  std::string anchorName = "KinematicAnchorFrame::" + robot().name();
  if(datastore().has(anchorName))
  {
    datastore().remove(anchorName);
  }
  datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) {
    // \todo
    return sva::PTransformd::Identity();
    // return sva::interpolate(robot.surfacePose(footManager_->surfaceName(Foot::Left)),
    //                         robot.surfacePose(footManager_->surfaceName(Foot::Right)), 0.5);
  });
}
