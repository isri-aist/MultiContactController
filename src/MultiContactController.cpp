#include <sys/syscall.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/OrientationTask.h>

#include <BaselineWalkingController/ConfigUtils.h>

#include <MultiContactController/MultiContactController.h>
// #include <MultiContactController/CentroidalManager.h>
// #include <MultiContactController/FootManager.h>

using namespace MMC;

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
  // if(config().has("FootTaskList"))
  // {
  //   for(const auto & footTaskConfig : config()("FootTaskList"))
  //   {
  //     Foot foot = strToFoot(footTaskConfig("foot"));
  //     footTasks_.emplace(
  //         foot, mc_tasks::MetaTaskLoader::load<mc_tasks::force::FirstOrderImpedanceTask>(solver(), footTaskConfig));
  //     footTasks_.at(foot)->name("FootTask_" + std::to_string(foot));
  //   }
  // }
  // else
  // {
  //   mc_rtc::log::warning("[MultiContactController] FootTaskList configuration is missing.");
  // }

  // Setup managers

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
  }

  return mc_control::fsm::Controller::run();
}

void MultiContactController::stop()
{
  // Clean up tasks
  solver().removeTask(comTask_);
  solver().removeTask(baseOriTask_);
  // for(const auto & foot : Feet::Both)
  // {
  //   solver().removeTask(footTasks_.at(foot));
  // }

  // Clean up managers

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
    return sva::PTransformd::Identity();
    // return sva::interpolate(robot.surfacePose(footManager_->surfaceName(Foot::Left)),
    //                         robot.surfacePose(footManager_->surfaceName(Foot::Right)), 0.5);
  });
}
