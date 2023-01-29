#include <mc_rtc/gui/Button.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/OrientationTask.h>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
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
    // ctl().solver().addTask(ctl().comTask_); // \todo temporary
    ctl().baseOriTask_->reset();
    ctl().solver().addTask(ctl().baseOriTask_);
    // limb tasks are added in LimbManager

    // Setup task stiffness interpolation
    comTaskStiffness_ = ctl().comTask_->dimStiffness();
    baseOriTaskStiffness_ = ctl().baseOriTask_->dimStiffness();
    // limb tasks are added with reset (i.e., the current value is the target value), so we do not interpolate stiffness
    constexpr double stiffnessInterpDuration = 1.0; // [sec]
    stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
        std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});

    // Reset managers
    ctl().limbManagerSet_->reset(ctl().config()("Contacts")("Initial", mc_rtc::Configuration{}));
    ctl().centroidalManager_->reset();
    ctl().enableManagerUpdate_ = true;

    // Setup anchor frame
    ctl().centroidalManager_->setAnchorFrame();

    // Add GUI of managers
    ctl().limbManagerSet_->addToGUI(*ctl().gui()); // \todo
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
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() <= stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      ctl().comTask_->stiffness(stiffnessRatio * comTaskStiffness_);
      ctl().baseOriTask_->stiffness(stiffnessRatio * baseOriTaskStiffness_);
    }
    else
    {
      stiffnessRatioFunc_.reset();
    }
  }

  return (phase_ == 3 && !stiffnessRatioFunc_);
}

void InitialState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("MCC::Initial", InitialState)
