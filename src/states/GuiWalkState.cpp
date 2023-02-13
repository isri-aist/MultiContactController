#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Form.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MathUtils.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/GuiWalkState.h>

using namespace MCC;

void GuiWalkState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  if(!(ctl().limbManagerSet_->count(leftFoot) && ctl().limbManagerSet_->count(rightFoot)))
  {
    mc_rtc::log::error_and_throw(
        "[GuiWalkState] The \"LeftFoot\" and \"RightFoot\" limbs, which are required, are missing.");
  }

  // Load configuration
  if(config_.has("configs"))
  {
    config_("configs")("footstepDuration", footstepDuration_);
    config_("configs")("doubleSupportRatio", doubleSupportRatio_);
    if(config_("configs").has("deltaTransLimit"))
    {
      deltaTransLimit_ = config_("configs")("deltaTransLimit");
      deltaTransLimit_[2] = mc_rtc::constants::toRad(deltaTransLimit_[2]);
    }
    if(config_("configs").has("midToFootTranss"))
    {
      for(const Limb & foot : {leftFoot, rightFoot})
      {
        config_("configs")("midToFootTranss")(std::to_string(foot), midToFootTranss_.at(foot));
      }
    }
  }

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "GuiWalk"},
                          mc_rtc::gui::Form(
                              "Walk",
                              [this](const mc_rtc::Configuration & config) {
                                sendWalkCommand(
                                    Eigen::Vector3d(config(walkConfigKeys_.at("x")), config(walkConfigKeys_.at("y")),
                                                    mc_rtc::constants::toRad(config(walkConfigKeys_.at("theta")))),
                                    config(walkConfigKeys_.at("last")));
                              },
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("x"), true, 0.0),
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("y"), true, 0.0),
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("theta"), true, 0.0),
                              mc_rtc::gui::FormIntegerInput(walkConfigKeys_.at("last"), true, 0)));

  output("OK");
}

bool GuiWalkState::run(mc_control::fsm::Controller &)
{
  return false;
}

void GuiWalkState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "GuiWalk"});
}

bool GuiWalkState::sendWalkCommand(const Eigen::Vector3d & targetTrans, int lastFootstepNum) const
{
  if(ctl().limbManagerSet_->contactCommandStacked())
  {
    mc_rtc::log::error(
        "[GuiWalkState] sendWalkCommand is available only when the contact command is not stacked in LimbManagerSet.");
    return false;
  }

  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  // The 2D variables (i.e., targetTrans, deltaTrans) represent the transformation relative to the initial pose,
  // while the 3D variables (i.e., initialFootMidpose, goalFootMidpose, footMidpose) represent the
  // transformation in the world frame.
  const sva::PTransformd & initialFootMidpose = projGround(
      sva::interpolate(ctl().limbTasks_.at(leftFoot)->targetPose(), ctl().limbTasks_.at(rightFoot)->targetPose(), 0.5));
  const sva::PTransformd & goalFootMidpose = convertTo3d(targetTrans) * initialFootMidpose;

  Limb foot = targetTrans.y() >= 0 ? leftFoot : rightFoot;
  sva::PTransformd footMidpose = initialFootMidpose;
  double startTime = ctl().t() + 2.0;

  while(convertTo2d(goalFootMidpose * footMidpose.inv()).norm() > 1e-6)
  {
    Eigen::Vector3d deltaTrans = convertTo2d(goalFootMidpose * footMidpose.inv());
    footMidpose = convertTo3d(clampDeltaTrans(deltaTrans, foot)) * footMidpose;

    ctl().limbManagerSet_->at(foot)->appendStepCommand(makeStepCommand(foot, footMidpose, startTime));

    foot = (std::equal_to<MCC::Limb>()(foot, leftFoot) ? rightFoot : leftFoot);
    startTime = startTime + footstepDuration_;
  }

  for(int i = 0; i < lastFootstepNum + 1; i++)
  {
    ctl().limbManagerSet_->at(foot)->appendStepCommand(makeStepCommand(foot, footMidpose, startTime));

    foot = (std::equal_to<MCC::Limb>()(foot, leftFoot) ? rightFoot : leftFoot);
    startTime = startTime + footstepDuration_;
  }

  return true;
}

Eigen::Vector3d GuiWalkState::clampDeltaTrans(const Eigen::Vector3d & deltaTrans, const Limb & foot) const
{
  Eigen::Vector3d deltaTransMax = deltaTransLimit_;
  Eigen::Vector3d deltaTransMin = -1 * deltaTransLimit_;
  if(std::equal_to<MCC::Limb>()(foot, leftFoot))
  {
    deltaTransMin.y() = 0;
  }
  else // if(std::equal_to<MCC::Limb>()(foot, rightFoot))
  {
    deltaTransMax.y() = 0;
  }
  return mc_filter::utils::clamp(deltaTrans, deltaTransMin, deltaTransMax);
}

StepCommand GuiWalkState::makeStepCommand(const Limb & foot,
                                          const sva::PTransformd & footMidpose,
                                          double startTime) const
{
  mc_rtc::Configuration stepCommandConfig;
  stepCommandConfig.add("limb", std::to_string(foot));
  stepCommandConfig.add("type", "Add");
  stepCommandConfig.add("startTime", startTime);
  stepCommandConfig.add("endTime", startTime + (1.0 - doubleSupportRatio_) * footstepDuration_);
  stepCommandConfig.add("pose", midToFootTranss_.at(foot) * footMidpose);
  mc_rtc::Configuration constraintConfig;
  constraintConfig.add("type", "Surface");
  constraintConfig.add("fricCoeff", 0.5);
  stepCommandConfig.add("constraint", constraintConfig);
  return StepCommand(stepCommandConfig);
}

EXPORT_SINGLE_STATE("MCC::GuiWalk", GuiWalkState)
