#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Form.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/GuiStepState.h>

using namespace MCC;

void GuiStepState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup GUI
  std::vector<std::string> limbs;
  for(const auto & limbManagerKV : *ctl().limbManagerSet_)
  {
    limbs.push_back(std::to_string(limbManagerKV.first));
  }
  std::vector<std::string> baseFrames = {"Control"};
  baseFrames.insert(baseFrames.end(), limbs.begin(), limbs.end());
  ctl().gui()->addElement(
      {ctl().name(), "GuiStep"},
      mc_rtc::gui::Form(
          "Command", [this](const mc_rtc::Configuration & config) { sendStepCommand(config); },
          mc_rtc::gui::FormComboInput(stepConfigKeys_.at("limb"), true, limbs, false, 0),
          mc_rtc::gui::FormComboInput(stepConfigKeys_.at("type"), true, {"Add", "Remove"}, false, 0),
          mc_rtc::gui::FormNumberInput(stepConfigKeys_.at("startTime"), true, 2.0),
          mc_rtc::gui::FormNumberInput(stepConfigKeys_.at("duration"), true, 1.0),
          mc_rtc::gui::FormArrayInput<Eigen::Vector3d>(stepConfigKeys_.at("xyz"), true, Eigen::Vector3d::Zero()),
          mc_rtc::gui::FormArrayInput<Eigen::Vector3d>(stepConfigKeys_.at("rpy"), true, Eigen::Vector3d::Zero()),
          mc_rtc::gui::FormComboInput(stepConfigKeys_.at("baseFrame"), true, baseFrames, false, 0),
          mc_rtc::gui::FormComboInput(stepConfigKeys_.at("constraintType"), true, {"Surface", "Grasp", "Empty"}, false,
                                      0)));

  output("OK");
}

bool GuiStepState::run(mc_control::fsm::Controller &)
{
  return false;
}

void GuiStepState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "GuiStep"});
}

bool GuiStepState::sendStepCommand(const mc_rtc::Configuration & config) const
{
  if(ctl().limbManagerSet_->contactCommandStacked())
  {
    mc_rtc::log::error(
        "[GuiStepState] sendStepCommand is available only when the contact command is not stacked in LimbManagerSet.");
    return false;
  }

  try
  {
    mc_rtc::Configuration stepCommandConfig;
    stepCommandConfig.add("limb", config(stepConfigKeys_.at("limb")));
    stepCommandConfig.add("type", config(stepConfigKeys_.at("type")));
    double startTime = ctl().t() + static_cast<double>(config(stepConfigKeys_.at("startTime")));
    double endTime = startTime + static_cast<double>(config(stepConfigKeys_.at("duration")));
    stepCommandConfig.add("startTime", startTime);
    stepCommandConfig.add("endTime", endTime);
    if(stepCommandConfig("type") == "Add")
    {
      Eigen::Vector3d rpy = config(stepConfigKeys_.at("rpy"));
      sva::PTransformd pose = sva::PTransformd(mc_rbdyn::rpyToMat(rpy.unaryExpr(&mc_rtc::constants::toRad)),
                                               config(stepConfigKeys_.at("xyz")));
      std::string baseFrame = config(stepConfigKeys_.at("baseFrame"));
      if(baseFrame != "Control")
      {
        Limb baseLimb = Limb(baseFrame);
        if(!ctl().limbManagerSet_->at(baseLimb)->getContactCommand(ctl().t()))
        {
          mc_rtc::log::error(
              "[GuiStepState] The base frame limb must be in contact, but the specified limb \"{}\" is not in contact.",
              std::to_string(baseLimb));
          return false;
        }
        pose = pose * ctl().limbTasks_.at(baseLimb)->targetPose();
      }
      stepCommandConfig.add("pose", pose);
      mc_rtc::Configuration constraintConfig;
      constraintConfig.add("type", config(stepConfigKeys_.at("constraintType")));
      constexpr double fricCoeff = 0.5;
      constraintConfig.add("fricCoeff", fricCoeff);
      stepCommandConfig.add("constraint", constraintConfig);
    }
    ctl().limbManagerSet_->at(Limb(stepCommandConfig("limb")))->appendStepCommand(StepCommand(stepCommandConfig));
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[GuiStepState] An exception occurred in sendStepCommand: {}", e.what());
    return false;
  }

  return true;
}

EXPORT_SINGLE_STATE("MCC::GuiStep", GuiStepState)
