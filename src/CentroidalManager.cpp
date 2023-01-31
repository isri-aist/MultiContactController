#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/OrientationTask.h>
#include <RBDyn/NumericalIntegration.h>

#include <ForceColl/WrenchDistribution.h>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MathUtils.h>
#include <MultiContactController/MultiContactController.h>

using namespace MCC;

void CentroidalManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("method", method);
  mcRtcConfig("centroidalGainP", centroidalGainP);
  mcRtcConfig("centroidalGainD", centroidalGainD);
  mcRtcConfig("useActualStateForMpc", useActualStateForMpc);
  mcRtcConfig("enableCentroidalFeedback", enableCentroidalFeedback);
  mcRtcConfig("useOnlyFootForAnchorFrame", useOnlyFootForAnchorFrame);
  mcRtcConfig("useTargetPoseForControlRobotAnchorFrame", useTargetPoseForControlRobotAnchorFrame);
  mcRtcConfig("useActualComForWrenchDist", useActualComForWrenchDist);
  mcRtcConfig("wrenchDistConfig", wrenchDistConfig);
}

void CentroidalManager::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_method", method);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainP", centroidalGainP);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainD", centroidalGainD);
  MC_RTC_LOG_HELPER(baseEntry + "_useActualStateForMpc", useActualStateForMpc);
  MC_RTC_LOG_HELPER(baseEntry + "_enableCentroidalFeedback", enableCentroidalFeedback);
  MC_RTC_LOG_HELPER(baseEntry + "_useOnlyFootForAnchorFrame", useOnlyFootForAnchorFrame);
  MC_RTC_LOG_HELPER(baseEntry + "_useTargetPoseForControlRobotAnchorFrame", useTargetPoseForControlRobotAnchorFrame);
  MC_RTC_LOG_HELPER(baseEntry + "_useActualComForWrenchDist", useActualComForWrenchDist);
}

void CentroidalManager::Configuration::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void CentroidalManager::ControlData::reset()
{
  *this = ControlData();
}

void CentroidalManager::ControlData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_mpc", mpcCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_planned", plannedCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_actual", actualCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_mpc", mpcCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_planned", plannedCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_actual", actualCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_wrench_planned", plannedWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_wrench_control", controlWrench);
}

void CentroidalManager::ControlData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

CentroidalManager::CentroidalManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
}

void CentroidalManager::reset()
{
  controlData_.reset();

  robotMass_ = ctl().robot().mass();
}

void CentroidalManager::update()
{
  // Set control data
  {
    const auto & baseOriLinkName = ctl().baseOriTask_->frame_->body();
    controlData_.actualCentroidalPose.translation() = ctl().realRobot().com();
    controlData_.actualCentroidalPose.rotation() = ctl().realRobot().bodyPosW(baseOriLinkName).rotation();
    controlData_.actualCentroidalVel.linear() = ctl().realRobot().comVelocity();
    controlData_.actualCentroidalVel.angular() = ctl().realRobot().bodyVelW(baseOriLinkName).angular();
  }
  if(config().useActualStateForMpc)
  {
    controlData_.mpcCentroidalPose = controlData_.actualCentroidalPose;
    controlData_.mpcCentroidalVel = controlData_.actualCentroidalVel;
  }
  else
  {
    // Task targets are the planned state in the previous step
    controlData_.mpcCentroidalPose.translation() = ctl().comTask_->com();
    controlData_.mpcCentroidalPose.rotation() = ctl().baseOriTask_->orientation();
    controlData_.mpcCentroidalVel.linear() = ctl().comTask_->refVel();
    controlData_.mpcCentroidalVel.angular() = ctl().baseOriTask_->refVel();
  }

  // Apply DCM feedback
  if(config().enableCentroidalFeedback)
  {
    // sva::transformError(A, B) corresponds to (B - A).
    sva::ForceVecd deltaControlWrench =
        -1 * config().centroidalGainP
            * sva::transformError(controlData_.plannedCentroidalPose, controlData_.actualCentroidalPose)
        + -1 * config().centroidalGainD * (controlData_.actualCentroidalVel - controlData_.plannedCentroidalVel);
    controlData_.controlWrench = controlData_.plannedWrench + deltaControlWrench;
  }

  // Distribute control wrench
  {
    contactList_ = ctl().limbManagerSet_->contactList(ctl().t());
    wrenchDist_ = std::make_shared<ForceColl::WrenchDistribution<Limb>>(contactList_, config().wrenchDistConfig);
    Eigen::Vector3d comForWrenchDist =
        (config().useActualComForWrenchDist ? ctl().realRobot().com() : ctl().comTask_->com());
    wrenchDist_->run(controlData_.controlWrench, comForWrenchDist);
  }

  // Set target pose of tasks
  {
    sva::PTransformd nextPlannedCentroidalPose;
    nextPlannedCentroidalPose.translation() =
        controlData_.mpcCentroidalPose.translation() + ctl().dt() * controlData_.mpcCentroidalVel.linear()
        + 0.5 * std::pow(ctl().dt(), 2) * controlData_.plannedCentroidalAccel.linear();
    constexpr double so3IntegrationPrec = 1e-8;
    nextPlannedCentroidalPose.rotation() =
        rbd::SO3Integration(Eigen::Quaterniond(controlData_.mpcCentroidalPose.rotation()),
                            controlData_.mpcCentroidalVel.angular(), controlData_.plannedCentroidalAccel.angular(),
                            ctl().dt(), so3IntegrationPrec, so3IntegrationPrec)
            .first.toRotationMatrix();
    sva::MotionVecd nextPlannedCentroidalVel =
        controlData_.mpcCentroidalVel + ctl().dt() * controlData_.plannedCentroidalAccel;

    ctl().comTask_->com(nextPlannedCentroidalPose.translation());
    ctl().comTask_->refVel(nextPlannedCentroidalVel.linear());
    ctl().comTask_->refAccel(controlData_.plannedCentroidalAccel.linear());
    ctl().baseOriTask_->orientation(nextPlannedCentroidalPose.rotation());
    ctl().baseOriTask_->refVel(nextPlannedCentroidalVel.angular());
    ctl().baseOriTask_->refAccel(controlData_.plannedCentroidalAccel.angular());
  }

  // Set target wrench of limb tasks
  {
    const auto & targetWrenchList = wrenchDist_->calcWrenchList();
    for(const auto & limbManagerKV : *ctl().limbManagerSet_)
    {
      sva::ForceVecd targetWrench = sva::ForceVecd::Zero();
      if(targetWrenchList.count(limbManagerKV.first))
      {
        targetWrench = targetWrenchList.at(limbManagerKV.first);
      }
      ctl().limbTasks_.at(limbManagerKV.first)->targetWrenchW(targetWrench);
    }
  }
}

void CentroidalManager::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void CentroidalManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {ctl().name(), config().name, "Config"}, mc_rtc::gui::Label("method", [this]() { return config().method; }),
      mc_rtc::gui::ArrayInput(
          "Centroidal P-Gain", {"ax", "ay", "az", "lx", "ly", "lz"},
          [this]() -> const sva::ImpedanceVecd & { return config().centroidalGainP; },
          [this](const Eigen::Vector6d & v) { config().centroidalGainP = sva::ImpedanceVecd(v); }),
      mc_rtc::gui::ArrayInput(
          "Centroidal D-Gain", {"ax", "ay", "az", "lx", "ly", "lz"},
          [this]() -> const sva::ImpedanceVecd & { return config().centroidalGainD; },
          [this](const Eigen::Vector6d & v) { config().centroidalGainD = sva::ImpedanceVecd(v); }),
      mc_rtc::gui::Checkbox(
          "useActualStateForMpc", [this]() { return config().useActualStateForMpc; },
          [this]() { config().useActualStateForMpc = !config().useActualStateForMpc; }),
      mc_rtc::gui::Checkbox(
          "enableCentroidalFeedback", [this]() { return config().enableCentroidalFeedback; },
          [this]() { config().enableCentroidalFeedback = !config().enableCentroidalFeedback; }),
      mc_rtc::gui::Checkbox(
          "useOnlyFootForAnchorFrame", [this]() { return config().useOnlyFootForAnchorFrame; },
          [this]() { config().useOnlyFootForAnchorFrame = !config().useOnlyFootForAnchorFrame; }),
      mc_rtc::gui::Checkbox(
          "useTargetPoseForControlRobotAnchorFrame",
          [this]() { return config().useTargetPoseForControlRobotAnchorFrame; },
          [this]() {
            config().useTargetPoseForControlRobotAnchorFrame = !config().useTargetPoseForControlRobotAnchorFrame;
          }),
      mc_rtc::gui::Checkbox(
          "useActualComForWrenchDist", [this]() { return config().useActualComForWrenchDist; },
          [this]() { config().useActualComForWrenchDist = !config().useActualComForWrenchDist; }));
}

void CentroidalManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config().name});
}

void CentroidalManager::addToLogger(mc_rtc::Logger & logger)
{
  config().addToLogger(config().name + "_Config", logger);
  controlData_.addToLogger(config().name + "_ControlData", logger);

  // \todo ZMP, support region
}

void CentroidalManager::removeFromLogger(mc_rtc::Logger & logger)
{
  config().removeFromLogger(logger);
  controlData_.removeFromLogger(logger);

  logger.removeLogEntries(this);
}

void CentroidalManager::setAnchorFrame()
{
  std::string anchorName = "KinematicAnchorFrame::" + ctl().robot().name();
  if(ctl().datastore().has(anchorName))
  {
    ctl().datastore().remove(anchorName);
  }
  ctl().datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) { return calcAnchorFrame(robot); });
}

sva::PTransformd CentroidalManager::calcAnchorFrame(const mc_rbdyn::Robot & robot) const
{
  bool isControlRobot = (&(ctl().robot()) == &robot);

  // Set list of weight and limb pose
  std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
  for(const auto & limbManagerKV : *ctl().limbManagerSet_)
  {
    if(config().useOnlyFootForAnchorFrame && limbManagerKV.first.group != Limb::Group::Foot)
    {
      continue;
    }

    double weight = limbManagerKV.second->getContactWeight(ctl().t());
    sva::PTransformd pose;
    if(config().useTargetPoseForControlRobotAnchorFrame && isControlRobot)
    {
      pose = ctl().limbTasks_.at(limbManagerKV.first)->targetPose(); // target pose
    }
    else
    {
      pose = ctl().limbTasks_.at(limbManagerKV.first)->surfacePose(); // control robot pose (i.e., IK result)
    }
    weightPoseList.emplace_back(weight, pose);
  }

  // Calculate weighted average
  if(weightPoseList.size() == 0)
  {
    return sva::PTransformd::Identity();
  }
  return calcWeightedAveragePose(weightPoseList);
}
