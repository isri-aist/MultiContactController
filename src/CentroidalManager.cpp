#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MomentumTask.h>
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

void CentroidalManager::ControlData::reset(const MultiContactController * const ctlPtr)
{
  *this = ControlData();
  plannedCentroidalPose = sva::PTransformd(ctlPtr->baseOriTask_->orientation(), ctlPtr->comTask_->com());
  plannedCentroidalVel = sva::MotionVecd(ctlPtr->baseOriTask_->refVel(), ctlPtr->comTask_->refVel());
}

void CentroidalManager::ControlData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_mpc", mpcCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_planned", plannedCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_actual", actualCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_mpc", mpcCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_planned", plannedCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_actual", actualCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalMomentum_mpc", mpcCentroidalMomentum);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalMomentum_planned", plannedCentroidalMomentum);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalMomentum_actual", actualCentroidalMomentum);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_planned", plannedCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_control", controlCentroidalWrench);
}

void CentroidalManager::ControlData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void CentroidalManager::RefData::reset()
{
  *this = RefData();
}

void CentroidalManager::RefData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_ref", centroidalPose);
}

void CentroidalManager::RefData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

CentroidalManager::CentroidalManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
}

void CentroidalManager::reset()
{
  refData_.reset();
  controlData_.reset(ctlPtr_);

  robotMass_ = ctl().robot().mass();
}

void CentroidalManager::update()
{
  // Set data
  refData_ = calcRefData(ctl().t());
  {
    const auto & baseOriLinkName = ctl().baseOriTask_->frame_->body();
    controlData_.actualCentroidalPose.translation() = ctl().realRobot().com();
    controlData_.actualCentroidalPose.rotation() = ctl().realRobot().bodyPosW(baseOriLinkName).rotation();
    controlData_.actualCentroidalVel.linear() = ctl().realRobot().comVelocity();
    controlData_.actualCentroidalVel.angular() = ctl().realRobot().bodyVelW(baseOriLinkName).angular();
    controlData_.actualCentroidalMomentum = rbd::computeCentroidalMomentum(
        ctl().realRobot().mb(), ctl().realRobot().mbc(), controlData_.actualCentroidalPose.translation());
  }
  if(config().useActualStateForMpc)
  {
    controlData_.mpcCentroidalPose = controlData_.actualCentroidalPose;
    controlData_.mpcCentroidalVel = controlData_.actualCentroidalVel;
    controlData_.mpcCentroidalMomentum = controlData_.actualCentroidalMomentum;
  }
  else
  {
    // Task targets are the planned state in the previous step
    controlData_.mpcCentroidalPose.translation() = ctl().comTask_->com();
    controlData_.mpcCentroidalPose.rotation() = ctl().baseOriTask_->orientation();
    controlData_.mpcCentroidalVel.linear() = ctl().comTask_->refVel();
    controlData_.mpcCentroidalVel.angular() = ctl().baseOriTask_->refVel();
    controlData_.mpcCentroidalMomentum = ctl().momentumTask_->momentum();
  }

  // Run MPC
  runMpc();

  // Apply centroidal feedback
  controlData_.controlCentroidalWrench = controlData_.plannedCentroidalWrench;
  if(config().enableCentroidalFeedback)
  {
    // sva::transformError(A, B) corresponds to (B - A).
    sva::ForceVecd deltaControlWrench =
        -1 * config().centroidalGainP
            * sva::transformError(controlData_.plannedCentroidalPose, controlData_.actualCentroidalPose)
        + -1 * config().centroidalGainD * (controlData_.actualCentroidalVel - controlData_.plannedCentroidalVel);
    controlData_.controlCentroidalWrench += deltaControlWrench;
  }

  // Distribute control wrench
  {
    contactList_ = ctl().limbManagerSet_->contactList(ctl().t());
    wrenchDist_ = std::make_shared<ForceColl::WrenchDistribution<Limb>>(contactList_, config().wrenchDistConfig);
    Eigen::Vector3d comForWrenchDist =
        (config().useActualComForWrenchDist ? ctl().realRobot().com() : ctl().comTask_->com());
    wrenchDist_->run(controlData_.controlCentroidalWrench, comForWrenchDist);
  }

  // Set target pose of tasks
  {
    controlData_.plannedCentroidalPose.translation() =
        controlData_.mpcCentroidalPose.translation() + ctl().dt() * controlData_.mpcCentroidalVel.linear()
        + 0.5 * std::pow(ctl().dt(), 2) * controlData_.plannedCentroidalAccel.linear();
    constexpr double so3IntegrationPrec = 1e-8;
    controlData_.plannedCentroidalPose.rotation() =
        rbd::SO3Integration(Eigen::Quaterniond(controlData_.mpcCentroidalPose.rotation()),
                            controlData_.mpcCentroidalVel.angular(), controlData_.plannedCentroidalAccel.angular(),
                            ctl().dt(), so3IntegrationPrec, so3IntegrationPrec)
            .first.toRotationMatrix();
    controlData_.plannedCentroidalVel =
        controlData_.mpcCentroidalVel + ctl().dt() * controlData_.plannedCentroidalAccel;

    ctl().comTask_->com(controlData_.plannedCentroidalPose.translation());
    ctl().comTask_->refVel(controlData_.plannedCentroidalVel.linear());
    ctl().comTask_->refAccel(controlData_.plannedCentroidalAccel.linear());
    ctl().baseOriTask_->orientation(controlData_.plannedCentroidalPose.rotation());
    ctl().baseOriTask_->refVel(controlData_.plannedCentroidalVel.angular());
    ctl().baseOriTask_->refAccel(controlData_.plannedCentroidalAccel.angular());
    ctl().momentumTask_->momentum(controlData_.plannedCentroidalMomentum);
    ctl().momentumTask_->refVel(Eigen::Vector6d::Zero());
    ctl().momentumTask_->refAccel(Eigen::Vector6d::Zero());
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

  // Update force visualization
  {
    ctl().gui()->removeCategory({ctl().name(), config().name, "ForceMarker"});
    wrenchDist_->addToGUI(*ctl().gui(), {ctl().name(), config().name, "ForceMarker"});
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
  refData_.addToLogger(config().name + "_Data", logger);
  controlData_.addToLogger(config().name + "_Data", logger);

  // \todo ZMP, support region
}

void CentroidalManager::removeFromLogger(mc_rtc::Logger & logger)
{
  config().removeFromLogger(logger);
  refData_.removeFromLogger(logger);
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

CentroidalManager::RefData CentroidalManager::calcRefData(double t) const
{
  RefData refData;

  Eigen::Vector3d meanPos = Eigen::Vector3d::Zero();
  {
    double totalWeight = 0;
    std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
    for(const auto & limbManagerKV : *ctl().limbManagerSet_)
    {
      if(limbManagerKV.first.group != Limb::Group::Foot)
      {
        // \todo add option
        continue;
      }
      double weight = limbManagerKV.second->getContactWeight(t);
      if(weight < std::numeric_limits<double>::min())
      {
        continue;
      }
      meanPos += weight * limbManagerKV.second->getLimbPose(t).translation();
      totalWeight += weight;
    }
    meanPos /= totalWeight;
  }

  refData.centroidalPose.translation() = meanPos + Eigen::Vector3d(0, 0, 0.7); // \todo

  return refData;
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
      pose = robot.surfacePose(ctl().limbTasks_.at(limbManagerKV.first)->surface()); // robot surface pose
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
