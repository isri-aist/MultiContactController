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
  mcRtcConfig("nominalCentroidalPose", nominalCentroidalPose);
  if(mcRtcConfig.has("limbWeightListForRefData"))
  {
    limbWeightListForRefData.clear();
    for(const auto & limbWeightKV : static_cast<std::map<std::string, double>>(mcRtcConfig("limbWeightListForRefData")))
    {
      limbWeightListForRefData.emplace(Limb(limbWeightKV.first), limbWeightKV.second);
    }
  }
  if(mcRtcConfig.has("limbWeightListForAnchorFrame"))
  {
    limbWeightListForAnchorFrame.clear();
    for(const auto & limbWeightKV :
        static_cast<std::map<std::string, double>>(mcRtcConfig("limbWeightListForAnchorFrame")))
    {
      limbWeightListForAnchorFrame.emplace(Limb(limbWeightKV.first), limbWeightKV.second);
    }
  }
  mcRtcConfig("centroidalGainP", centroidalGainP);
  mcRtcConfig("centroidalGainD", centroidalGainD);
  mcRtcConfig("useActualStateForMpc", useActualStateForMpc);
  mcRtcConfig("enableCentroidalFeedback", enableCentroidalFeedback);
  mcRtcConfig("useTargetPoseForControlRobotAnchorFrame", useTargetPoseForControlRobotAnchorFrame);
  mcRtcConfig("useActualComForWrenchDist", useActualComForWrenchDist);
  mcRtcConfig("wrenchDistConfig", wrenchDistConfig);
}

void CentroidalManager::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_method", method);
  MC_RTC_LOG_HELPER(baseEntry + "_nominalCentroidalPose", nominalCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainP", centroidalGainP);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainD", centroidalGainD);
  MC_RTC_LOG_HELPER(baseEntry + "_useActualStateForMpc", useActualStateForMpc);
  MC_RTC_LOG_HELPER(baseEntry + "_enableCentroidalFeedback", enableCentroidalFeedback);
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
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_actual", actualCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_planned", plannedZmp);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_control", controlZmp);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_actual", actualZmp);
  logger.addLogEntry(baseEntry + "_surfaceRegion_min", this,
                     [this]() -> const Eigen::Vector2d & { return surfaceRegionMinMax[0]; });
  logger.addLogEntry(baseEntry + "_surfaceRegion_max", this,
                     [this]() -> const Eigen::Vector2d & { return surfaceRegionMinMax[1]; });
  logger.addLogEntry(baseEntry + "_contactRegion_min", this,
                     [this]() -> const Eigen::Vector2d & { return contactRegionMinMax[0]; });
  logger.addLogEntry(baseEntry + "_contactRegion_max", this,
                     [this]() -> const Eigen::Vector2d & { return contactRegionMinMax[1]; });
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
    controlData_.actualCentroidalWrench = sva::ForceVecd::Zero();
    for(const auto & limbTaskKV : ctl().limbTasks_)
    {
      controlData_.actualCentroidalWrench +=
          ctl().realRobot().surfacePose(limbTaskKV.second->surface()).transMul(limbTaskKV.second->measuredWrench());
    }
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

  // Calculate ZMP and support region
  {
    Eigen::Vector3d zmpPlaneOrigin = calcAnchorFrame(ctl().robot()).translation();
    Eigen::Vector3d zmpPlaneNormal = Eigen::Vector3d::UnitZ();
    auto calcZmp = [&](const sva::ForceVecd & wrench) -> Eigen::Vector3d {
      Eigen::Vector3d zmp = zmpPlaneOrigin;
      if(wrench.force().z() > 0)
      {
        Eigen::Vector3d momentInZmpPlane = wrench.moment() - zmpPlaneOrigin.cross(wrench.force());
        zmp += zmpPlaneNormal.cross(momentInZmpPlane) / wrench.force().z();
      }
      return zmp;
    };
    controlData_.plannedZmp = calcZmp(controlData_.plannedCentroidalWrench);
    controlData_.controlZmp = calcZmp(controlData_.controlCentroidalWrench);
    controlData_.actualZmp = calcZmp(controlData_.actualCentroidalWrench);

    controlData_.surfaceRegionMinMax = calcSupportRegionMinMax(true);
    controlData_.contactRegionMinMax = calcSupportRegionMinMax(false);
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

  // Set list of weight and limb pose
  std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
  for(const auto & limbManagerKV : *ctl().limbManagerSet_)
  {
    if(config().limbWeightListForRefData.count(limbManagerKV.first) == 0)
    {
      continue;
    }

    double weight =
        config().limbWeightListForRefData.at(limbManagerKV.first) * limbManagerKV.second->getContactWeight(t);
    if(weight < std::numeric_limits<double>::min())
    {
      continue;
    }

    weightPoseList.emplace_back(weight, limbManagerKV.second->getLimbPose(t));
  }

  // Calculate weighted average
  if(weightPoseList.size() == 0)
  {
    mc_rtc::log::error_and_throw("[CentroidalManager] weightPoseList is empty in calcRefData.");
  }
  refData.centroidalPose = config().nominalCentroidalPose * projGround(calcWeightedAveragePose(weightPoseList), false);

  return refData;
}

std::array<Eigen::Vector2d, 2> CentroidalManager::calcSupportRegionMinMax(bool useSurfaceVertices) const
{
  if(contactList_.empty())
  {
    Eigen::Vector2d pos = ctl().robot().posW().translation().head<2>();
    return {pos, pos};
  }

  Eigen::Vector2d minPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d maxPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());
  for(const auto & contactKV : contactList_)
  {
    if(useSurfaceVertices)
    {
      const auto & surface = ctl().robot().surface(ctl().limbTasks_.at(contactKV.first)->surface());
      for(const auto & pos : calcSurfaceVertexList(surface, ctl().limbTasks_.at(contactKV.first)->targetPose()))
      {
        minPos = minPos.cwiseMin(pos.head<2>());
        maxPos = maxPos.cwiseMax(pos.head<2>());
      }
    }
    else
    {
      for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
      {
        const Eigen::Vector2d & pos = vertexWithRidge.vertex.head<2>();
        minPos = minPos.cwiseMin(pos);
        maxPos = maxPos.cwiseMax(pos);
      }
    }
  }

  return {minPos, maxPos};
}

sva::PTransformd CentroidalManager::calcAnchorFrame(const mc_rbdyn::Robot & robot) const
{
  bool isControlRobot = (&(ctl().robot()) == &robot);

  // Set list of weight and limb pose
  std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
  for(const auto & limbManagerKV : *ctl().limbManagerSet_)
  {
    if(config().limbWeightListForAnchorFrame.count(limbManagerKV.first) == 0)
    {
      continue;
    }

    double weight = config().limbWeightListForAnchorFrame.at(limbManagerKV.first)
                    * limbManagerKV.second->getContactWeight(ctl().t());
    if(weight < std::numeric_limits<double>::min())
    {
      continue;
    }

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
    return robot.posW();
  }
  return calcWeightedAveragePose(weightPoseList);
}
