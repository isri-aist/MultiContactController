#include <cmath>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/OrientationTask.h>

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
  mcRtcConfig("nominalCentroidalPoseBaseFrame", nominalCentroidalPoseBaseFrame);
  mcRtcConfig("refComZPolicy", refComZPolicy);
  if(mcRtcConfig.has("limbWeightListForRefData"))
  {
    limbWeightListForRefData.clear();
    for(const auto & limbWeightKV : static_cast<std::map<std::string, double>>(mcRtcConfig("limbWeightListForRefData")))
    {
      if(limbWeightKV.second <= 0.0)
      {
        mc_rtc::log::error_and_throw(
            "[CentroidalManager] Zero or negative weight found in limbWeightListForRefData, which should be excluded.");
      }
      limbWeightListForRefData.emplace(Limb(limbWeightKV.first), limbWeightKV.second);
    }
  }
  if(mcRtcConfig.has("limbWeightListForAnchorFrame"))
  {
    limbWeightListForAnchorFrame.clear();
    for(const auto & limbWeightKV :
        static_cast<std::map<std::string, double>>(mcRtcConfig("limbWeightListForAnchorFrame")))
    {
      if(limbWeightKV.second <= 0.0)
      {
        mc_rtc::log::error_and_throw("[CentroidalManager] Zero or negative weight found in "
                                     "limbWeightListForAnchorFrame, which should be excluded.");
      }
      limbWeightListForAnchorFrame.emplace(Limb(limbWeightKV.first), limbWeightKV.second);
    }
  }
  mcRtcConfig("centroidalGainP", centroidalGainP);
  mcRtcConfig("centroidalGainD", centroidalGainD);
  mcRtcConfig("lowPassCutoffPeriod", lowPassCutoffPeriod);
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
  MC_RTC_LOG_HELPER(baseEntry + "_nominalCentroidalPoseBaseFrame", nominalCentroidalPoseBaseFrame);
  MC_RTC_LOG_HELPER(baseEntry + "_refComZPolicy", refComZPolicy);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainP", centroidalGainP);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalGainD", centroidalGainD);
  MC_RTC_LOG_HELPER(baseEntry + "_lowPassCutoffPeriod", lowPassCutoffPeriod);
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
  plannedCentroidalMomentum = ctlPtr->momentumTask_->momentum();
}

void CentroidalManager::ControlData::setMpcState(bool useActualStateForMpc)
{
  mpcCentroidalPose = (useActualStateForMpc ? actualCentroidalPose : plannedCentroidalPose);
  mpcCentroidalVel = (useActualStateForMpc ? actualCentroidalVel : plannedCentroidalVel);
  mpcCentroidalMomentum = (useActualStateForMpc ? actualCentroidalMomentum : plannedCentroidalMomentum);
}

void CentroidalManager::ControlData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_planned", plannedCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalPose_actual", actualCentroidalPose);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_planned", plannedCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalVel_actual", actualCentroidalVel);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalMomentum_planned", plannedCentroidalMomentum);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalMomentum_actual", actualCentroidalMomentum);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_planned", plannedCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_control", controlCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_projectedControl", projectedControlCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_centroidalWrench_actual", actualCentroidalWrench);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_planned", plannedZmp);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_control", controlZmp);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_projectedControl", projectedControlZmp);
  MC_RTC_LOG_HELPER(baseEntry + "_zmp_actual", actualZmp);
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

CentroidalManager::CentroidalManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & // mcRtcConfig
                                     )
: ctlPtr_(ctlPtr)
{
}

void CentroidalManager::reset()
{
  refData_.reset();
  controlData_.reset(ctlPtr_);

  robotMass_ = ctl().robot().mass();
  {
    sva::RBInertiad totalInertia(0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());
    sva::PTransformd comPoseInv = sva::PTransformd(ctl().robot().com()).inv();
    for(size_t i = 0; i < ctl().robot().mb().nrBodies(); i++)
    {
      const auto & bodyPose = ctl().robot().bodyPosW()[i];
      const auto & bodyInertia = ctl().robot().mb().body(i).inertia();
      totalInertia += (bodyPose * comPoseInv).dualMul(bodyInertia);
    }
    robotInertiaMat_ = totalInertia.inertia();
  }

  lowPass_.dt(ctl().solver().dt());
  lowPass_.reset(sva::MotionVecd::Zero());

  nominalCentroidalPoseList_.emplace(ctl().t(), config().nominalCentroidalPose);
}

void CentroidalManager::update()
{
  // Set data
  refData_ = calcRefData(ctl().t());
  {
    const auto & baseOriLinkName = ctl().baseOriTask_->frame_->body();
    controlData_.actualCentroidalPose.translation() = ctl().realRobot().com();
    controlData_.actualCentroidalPose.rotation() = ctl().realRobot().bodyPosW(baseOriLinkName).rotation();
    if(lowPass_.cutoffPeriod() != config().lowPassCutoffPeriod)
    {
      lowPass_.cutoffPeriod(config().lowPassCutoffPeriod);
    }
    lowPass_.update(
        sva::MotionVecd(ctl().realRobot().bodyVelW(baseOriLinkName).angular(), ctl().realRobot().comVelocity()));
    controlData_.actualCentroidalVel = lowPass_.eval();
    controlData_.actualCentroidalMomentum = rbd::computeCentroidalMomentum(
        ctl().realRobot().mb(), ctl().realRobot().mbc(), controlData_.actualCentroidalPose.translation());
    controlData_.actualCentroidalWrench = sva::ForceVecd::Zero();
    for(const auto & limbTaskKV : ctl().limbTasks_)
    {
      sva::PTransformd limbPoseFromCom = ctl().realRobot().frame(limbTaskKV.second->frame().name()).position()
                                         * sva::PTransformd(controlData_.actualCentroidalPose.translation()).inv();
      controlData_.actualCentroidalWrench += limbPoseFromCom.transMul(limbTaskKV.second->measuredWrench());
    }
  }
  controlData_.setMpcState(config().useActualStateForMpc);

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
    wrenchDist_ = std::make_shared<ForceColl::WrenchDistribution>(ForceColl::getContactVecFromMap(contactList_),
                                                                  config().wrenchDistConfig);
    Eigen::Vector3d comForWrenchDist =
        (config().useActualComForWrenchDist ? controlData_.actualCentroidalPose.translation()
                                            : controlData_.plannedCentroidalPose.translation());
    wrenchDist_->run(controlData_.controlCentroidalWrench, comForWrenchDist);
    controlData_.projectedControlCentroidalWrench = wrenchDist_->resultTotalWrench_;
  }

  // Update planned state for the next time step
  {
    controlData_.plannedCentroidalPose.translation() =
        controlData_.mpcCentroidalPose.translation()
        + ctl().dt()
              * (controlData_.mpcCentroidalVel.linear()
                 + 0.5 * ctl().dt() * controlData_.plannedCentroidalAccel.linear());
    Eigen::Vector3d deltaAngular =
        ctl().dt()
        * (controlData_.mpcCentroidalVel.angular() + 0.5 * ctl().dt() * controlData_.plannedCentroidalAccel.angular());
    Eigen::AngleAxisd deltaAngleAxis(Eigen::Quaterniond::Identity());
    if(deltaAngular.norm() > 1e-10)
    {
      deltaAngleAxis = Eigen::AngleAxisd(deltaAngular.norm(), deltaAngular.normalized());
    }
    // \todo The reverse order of rotation multiplication seems to be correct, but then the rotation diverges in
    // CentroidalManagerSRB
    controlData_.plannedCentroidalPose.rotation() =
        controlData_.mpcCentroidalPose.rotation() * deltaAngleAxis.toRotationMatrix().transpose();
    controlData_.plannedCentroidalVel =
        controlData_.mpcCentroidalVel + ctl().dt() * controlData_.plannedCentroidalAccel;
  }

  // Set target state of tasks
  {
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
    const auto & targetWrenchList = ForceColl::calcWrenchList(contactList_, wrenchDist_->resultWrenchRatio_);
    for(const auto & limbManagerKV : *ctl().limbManagerSet_)
    {
      sva::ForceVecd targetWrench;
      if(targetWrenchList.count(limbManagerKV.first) == 0)
      {
        targetWrench = sva::ForceVecd::Zero();
      }
      else
      {
        targetWrench = targetWrenchList.at(limbManagerKV.first);
      }
      ctl().limbTasks_.at(limbManagerKV.first)->targetWrenchW(targetWrench);
    }
  }

  // Calculate ZMP and contact region
  {
    Eigen::Vector3d zmpPlaneOrigin = calcAnchorFrame(ctl().robot()).translation();
    Eigen::Vector3d zmpPlaneNormal = Eigen::Vector3d::UnitZ();
    auto calcZmp = [&](const sva::ForceVecd & wrench, const Eigen::Vector3d & momentOrigin) {
      Eigen::Vector3d zmp = zmpPlaneOrigin;
      if(wrench.force().z() > 0)
      {
        Eigen::Vector3d momentAroundZmpPlane = wrench.moment() + (momentOrigin - zmpPlaneOrigin).cross(wrench.force());
        zmp += zmpPlaneNormal.cross(momentAroundZmpPlane) / wrench.force().z();
      }
      return zmp;
    };
    controlData_.plannedZmp =
        calcZmp(controlData_.plannedCentroidalWrench, controlData_.mpcCentroidalPose.translation());
    controlData_.controlZmp =
        calcZmp(controlData_.controlCentroidalWrench, controlData_.mpcCentroidalPose.translation());
    controlData_.projectedControlZmp =
        calcZmp(controlData_.projectedControlCentroidalWrench, controlData_.mpcCentroidalPose.translation());
    controlData_.actualZmp =
        calcZmp(controlData_.actualCentroidalWrench, controlData_.actualCentroidalPose.translation());

    controlData_.contactRegionMinMax = calcContactRegionMinMax();
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
  Eigen::Vector3d centroidMarkerSize;
  centroidMarkerSize << robotInertiaMat_(1, 1) + robotInertiaMat_(2, 2) - robotInertiaMat_(0, 0),
      robotInertiaMat_(2, 2) + robotInertiaMat_(0, 0) - robotInertiaMat_(1, 1),
      robotInertiaMat_(0, 0) + robotInertiaMat_(1, 1) - robotInertiaMat_(2, 2);
  centroidMarkerSize = ((2.0 / robotMass_) * centroidMarkerSize).cwiseSqrt();
  gui.addElement({ctl().name(), config().name, "Status"},
                 mc_rtc::gui::Ellipsoid(
                     "plannedCentroidalPose", centroidMarkerSize,
                     [this]() -> const sva::PTransformd & { return controlData_.plannedCentroidalPose; },
                     mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.8)));
  gui.addElement({ctl().name(), config().name, "Config"},
                 mc_rtc::gui::Label("method", [this]() -> const std::string & { return config().method; }),
                 mc_rtc::gui::ComboInput(
                     "nominalCentroidalPoseBaseFrame", {"LimbAveragePose", "World"},
                     [this]() -> const std::string & { return config().nominalCentroidalPoseBaseFrame; },
                     [this](const std::string & v) { config().nominalCentroidalPoseBaseFrame = v; }),
                 mc_rtc::gui::ComboInput(
                     "refComZPolicy", {"Average", "Constant", "Min", "Max"},
                     [this]() -> const std::string & { return config().refComZPolicy; },
                     [this](const std::string & v) { config().refComZPolicy = v; }),
                 mc_rtc::gui::ArrayInput(
                     "Centroidal P-Gain", {"ax", "ay", "az", "lx", "ly", "lz"},
                     [this]() -> const sva::ImpedanceVecd & { return config().centroidalGainP; },
                     [this](const Eigen::Vector6d & v) { config().centroidalGainP = sva::ImpedanceVecd(v); }),
                 mc_rtc::gui::ArrayInput(
                     "Centroidal D-Gain", {"ax", "ay", "az", "lx", "ly", "lz"},
                     [this]() -> const sva::ImpedanceVecd & { return config().centroidalGainD; },
                     [this](const Eigen::Vector6d & v) { config().centroidalGainD = sva::ImpedanceVecd(v); }),
                 mc_rtc::gui::NumberInput(
                     "lowPassCutoffPeriod", [this]() { return config().lowPassCutoffPeriod; },
                     [this](double v) { config().lowPassCutoffPeriod = v; }),
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
                       config().useTargetPoseForControlRobotAnchorFrame =
                           !config().useTargetPoseForControlRobotAnchorFrame;
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

  logger.addLogEntry(config().name + "_nominalCentroidalPose", this,
                     [this]() { return getNominalCentroidalPose(ctl().t()); });
  MC_RTC_LOG_HELPER(config().name + "_Robot_mass", robotMass_);
  logger.addLogEntry(config().name + "_Robot_momentOfInertia", this,
                     [this]() -> Eigen::Vector3d { return robotInertiaMat_.diagonal(); });
}

void CentroidalManager::removeFromLogger(mc_rtc::Logger & logger)
{
  config().removeFromLogger(logger);
  refData_.removeFromLogger(logger);
  controlData_.removeFromLogger(logger);

  logger.removeLogEntries(this);
}

bool CentroidalManager::appendNominalCentroidalPose(double t, const sva::PTransformd & nominalCentroidalPose)
{
  if(t < ctl().t())
  {
    mc_rtc::log::error("[CentroidalManager] Ignore a nominal centroidal pose with past time: {} < {}", t, ctl().t());
    return false;
  }
  if(!nominalCentroidalPoseList_.empty())
  {
    double lastTime = nominalCentroidalPoseList_.rbegin()->first;
    if(t < lastTime)
    {
      mc_rtc::log::error("[CentroidalManager] Ignore a nominal centroidal pose earlier than the last one: {} < {}", t,
                         lastTime);
      return false;
    }
  }

  nominalCentroidalPoseList_.emplace(t, nominalCentroidalPose);

  return true;
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

bool CentroidalManager::isFinished(const double t) const
{
  if(nominalCentroidalPoseList_.empty())
  {
    return true;
  }
  return t > nominalCentroidalPoseList_.rbegin()->first;
}

CentroidalManager::RefData CentroidalManager::calcRefData(double t) const
{
  RefData refData;

  sva::PTransformd nominalCentroidalPose = getNominalCentroidalPose(t);
  if(config().nominalCentroidalPoseBaseFrame == "World")
  {
    refData.centroidalPose = nominalCentroidalPose;
  }
  else // if(config().nominalCentroidalPoseBaseFrame == "LimbAveragePose")
  {
    refData.centroidalPose = nominalCentroidalPose * projGround(calcLimbAveragePoseForRefData(t, false), false);
  }

  return refData;
}

sva::PTransformd CentroidalManager::calcLimbAveragePoseForRefData(double t, bool recursive) const
{
  // Set weightPoseList
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

  // Calculate average pose
  if(weightPoseList.size() > 0)
  {
    sva::PTransformd averagePose = calcWeightedAveragePose(weightPoseList);
    if(config().refComZPolicy == "Constant")
    {
      averagePose.translation().z() = 0.0;
    }
    else if(config().refComZPolicy == "Min" || config().refComZPolicy == "Max")
    {
      double posZ = weightPoseList.front().second.translation().z();
      for(const auto & weightPoseKV : weightPoseList)
      {
        if(config().refComZPolicy == "Min")
        {
          posZ = std::min(posZ, weightPoseKV.second.translation().z());
        }
        else if(config().refComZPolicy == "Max")
        {
          posZ = std::max(posZ, weightPoseKV.second.translation().z());
        }
      }
      averagePose.translation().z() = posZ;
    }
    return averagePose;
  }
  else
  {
    if(recursive)
    {
      mc_rtc::log::error_and_throw(
          "[CentroidalManager] weightPoseList should not be empty in recursive call of calcLimbAveragePoseForRefData.");
    }

    // Calculate closestContactTimes
    std::unordered_set<Limb> limbs;
    for(const auto & weightKV : config().limbWeightListForRefData)
    {
      limbs.insert(weightKV.first);
    }
    const auto & closestContactTimes = ctl().limbManagerSet_->getClosestContactTimes(t, limbs);

    // Calculate closestAveragePoses
    std::array<sva::PTransformd, 2> closestAveragePoses;
    for(int i = 0; i < 2; i++)
    {
      if(std::isnan(closestContactTimes[i]))
      {
        mc_rtc::log::error_and_throw(
            "[CentroidalManager] closestContactTimes[{}] is NaN in calcLimbAveragePoseForRefData.", i);
      }
      closestAveragePoses[i] = calcLimbAveragePoseForRefData(closestContactTimes[i], true);
    }

    return sva::interpolate(closestAveragePoses[0], closestAveragePoses[1], 0.5);
  }
}

sva::PTransformd CentroidalManager::getNominalCentroidalPose(double t) const
{
  auto it = nominalCentroidalPoseList_.upper_bound(t);
  if(it == nominalCentroidalPoseList_.begin())
  {
    mc_rtc::log::error_and_throw(
        "[CentroidalManager] Past time is specified in getNominalCentroidalPose. specified time: {}, current time: {}",
        t, ctl().t());
  }
  it--;
  return it->second;
}

std::array<Eigen::Vector2d, 2> CentroidalManager::calcContactRegionMinMax() const
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
    for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
    {
      const Eigen::Vector2d & pos = vertexWithRidge.vertex.head<2>();
      minPos = minPos.cwiseMin(pos);
      maxPos = maxPos.cwiseMax(pos);
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
      pose = robot.frame(ctl().limbTasks_.at(limbManagerKV.first)->frame().name()).position(); // robot limb pose
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
