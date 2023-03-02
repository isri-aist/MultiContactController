#include <functional>

#include <mc_rbdyn/rpy_utils.h>

#include <CCC/Constants.h>
#include <CCC/PreviewControlCentroidal.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/centroidal/CentroidalManagerPC.h>

using namespace MCC;

void CentroidalManagerPC::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  if(mcRtcConfig.has("mpcWeightParam"))
  {
    mcRtcConfig("mpcWeightParam")("pos", mpcWeightParam.pos);
    mcRtcConfig("mpcWeightParam")("wrench", mpcWeightParam.wrench);
    mcRtcConfig("mpcWeightParam")("jerk", mpcWeightParam.jerk);
    mcRtcConfig("mpcWeightParam")("wrenchDistConfig", mpcWeightParam.wrench_dist_config);
  }
}

void CentroidalManagerPC::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  CentroidalManager::Configuration::addToLogger(baseEntry, logger);

  MC_RTC_LOG_HELPER(baseEntry + "_horizonDuration", horizonDuration);
  MC_RTC_LOG_HELPER(baseEntry + "_horizonDt", horizonDt);
}

CentroidalManagerPC::CentroidalManagerPC(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerPC::reset()
{
  CentroidalManager::reset();

  // Set robotMomentOfInertia_
  {
    sva::RBInertiad totalInertia(0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());
    for(const auto & body : ctl().robot().mb().bodies())
    {
      totalInertia += body.inertia();
    }
    robotMomentOfInertia_ = totalInertia.inertia().diagonal();
  }

  // Setup preview control
  pc_ = std::make_shared<CCC::PreviewControlCentroidal>(robotMass_, robotMomentOfInertia_, config_.horizonDuration,
                                                        config_.horizonDt, config_.mpcWeightParam);
}

void CentroidalManagerPC::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  MC_RTC_LOG_HELPER(config().name + "_Robot_momentOfInertia", robotMomentOfInertia_);
}

void CentroidalManagerPC::runMpc()
{
  CCC::PreviewControlCentroidal::InitialParam initialParam;
  initialParam.pos.linear() = controlData_.mpcCentroidalPose.translation();
  initialParam.pos.angular() = mc_rbdyn::rpyFromMat(controlData_.mpcCentroidalPose.rotation());
  initialParam.vel = controlData_.mpcCentroidalVel;
  initialParam.acc = controlData_.plannedCentroidalAccel;

  controlData_.plannedCentroidalWrench = pc_->planOnce(
      calcMpcMotionParam(ctl().t()), std::bind(&CentroidalManagerPC::calcMpcRefData, this, std::placeholders::_1),
      initialParam, ctl().t(), ctl().dt());
  controlData_.plannedCentroidalMomentum.force() += ctl().dt() * controlData_.plannedCentroidalWrench.force()
                                                    - robotMass_ * Eigen::Vector3d(0.0, 0.0, CCC::constants::g);
  controlData_.plannedCentroidalMomentum.moment() += ctl().dt() * controlData_.plannedCentroidalWrench.moment();
  controlData_.plannedCentroidalAccel.linear() =
      controlData_.plannedCentroidalWrench.force() / robotMass_ - Eigen::Vector3d(0.0, 0.0, CCC::constants::g);
  controlData_.plannedCentroidalAccel.angular() =
      controlData_.plannedCentroidalWrench.moment().cwiseQuotient(robotMomentOfInertia_);
}

CCC::PreviewControlCentroidal::MotionParam CentroidalManagerPC::calcMpcMotionParam(double t) const
{
  CCC::PreviewControlCentroidal::MotionParam motionParam;

  motionParam.contact_list = ForceColl::getContactVecFromMap(ctl().limbManagerSet_->contactList(t));

  return motionParam;
}

CCC::PreviewControlCentroidal::RefData CentroidalManagerPC::calcMpcRefData(double t) const
{
  CCC::PreviewControlCentroidal::RefData mpcRefData;

  const auto & refData = calcRefData(t);
  mpcRefData.pos.linear() = refData.centroidalPose.translation();
  mpcRefData.pos.angular() = mc_rbdyn::rpyFromMat(refData.centroidalPose.rotation());

  return mpcRefData;
}
