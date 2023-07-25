#include <functional>

#include <CCC/Constants.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/centroidal/CentroidalManagerSRB.h>

namespace
{
/** \brief Calculate Euler angles from rotation matrix. */
Eigen::Vector3d eulerAnglesFromRot(const Eigen::Matrix3d & rot)
{
  double euler0 = std::atan2(rot(1, 0), rot(0, 0));
  double sin0 = std::sin(euler0);
  double cos0 = std::cos(euler0);
  double euler1 = std::atan2(-1 * rot(2, 0), rot(0, 0) * cos0 + rot(1, 0) * sin0);
  double euler2 = std::atan2(rot(0, 2) * sin0 - rot(1, 2) * cos0, -1 * rot(0, 1) * sin0 + rot(1, 1) * cos0);
  return Eigen::Vector3d(euler0, euler1, euler2);
}
} // namespace

using namespace MCC;

void CentroidalManagerSRB::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  mcRtcConfig("ddpMaxIter", ddpMaxIter);
  if(mcRtcConfig.has("mpcWeightParam"))
  {
    mcRtcConfig("mpcWeightParam")("runningPos", mpcWeightParam.running_pos);
    mcRtcConfig("mpcWeightParam")("runningOri", mpcWeightParam.running_ori);
    mcRtcConfig("mpcWeightParam")("runningLinearVel", mpcWeightParam.running_linear_vel);
    mcRtcConfig("mpcWeightParam")("runningAngularVel", mpcWeightParam.running_angular_vel);
    mcRtcConfig("mpcWeightParam")("runningForce", mpcWeightParam.running_force);
    mcRtcConfig("mpcWeightParam")("terminalPos", mpcWeightParam.terminal_pos);
    mcRtcConfig("mpcWeightParam")("terminalOri", mpcWeightParam.terminal_ori);
    mcRtcConfig("mpcWeightParam")("terminalLinearVel", mpcWeightParam.terminal_linear_vel);
    mcRtcConfig("mpcWeightParam")("terminalAngularVel", mpcWeightParam.terminal_angular_vel);
  }
}

void CentroidalManagerSRB::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  CentroidalManager::Configuration::addToLogger(baseEntry, logger);

  MC_RTC_LOG_HELPER(baseEntry + "_horizonDuration", horizonDuration);
  MC_RTC_LOG_HELPER(baseEntry + "_horizonDt", horizonDt);
  MC_RTC_LOG_HELPER(baseEntry + "_ddpMaxIter", ddpMaxIter);
}

CentroidalManagerSRB::CentroidalManagerSRB(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerSRB::reset()
{
  CentroidalManager::reset();

  ddp_ = std::make_shared<CCC::DdpSingleRigidBody>(
      robotMass_, config_.horizonDt, static_cast<int>(std::floor(config_.horizonDuration / config_.horizonDt)),
      config_.mpcWeightParam);
  ddp_->ddp_solver_->config().max_iter = config_.ddpMaxIter;
}

void CentroidalManagerSRB::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_DDP_computationDuration", this,
                     [this]() { return ddp_->ddp_solver_->computationDuration().solve; });
  logger.addLogEntry(config_.name + "_DDP_iter", this, [this]() {
    return ddp_->ddp_solver_->traceDataList().empty() ? 0 : ddp_->ddp_solver_->traceDataList().back().iter;
  });
}

void CentroidalManagerSRB::runMpc()
{
  CCC::DdpSingleRigidBody::InitialParam initialParam;
  initialParam.pos = controlData_.mpcCentroidalPose.translation();
  initialParam.ori = eulerAnglesFromRot(controlData_.mpcCentroidalPose.rotation().transpose());
  initialParam.linear_vel = controlData_.mpcCentroidalVel.linear();
  initialParam.angular_vel = controlData_.mpcCentroidalVel.angular();
  initialParam.u_list = ddp_->ddp_solver_->controlData().u_list;
  if(!initialParam.u_list.empty())
  {
    for(int i = 0; i < ddp_->ddp_solver_->config().horizon_steps; i++)
    {
      double tmpTime = ctl().t() + i * ddp_->ddp_problem_->dt();
      int inputDim = ddp_->ddp_problem_->inputDim(tmpTime);
      if(initialParam.u_list[i].size() != inputDim)
      {
        initialParam.u_list[i].setZero(inputDim);
      }
    }
  }

  Eigen::VectorXd plannedForceScales = ddp_->planOnce(
      std::bind(&CentroidalManagerSRB::calcMpcMotionParam, this, std::placeholders::_1),
      std::bind(&CentroidalManagerSRB::calcMpcRefData, this, std::placeholders::_1), initialParam, ctl().t());

  const auto & motionParam = calcMpcMotionParam(ctl().t());
  controlData_.plannedCentroidalWrench = ForceColl::calcTotalWrench(motionParam.contact_list, plannedForceScales,
                                                                    controlData_.mpcCentroidalPose.translation());
  controlData_.plannedCentroidalMomentum =
      sva::ForceVecd(motionParam.inertia_mat * controlData_.plannedCentroidalVel.angular(),
                     robotMass_ * controlData_.plannedCentroidalVel.linear());
  controlData_.plannedCentroidalAccel.linear() =
      controlData_.plannedCentroidalWrench.force() / robotMass_ - Eigen::Vector3d(0.0, 0.0, CCC::constants::g);
  controlData_.plannedCentroidalAccel.angular() = motionParam.inertia_mat.llt().solve(
      -1 * controlData_.plannedCentroidalVel.angular().cross(controlData_.plannedCentroidalMomentum.moment())
      + controlData_.plannedCentroidalWrench.moment());
}

CCC::DdpSingleRigidBody::MotionParam CentroidalManagerSRB::calcMpcMotionParam(double t) const
{
  CCC::DdpSingleRigidBody::MotionParam motionParam;

  motionParam.contact_list = ForceColl::getContactVecFromMap(ctl().limbManagerSet_->contactList(t));
  motionParam.inertia_mat = robotInertiaMat_;

  return motionParam;
}

CCC::DdpSingleRigidBody::RefData CentroidalManagerSRB::calcMpcRefData(double t) const
{
  CCC::DdpSingleRigidBody::RefData refData;

  const auto & refDataBase = calcRefData(t);
  refData.pos = refDataBase.centroidalPose.translation();
  refData.ori = eulerAnglesFromRot(refDataBase.centroidalPose.rotation().transpose());

  return refData;
}
