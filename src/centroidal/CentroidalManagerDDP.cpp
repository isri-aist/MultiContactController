#include <functional>

#include <CCC/Constants.h>
#include <CCC/DdpCentroidal.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/centroidal/CentroidalManagerDDP.h>

using namespace MCC;

void CentroidalManagerDDP::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  mcRtcConfig("ddpMaxIter", ddpMaxIter);
  if(mcRtcConfig.has("weightParam"))
  {
    mcRtcConfig("weightParam")("running_pos", weightParam.running_pos);
    mcRtcConfig("weightParam")("running_linear_momentum", weightParam.running_linear_momentum);
    mcRtcConfig("weightParam")("running_angular_momentum", weightParam.running_angular_momentum);
    mcRtcConfig("weightParam")("running_force", weightParam.running_force);
    mcRtcConfig("weightParam")("terminal_pos", weightParam.terminal_pos);
    mcRtcConfig("weightParam")("terminal_linear_momentum", weightParam.terminal_linear_momentum);
    mcRtcConfig("weightParam")("terminal_angular_momentum", weightParam.terminal_angular_momentum);
  }
}

void CentroidalManagerDDP::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  CentroidalManager::Configuration::addToLogger(baseEntry, logger);

  MC_RTC_LOG_HELPER(baseEntry + "_horizonDuration", horizonDuration);
  MC_RTC_LOG_HELPER(baseEntry + "_horizonDt", horizonDt);
  MC_RTC_LOG_HELPER(baseEntry + "_ddpMaxIter", ddpMaxIter);
}

CentroidalManagerDDP::CentroidalManagerDDP(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerDDP::reset()
{
  CentroidalManager::reset();

  ddp_ = std::make_shared<CCC::DdpCentroidal>(robotMass_, config_.horizonDt,
                                              static_cast<int>(std::floor(config_.horizonDuration / config_.horizonDt)),
                                              config_.weightParam);
  ddp_->ddp_solver_->config().max_iter = config_.ddpMaxIter;
}

void CentroidalManagerDDP::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_DDP_computationDuration", this,
                     [this]() { return ddp_->ddp_solver_->computationDuration().solve; });
  logger.addLogEntry(config_.name + "_DDP_iter", this, [this]() {
    return ddp_->ddp_solver_->traceDataList().empty() ? 0 : ddp_->ddp_solver_->traceDataList().back().iter;
  });
}

void CentroidalManagerDDP::runMpc()
{
  CCC::DdpCentroidal::InitialParam initialParam;
  initialParam.pos = controlData_.mpcCentroidalPose.translation();
  initialParam.vel = controlData_.mpcCentroidalVel.linear();
  initialParam.angular_momentum = controlData_.mpcCentroidalMomentum.moment();
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
      std::bind(&CentroidalManagerDDP::calcMotionParam, this, std::placeholders::_1),
      std::bind(&CentroidalManagerDDP::calcRefData, this, std::placeholders::_1), initialParam, ctl().t());

  const auto & motionParam = calcMotionParam(ctl().t());
  Eigen::Vector6d totalWrench = motionParam.calcTotalWrench(plannedForceScales);
  controlData_.plannedCentroidalWrench = sva::ForceVecd(totalWrench.tail<3>(), totalWrench.head<3>());
  controlData_.plannedCentroidalMomentum = sva::ForceVecd(ddp_->ddp_solver_->controlData().x_list[1].segment<3>(6),
                                                          ddp_->ddp_solver_->controlData().x_list[1].segment<3>(3));
  controlData_.plannedCentroidalAccel.linear() =
      controlData_.plannedCentroidalWrench.force() / robotMass_ - Eigen::Vector3d(0.0, 0.0, CCC::constants::g);
  controlData_.plannedCentroidalAccel.angular().setZero(); // \todo
}

CCC::DdpCentroidal::MotionParam CentroidalManagerDDP::calcMotionParam(double t) const
{
  CCC::DdpCentroidal::MotionParam motionParam;
  motionParam.vertex_ridge_list; // \todo
  return motionParam;
}

CCC::DdpCentroidal::RefData CentroidalManagerDDP::calcRefData(double t) const
{
  CCC::DdpCentroidal::RefData refData;
  refData.pos; // \todo
  return refData;
}
