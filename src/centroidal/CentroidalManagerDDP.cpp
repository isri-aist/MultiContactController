#include <functional>
#include <limits>

#include <CCC/Constants.h>
#include <CCC/DdpCentroidal.h>

#include <ForceColl/Contact.h>

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
  if(mcRtcConfig.has("mpcWeightParam"))
  {
    mcRtcConfig("mpcWeightParam")("runningPos", mpcWeightParam.running_pos);
    mcRtcConfig("mpcWeightParam")("runningLinearMomentum", mpcWeightParam.running_linear_momentum);
    mcRtcConfig("mpcWeightParam")("runningAngularMomentum", mpcWeightParam.running_angular_momentum);
    mcRtcConfig("mpcWeightParam")("runningForce", mpcWeightParam.running_force);
    mcRtcConfig("mpcWeightParam")("terminalPos", mpcWeightParam.terminal_pos);
    mcRtcConfig("mpcWeightParam")("terminalLinearMomentum", mpcWeightParam.terminal_linear_momentum);
    mcRtcConfig("mpcWeightParam")("terminalAngularMomentum", mpcWeightParam.terminal_angular_momentum);
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
                                              config_.mpcWeightParam);
  ddp_->ddp_solver_->config().max_iter = config_.ddpMaxIter;
  ddp_->ddp_solver_->config().initial_lambda = 1e-6;
  ddp_->ddp_solver_->config().lambda_min = 1e-8;
  ddp_->ddp_solver_->config().lambda_thre = 1e-7;
}

void CentroidalManagerDDP::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_ControlData_CoM_ref", this, [this]() { return calcRefData(ctl().t()).pos; });

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

  const auto & contactList = ctl().limbManagerSet_->contactList(t);
  {
    int colNum = 0;
    for(const auto & contactKV : contactList)
    {
      colNum += static_cast<int>(contactKV.second->graspMat_.cols());
    }
    motionParam.vertex_ridge_list.resize(6, colNum);
  }
  {
    int colIdx = 0;
    for(const auto & contactKV : contactList)
    {
      for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
      {
        motionParam.vertex_ridge_list.topRows<3>().middleCols(colIdx, vertexWithRidge.ridgeList.size()).colwise() =
            vertexWithRidge.vertex;
        for(const auto & ridge : vertexWithRidge.ridgeList)
        {
          motionParam.vertex_ridge_list.bottomRows<3>().col(colIdx) = ridge;
          colIdx++;
        }
      }
    }
  }

  return motionParam;
}

CCC::DdpCentroidal::RefData CentroidalManagerDDP::calcRefData(double t) const
{
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

  CCC::DdpCentroidal::RefData refData;
  refData.pos = meanPos + Eigen::Vector3d(0, 0, 0.7); // \todo
  return refData;
}
