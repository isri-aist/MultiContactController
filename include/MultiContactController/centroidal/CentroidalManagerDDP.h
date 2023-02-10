#pragma once

#include <CCC/DdpCentroidal.h>

#include <MultiContactController/CentroidalManager.h>

namespace MCC
{
/** \brief Centroidal manager with DDP.

    Centroidal manager calculates the target of robot centroidal state through trajectory planning and feedback control.
 */
class CentroidalManagerDDP : public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    //! Horizon duration [sec]
    double horizonDuration = 2.0;

    //! Horizon dt [sec]
    double horizonDt = 0.05;

    //! DDP maximum iteration
    int ddpMaxIter = 1;

    //! Weight parameter of MPC objective function
    CCC::DdpCentroidal::WeightParam mpcWeightParam;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;

    /** \brief Add entries to the logger. */
    virtual void addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger) override;
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerDDP(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
   */
  virtual void reset() override;

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger) override;

protected:
  /** \brief Accessor to the configuration. */
  inline virtual Configuration & config() override
  {
    return config_;
  }

  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates controlData_.planned(CentroidalAccel|CentroidalMomentum|CentroidalWrench) from
     controlData_.mpc(mpcCentroidalPose|mpcCentroidalVel|mpcCentroidalMomentum).
   */
  virtual void runMpc() override;

  /** \brief Calculate motion parameter of MPC.
      \param t time
   */
  CCC::DdpCentroidal::MotionParam calcMpcMotionParam(double t) const;

  /** \brief Calculate reference data of MPC.
      \param t time
   */
  CCC::DdpCentroidal::RefData calcMpcRefData(double t) const;

protected:
  //! Configuration
  Configuration config_;

  //! DDP
  std::shared_ptr<CCC::DdpCentroidal> ddp_;
};
} // namespace MCC
