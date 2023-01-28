#pragma once

#include <CCC/DdpCentroidal.h>

#include <MultiContactController/CentroidalManager.h>

namespace MCC
{
/** \brief Centroidal manager with DDP.

    Centroidal manager calculates the target of robot centroidal state through trajectory planning and feedback control.
 */
class CentroidalManagerDdpCentroidal : public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override{};
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerDdpCentroidal(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {})
  : CentroidalManager(ctlPtr, mcRtcConfig){};

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

protected:
  /** \brief Accessor to the configuration. */
  inline virtual Configuration & config() override
  {
    return config_;
  }

  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() override {}

protected:
  //! Configuration
  Configuration config_;

  //! DDP
  std::shared_ptr<CCC::DdpCentroidal> ddp_;
};
} // namespace MCC
