#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <MultiContactController/CommandTypes.h>
#include <MultiContactController/LimbTypes.h>

namespace mc_rbdyn
{
class Robot;
}

namespace ForceColl
{
template<class PatchID>
class WrenchDistribution;
} // namespace ForceColl

namespace MCC
{
class MultiContactController;

/** \brief Centroidal manager.

    Centroidal manager calculates the target of robot centroidal state through trajectory planning and feedback control.
 */
class CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "CentroidalManager";

    //! Method
    std::string method = "";

    //! Whether to use actual state for MPC
    bool useActualStateForMpc = false;

    //! Whether to enable DCM feedback
    bool enableFeedback = true;

    //! Whether to only use foot surfaces to calculate anchor frame
    bool useOnlyFootForAnchorFrame = true;

    //! Whether to use target surface pose for anchor frame of control robot
    bool useTargetPoseForControlRobotAnchorFrame = true;

    //! Whether to use actual CoM for wrench distribution
    bool useActualComForWrenchDist = true;

    //! Configuration for wrench distribution
    mc_rtc::Configuration wrenchDistConfig;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig){}; // \todo
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {})
  : ctlPtr_(ctlPtr){};

  /** \brief Reset.

      This method should be called once when controller is reset.
   */
  virtual void reset(){};

  /** \brief Update.

      This method should be called once every control cycle.
   */
  virtual void update(){};

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  virtual void stop(){};

  /** \brief Const accessor to the configuration. */
  virtual const Configuration & config() const = 0;

  /** \brief Add entries to the GUI. */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui){};

  /** \brief Remove entries from the GUI. */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder & gui){};

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger){};

  /** \brief Remove entries from the logger. */
  virtual void removeFromLogger(mc_rtc::Logger & logger){};

  /** \brief Set anchor frame. */
  void setAnchorFrame();

protected:
  /** \brief Const accessor to the controller. */
  inline const MultiContactController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline MultiContactController & ctl()
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the configuration. */
  virtual Configuration & config() = 0;

  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() = 0;

  /** \brief Calculate anchor frame.
      \param robot robot
   */
  sva::PTransformd calcAnchorFrame(const mc_rbdyn::Robot & robot) const;

protected:
  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Robot mass [kg]
  double robotMass_ = 0;

  //! CoM used as the initial state of MPC
  Eigen::Vector3d mpcCom_ = Eigen::Vector3d::Zero();

  //! CoM velocity used as the initial state of MPC
  Eigen::Vector3d mpcComVel_ = Eigen::Vector3d::Zero();

  //! Wrench distribution
  std::shared_ptr<ForceColl::WrenchDistribution<Limb>> wrenchDist_;

  //! Contact list
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList_;
};
} // namespace MCC
