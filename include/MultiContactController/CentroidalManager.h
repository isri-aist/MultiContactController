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

    //! Feedback gain of centroidal pose
    sva::ImpedanceVecd centroidalGainP = sva::ImpedanceVecd::Zero();

    //! Feedback gain of centroidal velocity
    sva::ImpedanceVecd centroidalGainD = sva::ImpedanceVecd::Zero();

    //! Whether to use actual state for MPC
    bool useActualStateForMpc = false;

    //! Whether to enable centroidal feedback
    bool enableCentroidalFeedback = true;

    //! Whether to only use foot surfaces to calculate anchor frame
    bool useOnlyFootForAnchorFrame = true;

    //! Whether to use target surface pose for anchor frame of control robot
    bool useTargetPoseForControlRobotAnchorFrame = true;

    //! Whether to use actual CoM for wrench distribution
    bool useActualComForWrenchDist = true;

    //! Configuration for wrench distribution
    mc_rtc::Configuration wrenchDistConfig;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig);

    /** \brief Add entries to the logger. */
    virtual void addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger);

    /** \brief Remove entries from the logger. */
    virtual void removeFromLogger(mc_rtc::Logger & logger);
  };

  /** \brief Control data. */
  struct ControlData
  {
    //! Centroidal pose used as the initial state of MPC
    sva::PTransformd mpcCentroidalPose = sva::PTransformd::Identity();

    //! Planned centroidal pose
    sva::PTransformd plannedCentroidalPose = sva::PTransformd::Identity();

    //! Actual centroidal pose
    sva::PTransformd actualCentroidalPose = sva::PTransformd::Identity();

    //! Centroidal velocity used as the initial state of MPC
    sva::MotionVecd mpcCentroidalVel = sva::MotionVecd::Zero();

    //! Planned centroidal velocity
    sva::MotionVecd plannedCentroidalVel = sva::MotionVecd::Zero();

    //! Actual centroidal velocity
    sva::MotionVecd actualCentroidalVel = sva::MotionVecd::Zero();

    //! Planned centroidal acceleration
    sva::MotionVecd plannedCentroidalAccel = sva::MotionVecd::Zero();

    //! Centroidal momentum used as the initial state of MPC
    sva::ForceVecd mpcCentroidalMomentum = sva::ForceVecd::Zero();

    //! Planned centroidal momentum
    sva::ForceVecd plannedCentroidalMomentum = sva::ForceVecd::Zero();

    //! Actual centroidal momentum
    sva::ForceVecd actualCentroidalMomentum = sva::ForceVecd::Zero();

    //! Planned centroidal wrench
    sva::ForceVecd plannedCentroidalWrench = sva::ForceVecd::Zero();

    //! Control centroidal wrench
    sva::ForceVecd controlCentroidalWrench = sva::ForceVecd::Zero();

    /** \brief Reset. */
    void reset(const MultiContactController * const ctlPtr);

    /** \brief Add entries to the logger. */
    virtual void addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger);

    /** \brief Remove entries from the logger. */
    virtual void removeFromLogger(mc_rtc::Logger & logger);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
   */
  virtual void reset();

  /** \brief Update.

      This method should be called once every control cycle.
   */
  virtual void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  virtual void stop();

  /** \brief Const accessor to the configuration. */
  virtual const Configuration & config() const = 0;

  /** \brief Add entries to the GUI. */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  virtual void removeFromLogger(mc_rtc::Logger & logger);

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

      This method calculates controlData_.planned(CentroidalAccel|CentroidalMomentum|CentroidalWrench) from
     controlData_.mpc(mpcCentroidalPose|mpcCentroidalVel|mpcCentroidalMomentum).
   */
  virtual void runMpc() = 0;

  /** \brief Calculate anchor frame.
      \param robot robot
   */
  sva::PTransformd calcAnchorFrame(const mc_rbdyn::Robot & robot) const;

protected:
  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Control data
  ControlData controlData_;

  //! Robot mass [kg]
  double robotMass_ = 0;

  //! Wrench distribution
  std::shared_ptr<ForceColl::WrenchDistribution<Limb>> wrenchDist_;

  //! Contact list
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList_;
};
} // namespace MCC
