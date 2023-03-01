#pragma once

#include <mc_filter/LowPass.h>
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

    //! Nominal centroidal pose
    sva::PTransformd nominalCentroidalPose = sva::PTransformd(Eigen::Vector3d(0.0, 0.0, 1.0));

    //! Policy for determining the reference CoM Z position ("Average", "Constant", "Min", or "Max")
    std::string refComZPolicy = "Average";

    //! Limb weight list to calculate reference data
    std::unordered_map<Limb, double> limbWeightListForRefData = {{Limb("LeftFoot"), 1.0}, {Limb("RightFoot"), 1.0}};

    //! Limb weight list to calculate anchor frame
    std::unordered_map<Limb, double> limbWeightListForAnchorFrame = {{Limb("LeftFoot"), 1.0}, {Limb("RightFoot"), 1.0}};

    //! Feedback gain of centroidal pose
    sva::ImpedanceVecd centroidalGainP = sva::ImpedanceVecd::Zero();

    //! Feedback gain of centroidal velocity
    sva::ImpedanceVecd centroidalGainD = sva::ImpedanceVecd::Zero();

    //! Cutoff period of low-pass filter for velocity calculation [sec]
    double lowPassCutoffPeriod = 0.1;

    //! Whether to use actual state for MPC
    bool useActualStateForMpc = false;

    //! Whether to enable centroidal feedback
    bool enableCentroidalFeedback = true;

    //! Whether to use target limb pose for anchor frame of control robot
    bool useTargetPoseForControlRobotAnchorFrame = true;

    //! Whether to use actual CoM for wrench distribution
    bool useActualComForWrenchDist = false;

    //! Configuration for wrench distribution
    mc_rtc::Configuration wrenchDistConfig;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig);

    /** \brief Add entries to the logger. */
    virtual void addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger);

    /** \brief Remove entries from the logger. */
    virtual void removeFromLogger(mc_rtc::Logger & logger);
  };

  /** \brief Reference data. */
  struct RefData
  {
    //! Centroidal pose
    sva::PTransformd centroidalPose = sva::PTransformd::Identity();

    /** \brief Reset. */
    void reset();

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

    //! Planned centroidal wrench (moment origin is CoM)
    sva::ForceVecd plannedCentroidalWrench = sva::ForceVecd::Zero();

    //! Control centroidal wrench (moment origin is CoM)
    sva::ForceVecd controlCentroidalWrench = sva::ForceVecd::Zero();

    //! Projected control centroidal wrench (moment origin is CoM)
    sva::ForceVecd projectedControlCentroidalWrench = sva::ForceVecd::Zero();

    //! Actual centroidal wrench (moment origin is CoM)
    sva::ForceVecd actualCentroidalWrench = sva::ForceVecd::Zero();

    //! Planned ZMP (calculated from all limb wrench)
    Eigen::Vector3d plannedZmp = Eigen::Vector3d::Zero();

    //! Control ZMP (calculated from all limb wrench)
    Eigen::Vector3d controlZmp = Eigen::Vector3d::Zero();

    //! Projected control ZMP (calculated from all limb wrench)
    Eigen::Vector3d projectedControlZmp = Eigen::Vector3d::Zero();

    //! Actual ZMP (calculated from all limb wrench)
    Eigen::Vector3d actualZmp = Eigen::Vector3d::Zero();

    //! Min/max points of contact region
    std::array<Eigen::Vector2d, 2> contactRegionMinMax = {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};

    /** \brief Reset.
        \param ctlPtr pointer to controller
     */
    void reset(const MultiContactController * const ctlPtr);

    /** \brief Set centroidal state for MPC.
        \param useActualStateForMpc whether to use actual state for MPC
     */
    void setMpcState(bool useActualStateForMpc);

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

  /** \brief Append a nominal centroidal pose
      \param t time
      \param nominalCentroidalPose nominal centroidal pose to append
      \return whether nominalCentroidalPose is appended
  */
  bool appendNominalCentroidalPose(double t, const sva::PTransformd & nominalCentroidalPose);

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

  /** \brief Calculate reference data.
      \param t time
   */
  RefData calcRefData(double t) const;

  /** \brief Calculate limb average pose for reference data.
      \param t time
      \param recursive whether it is called recursively
   */
  sva::PTransformd calcLimbAveragePoseForRefData(double t, bool recursive) const;

  /** \brief Get nominal centroidal pose
      \param t time
   */
  sva::PTransformd getNominalCentroidalPose(double t) const;

  /** \brief Calculate min/max points of contact region. */
  std::array<Eigen::Vector2d, 2> calcContactRegionMinMax() const;

  /** \brief Calculate anchor frame.
      \param robot robot
   */
  sva::PTransformd calcAnchorFrame(const mc_rbdyn::Robot & robot) const;

protected:
  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Reference data
  RefData refData_;

  //! Control data
  ControlData controlData_;

  //! Robot mass [kg]
  double robotMass_ = 0;

  //! Low-pass filter for velocity calculation
  mc_filter::LowPass<sva::MotionVecd> lowPass_ = mc_filter::LowPass<sva::MotionVecd>(0.005, 0.01);

  //! Wrench distribution
  std::shared_ptr<ForceColl::WrenchDistribution> wrenchDist_;

  //! Contact list
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList_;

  //! Nominal centroidal pose list
  std::map<double, sva::PTransformd> nominalCentroidalPoseList_;
};
} // namespace MCC
