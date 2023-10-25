#pragma once

#include <mc_control/fsm/Controller.h>

#include <MultiContactController/LimbTypes.h>

namespace mc_tasks
{
struct CoMTask;
struct OrientationTask;
struct MomentumTask;

namespace force
{
struct FirstOrderImpedanceTask;
}
} // namespace mc_tasks

namespace MCC
{
class LimbManagerSet;
class CentroidalManager;
class PostureManager;

/** \brief Humanoid multi-contact motion controller. */
struct MultiContactController : public mc_control::fsm::Controller
{
public:
  /** \brief Constructor. */
  MultiContactController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & _config);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Get controller name. */
  inline const std::string & name() const
  {
    return name_;
  }

  /** \brief Get current time. */
  inline double t() const noexcept
  {
    return t_;
  }

  /** \brief Get timestep. */
  inline double dt() const
  {
    return solver().dt();
  }

  /** \brief Set default anchor. */
  void setDefaultAnchor();

public:
  //! CoM task
  std::shared_ptr<mc_tasks::CoMTask> comTask_;

  //! Base link orientation task
  std::shared_ptr<mc_tasks::OrientationTask> baseOriTask_;

  //! Momentum task
  std::shared_ptr<mc_tasks::MomentumTask> momentumTask_;

  //! Limb tasks
  std::unordered_map<Limb, std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask>> limbTasks_;

  //! Limb manager set
  std::shared_ptr<LimbManagerSet> limbManagerSet_;

  //! Centroidal manager
  std::shared_ptr<CentroidalManager> centroidalManager_;

  //! Posture manager
  std::shared_ptr<PostureManager> postureManager_;

  //! Whether to enable manager update
  bool enableManagerUpdate_ = false;

  //! Whether to save last base pose when stopping this controller
  bool saveLastBasePose_ = false;

protected:
  //! Controller name
  std::string name_ = "MCC";

  //! Current time [sec]
  double t_ = 0;
};
} // namespace MCC
