#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/PostureTask.h>

namespace mc_rbdyn
{
class Robot;
}

namespace MCC
{
class MultiContactController;

/** \brief Posture manager.

    Posture manager calculates the target of robot posture.
 */
class PostureManager
{
public:
  typedef std::map<std::string, std::vector<double>> PostureMap;

  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "PostureManager";

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  PostureManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

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

  // TODO: implement GUI functions

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  virtual void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Append a nominal centroidal pose
      \param t time
      \param PostureMap map from joint names to joint angle
      \return whether nominalCentroidalPose is appended
  */
  virtual bool appendNominalPosture(double t, const PostureMap & joints);

  /** \brief Get nominal posture.
      \param t time
  */
  virtual PostureMap getNominalPosture(double t) const;

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

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const
  {
    return config_;
  }

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Pointer to posture task in controller
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;

  //! Nominal posture list
  std::map<double, PostureMap> nominalPostureList_;
};
} // namespace MCC
