#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

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
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "PostureManager";

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
    //! Nominal posture
    std::vector<std::vector<double>> posture;

    /** \brief Reset. */
    void reset();

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

  /** \brief Const accessor to the configuration. */
  virtual const Configuration & config();

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
      \param jointNames joint names to append
      \param posture nominal postures to append
      \return whether nominalCentroidalPose is appended
  */
  bool appendNominalPosture(double t, const std::vector<std::string> &jointNames,
                            const std::vector<std::vector<double>> &posture);

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
  virtual Configuration & config();

  /** \brief Calculate reference data.
      \param t time
   */
  RefData calcRefData(double t) const;

protected:
  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Reference data
  RefData refData_;

  //! Nominal posture list
  std::map<double, std::vector<std::vector<double> > > nominalPostureList_;
};
} // namespace MCC
