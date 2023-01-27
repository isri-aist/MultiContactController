#pragma once

#include <mc_rtc/gui/StateBuilder.h>

#include <TrajColl/CubicInterpolator.h>

#include <MultiContactController/SwingTraj.h>

namespace MCC
{
/** \brief Simple limb swing trajectory with cubic spline.

    The position is interpolated by a single 3D cubic spline. The rotation is interpolated by a single cubic
   interpolator.
 */
class SwingTrajCubicSplineSimple : public SwingTraj
{
public:
  /** \brief Configuration. */
  struct Configuration : public SwingTraj::Configuration
  {
    //! Duration ratio to withdraw limb
    double withdrawDurationRatio = 0.2;

    //! Position offset to withdraw limb [m]
    Eigen::Vector3d withdrawOffset = Eigen::Vector3d(0, 0, 0.03);

    //! Duration ratio to approach limb
    double approachDurationRatio = 0.2;

    //! Position offset to approach limb [m]
    Eigen::Vector3d approachOffset = Eigen::Vector3d(0, 0, 0.03);

    //! Position offset to swing limb [m]
    Eigen::Vector3d swingOffset = Eigen::Vector3d(0, 0, 0.1);

    /** \brief Constructor.

        This is necessary for https://stackoverflow.com/q/53408962
    */
    Configuration() {}

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

public:
  //! Default configuration
  static inline Configuration defaultConfig_;

  /** \brief Load mc_rtc configuration to the default configuration.
      \param mcRtcConfig mc_rtc configuration
  */
  static void loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Add entries of default configuration to the GUI.
      \param gui GUI
      \param category category of GUI entries
   */
  static void addConfigToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

  /** \brief Remove entries of default configuration from the GUI.
      \param gui GUI
      \param category category of GUI entries
   */
  static void removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

public:
  /** \brief Constructor.
      \param commandType type of contact command
      \param isContact whether the limb is contacting
      \param startPose start pose
      \param endPose pose end pose
      \param startTime start time
      \param endTime end time
      \param taskGain IK task gain
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTrajCubicSplineSimple(const ContactCommand::Type & commandType,
                             bool isContact,
                             const sva::PTransformd & startPose,
                             const sva::PTransformd & endPose,
                             double startTime,
                             double endTime,
                             const TaskGain & taskGain,
                             const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Get type of limb swing trajectory. */
  inline virtual std::string type() const override
  {
    return "CubicSplineSimple";
  }

  /** \brief Calculate the pose of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::PTransformd pose(double t) const override;

  /** \brief Calculate the velocity of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd vel(double t) const override;

  /** \brief Calculate the acceleration of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd accel(double t) const override;

  /** \brief Calculate the IK task gain of the swing trajectory at a specified time.
      \param t time
  */
  virtual TaskGain taskGain(double t) const override;

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

protected:
  //! Configuration
  Configuration config_ = defaultConfig_;

  //! Position function
  std::shared_ptr<TrajColl::PiecewiseFunc<Eigen::Vector3d>> posFunc_;

  //! Rotation function
  std::shared_ptr<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> rotFunc_;

  //! Stiffness ratio function
  std::shared_ptr<TrajColl::CubicInterpolator<double>> stiffnessRatioFunc_;
};
} // namespace MCC
