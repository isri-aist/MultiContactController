#pragma once

#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <MultiContactController/CommandTypes.h>
#include <MultiContactController/RobotUtils.h>

namespace MCC
{
/** \brief Limb swing trajectory. */
class SwingTraj
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    virtual void load(const mc_rtc::Configuration & // mcRtcConfig
    )
    {
    }
  };

public:
  /** \brief Constructor.
      \param commandType type of swing command
      \param isContact whether the limb is contacting
      \param startPose start pose
      \param endPose pose end pose
      \param startTime start time
      \param endTime end time
      \param taskGain IK task gain
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTraj(const SwingCommand::Type & commandType,
            bool isContact,
            const sva::PTransformd & startPose,
            const sva::PTransformd & endPose,
            double startTime,
            double endTime,
            const TaskGain & taskGain,
            const mc_rtc::Configuration & = {} // mcRtcConfig
            )
  : commandType_(commandType), isContact_(isContact), startPose_(startPose), endPose_(endPose), startTime_(startTime),
    endTime_(endTime), taskGain_(taskGain)
  {
  }

  /** \brief Get type of limb swing trajectory. */
  virtual std::string type() const = 0;

  /** \brief Calculate the pose of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::PTransformd pose(double t) const = 0;

  /** \brief Calculate the velocity of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd vel(double t) const = 0;

  /** \brief Calculate the acceleration of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd accel(double t) const = 0;

  /** \brief Calculate the IK task gain of the swing trajectory at a specified time.
      \param t time
  */
  inline virtual TaskGain taskGain(double // t
  ) const
  {
    return taskGain_;
  }

  /** \brief Notify touch down detection.
      \param t time
  */
  inline virtual void touchDown(double t)
  {
    touchDownTime_ = t;
  }

  /** \brief Const accessor to the configuration. */
  virtual const Configuration & config() const = 0;

protected:
  /** \brief Accessor to the configuration. */
  virtual Configuration & config() = 0;

public:
  //! Type of swing command
  SwingCommand::Type commandType_;

  //! Whether the limb is contacting
  bool isContact_;

  //! Start pose
  sva::PTransformd startPose_;

  //! End pose
  sva::PTransformd endPose_;

  //! Start time [sec]
  double startTime_;

  //! End time [sec]
  double endTime_;

  //! IK task gain
  TaskGain taskGain_;

protected:
  //! Time when touch down is detected (-1 if not detected)
  double touchDownTime_ = -1;
};
} // namespace MCC
