#pragma once

#include <mc_rtc/Configuration.h>

namespace ForceColl
{
class Contact;
}

namespace MCC
{
/** \brief Contact constraint. */
using ContactConstraint = ForceColl::Contact;

/** \brief Swing command. */
struct SwingCommand
{
  /** \brief Type of swing command. */
  enum class Type
  {
    //! Command to add contact
    Add = 0,

    //! Command to remove contact
    Remove
  };

  static inline const std::unordered_map<std::string, Type> strToType = {{"Add", Type::Add}, {"Remove", Type::Remove}};

  /** \brief Constructor.
      \param _type type of swing command
      \param _startTime time to start swinging the limb
      \param _endTime time to end swinging the limb
      \param _pose pose (only used for Type::Add)
      \param _config configuration for swing trajectory
   */
  SwingCommand(const Type & _type,
               double _startTime,
               double _endTime,
               const sva::PTransformd & _pose,
               const mc_rtc::Configuration & _config = {})
  : type(_type), startTime(_startTime), endTime(_endTime), pose(_pose), config(_config)
  {
  }

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration

      An example of \p mcRtcConfig is as follows.
      @code
      type: Add
      startTime: 2.0
      endTime: 5.0
      pose:
        translation: [1.0, 0.0, 0.0]
      config:
        approachOffset: [0.0, 0.0, 0.1]
      @endcode
   */
  SwingCommand(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Set base time.
      \param baseTime base time

      Overwrites all times in the command, assuming they are expressed relative to baseTime.
   */
  void setBaseTime(double baseTime);

  //! Type of swing command
  Type type;

  //! Time to start swinging the limb
  double startTime;

  //! Time to end swinging the limb
  double endTime;

  //! Pose (only used for Type::Add)
  sva::PTransformd pose = sva::PTransformd::Identity();

  //! Configuration for swing trajectory
  mc_rtc::Configuration config = {};
};

/** \brief Contact command. */
struct ContactCommand
{
  /** \brief Constructor.
      \param _time time
      \param _constraint contact constraint (nullptr is not allowed)
   */
  ContactCommand(double _time, const std::shared_ptr<ContactConstraint> & _constraint);

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration

      An example of \p mcRtcConfig is as follows.
      @code
      time: 2.0
      constraint:
        type: Surface
        name: ContactName
        fricCoeff: 1.0
        verticesName: VerticesName
        pose:
          translation: [1.0, 0.0, 0.0]
      @endcode
   */
  ContactCommand(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Set base time.
      \param baseTime base time

      Overwrites all times in the command, assuming they are expressed relative to baseTime.
   */
  void setBaseTime(double baseTime);

  //! Time
  double time;

  //! Contact constraint
  std::shared_ptr<ContactConstraint> constraint;
};

/** \brief Gripper command. */
struct GripperCommand
{
  /** \brief Constructor.
      \param _time time
      \param _name gripper name
      \param _config configuration for gripper command
   */
  GripperCommand(double _time, const std::string & _name, const mc_rtc::Configuration & _config)
  : time(_time), name(_name), config(_config)
  {
  }

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration

      An example of \p mcRtcConfig is as follows.
      @code
      time: 2.0
      name: l_gripper
      config:
        opening: 0.0 # Close gripper
      @endcode
   */
  GripperCommand(const mc_rtc::Configuration & mcRtcConfig)
  : GripperCommand(mcRtcConfig("time"), mcRtcConfig("name"), mcRtcConfig("config"))
  {
  }

  /** \brief Set base time.
      \param baseTime base time

      Overwrites all times in the command, assuming they are expressed relative to baseTime.
   */
  void setBaseTime(double baseTime);

  //! Time
  double time;

  //! Gripper name
  std::string name;

  //! Configuration for gripper command
  mc_rtc::Configuration config;
};

/** \brief Command for a single contact step. */
struct StepCommand
{
  /** \brief Constructor.
      \param _swingCommand swing command
      \param _contactCommandList contact command list
      \param _gripperCommandList gripper command list
   */
  StepCommand(const std::shared_ptr<SwingCommand> & _swingCommand,
              const std::map<double, std::shared_ptr<ContactCommand>> & _contactCommandList,
              const std::map<double, std::shared_ptr<GripperCommand>> & _gripperCommandList)
  : swingCommand(_swingCommand), contactCommandList(_contactCommandList), gripperCommandList(_gripperCommandList)
  {
  }

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
   */
  StepCommand(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Set base time.
      \param baseTime base time

      Overwrites all times in the command, assuming they are expressed relative to baseTime.
   */
  void setBaseTime(double baseTime);

  //! Swing command
  std::shared_ptr<SwingCommand> swingCommand;

  //! Contact command list
  std::map<double, std::shared_ptr<ContactCommand>> contactCommandList;

  //! Gripper command list
  std::map<double, std::shared_ptr<GripperCommand>> gripperCommandList;
};
} // namespace MCC
