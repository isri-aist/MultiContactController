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
   */
  SwingCommand(const mc_rtc::Configuration & mcRtcConfig);

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
      \param _pose surface pose
      \param _constraint contact constraint
   */
  ContactCommand(double _time, const sva::PTransformd & _pose, const std::shared_ptr<ContactConstraint> & _constraint)
  : time(_time), pose(_pose), constraint(_constraint)
  {
    assert(constraint);
  }

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
   */
  ContactCommand(const mc_rtc::Configuration & mcRtcConfig);

  //! Time
  double time;

  //! Pose
  sva::PTransformd pose;

  //! Contact constraint
  std::shared_ptr<ContactConstraint> constraint;
};

/** \brief Command for a single contact step. */
struct StepCommand
{
  /** \brief Constructor.
      \param _swingCommand swing command
      \param _contactCommandList contact command list
   */
  StepCommand(const std::shared_ptr<SwingCommand> & _swingCommand,
              const std::map<double, std::shared_ptr<ContactCommand>> & _contactCommandList)
  : swingCommand(_swingCommand), contactCommandList(_contactCommandList)
  {
  }

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
   */
  StepCommand(const mc_rtc::Configuration & mcRtcConfig);

  //! Swing command
  std::shared_ptr<SwingCommand> swingCommand;

  //! Contact command list
  std::map<double, std::shared_ptr<ContactCommand>> contactCommandList;
};
} // namespace MCC
