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

/** \brief Contact command. */
struct ContactCommand
{
  /** \brief Type of contact command. */
  enum class Type
  {
    //! Command to add contact
    Add = 0,

    //! Command to remove contact
    Remove
  };

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
   */
  ContactCommand(const mc_rtc::Configuration & mcRtcConfig);

  //! Type of contact command
  Type type;

  //! Time to start swinging the limb
  double startTime = 0;

  //! Time to end swinging the limb
  double endTime = 0;

  //! Time to remove contact (same as startTime by default)
  double removeTime = 0;

  //! Time to add contact (only used for Type::Add, same as endTime by default)
  double addTime = 0;

  //! Goal pose of surface (only used for Type::Add)
  sva::PTransformd surfacePose = sva::PTransformd::Identity();

  //! Constraint (only used for Type::Add)
  std::shared_ptr<ContactConstraint> constraint = nullptr;

  //! Configuration for swing trajectory
  mc_rtc::Configuration swingTrajConfig = {};
};

/** \brief Contact state. */
struct ContactState
{
  /** \brief Constructor.
      \param surfacePose surface pose
      \param constraint contact constraint
   */
  ContactState(const sva::PTransformd & _surfacePose, const std::shared_ptr<ContactConstraint> & _constraint)
  : surfacePose(_surfacePose), constraint(_constraint)
  {
  }

  //! Surface pose
  sva::PTransformd surfacePose;

  //! Contact constraint
  std::shared_ptr<ContactConstraint> constraint;
};
} // namespace MCC
