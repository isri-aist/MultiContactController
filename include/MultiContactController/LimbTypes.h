#pragma once

#include <string>

namespace MCC
{
/** \brief Limb. */
struct Limb
{
  /** \brief Limb type. */
  enum class Type
  {
    //! Hand
    Hand = 0,

    //! Foot
    Foot,

    //! Other
    Other
  };

  /** \brief Constructor.
      \param _name limb name
      \param _type limb type (automatically set if "hand" or "foot" (case-insensitive) is included in the name)
  */
  Limb(const std::string & _name, const Type & _type = Type::Other) : name(_name), type(_type) {}

  //! Limb name
  std::string name;

  //! Limb type
  Type type;
};
} // namespace MCC

namespace std
{
/** \brief Convert limb to string. */
std::string to_string(const MCC::Limb & limb);
} // namespace std
