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

/** \brief Equal operator of Limb.

    This is required to use Limb as a key in std::unordered_map.
*/
template<>
struct equal_to<MCC::Limb>
{
  bool operator()(const MCC::Limb & lhs, const MCC::Limb & rhs) const;
};

/** \brief Hash operator of Limb.

    This is required to use Limb as a key in std::unordered_map.
*/
template<>
struct hash<MCC::Limb>
{
  size_t operator()(const MCC::Limb & limb) const;
};
} // namespace std
