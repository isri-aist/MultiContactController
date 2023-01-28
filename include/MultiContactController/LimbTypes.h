#pragma once

#include <string>

namespace MCC
{
/** \brief Limb. */
struct Limb
{
  /** \brief Limb group.

      This is defined only for namespace.
  */
  struct Group
  {
    //! Hand group
    static inline std::string Hand = "Hand";

    //! Foot group
    static inline std::string Foot = "Foot";
  };

  /** \brief Constructor.
      \param _name limb name
      \param _group limb group

      limb group is automatically set if _group is empty and _name contains "Hand" or "Foot" (case-sensitive).
  */
  Limb(const std::string & _name, const std::string & _group = "");

  //! Limb name
  std::string name;

  //! Limb group
  std::string group;
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
