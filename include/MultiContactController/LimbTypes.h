#pragma once

#include <string>

namespace MCC
{
/** \brief Limb.

    In this controller, "limb" refers to a contact patch of robot, such as hands, feet, and knees.
 */
struct Limb
{
  /** \brief Limb group.

      This is defined only for namespace.
  */
  struct Group
  {
    //! Hand group
    static inline const std::string Hand = "Hand";

    //! Foot group
    static inline const std::string Foot = "Foot";
  };

  /** \brief Constructor.
      \param _name limb name
      \param _group limb group

      Limb group is automatically set if _group is empty and _name contains "Hand" or "Foot" (case-sensitive). Different
      limbs must have different names; limbs with the same name but different groups are not allowed.
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
