#pragma once

#include <unordered_set>

#include <MultiContactController/LimbManager.h>

namespace MCC
{
/** \brief Set of LimbManager. */
class LimbManagerSet : public std::unordered_map<Limb, std::shared_ptr<LimbManager>>
{
public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  LimbManagerSet(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.
      \param constraintSetConfig mc_rtc configuration for contact constraint set

      This method should be called once when controller is reset.
  */
  void reset(const mc_rtc::Configuration & constraintSetConfig);

  /** \brief Update.

      This method should be called once every control cycle.
  */
  void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Get contact constraint list at the specified time.
      \param t time
   */
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList(double t);

  /** \brief Get limbs of the specified group.
      \param group limb group
   */
  inline const std::unordered_set<Limb> & limbsFromGroup(const std::string & group) const
  {
    return groupLimbsMap_.at(group);
  }

protected:
  //! Map from limb group to limb
  std::unordered_map<std::string, std::unordered_set<Limb>> groupLimbsMap_;
};
} // namespace MCC
