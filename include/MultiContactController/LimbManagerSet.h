#pragma once

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

  /** \brief Get contact constraint list at the specified time.
      \param t time
   */
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList(double t);
};
} // namespace MCC
