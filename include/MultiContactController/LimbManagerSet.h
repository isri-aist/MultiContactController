#pragma once

#include <unordered_set>

#include <MultiContactController/LimbManager.h>

namespace MCC
{
/** \brief Set of LimbManager. */
class LimbManagerSet : public std::unordered_map<Limb, std::shared_ptr<LimbManager>>
{
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "LimbManagerSet";

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration

      mcRtcConfig has the configuration of LimbManagerSet in the root entry and the configuration of each LimbManager
     under the "LimbManager" entry. Each LimbManager configuration is overwritten in the order of default, limb group,
     limb name.

      An example of \p mcRtcConfig is as follows.
      @code
      LimbManager:
        default:
          entryA: commonValueA
          entryB: commonValueB
          entryC: commonValueC
        Hand:
          entryB: groupSpecificValueB
        LeftHand:
          entryC: limbSpacificValueC
      @endcode
  */
  LimbManagerSet(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.
      \param constraintSetConfig mc_rtc configuration for contact constraint set

      This method should be called once when controller is reset.

      An example of \p constraintSetConfig is as follows.
      @code
      - limb: LeftFoot
        # configuration for ContactConstraint
        type: Surface
        fricCoeff: 0.5
      - limb: RightFoot
        type: Surface
        fricCoeff: 0.5
      @endcode
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

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Get contact constraint list at the specified time.
      \param t time
   */
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList(double t) const;

  /** \brief Get whether future contact command is stacked. */
  bool contactCommandStacked() const;

  /** \brief Get whether any limbs are executing swing motion. */
  bool isExecutingLimbSwing() const;

  /** \brief Get limbs of the specified group.
      \param group limb group
   */
  inline const std::unordered_set<Limb> & limbsFromGroup(const std::string & group) const
  {
    return groupLimbsMap_.at(group);
  }

  /** \brief Get the closest contact times to the specified time.
      \param t time
      \param limbs limbs to check contact
      \return array consisting of the closest times before and after the specified time
   */
  std::array<double, 2> getClosestContactTimes(double t, const std::unordered_set<Limb> & limbs) const;

protected:
  /** \brief Const accessor to the controller. */
  inline const MultiContactController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline MultiContactController & ctl()
  {
    return *ctlPtr_;
  }

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  MultiContactController * ctlPtr_ = nullptr;

  //! Map from limb group to limbs
  std::unordered_map<std::string, std::unordered_set<Limb>> groupLimbsMap_;
};
} // namespace MCC
