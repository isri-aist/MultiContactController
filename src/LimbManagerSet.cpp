#include <cmath>
#include <limits>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Label.h>

#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/swing/SwingTrajCubicSplineSimple.h>

using namespace MCC;

void LimbManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
}

LimbManagerSet::LimbManagerSet(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);

  if(mcRtcConfig.has("SwingTraj"))
  {
    SwingTrajCubicSplineSimple::loadDefaultConfig(
        mcRtcConfig("SwingTraj")("CubicSplineSimple", mc_rtc::Configuration{}));
  }

  for(const auto & limbTaskKV : ctl().limbTasks_)
  {
    mc_rtc::Configuration limbManagerConfig;
    if(mcRtcConfig.has("LimbManager"))
    {
      limbManagerConfig.load(mcRtcConfig("LimbManager")("default", mc_rtc::Configuration{})); // deep copy
      limbManagerConfig.load(mcRtcConfig("LimbManager")(limbTaskKV.first.group, mc_rtc::Configuration{}));
      limbManagerConfig.load(mcRtcConfig("LimbManager")(std::to_string(limbTaskKV.first), mc_rtc::Configuration{}));
    }
    this->emplace(limbTaskKV.first, std::make_shared<LimbManager>(ctlPtr, limbTaskKV.first, limbManagerConfig));

    groupLimbsMap_[limbTaskKV.first.group].insert(limbTaskKV.first);
  }
}

void LimbManagerSet::reset(const mc_rtc::Configuration & constraintSetConfig)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->reset(constraintSetConfig(std::to_string(limbManagerKV.first), mc_rtc::Configuration{}));
  }
}

void LimbManagerSet::update()
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->update();
  }
}

void LimbManagerSet::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->stop();
  }
}

std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> LimbManagerSet::contactList(double t) const
{
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList;
  for(const auto & limbManagerKV : *this)
  {
    const auto & contactCommand = limbManagerKV.second->getContactCommand(t);
    if(contactCommand)
    {
      contactList.emplace(limbManagerKV.first, contactCommand->constraint);
    }
  }
  return contactList;
}

bool LimbManagerSet::contactCommandStacked() const
{
  for(const auto & limbManagerKV : *this)
  {
    const auto & contactCommandList = limbManagerKV.second->contactCommandList();
    if(contactCommandList.upper_bound(ctl().t()) != contactCommandList.end())
    {
      return true;
    }
  }
  return false;
}

void LimbManagerSet::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->addToGUI(gui);
  }

  for(const auto & groupLimbsKV : groupLimbsMap_)
  {
    const auto & group = groupLimbsKV.first;
    const auto & limbs = groupLimbsKV.second;

    gui.addElement({ctl().name(), config_.name, "ImpedanceGains", group}, mc_rtc::gui::Label("limbs", [this, limbs]() {
                     std::string s;
                     for(const auto & limb : limbs)
                     {
                       if(!s.empty())
                       {
                         s += " / ";
                       }
                       s += std::to_string(limb);
                     }
                     return s;
                   }));
    for(const auto & impGainKV : this->begin()->second->config_.impGains)
    {
      const auto & impGainType = impGainKV.first;
      gui.addElement({ctl().name(), config_.name, "ImpedanceGains", group, impGainType},
                     mc_rtc::gui::ArrayInput(
                         "Damper", {"cx", "cy", "cz", "fx", "fy", "fz"},
                         [this, limbs, impGainType]() -> const sva::ImpedanceVecd & {
                           return this->at(*limbs.begin())->config_.impGains.at(impGainType).damper().vec();
                         },
                         [this, limbs, impGainType](const Eigen::Vector6d & v) {
                           for(const auto & limb : limbs)
                           {
                             this->at(limb)->config_.impGains.at(impGainType).damper().vec(v);
                             this->at(limb)->requireImpGainUpdate_ = true;
                           }
                         }));
      gui.addElement({ctl().name(), config_.name, "ImpedanceGains", group, impGainType},
                     mc_rtc::gui::ArrayInput(
                         "Spring", {"cx", "cy", "cz", "fx", "fy", "fz"},
                         [this, limbs, impGainType]() -> const sva::ImpedanceVecd & {
                           return this->at(*limbs.begin())->config_.impGains.at(impGainType).spring().vec();
                         },
                         [this, limbs, impGainType](const Eigen::Vector6d & v) {
                           for(const auto & limb : limbs)
                           {
                             this->at(limb)->config_.impGains.at(impGainType).spring().vec(v);
                             this->at(limb)->requireImpGainUpdate_ = true;
                           }
                         }));
      gui.addElement({ctl().name(), config_.name, "ImpedanceGains", group, impGainType},
                     mc_rtc::gui::ArrayInput(
                         "Wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                         [this, limbs, impGainType]() -> const sva::ImpedanceVecd & {
                           return this->at(*limbs.begin())->config_.impGains.at(impGainType).wrench().vec();
                         },
                         [this, limbs, impGainType](const Eigen::Vector6d & v) {
                           for(const auto & limb : limbs)
                           {
                             this->at(limb)->config_.impGains.at(impGainType).wrench().vec(v);
                             this->at(limb)->requireImpGainUpdate_ = true;
                           }
                         }));
    }
  }

  SwingTrajCubicSplineSimple::addConfigToGUI(gui, {ctl().name(), config_.name, "SwingTraj", "CubicSplineSimple"});
}

void LimbManagerSet::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name});

  SwingTrajCubicSplineSimple::removeConfigFromGUI(gui, {ctl().name(), config_.name, "SwingTraj", "CubicSplineSimple"});

  // GUI of each LimbManager is not removed here (removed via stop method)
}

void LimbManagerSet::addToLogger(mc_rtc::Logger & logger)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->addToLogger(logger);
  }

  constexpr bool enableDebugLog = false;
  if(enableDebugLog)
  {
    logger.addLogEntry(config_.name + "_closestContactTimes", this, [this]() {
      std::unordered_set<Limb> limbs;
      for(const auto & limbManagerKV : *this)
      {
        limbs.insert(limbManagerKV.first);
      }
      return getClosestContactTimes(ctl().t(), limbs);
    });
  }
}

void LimbManagerSet::removeFromLogger(mc_rtc::Logger & // logger
)
{
  // Log of each LimbManager is not removed here (removed via stop method)
}

std::array<double, 2> LimbManagerSet::getClosestContactTimes(double t, const std::unordered_set<Limb> & limbs) const
{
  std::array<double, 2> closestContactTimes = {std::numeric_limits<double>::quiet_NaN(),
                                               std::numeric_limits<double>::quiet_NaN()};
  for(const auto & limb : limbs)
  {
    const auto & closestContactTimesLimb = this->at(limb)->getClosestContactTimes(t);
    if(!std::isnan(closestContactTimesLimb[0]))
    {
      if(std::isnan(closestContactTimes[0]) || (closestContactTimesLimb[0] > closestContactTimes[0]))
      {
        closestContactTimes[0] = closestContactTimesLimb[0];
      }
    }
    if(!std::isnan(closestContactTimesLimb[1]))
    {
      if(std::isnan(closestContactTimes[1]) || (closestContactTimesLimb[1] < closestContactTimes[1]))
      {
        closestContactTimes[1] = closestContactTimesLimb[1];
      }
    }
  }
  return closestContactTimes;
}
