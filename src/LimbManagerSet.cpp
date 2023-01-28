#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>

using namespace MCC;

LimbManagerSet::LimbManagerSet(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
{
  for(const auto & limbTaskKV : ctlPtr->limbTasks_)
  {
    this->emplace(limbTaskKV.first, std::make_shared<LimbManager>(
                                        ctlPtr, limbTaskKV.first,
                                        mcRtcConfig(std::to_string(limbTaskKV.first), mc_rtc::Configuration{})));

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
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->stop();
  }
}

std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> LimbManagerSet::contactList(double t)
{
  std::unordered_map<Limb, std::shared_ptr<ContactConstraint>> contactList;
  for(const auto & limbManagerKV : *this)
  {
    const auto & contactState = limbManagerKV.second->getContactState(t);
    if(contactState)
    {
      contactList.emplace(limbManagerKV.first, contactState->constraint);
    }
  }
  return contactList;
}

void addToGUI(mc_rtc::gui::StateBuilder & gui) {}

void LimbManagerSet::addToLogger(mc_rtc::Logger & logger)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->addToLogger(logger);
  }
}
