#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MultiContactController.h>

using namespace MCC;

LimbManagerSet::LimbManagerSet(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
{
  for(const auto & limbManagerConfig : mcRtcConfig)
  {
    const auto & limb = Limb(limbManagerConfig("limb"));
    if(ctlPtr->limbTasks_.count(limb) == 0)
    {
      mc_rtc::log::error_and_throw("[LimbManagerSet] LimbTask of {} is missing.", std::to_string(limb));
    }
    this->emplace(limb, std::make_shared<LimbManager>(ctlPtr, limb, limbManagerConfig));
  }
}

void LimbManagerSet::reset(const mc_rtc::Configuration & constraintSetConfig)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->reset(constraintSetConfig(std::to_string(limbManagerKV.first)));
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
