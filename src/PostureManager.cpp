#include <cmath>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/PostureTask.h>

#include <MultiContactController/PostureManager.h>
#include <MultiContactController/MultiContactController.h>

using namespace MCC;

void PostureManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
}

PostureManager::PostureManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
  : ctlPtr_(ctlPtr)
{
  postureTask_ = ctl().getPostureTask(ctl().robot().name());
  if (postureTask_ == nullptr) {
    mc_rtc::log::error_and_throw("[PostureManager] PostureTask does not exist");
  }
  config_.load(mcRtcConfig);
}

void PostureManager::reset()
{
  nominalPostureList_.clear();
}

void PostureManager::update()
{
  // Set data
  PostureManager::PostureMap ref = getNominalPosture(ctl().t());
  postureTask_->target(ref); // this function will do nothing if ref is empty

  // TODO: update postureTask_->refVel and postureTask_->refAcc
}

void PostureManager::stop()
{
  removeFromLogger(ctl().logger());
}

void PostureManager::addToLogger(mc_rtc::Logger & logger)
{
  std::vector<double> flatten;
  for (auto p : postureTask_->posture()) {
    flatten.insert(flatten.end(), p.begin(), p.end());
  }
  logger.addLogEntry(config().name + "_nominalPosture", this,
                     [flatten]() { return flatten; });
}

void PostureManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

PostureManager::PostureMap PostureManager::getNominalPosture(double t) const
{
  // if nominalPostureList_ is empty, return empty map
  PostureManager::PostureMap postures;
  if (!nominalPostureList_.empty()) {
    auto it = nominalPostureList_.upper_bound(t);
    if(it == nominalPostureList_.begin()) {
      mc_rtc::log::error_and_throw(
        "[PostureManager] Past time is specified in {}. specified time: {}, current time: {}",
        __func__, t, ctl().t());
    }
    it--;
    postures = it->second;
  }
  return postures;
}

bool PostureManager::appendNominalPosture(double t, const PostureManager::PostureMap &postures)
{
  if (t < ctl().t()) {
    mc_rtc::log::error("[PostureManager] Ignore a nominal posture with past time: {} < {}", t, ctl().t());
    return false;
  }
  nominalPostureList_.emplace(t, postures);
  return true;
}
