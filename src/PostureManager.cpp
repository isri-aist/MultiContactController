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

void PostureManager::RefData::reset()
{
  *this = RefData();
}

void PostureManager::RefData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  std::vector<double> flatten;
  for (auto p : posture) {
    flatten.insert(flatten.end(), p.begin(), p.end());
  }
  logger.addLogEntry(baseEntry + "_nominalPosture", this,
                     [flatten]() { return flatten; });
}

void PostureManager::RefData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
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
  refData_.reset();
  nominalPostureList_.emplace(ctl().t(), postureTask_->posture());
}

void PostureManager::update()
{
  // Set data
  refData_ = calcRefData(ctl().t());
  postureTask_->posture(refData_.posture);

  // TODO: update postureTask_->refVel and postureTask_->refAcc
}

void PostureManager::stop()
{
  removeFromLogger(ctl().logger());
}

void PostureManager::addToLogger(mc_rtc::Logger & logger)
{
  refData_.addToLogger(config().name + "_Data", logger);
}

void PostureManager::removeFromLogger(mc_rtc::Logger & logger)
{
  refData_.removeFromLogger(logger);
  logger.removeLogEntries(this);
}

std::vector<std::vector<double> > PostureManager::getNominalPosture(double t) const
{
  if (nominalPostureList_.empty()) {
    // if nominalPostureList_ is not defined, return current posture task
    return postureTask_->posture();
  } else {
    auto it = nominalPostureList_.upper_bound(t);
    if(it == nominalPostureList_.begin()) {
      mc_rtc::log::error_and_throw(
        "[PostureManager] Past time is specified in {}. specified time: {}, current time: {}",
        __func__, t, ctl().t());
    }
    it--;
    return it->second;
  }
}

bool PostureManager::appendNominalPosture(double t, const std::vector<std::string> &jointNames,
                                          const std::vector<std::vector<double>> &posture)
{
  if (t < ctl().t()) {
    mc_rtc::log::error("[PostureManager] Ignore a nominal posture with past time: {} < {}", t, ctl().t());
    return false;
  }

  if (jointNames.size() != posture.size()) {
      mc_rtc::log::error("[PostureManager] size of jointNames({}) and posture({}) are different",
                         jointNames.size(), posture.size());
      return false;
  }

  std::vector<std::vector<double> > refPosture, lastPosture;
  if (!nominalPostureList_.empty()) {
    double lastTime = nominalPostureList_.rbegin()->first;
    if (t < lastTime) {
      mc_rtc::log::error("[PostureManager] Ignore a nominal posture earlier than the last one: {} < {}",
                         t, lastTime);
      return false;
    }
    lastPosture = nominalPostureList_.rbegin()->second;
  } else {
    lastPosture = postureTask_->posture();
  }
  std::copy(lastPosture.begin(), lastPosture.end(), std::back_inserter(refPosture));

  for (unsigned int i = 0; i < jointNames.size(); i++) {
    std::string jn = jointNames[i];
    int jIndex = ctl().robot().mb().jointIndexByName(jn);
    if (jIndex < 0) {
      mc_rtc::log::error("[PostureManager] Ignore joint {} which does not exist", jn);
      continue;
    }
    if (static_cast<size_t>(jIndex) < refPosture.size()) {
      refPosture[jIndex] = posture[i];
    } else {
      mc_rtc::log::error("[PostureManager] Ignore joint index {} for {} which exceeds size of posture ({})",
                         jIndex, jn, refPosture.size());
      continue;
    }
  }

  nominalPostureList_.emplace(t, refPosture);
  return true;
}

PostureManager::RefData PostureManager::calcRefData(double t) const
{
  RefData refData;

  // TODO: should be interpolated?
  std::vector<std::vector<double> > nominalPosture = getNominalPosture(t);
  refData.posture = nominalPosture;

  return refData;
}
