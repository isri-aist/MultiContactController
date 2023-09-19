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

void PostureManager::Configuration::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
}

void PostureManager::Configuration::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void PostureManager::RefData::reset()
{
  *this = RefData();
}

void PostureManager::RefData::addToLogger(const std::string & baseEntry, mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(baseEntry + "_posture_ref", posture);
}

void PostureManager::RefData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

PostureManager::PostureManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
  : ctlPtr_(ctlPtr)
{
}

void PostureManager::reset()
{
  refData_.reset();

  auto postureTask = ctl().getPostureTask(ctl().robot().name());
  if (postureTask == nullptr) {
    mc_rtc::log::error("[PostureManager] PostureTask does not exist");
    return;
  }
  nominalPostureList_.emplace(ctl().t(), postureTask->posture());
}

void PostureManager::update()
{
  // Set data
  refData_ = calcRefData(ctl().t());
  
  auto postureTask = ctl().getPostureTask(ctl().robot().name());
  postureTask->posture(refData_.posture());

  // TODO: update postureTask->refVel and postureTask->refAcc
}

void PostureManager::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void PostureManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
}

void PostureManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  // gui.removeCategory({ctl().name(), config().name});
}

void PostureManager::addToLogger(mc_rtc::Logger & logger)
{
  config().addToLogger(config().name + "_Config", logger);
  refData_.addToLogger(config().name + "_Data", logger);

  logger.addLogEntry(config().name + "_nominalPosture", this,
                     [this]() { return getNominalPosture(ctl().t()); });
}

void PostureManager::removeFromLogger(mc_rtc::Logger & logger)
{
  config().removeFromLogger(logger);
  refData_.removeFromLogger(logger);

  logger.removeLogEntries(this);
}

std::vector<std::vector<double> > PostureManager::getNominalPosture(double t) const
{
  auto it = nominalPostureList_.upper_bound(t);
  if(it == nominalPostureList_.begin())
  {
    mc_rtc::log::error_and_throw(
        "[PostureManager] Past time is specified in {}. specified time: {}, current time: {}",
        __func__, t, ctl().t());
  }
  it--;
  return it->second;
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

  std::vector<std::vector<double> > refPosture;
  if (!nominalPostureList_.empty()) {
    double lastTime = nominalPostureList_.rbegin()->first;
    if (t < lastTime) {
      mc_rtc::log::error("[PostureManager] Ignore a nominal posture earlier than the last one: {} < {}",
                         t, lastTime);
      return false;
    }
    std::vector<std::vector<double> > lastPosture = nominalPostureList_.rbegin()->second;
    std::copy(lastPosture.begin(), lastPosture.rbegin()->second.end(), std::back_inserter(refPosture));
  } else {
    std::vector<std::vector<double> > postureInTask = ctl().getPostureTask(ctl().robot().name())->posture();
    std::copy(postureInTask.begin(), postureInTask.end(), std::back_inserter(refPosture));
  }

  for (int i = 0; i < jointNames.size(); i++) {
    std::string jn = jointNames[i];
    int jIndex = ctl().robot().mb().jointIndexByName(jn);
    if (jIndex < 0) {
      mc_rtc::log::error("[PostureManager] Ignore joint {} which does not exist", jn);
      continue;
    }
    if (jIndex < refPosture.size()) {
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
  refData.nominalPosture = nominalPosture;

  return refData;
}
