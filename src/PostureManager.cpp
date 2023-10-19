#include <cmath>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/PostureTask.h>

#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/PostureManager.h>

using namespace MCC;

void PostureManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
}

PostureManager::PostureManager(MultiContactController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  postureTask_ = ctl().getPostureTask(ctl().robot().name());
  if(postureTask_ == nullptr)
  {
    mc_rtc::log::error_and_throw("[PostureManager] PostureTask does not exist");
  }
  config_.load(mcRtcConfig);
}

void PostureManager::reset(const PostureManager::PostureMap & initialPosture)
{
  nominalPostureList_.clear();
  nominalPostureList_.emplace(ctl().t(), initialPosture);
}

void PostureManager::reset()
{
  // empty map (= no modification for current PostureTask)
  PostureManager::PostureMap posture;
  reset(posture);
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
  auto q = postureTask_->posture();
  for(unsigned int i = 0; i < q.size(); i++)
  {
    if(q[i].size() == 1)
    { // only consider 1DoF joints (posture includes Root and fixed joints)
      std::string jname = ctl().robot().mb().joint(i).name();
      std::replace(jname.begin(), jname.end(), '_', '-'); // underbar creates a new layer
      logger.addLogEntry(config().name + "_nominalPosture_" + jname, this,
                         [this, i]() { return postureTask_->posture()[i][0]; });
    }
  }
}

void PostureManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

PostureManager::PostureMap PostureManager::getNominalPosture(double t) const
{
  // if nominalPostureList_ is empty or specified past time, return empty map
  PostureManager::PostureMap postures;
  if(!nominalPostureList_.empty())
  {
    auto it = nominalPostureList_.upper_bound(t);
    if(it == nominalPostureList_.begin())
    {
      mc_rtc::log::error_and_throw(
          "[PostureManager] Past time is specified in {}. specified time: {}, current time: {}", __func__, t,
          ctl().t());
    }
    it--;
    postures = it->second;
  }
  return postures;
}

bool PostureManager::appendNominalPosture(double t, const PostureManager::PostureMap & postures)
{
  if(t < ctl().t())
  {
    mc_rtc::log::error("[PostureManager] Ignore a nominal posture with past time: {} < {}", t, ctl().t());
    return false;
  }
  nominalPostureList_.emplace(t, postures);
  return true;
}
