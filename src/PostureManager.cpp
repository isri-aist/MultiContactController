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
  // Default argument is empty map (= no modification for current PostureTask)
  nominalPostureList_.clear();
  nominalPostureList_.emplace(ctl().t(), initialPosture);
}

void PostureManager::update()
{
  // Set data
  PostureMap nominalPosture = getNominalPosture(ctl().t());
  postureTask_->target(nominalPosture); // this function will do nothing if nominalPosture is empty

  // \todo update postureTask_->refVel and postureTask_->refAcc
}

void PostureManager::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void PostureManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({ctl().name(), config().name, "Status"}, mc_rtc::gui::Label("Number of specified joints", [this]() {
                   return getNominalPosture(ctl().t()).size();
                 }));
}

void PostureManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config().name});
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
  // If nominalPostureList_ is empty or specified past time, return empty map
  PostureManager::PostureMap nominalPosture;
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
    nominalPosture = it->second;
  }
  return nominalPosture;
}

bool PostureManager::appendNominalPosture(double t, const PostureManager::PostureMap & nominalPosture)
{
  if(t < ctl().t())
  {
    mc_rtc::log::error("[PostureManager] Ignore a nominal posture with past time: {} < {}", t, ctl().t());
    return false;
  }
  if(!nominalPostureList_.empty())
  {
    double lastTime = nominalPostureList_.rbegin()->first;
    if(t < lastTime)
    {
      mc_rtc::log::error("[PostureManager] Ignore a nominal posture earlier than the last one: {} < {}", t, lastTime);
      return false;
    }
  }
  nominalPostureList_.emplace(t, nominalPosture);
  return true;
}

bool PostureManager::isFinished(const double t) const
{
  if(nominalPostureList_.empty())
  {
    return true;
  }
  return t > nominalPostureList_.rbegin()->first;
}
