#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/OrientationTask.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/CentroidalManager.h>
#include <MultiContactController/LimbManagerSet.h>
#include <MultiContactController/MathUtils.h>
#include <MultiContactController/MultiContactController.h>

using namespace MCC;

void CentroidalManager::setAnchorFrame()
{
  std::string anchorName = "KinematicAnchorFrame::" + ctl().robot().name();
  if(ctl().datastore().has(anchorName))
  {
    ctl().datastore().remove(anchorName);
  }
  ctl().datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) { return calcAnchorFrame(robot); });
}

sva::PTransformd CentroidalManager::calcAnchorFrame(const mc_rbdyn::Robot & robot) const
{
  bool isControlRobot = (&(ctl().robot()) == &robot);

  // Set list of weight and limb pose
  std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
  for(const auto & limbManagerKV : *ctl().limbManagerSet_)
  {
    if(config().useOnlyFootForAnchorFrame && limbManagerKV.first.group != Limb::Group::Foot)
    {
      continue;
    }

    double weight = limbManagerKV.second->getContactWeight(ctl().t());
    sva::PTransformd pose;
    if(config().useTargetPoseForControlRobotAnchorFrame && isControlRobot)
    {
      pose = ctl().limbTasks_.at(limbManagerKV.first)->targetPose(); // target pose
    }
    else
    {
      pose = ctl().limbTasks_.at(limbManagerKV.first)->surfacePose(); // control robot pose (i.e., IK result)
    }
    weightPoseList.emplace_back(weight, pose);
  }

  // Calculate weighted average
  if(weightPoseList.size() == 0)
  {
    return sva::PTransformd::Identity();
  }
  return calcWeightedAveragePose(weightPoseList);
}
