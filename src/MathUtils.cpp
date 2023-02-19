#include <mc_rtc/logging.h>

#include <MultiContactController/MathUtils.h>

using namespace MCC;

sva::PTransformd MCC::calcWeightedAveragePose(const std::vector<std::pair<double, sva::PTransformd>> & weightPoseList)
{
  if(weightPoseList.size() == 0)
  {
    mc_rtc::log::error_and_throw("[calcWeightedAveragePose] weightPoseList is empty.");
  }

  for(const auto & weightPoseKV : weightPoseList)
  {
    if(weightPoseKV.first <= 0.0)
    {
      mc_rtc::log::error_and_throw("[calcWeightedAveragePose] Weight must be positive but is {}.", weightPoseKV.first);
    }
  }

  double weight = weightPoseList[0].first;
  sva::PTransformd pose = weightPoseList[0].second;
  for(size_t i = 1; i < weightPoseList.size(); i++)
  {
    weight += weightPoseList[i].first;
    pose = sva::interpolate(pose, weightPoseList[i].second, weightPoseList[i].first / weight);
  }
  return pose;
}
