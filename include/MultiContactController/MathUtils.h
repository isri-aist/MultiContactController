#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace MCC
{
/** \brief Calculate weighted average of poses.
    \param weightPoseList list of weight and pose
 */
sva::PTransformd calcWeightedAveragePose(const std::vector<std::pair<double, sva::PTransformd>> & weightPoseList);
} // namespace MCC
