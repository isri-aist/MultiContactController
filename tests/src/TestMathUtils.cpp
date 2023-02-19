/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <MultiContactController/MathUtils.h>

TEST(TestMathUtils, calcWeightedAveragePose)
{
  for(int poseNum = 1; poseNum <= 100; poseNum++)
  {
    for(int i = 0; i < 100; i++)
    {
      double totalWeight = 0;
      Eigen::Vector3d averagePos = Eigen::Vector3d::Zero();
      std::vector<std::pair<double, sva::PTransformd>> weightPoseList;
      for(int poseIdx = 0; poseIdx < poseNum; poseIdx++)
      {
        double weight = 10.0 * std::abs(Eigen::Matrix<double, 1, 1>::Random()[0]) + 1e-10;
        sva::PTransformd pose = sva::PTransformd(Eigen::Quaterniond::UnitRandom(), 100.0 * Eigen::Vector3d::Random());

        totalWeight += weight;
        averagePos += weight * pose.translation();
        weightPoseList.emplace_back(weight, pose);
      }
      averagePos /= totalWeight;
      sva::PTransformd averagePose = MCC::calcWeightedAveragePose(weightPoseList);

      EXPECT_FALSE(averagePose.translation().array().isNaN().any() || averagePose.translation().array().isInf().any());
      EXPECT_FALSE(averagePose.rotation().array().isNaN().any() || averagePose.rotation().array().isInf().any());
      EXPECT_LT((averagePose.translation() - averagePos).norm(), 1e-10);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
