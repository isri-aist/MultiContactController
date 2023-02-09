/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <ForceColl/Contact.h>

#include <MultiContactController/CommandTypes.h>

TEST(TestCommandTypes, setBaseTime)
{
  // Create command
  auto swingCommand =
      std::make_shared<MCC::SwingCommand>(MCC::SwingCommand::Type::Add, 1.0, 3.0, sva::PTransformd::Identity());
  std::map<double, std::shared_ptr<MCC::ContactCommand>> contactCommandList = {
      {2.0, std::make_shared<MCC::ContactCommand>(
                2.0, std::make_shared<ForceColl::SurfaceContact>("Contact1", 1.0,
                                                                 std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()},
                                                                 sva::PTransformd::Identity()))},
      {5.0, std::make_shared<MCC::ContactCommand>(5.0, std::make_shared<ForceColl::GraspContact>(
                                                           "Contact2", 1.0,
                                                           std::vector<sva::PTransformd>{sva::PTransformd::Identity()},
                                                           sva::PTransformd::Identity()))}};
  std::map<double, std::shared_ptr<MCC::GripperCommand>> gripperCommandList = {
      {10.0, std::make_shared<MCC::GripperCommand>(10.0, "Gripper1", mc_rtc::Configuration())}};
  auto stepCommand = MCC::StepCommand(swingCommand, contactCommandList, gripperCommandList);

  // Copy command
  auto swingCommandCopied = std::make_shared<MCC::SwingCommand>(*swingCommand);
  std::map<double, std::shared_ptr<MCC::ContactCommand>> contactCommandListCopied;
  for(const auto & contactCommandKV : contactCommandList)
  {
    contactCommandListCopied.emplace(contactCommandKV.first,
                                     std::make_shared<MCC::ContactCommand>(*contactCommandKV.second));
  }
  std::map<double, std::shared_ptr<MCC::GripperCommand>> gripperCommandListCopied;
  for(const auto & gripperCommandKV : gripperCommandList)
  {
    gripperCommandListCopied.emplace(gripperCommandKV.first,
                                     std::make_shared<MCC::GripperCommand>(*gripperCommandKV.second));
  }

  // Set base time
  double baseTime = 12.3;
  stepCommand.setBaseTime(baseTime);

  // Check swing command
  EXPECT_EQ(stepCommand.swingCommand, swingCommand);
  EXPECT_NE(stepCommand.swingCommand, swingCommandCopied);
  EXPECT_DOUBLE_EQ(stepCommand.swingCommand->startTime, swingCommandCopied->startTime + baseTime);
  EXPECT_DOUBLE_EQ(stepCommand.swingCommand->endTime, swingCommandCopied->endTime + baseTime);

  // Check contact command
  {
    EXPECT_EQ(stepCommand.contactCommandList.size(), contactCommandList.size());
    EXPECT_EQ(stepCommand.contactCommandList.size(), contactCommandListCopied.size());
    auto it = stepCommand.contactCommandList.begin();
    auto itSrc = contactCommandList.begin();
    auto itCopied = contactCommandListCopied.begin();
    while(it != stepCommand.contactCommandList.end())
    {
      EXPECT_EQ(it->second, itSrc->second);
      EXPECT_NE(it->second, itCopied->second);
      EXPECT_DOUBLE_EQ(it->first, itCopied->first + baseTime);
      EXPECT_DOUBLE_EQ(it->second->time, itCopied->second->time + baseTime);
      it++;
      itSrc++;
      itCopied++;
    }
  }

  // Check gripper command
  {
    EXPECT_EQ(stepCommand.gripperCommandList.size(), gripperCommandList.size());
    EXPECT_EQ(stepCommand.gripperCommandList.size(), gripperCommandListCopied.size());
    auto it = stepCommand.gripperCommandList.begin();
    auto itSrc = gripperCommandList.begin();
    auto itCopied = gripperCommandListCopied.begin();
    while(it != stepCommand.gripperCommandList.end())
    {
      EXPECT_EQ(it->second, itSrc->second);
      EXPECT_NE(it->second, itCopied->second);
      EXPECT_DOUBLE_EQ(it->first, itCopied->first + baseTime);
      EXPECT_DOUBLE_EQ(it->second->time, itCopied->second->time + baseTime);
      it++;
      itSrc++;
      itCopied++;
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
