#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <TrajColl/CubicSpline.h>

#include <MultiContactController/swing/SwingTrajCubicSplineSimple.h>

using namespace MCC;

void SwingTrajCubicSplineSimple::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("withdrawOffset", withdrawOffset);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("approachOffset", approachOffset);
  mcRtcConfig("swingOffset", swingOffset);
}

void SwingTrajCubicSplineSimple::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajCubicSplineSimple::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
                                                const std::vector<std::string> & category)
{
  gui.addElement(
      category,
      mc_rtc::gui::NumberInput(
          "withdrawDurationRatio", []() { return defaultConfig_.withdrawDurationRatio; },
          [](double v) { defaultConfig_.withdrawDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "withdrawOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.withdrawOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.withdrawOffset = v; }),
      mc_rtc::gui::NumberInput(
          "approachDurationRatio", []() { return defaultConfig_.approachDurationRatio; },
          [](double v) { defaultConfig_.approachDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "approachOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.approachOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.approachOffset = v; }),
      mc_rtc::gui::ArrayInput(
          "swingOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.swingOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.swingOffset = v; }));
}

void SwingTrajCubicSplineSimple::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                     const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajCubicSplineSimple::SwingTrajCubicSplineSimple(const ContactCommand::Type & commandType,
                                                       bool isContact,
                                                       const sva::PTransformd & startPose,
                                                       const sva::PTransformd & endPose,
                                                       double startTime,
                                                       double endTime,
                                                       const TaskGain & taskGain,
                                                       const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(commandType, isContact, startPose, endPose, startTime, endTime, taskGain, mcRtcConfig),
  posFunc_(std::make_shared<TrajColl::PiecewiseFunc<Eigen::Vector3d>>()),
  rotFunc_(std::make_shared<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>())
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (endTime_ - startTime_);
  double approachDuration = config_.approachDurationRatio * (endTime_ - startTime_);

  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroVelBC(TrajColl::BoundaryConstraintType::Velocity,
                                                          Eigen::Vector3d::Zero());
  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroAccelBC(TrajColl::BoundaryConstraintType::Acceleration,
                                                            Eigen::Vector3d::Zero());

  if(commandType_ == ContactCommand::Type::Add)
  {
    // Spline to withdraw limb
    // Pos
    std::map<double, Eigen::Vector3d> withdrawPosWaypoints;
    std::shared_ptr<TrajColl::CubicSpline<Eigen::Vector3d>> withdrawPosSpline;
    if(isContact_)
    {
      withdrawPosWaypoints = {
          {startTime_, startPose_.translation()},
          {startTime_ + withdrawDuration, (sva::PTransformd(config_.withdrawOffset) * startPose_).translation()}};
      withdrawPosSpline =
          std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroVelBC, zeroAccelBC, withdrawPosWaypoints);
      withdrawPosSpline->calcCoeff();
      posFunc_->appendFunc(startTime_ + withdrawDuration, withdrawPosSpline);
    }
    // Rot
    rotFunc_->appendPoint(std::make_pair(startTime_, startPose_.rotation().transpose()));
    if(isContact_)
    {
      rotFunc_->appendPoint(std::make_pair(startTime_ + withdrawDuration, startPose_.rotation().transpose()));
    }

    // Spline to approach limb
    // Pos
    std::map<double, Eigen::Vector3d> approachPosWaypoints = {
        {endTime_ - approachDuration, (sva::PTransformd(config_.approachOffset) * endPose_).translation()},
        {endTime_, endPose_.translation()}};
    auto approachPosSpline =
        std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroAccelBC, zeroVelBC, approachPosWaypoints);
    approachPosSpline->calcCoeff();
    posFunc_->appendFunc(endTime_, approachPosSpline);
    // Rot
    rotFunc_->appendPoint(std::make_pair(endTime_ - approachDuration, endPose_.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(endTime_, endPose_.rotation().transpose()));

    // Spline to swing limb
    // Pos
    std::shared_ptr<TrajColl::CubicSpline<Eigen::Vector3d>> swingPosSpline;
    if(isContact_)
    {
      std::map<double, Eigen::Vector3d> swingPosWaypoints = {
          *withdrawPosWaypoints.rbegin(),
          {0.5 * (startTime_ + endTime_),
           (sva::PTransformd(config_.swingOffset) * sva::interpolate(startPose_, endPose_, 0.5)).translation()},
          *approachPosWaypoints.begin()};
      swingPosSpline = std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(
          3,
          TrajColl::BoundaryConstraint<Eigen::Vector3d>(
              TrajColl::BoundaryConstraintType::Velocity,
              withdrawPosSpline->derivative(startTime_ + withdrawDuration, 1)),
          TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity,
                                                        approachPosSpline->derivative(endTime_ - approachDuration, 1)),
          swingPosWaypoints);
    }
    else
    {
      std::map<double, Eigen::Vector3d> swingPosWaypoints = {{startTime_, startPose_.translation()},
                                                             *approachPosWaypoints.begin()};
      swingPosSpline = std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(
          3, zeroVelBC,
          TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraint<Eigen::Vector3d>(
              TrajColl::BoundaryConstraintType::Velocity,
              approachPosSpline->derivative(endTime_ - approachDuration, 1))),
          swingPosWaypoints);
    }
    swingPosSpline->calcCoeff();
    posFunc_->appendFunc(endTime_ - approachDuration, swingPosSpline);
    // Rot
    rotFunc_->calcCoeff();
  }
  else // if(commandType_ == ContactCommand::Type::Remove)
  {
    // Spline to withdraw limb
    // Pos
    std::map<double, Eigen::Vector3d> withdrawPosWaypoints = {
        {startTime_, startPose_.translation()},
        {startTime_ + withdrawDuration, (sva::PTransformd(config_.withdrawOffset) * startPose_).translation()}};
    std::shared_ptr<TrajColl::CubicSpline<Eigen::Vector3d>> withdrawPosSpline =
        std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroVelBC, zeroVelBC, withdrawPosWaypoints);
    withdrawPosSpline->calcCoeff();
    posFunc_->appendFunc(startTime_ + withdrawDuration, withdrawPosSpline);
    // Rot
    rotFunc_->appendPoint(std::make_pair(startTime_, startPose_.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(endTime_, startPose_.rotation().transpose()));
    rotFunc_->calcCoeff();

    // Constant to stay limb
    // Pos
    posFunc_->appendFunc(endTime_,
                         std::make_shared<TrajColl::Constant<Eigen::Vector3d>>(withdrawPosWaypoints.rbegin()->second));

    // Stiffness interpolation
    stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
        std::map<double, double>{{startTime_, 1.0},
                                 {startTime_ + withdrawDuration, 1.0},
                                 {((startTime_ + withdrawDuration) + endTime_) / 2, 0.0},
                                 {endTime_, 0.0}});
  }
}

sva::PTransformd SwingTrajCubicSplineSimple::pose(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t = touchDownTime_;
  }
  return sva::PTransformd((*rotFunc_)(t).transpose(), (*posFunc_)(t));
}

sva::MotionVecd SwingTrajCubicSplineSimple::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(rotFunc_->derivative(t, 1), posFunc_->derivative(t, 1));
  }
}

sva::MotionVecd SwingTrajCubicSplineSimple::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(rotFunc_->derivative(t, 2), posFunc_->derivative(t, 2));
  }
}

TaskGain SwingTrajCubicSplineSimple::taskGain(double t) const
{
  if(commandType_ == ContactCommand::Type::Add)
  {
    return taskGain_;
  }
  else // if(commandType_ == ContactCommand::Type::Remove)
  {
    return TaskGain((*stiffnessRatioFunc_)(t)*taskGain_.stiffness);
  }
}
