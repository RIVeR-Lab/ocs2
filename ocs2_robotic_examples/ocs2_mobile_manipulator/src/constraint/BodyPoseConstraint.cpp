// LAST UPDATE: 2022.03.03
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
//

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/BodyPoseConstraint.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BodyPoseConstraint::BodyPoseConstraint(size_t modalMode, const ReferenceManager& referenceManager)
  : StateConstraint(ConstraintOrder::Linear),
    modalMode_(modalMode),
    referenceManagerPtr_(&referenceManager) 
{
  // NUA TODO: ADD 6 DOF BASE VERSION!
  switch (modalMode_)
  {
    case 0:
      numPosConst_ = 2;
      numOriConst_ = 1;
      numConst_ = numPosConst_ + numOriConst_;
      break;
    
    default:
      std::cout << "[BodyPoseConstraint::BodyPoseConstraint] WARNING: Undefined modal mode: " << modalMode_ << std:: endl;
      numPosConst_ = 0;
      numOriConst_ = 0;
      numConst_ = 0;
      break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t BodyPoseConstraint::getNumConstraints(scalar_t time) const 
{
  return numConst_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t BodyPoseConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const 
{
  std::cout << "[BodyPoseConstraint::getValue] WARNING: Undefined modal mode: " << modalMode_ << std:: endl;

  vector_t constraint(numConst_);

  if (modalMode_ == 0)
  {
    const auto targetPositionOrientation = interpolateTargetBodyPose(time);

    for (size_t i = 0; i < numPosConst_; i++)
    {
      constraint[i] = abs(state[i] - targetPositionOrientation.first[i]);
    }

    auto target_euler = targetPositionOrientation.second.toRotationMatrix().eulerAngles(0, 1, 2);
    constraint[numPosConst_] = abs(state[numPosConst_] - target_euler[2]);
  }
  else
  {
    std::cout << "[BodyPoseConstraint::getValue] WARNING: Undefined modal mode: " << modalMode_ << std:: endl;
    constraint.setZero();
  }

  std::cout << "[BodyPoseConstraint::getValue] END" << std:: endl;

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation BodyPoseConstraint::getLinearApproximation(scalar_t time, 
                                                                             const vector_t& state,
                                                                             const PreComputation& preComputation) const 
{
  auto approximation = VectorFunctionLinearApproximation(numConst_, state.rows(), 0);
  const auto targetPositionOrientation = interpolateTargetBodyPose(time);

  for (size_t i = 0; i < numPosConst_; i++)
  {
    approximation.f[i] = abs(state[i] - targetPositionOrientation.first[i]);
  }
  
  auto target_euler = targetPositionOrientation.second.toRotationMatrix().eulerAngles(0, 1, 2);
  
  approximation.f[numPosConst_] = abs(state[numPosConst_] - target_euler[2]);
  approximation.dfdx.setIdentity();

  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto BodyPoseConstraint::interpolateTargetBodyPose(scalar_t time) const -> std::pair<vector_t, quaternion_t> 
{
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) 
  {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index];
    const auto& rhs = stateTrajectory[index + 1];

    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } 
  else 
  {  
    // stateTrajectory.size() == 1
    position = stateTrajectory.front().head<3>();
    orientation = quaternion_t(stateTrajectory.front().tail<4>());
  }

  return {position, orientation};
}

}  // namespace mobile_manipulator
}  // namespace ocs2
