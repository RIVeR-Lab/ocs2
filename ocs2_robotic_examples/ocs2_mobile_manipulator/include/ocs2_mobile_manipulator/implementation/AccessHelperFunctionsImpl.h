// LAST UPDATE: 2022.04.10
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const RobotModelInfo& info) 
{
  //std::cout << "[AccessHelperFunctionsImpl::getBasePosition] START" << std::endl;

  //assert(state.rows() == stateDim);
  //assert(state.cols() == 1);

  // Resolve the position vector based on robot type.
  switch (info.robotModelType) 
  {
    case RobotModelType::MobileBase: 
    {
      // For wheel-based, we assume 2D base position
      return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
    }

    case RobotModelType::RobotArm: 
    {
      // For arm, we assume robot is at identity pose
      return Eigen::Matrix<SCALAR, 3, 1>::Zero();
    }

    case RobotModelType::MobileManipulator: 
    {
      // For wheel-based, we assume 2D base position
      return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
    }

    default:
      throw std::invalid_argument("[AccessHelperFunctionsImpl::getBasePosition] ERROR: Invalid manipulator model type!");
  }

  //std::cout << "[AccessHelperFunctionsImpl::getBasePosition] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const RobotModelInfo& info) 
{
  //std::cout << "[AccessHelperFunctionsImpl::getBaseOrientation] START" << std::endl;

  //assert(state.rows() == stateDim);
  //assert(state.cols() == 1);
  
  // Resolve the orientation vector based on robot type.
  switch (info.robotModelType) 
  {
    case RobotModelType::MobileBase: 
    {
      // For wheel-based, we assume only yaw
      //std::cout << "[AccessHelperFunctionsImpl::getBaseOrientation] END MobileBase" << std::endl;
      return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
    }

    case RobotModelType::RobotArm: 
    {
      // For arm, we assume robot is at identity pose
      //std::cout << "[AccessHelperFunctionsImpl::getBaseOrientation] END RobotArm" << std::endl;
      return Eigen::Quaternion<SCALAR>::Identity();
    }

    case RobotModelType::MobileManipulator: 
    {
      // For wheel-based, we assume only yaw
      //std::cout << "[AccessHelperFunctionsImpl::getBaseOrientation] END MobileManipulator" << std::endl;
      return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
    }

    default:
      throw std::invalid_argument("[AccessHelperFunctionsImpl::getBaseOrientation] ERROR: Invalid manipulator model type!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const RobotModelInfo& info) 
{
  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] START" << std::endl;

  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] DEBUG INF" << std::endl;
  while(1);

  auto infoTmp = info;

  //auto stateDimBase = info.mobileBase.stateDim;
  auto stateDimBase = getStateDimBase(infoTmp);
  //auto stateDimArm = info.robotArm.stateDim;
  auto stateDimArm = getStateDimArm(infoTmp);
  //auto stateDim = stateDimBase + stateDimArm;

  //assert(state.rows() == stateDim);
  //assert(state.cols() == 1);
  
  const size_t startRow = stateDimBase;

  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] END" << std::endl;

  return Eigen::Block<Derived, -1, 1>(state.derived(), startRow, 0, stateDimArm, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const RobotModelInfo& info) 
{
  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] START" << std::endl;

  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] DEBUG INF" << std::endl;
  while(1);

  auto infoTmp = info;
  auto stateDimBase = getStateDimBase(infoTmp);
  auto stateDimArm = getStateDimArm(infoTmp);

  //assert(state.rows() == stateDim);
  //assert(state.cols() == 1);

  // Resolve for arm dof start index
  const size_t startRow = stateDimBase;
  
  std::cout << "[AccessHelperFunctionsImpl::getArmJointAngles] END" << std::endl;

  return Eigen::Block<const Derived, -1, 1>(state.derived(), startRow, 0, stateDimArm, 1);
}

}  // namespace mobile_manipulator
}  // namespace ocs2
