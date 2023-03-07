// LAST UPDATE: 2022.03.06
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#pragma once

#include <Eigen/Core>

//#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/RobotModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Provides read/write access to the base position.
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const RobotModelInfo& info);

/**
 * Provides read/write access to the base orientation.
 */
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const RobotModelInfo& info);

/**
 * Provides read/write access to the arm joint angles.
 */
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const RobotModelInfo& info);

/**
 * Provides read access to the arm joint angles.
 */
template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const RobotModelInfo& info);

}  // namespace mobile_manipulator
}  // namespace ocs2

#include "implementation/AccessHelperFunctionsImpl.h"
