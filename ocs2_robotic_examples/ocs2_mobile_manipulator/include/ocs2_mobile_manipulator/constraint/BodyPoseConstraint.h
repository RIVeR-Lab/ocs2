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

#pragma once

#include <memory>

//#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
//#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {
namespace mobile_manipulator {

class BodyPoseConstraint final : public StateConstraint 
{
  public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;

    BodyPoseConstraint(size_t modalMode, const ReferenceManager& referenceManager);
    
    ~BodyPoseConstraint() override = default;
    
    BodyPoseConstraint* clone() const override 
    { 
      return new BodyPoseConstraint(modalMode_, *referenceManagerPtr_); 
    }

    size_t getNumConstraints(scalar_t time) const override;
    
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
    
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, 
                                                             const vector_t& state,
                                                             const PreComputation& preComputation) const override;

  private:
    BodyPoseConstraint(const BodyPoseConstraint& other) = default;
    
    std::pair<vector_t, quaternion_t> interpolateTargetBodyPose(scalar_t time) const;

    size_t modalMode_;
    size_t numPosConst_;
    size_t numOriConst_;
    size_t numConst_;

    vector3_t targetBodyPosition_;
    quaternion_t targetBodyOrientation_;
    
    const ReferenceManager* referenceManagerPtr_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
