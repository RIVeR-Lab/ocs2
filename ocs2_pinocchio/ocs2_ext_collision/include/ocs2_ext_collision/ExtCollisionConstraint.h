// LAST UPDATE: 2022.12.21
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

//#include <voxblox/interpolator/interpolator.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_ext_collision/ExtCollision.h>
#include <ocs2_ext_collision/PointsOnRobot.h>

namespace ocs2 {

//using typename voxblox::EsdfCachingVoxel;
//using voxblox::Interpolator;

/**
 *  This class provides a variant of the Extrenal-collision constraints, which allows for caching. Therefore It is the user's
 *  responsibility to call the required updates on the PinocchioInterface in pre-computation requests.
 */
class ExtCollisionConstraint : public StateConstraint 
{
  public:
    /**
     * Constructor
     *
     * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
     * @param [in] extCollisionPinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
     */
    ExtCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, 
                           ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                           std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                           ocs2::scalar_t maxDistance,
                           std::shared_ptr<ExtMapUtility> emuPtr,
                           size_t modalMode,
                           size_t stateDim);

    ~ExtCollisionConstraint() override = default;

    size_t getNumConstraints(scalar_t time) const final;

    /** Get the Extrenal collision distance values
     *
     * @note Requires pinocchio::forwardKinematics().
     */
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const final;

    /** Get the Extrenal collision distance approximation
     *
     * @note Requires pinocchio::forwardKinematics(),
     *                pinocchio::updateGlobalPlacements(),
     *                pinocchio::computeJointJacobians().
     * @note In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
     * you should also call tham as well.
     */
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, 
                                                             const vector_t& state,
                                                             const PreComputation& preComputation) const final;

  protected:
    /** Get the pinocchio interface updated with the requested computation. */
    virtual const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const = 0;

    ExtCollisionConstraint(const ExtCollisionConstraint& rhs);

    ExtCollision extCollision_;
    
    std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;

    //std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator_;
};

}  // namespace ocs2
