/******************************************************************************
Copyright (c) 2020, Neset Unver Akmandor. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>

#include <voxblox/interpolator/interpolator.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_ext_collision/ExtCollision.h>
#include <ocs2_ext_collision/PointsOnRobot.h>

namespace ocs2 {

using typename voxblox::EsdfCachingVoxel;
using voxblox::Interpolator;

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
     * @param [in] minimumDistance: The minimum allowed distance between collision pairs.
     */
    ExtCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, 
                          ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                          scalar_t minimumDistance);

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

    std::shared_ptr<const PointsOnRobot> pointsOnRobot;
    std::shared_ptr<Interpolator<EsdfCachingVoxel>> interpolator;
};

}  // namespace ocs2