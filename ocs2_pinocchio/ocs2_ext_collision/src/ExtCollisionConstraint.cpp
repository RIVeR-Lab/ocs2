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

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ext_collision/ExtCollisionConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraint::ExtCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                               ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                               std::shared_ptr<PointsOnRobot> pointsOnRobotPtr)
  : StateConstraint(ConstraintOrder::Linear),
    extCollision_(std::move(extCollisionPinocchioGeometryInterface), pointsOnRobotPtr),
    mappingPtr_(mapping.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraint::ExtCollisionConstraint(const ExtCollisionConstraint& rhs)
  : StateConstraint(rhs), extCollision_(rhs.extCollision_), mappingPtr_(rhs.mappingPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ExtCollisionConstraint::getNumConstraints(scalar_t time) const 
{
  return extCollision_.getNumPointsOnRobot();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const 
{
  std::cout << "[ExtCollisionConstraint::getValue] START" << std::endl;

  auto pinocchioInterface = getPinocchioInterface(preComputation);

  std::cout << "[ExtCollisionConstraint::getValue] END" << std::endl;

  return extCollision_.getValue(pinocchioInterface, *mappingPtr_, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ExtCollisionConstraint::getLinearApproximation(scalar_t time, 
                                                                                 const vector_t& state,
                                                                                 const PreComputation& preComputation) const 
{
  std::cout << "[ExtCollisionConstraint::getLinearApproximation] START" << std::endl;

  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  mappingPtr_->setPinocchioInterface(pinocchioInterface);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = extCollision_.getLinearApproximation(pinocchioInterface);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);

  std::cout << "[ExtCollisionConstraint::getLinearApproximation] END" << std::endl;

  return constraint;
}

}  // namespace ocs2
