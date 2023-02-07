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

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ext_collision/ExtCollisionConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraint::ExtCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                               ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                               std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                               ocs2::scalar_t maxDistance,
                                               std::shared_ptr<ExtMapUtility> emuPtr)
  : StateConstraint(ConstraintOrder::Linear),
    extCollision_(std::move(extCollisionPinocchioGeometryInterface), pointsOnRobotPtr, maxDistance, emuPtr),
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
  //std::cout << "[ExtCollisionConstraint::getValue] START" << std::endl;

  auto pinocchioInterface = getPinocchioInterface(preComputation);

  //std::cout << "[ExtCollisionConstraint::getValue] END" << std::endl;

  return extCollision_.getValue(pinocchioInterface, *mappingPtr_, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ExtCollisionConstraint::getLinearApproximation(scalar_t time, 
                                                                                 const vector_t& state,
                                                                                 const PreComputation& preComputation) const 
{
  //std::cout << "[ExtCollisionConstraint::getLinearApproximation] START" << std::endl;

  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  mappingPtr_->setPinocchioInterface(pinocchioInterface);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = extCollision_.getLinearApproximation(pinocchioInterface);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);

  //std::cout << "[ExtCollisionConstraint::getLinearApproximation] END" << std::endl;

  return constraint;
}

}  // namespace ocs2
