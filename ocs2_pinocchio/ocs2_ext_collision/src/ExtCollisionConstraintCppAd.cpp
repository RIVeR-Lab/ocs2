// LAST UPDATE: 2022.03.04
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ext_collision/ExtCollisionConstraintCppAd.h>

namespace {

void defaultUpdatePinocchioInterface(const ocs2::vector_t&, ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>&){};

}  // unnamed namespace

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraintCppAd::ExtCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                         const PinocchioStateInputMapping<scalar_t>& mapping,
                                                         const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                         //ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                                                         //size_t modalMode,
                                                         std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                                         ocs2::scalar_t maxDistance,
                                                         std::shared_ptr<ExtMapUtility> emuPtr,
                                                         const std::string& modelName, 
                                                         const std::string& modelFolder,
                                                         bool recompileLibraries, 
                                                         bool verbose)
  : ExtCollisionConstraintCppAd(std::move(pinocchioInterface), 
                                mapping,
                                mappingCppAd, 
                                //std::move(extCollisionPinocchioGeometryInterface), 
                                //modalMode,
                                pointsOnRobotPtr,
                                maxDistance,
                                emuPtr,
                                defaultUpdatePinocchioInterface, 
                                modelName, 
                                modelFolder, 
                                recompileLibraries, 
                                verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraintCppAd::ExtCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                         const PinocchioStateInputMapping<scalar_t>& mapping,
                                                         const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                         //ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                                         //size_t modalMode,
                                                         std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                                         ocs2::scalar_t maxDistance,
                                                         std::shared_ptr<ExtMapUtility> emuPtr,
                                                         update_pinocchio_interface_callback updateCallback, 
                                                         const std::string& modelName,
                                                         const std::string& modelFolder, 
                                                         bool recompileLibraries, 
                                                         bool verbose)
  : StateConstraint(ConstraintOrder::Linear),
    pinocchioInterface_(std::move(pinocchioInterface)),
    extCollisionCppAd_(pinocchioInterface_, 
                       mappingCppAd,
                       //std::move(extCollisionPinocchioGeometryInterface), 
                       //modalMode,
                       pointsOnRobotPtr,
                       maxDistance,
                       emuPtr,
                       modelName, 
                       modelFolder,
                       recompileLibraries, 
                       verbose),
    mappingPtr_(mapping.clone()),
    updateCallback_(std::move(updateCallback)) 
{
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionConstraintCppAd::ExtCollisionConstraintCppAd(const ExtCollisionConstraintCppAd& rhs)
    : StateConstraint(rhs),
      pinocchioInterface_(rhs.pinocchioInterface_),
      extCollisionCppAd_(rhs.extCollisionCppAd_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      updateCallback_(rhs.updateCallback_) {
      mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ExtCollisionConstraintCppAd::getNumConstraints(scalar_t time) const 
{
  return extCollisionCppAd_.getNumPointsOnRobot();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionConstraintCppAd::getValue(scalar_t time, const vector_t& state, const PreComputation&) const 
{
  std::cout << "[ExtCollisionConstraintCppAd::getValue(3)] START" << std::endl;

  std::cout << "[ExtCollisionConstraintCppAd::getValue(3)] DEBUG INF" << std::endl;
  while(1);

  const auto q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);

  std::cout << "[ExtCollisionConstraintCppAd::getValue(3)] END" << std::endl;

  return extCollisionCppAd_.getValue(pinocchioInterface_, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionConstraintCppAd::getValue(scalar_t time, 
                                               const vector_t& state, 
                                               const vector_t& fullState, 
                                               const PreComputation&) const 
{
  //std::cout << "[ExtCollisionConstraintCppAd::getValue(4)] START" << std::endl;

  const auto q = mappingPtr_->getPinocchioJointPosition(state, fullState);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);

  //std::cout << "[ExtCollisionConstraintCppAd::getValue(4)] END" << std::endl;

  return extCollisionCppAd_.getValue(pinocchioInterface_, state, fullState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ExtCollisionConstraintCppAd::getLinearApproximation(scalar_t time, 
                                                                                      const vector_t& state,
                                                                                      const PreComputation&) const 
{
  std::cout << "[ExtCollisionConstraintCppAd::getLinearApproximation(3)] START" << std::endl;

  std::cout << "[ExtCollisionConstraintCppAd::getLinearApproximation(3)] DEBUG INF" << std::endl;
  while(1);

  const auto q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  updateCallback_(state, pinocchioInterface_);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = extCollisionCppAd_.getLinearApproximation(pinocchioInterface_, q);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
  
  std::cout << "[ExtCollisionConstraintCppAd::getLinearApproximation(3)] END" << std::endl;

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ExtCollisionConstraintCppAd::getLinearApproximation(scalar_t time, 
                                                                                      const vector_t& state,
                                                                                      const vector_t& fullState,
                                                                                      const PreComputation&) const 
{
  //std::cout << "[ExtCollisionConstraintCppAd::getLinearApproximation(4)] START" << std::endl;

  const auto q = mappingPtr_->getPinocchioJointPosition(state, fullState);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  updateCallback_(state, pinocchioInterface_);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = extCollisionCppAd_.getLinearApproximation(pinocchioInterface_, state, fullState);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
  
  //std::cout << "[ExtCollisionConstraintCppAd::getLinearApproximation(4)] END" << std::endl;

  return constraint;
}

}  // namespace ocs2
