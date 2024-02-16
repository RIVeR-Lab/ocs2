/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

namespace {

void defaultUpdatePinocchioInterface(const ocs2::vector_t&, ocs2::PinocchioInterfaceTpl<ocs2::scalar_t>&){};

}  // unnamed namespace

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                           const PinocchioStateInputMapping<scalar_t>& mapping,
                                                           const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                           PinocchioGeometryInterface pinocchioGeometryInterface, 
                                                           RobotModelInfo robotModelInfo,
                                                           scalar_t minimumDistance,
                                                           const std::string modelName, 
                                                           const std::string modelFolder,
                                                           bool recompileLibraries, 
                                                           bool verbose)
  : SelfCollisionConstraintCppAd(std::move(pinocchioInterface), 
                                 mapping, 
                                 mappingCppAd,
                                 std::move(pinocchioGeometryInterface), 
                                 robotModelInfo,
                                 minimumDistance,
                                 defaultUpdatePinocchioInterface, 
                                 modelName, 
                                 modelFolder, 
                                 recompileLibraries, 
                                 verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                           const PinocchioStateInputMapping<scalar_t>& mapping,
                                                           const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                           PinocchioGeometryInterface pinocchioGeometryInterface, 
                                                           RobotModelInfo& robotModelInfo,
                                                           scalar_t minimumDistance,
                                                           update_pinocchio_interface_callback updateCallback, 
                                                           const std::string& modelName,
                                                           const std::string& modelFolder, 
                                                           bool recompileLibraries, 
                                                           bool verbose)
  : StateConstraint(ConstraintOrder::Linear),
    pinocchioInterface_(std::move(pinocchioInterface)),
    selfCollision_(pinocchioInterface_, 
                   std::move(pinocchioGeometryInterface), 
                   mappingCppAd,
                   robotModelInfo,
                   minimumDistance, 
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
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(const SelfCollisionConstraintCppAd& rhs)
  : StateConstraint(rhs),
    pinocchioInterface_(rhs.pinocchioInterface_),
    selfCollision_(rhs.selfCollision_),
    mappingPtr_(rhs.mappingPtr_->clone()),
    updateCallback_(rhs.updateCallback_) 
{
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t SelfCollisionConstraintCppAd::getNumConstraints(scalar_t time) const 
{
  return selfCollision_.getNumCollisionPairs();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionConstraintCppAd::getValue(scalar_t time, const vector_t& state, const PreComputation&) const 
{
  std::cout << "[SelfCollisionConstraintCppAd::getValue(3)] DEBUG INF" << std::endl;
  while(1);

  const auto q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);

  return selfCollision_.getValue(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionConstraintCppAd::getValue(scalar_t time, 
                                                const vector_t& state, 
                                                const vector_t& fullState, 
                                                const PreComputation&) const 
{
  //std::cout << "[SelfCollisionConstraintCppAd::getValue(4)] START" << std::endl;

  //std::cout << "[SelfCollisionConstraintCppAd::getValue(4)] NumConstraints: " << getNumConstraints(time) << std::endl;

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = mappingPtr_->getPinocchioJointPosition(state, fullState);
  
  pinocchio::forwardKinematics(model, data, q);

  //std::cout << "[SelfCollisionConstraintCppAd::getValue(4)] END" << std::endl;

  return selfCollision_.getValue(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SelfCollisionConstraintCppAd::getLinearApproximation(scalar_t time, 
                                                                                       const vector_t& state,
                                                                                       const PreComputation&) const 
{
  std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] DEBUG INF" << std::endl;
  while(1);

  const auto q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  updateCallback_(state, pinocchioInterface_);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = selfCollision_.getLinearApproximation(pinocchioInterface_, q);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SelfCollisionConstraintCppAd::getLinearApproximation(scalar_t time, 
                                                                                       const vector_t& state,
                                                                                       const vector_t& fullState,
                                                                                       const PreComputation&) const 
{
  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] START" << std::endl;

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE getPinocchioJointPosition" << std::endl;
  const auto q = mappingPtr_->getPinocchioJointPosition(state, fullState);

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE getModel" << std::endl;
  const auto& model = pinocchioInterface_.getModel();

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE getData" << std::endl;
  auto& data = pinocchioInterface_.getData();

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE forwardKinematics" << std::endl;
  pinocchio::forwardKinematics(model, data, q);

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE updateCallback_" << std::endl;
  //updateCallback_(state, pinocchioInterface_);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  //std::tie(constraint.f, dfdq) = selfCollision_.getLinearApproximation(pinocchioInterface_, q);

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE getLinearApproximation" << std::endl;
  std::tie(constraint.f, dfdq) = selfCollision_.getLinearApproximation(pinocchioInterface_, state, fullState);
  dfdv.setZero(dfdq.rows(), dfdq.cols());

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] BEFORE getOcs2Jacobian" << std::endl;
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] constraint.f: " << std::endl;
  //std::cout << constraint.f << std::endl;

  //std::cout << "[SelfCollisionConstraintCppAd::getLinearApproximation] END" << std::endl;

  return constraint;
}

}  // namespace ocs2
