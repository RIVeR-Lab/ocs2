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
                                                         ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                                                         std::shared_ptr<PointsOnRobot> pointsOnRobotPtr, 
                                                         const std::string& modelName, 
                                                         const std::string& modelFolder,
                                                         bool recompileLibraries, 
                                                         bool verbose)
  : ExtCollisionConstraintCppAd(std::move(pinocchioInterface), 
                                mapping, 
                                std::move(extCollisionPinocchioGeometryInterface), 
                                pointsOnRobotPtr,
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
                                                         ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                                         std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                                         update_pinocchio_interface_callback updateCallback, 
                                                         const std::string& modelName,
                                                         const std::string& modelFolder, 
                                                         bool recompileLibraries, 
                                                         bool verbose)
  : StateConstraint(ConstraintOrder::Linear),
    pinocchioInterface_(std::move(pinocchioInterface)),
    extCollisionCppAd_(pinocchioInterface_, 
                  std::move(extCollisionPinocchioGeometryInterface), 
                  pointsOnRobotPtr,
                  modelName, 
                  modelFolder,
                  recompileLibraries, 
                  verbose),
    mappingPtr_(mapping.clone()),
    updateCallback_(std::move(updateCallback)) 
{
  std::cout << "[ExtCollisionConstraintCppAd::ExtCollisionConstraintCppAd] START" << std::endl;

  mappingPtr_->setPinocchioInterface(pinocchioInterface_);

  std::cout << "[ExtCollisionConstraintCppAd::ExtCollisionConstraintCppAd] END" << std::endl;
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
  return extCollisionCppAd_.getNumCollisionPairs();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionConstraintCppAd::getValue(scalar_t time, const vector_t& state, const PreComputation&) const 
{
  const auto q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);

  return extCollisionCppAd_.getValue(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ExtCollisionConstraintCppAd::getLinearApproximation(scalar_t time, 
                                                                                      const vector_t& state,
                                                                                      const PreComputation&) const 
{
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
  return constraint;
}

}  // namespace ocs2
