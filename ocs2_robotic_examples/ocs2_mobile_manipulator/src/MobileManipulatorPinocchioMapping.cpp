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

#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>::MobileManipulatorPinocchioMappingTpl(RobotModelInfo info)
  : modelInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>* MobileManipulatorPinocchioMappingTpl<SCALAR>::clone() const 
{
  return new MobileManipulatorPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t 
{
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
  -> vector_t 
{
  auto modeStateDim = modelInfo_.modeStateDim;
  vector_t vPinocchio = vector_t::Zero(modeStateDim);

  // Set velocity model based on model mode
  switch (modelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      vPinocchio << cos(theta) * v, sin(theta) * v, input(1);
      break;
    }

    case ModelMode::ArmMotion:
    {
      vPinocchio = input;
      break;
    }

    case ModelMode::WholeBodyMotion:
    {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      vPinocchio << cos(theta) * v, sin(theta) * v, input(1), input.tail(modelInfo_.robotArm.inputDim);
      break;
    }

    default: 
      throw std::runtime_error("[MobileManipulatorPinocchioMapping::getPinocchioJointVelocity] ERROR: Invalid manipulator model type!");
  }
  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
  -> std::pair<matrix_t, matrix_t> 
{
  // Set jacobian model based on model type
  switch (modelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      const auto stateDim = 3;
      const auto inputDim = 2;

      assert(Jq.rows() == stateDim);
      assert(Jq.cols() == stateDim);
      assert(Jv.rows() == stateDim);
      assert(Jv.cols() == stateDim);

      matrix_t dfdu(stateDim, inputDim);
      Eigen::Matrix<SCALAR, stateDim, inputDim> dxdu_base;
      
      const SCALAR theta = state(2);
      dxdu_base << cos(theta), SCALAR(0),
                   sin(theta), SCALAR(0),
                   SCALAR(0), SCALAR(1.0);
      
      dfdu.template leftCols<inputDim>() = Jv.template leftCols<stateDim>() * dxdu_base;
      
      return {Jq, dfdu};
    }
      
    case ModelMode::ArmMotion:
    {
      const auto stateDim = modelInfo_.robotArm.stateDim;
      const auto inputDim = modelInfo_.robotArm.inputDim;

      assert(Jq.rows() == stateDim);
      assert(Jq.cols() == stateDim);
      assert(Jv.rows() == stateDim);
      assert(Jv.cols() == stateDim);

      return {Jq, Jv};
    }

    case ModelMode::WholeBodyMotion:
    {
      const auto modeStateDim = modelInfo_.modeStateDim;
      const auto modeInputDim = modelInfo_.modeInputDim;

      assert(Jq.rows() == modeStateDim);
      assert(Jq.cols() == modeStateDim);
      assert(Jv.rows() == modeStateDim);
      assert(Jv.cols() == modeStateDim);

      matrix_t dfdu(modeStateDim, modeInputDim);

      const auto stateDimBase = 3;
      const auto inputDimBase = 2;
      const auto stateDimArm = modelInfo_.robotArm.stateDim;

      Eigen::Matrix<SCALAR, stateDimBase, inputDimBase> dxdu_base;
    
      const SCALAR theta = state(2);
      dxdu_base << cos(theta), SCALAR(0),
                    sin(theta), SCALAR(0),
                    SCALAR(0), SCALAR(1.0);
      
      dfdu.template leftCols<inputDimBase>() = Jv.template leftCols<stateDimBase>() * dxdu_base;
      dfdu.template rightCols(stateDimArm) = Jv.template rightCols(stateDimArm);
      
      return {Jq, dfdu};
    }
      
    default: 
      throw std::runtime_error("[MobileManipulatorPinocchioMapping::getOcs2Jacobian] ERROR: Invalid manipulator model type!");
      break;
  }
}

// explicit template instantiation
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace ocs2
