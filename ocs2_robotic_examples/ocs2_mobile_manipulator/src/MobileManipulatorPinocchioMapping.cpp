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
const RobotModelInfo& MobileManipulatorPinocchioMappingTpl<SCALAR>::getRobotModelInfo() const
{ 
  return modelInfo_; 
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t 
{
  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(1)] START" << std::endl;

  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(1)] DEBUG INF" << std::endl;
  while(1);

  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(1)] END" << std::endl;

  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state, const vector_t& fullState) const -> vector_t 
{
  //std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] START" << std::endl;

  auto modeStateDim = modelInfo_.modeStateDim;
  switch (modelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {
      if (state.size() != fullState.size() || state.size() != 3)
      {
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] fullState.size(): " << fullState.size() << std::endl;
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] state.size(): " << state.size() << std::endl;
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] DEBUG INF MobileBase" << std::endl;
        while(1);
      }
      return state;
    }

    case RobotModelType::RobotArm:
    {
      if (state.size() != fullState.size() || state.size() != 6)
      {
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] fullState.size(): " << fullState.size() << std::endl;
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] state.size(): " << state.size() << std::endl;
        std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] DEBUG INF RobotArm" << std::endl;
        while(1);
      }
      return state;
    }

    case RobotModelType::MobileManipulator:
    {
      switch (modelInfo_.modelMode) 
      {
        case ModelMode::BaseMotion:
        {
          //std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] BaseMotion" << std::endl;
          
          if (fullState.size() != 9 || state.size() != 3)
          {
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] fullState.size(): " << fullState.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] state.size(): " << state.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] DEBUG INF BaseMotion" << std::endl;
            while(1);
          }
          
          vector_t pinocchioState = fullState;
          pinocchioState.head(modeStateDim) = state;
          return pinocchioState;
        }

        case ModelMode::ArmMotion:
        {
          //std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] ArmMotion" << std::endl;
          
          if (fullState.size() != 9 || state.size() != 6)
          {
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] fullState.size(): " << fullState.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] state.size(): " << state.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] DEBUG INF ArmMotion" << std::endl;
            while(1);
          }
          
          vector_t pinocchioState = fullState;
          pinocchioState.tail(modeStateDim) = state;
          
          return pinocchioState;
          //return state;
        }

        case ModelMode::WholeBodyMotion:
        {
          //std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] WholeBodyMotion" << std::endl;
          
          if (fullState.size() != 9 || state.size() != 9)
          {
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] fullState.size(): " << fullState.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] state.size(): " << state.size() << std::endl;
            std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointPosition(2)] DEBUG INF WholeBodyMotion" << std::endl;
            while(1);
          }
          
          return state;
        }

        default: 
          throw std::runtime_error("[MobileManipulatorPinocchioMapping::getPinocchioJointVelocity] ERROR: Invalid manipulator model type!");
      }
    }
  
    default:
      break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
  -> vector_t 
{
  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] START" << std::endl;

  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] DEBUG INF" << std::endl;
  while(1);

  auto modeStateDim = modelInfo_.modeStateDim;
  vector_t pinocchioStateVelo = vector_t::Zero(modeStateDim);

  // Set velocity model based on model mode
  switch (modelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] BaseMotion" << std::endl;
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      pinocchioStateVelo << cos(theta) * v, sin(theta) * v, input(1);
      break;
    }

    case ModelMode::ArmMotion:
    {
      std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] ArmMotion" << std::endl;
      pinocchioStateVelo = input;
      break;
    }

    case ModelMode::WholeBodyMotion:
    {
      std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] WholeBodyMotion" << std::endl;
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      pinocchioStateVelo << cos(theta) * v, sin(theta) * v, input(1), input.tail(modelInfo_.robotArm.inputDim);
      break;
    }

    default: 
      throw std::runtime_error("[MobileManipulatorPinocchioMapping::getPinocchioJointVelocity] ERROR: Invalid manipulator model type!");
  }

  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity] END" << std::endl;

  return pinocchioStateVelo;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& fullStateVelo, const vector_t& input) const
  -> vector_t 
{
  std::cout << "[MobileManipulatorPinocchioMappingTpl::getPinocchioJointVelocity(3)] DEBUG INF" << std::endl;
  while(1);

  auto modeStateDim = modelInfo_.modeStateDim;
  vector_t pinocchioStateVelo = fullStateVelo;

  // Set velocity model based on model mode
  switch (modelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame

      pinocchioStateVelo(0) = cos(theta) * v;
      pinocchioStateVelo(1) = sin(theta) * v;
      pinocchioStateVelo(2) = input(1);
      break;
    }

    case ModelMode::ArmMotion:
    {
      pinocchioStateVelo.tail(modelInfo_.robotArm.inputDim) = input;
      break;
    }

    case ModelMode::WholeBodyMotion:
    {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame

      pinocchioStateVelo(0) = cos(theta) * v;
      pinocchioStateVelo(1) = sin(theta) * v;
      pinocchioStateVelo(2) = input(1);
      pinocchioStateVelo.tail(modelInfo_.robotArm.inputDim) = input;
      break;
    }

    default: 
      throw std::runtime_error("[MobileManipulatorPinocchioMapping::getPinocchioJointVelocity] ERROR: Invalid manipulator model type!");
  }
  return pinocchioStateVelo;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
  -> std::pair<matrix_t, matrix_t> 
{
  //std::cout << "[MobileManipulatorPinocchioMappingTpl::getOcs2Jacobian] START" << std::endl;
  
  //std::cout << "[MobileManipulatorPinocchioMappingTpl::getOcs2Jacobian] DEBUG INF" << std::endl;
  //while(1);

  auto modelInfo = modelInfo_;
  const auto modeStateDim = getModeStateDim(modelInfo);
  const auto modeInputDim = getModeInputDim(modelInfo);

  assert(state.size() == modeStateDim);
  assert(Jq.rows() == modeStateDim);
  assert(Jq.cols() == modeStateDim);
  assert(Jv.rows() == modeStateDim);
  assert(Jv.cols() == modeStateDim);

  // Set jacobian model based on model type
  switch (modelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      const auto stateDimBase = 3;
      const auto inputDimBase = 2;

      matrix_t dfdu(modeStateDim, modeInputDim);
      Eigen::Matrix<SCALAR, 3, 2> dxdu_base;
      
      const SCALAR theta = state(2);
      dxdu_base << cos(theta), SCALAR(0),
                   sin(theta), SCALAR(0),
                   SCALAR(0), SCALAR(1.0);
      
      dfdu.template leftCols<inputDimBase>() = Jv.template leftCols<stateDimBase>() * dxdu_base;
      
      return {Jq, dfdu};
    }
      
    case ModelMode::ArmMotion:
    {
      return {Jq, Jv};
    }

    case ModelMode::WholeBodyMotion:
    {
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
