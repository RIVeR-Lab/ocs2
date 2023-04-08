/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace {

void defaultUpdatePinocchioInterface(const ocs2::ad_vector_t&, ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>&) {}

}  // unnamed namespace

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface,
                                                                         const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                         RobotModelInfo robotModelInfo,
                                                                         const std::string& modelName,
                                                                         const std::string& modelFolder, 
                                                                         bool recompileLibraries,
                                                                         bool verbose)

  : PinocchioEndEffectorKinematicsCppAd(pinocchioInterface, 
                                        mapping, 
                                        robotModelInfo,
                                        &defaultUpdatePinocchioInterface, 
                                        modelName, 
                                        modelFolder, 
                                        recompileLibraries, 
                                        verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface, 
                                                                         const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                         RobotModelInfo robotModelInfo,
                                                                         update_pinocchio_interface_callback updateCallback,
                                                                         const std::string& modelName, 
                                                                         const std::string& modelFolder, 
                                                                         bool recompileLibraries, 
                                                                         bool verbose)
  : robotModelInfo_(robotModelInfo)
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START" << std::endl;

  endEffectorFrameNames_.clear();
  endEffectorFrameNames_.push_back(robotModelInfo_.robotArm.eeFrame);

  for (const auto& bodyName : {robotModelInfo_.robotArm.eeFrame}) 
  {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }

  // Initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // Set pinocchioInterface to mapping
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
  mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  // NUA TODO: FIX IT !!!!!!!!!!!!!!!!!!!!!!!!!
  const int stateDim = getStateDimTmp(robotModelInfo_);
  const int modeStateDim = getModeStateDim(robotModelInfo_);
  const int modeInputDim = getModeInputDim(robotModelInfo_);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modelName: " << modelName << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modelFolder: " << modelFolder << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] stateDim: " << stateDim << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modeStateDim: " << modeStateDim << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modeInputDim: " << modeInputDim << std::endl;
  
  /*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START getPositionCppAd" << std::endl;
  auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
  {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = getPositionCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
  };
  positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, 
                                                      modeStateDim,
                                                      modelName + "_position", 
                                                      modelFolder));
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END getPositionCppAd" << std::endl;
  */

  ///*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START getPositionCppAd" << std::endl;
  auto positionFunc = [&, this](const ad_vector_t& x, const ad_vector_t& x_full, ad_vector_t& y) 
  {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = getPositionCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, x_full);
  };
  positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, 
                                                      modeStateDim,
                                                      stateDim,
                                                      modelName + "_position", 
                                                      modelFolder));
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END getPositionCppAd" << std::endl;
  //*/
  
  /*
  // Set velocity function
  auto velocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
  {
    const ad_vector_t state = x.head(modeStateDim);
    const ad_vector_t input = x.tail(modeInputDim);
    updateCallback(state, pinocchioInterfaceCppAd);
    y = getVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
  };
  velocityCppAdInterfacePtr_.reset(new CppAdInterface(velocityFunc, 
                                                      modeStateDim + modeInputDim, 
                                                      modelName + "_velocity", 
                                                      modelFolder));
  */

  /*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START orientationErrorCppAdInterfacePtr_" << std::endl;
  // Set Orientation function
  auto orientationFunc = [&, this](const ad_vector_t& x, const ad_vector_t& params, ad_vector_t& y) 
  {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = getOrientationErrorCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, params);
  };
  orientationErrorCppAdInterfacePtr_.reset(new CppAdInterface(orientationFunc, 
                                                              modeStateDim, 
                                                              4 * endEffectorFrameIds_.size(), 
                                                              modelName + "_orientation", 
                                                              modelFolder));
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END orientationErrorCppAdInterfacePtr_" << std::endl;
  */
  ///*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START orientationErrorCppAdInterfacePtr_" << std::endl;
  auto orientationFunc = [&, this](const ad_vector_t& x, const ad_vector_t& params, ad_vector_t& y) 
  {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = getOrientationErrorCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, params);
  };
  orientationErrorCppAdInterfacePtr_.reset(new CppAdInterface(orientationFunc, 
                                                              modeStateDim, 
                                                              stateDim + 4 * endEffectorFrameIds_.size(), 
                                                              modelName + "_orientation", 
                                                              modelFolder));
  //*/
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END orientationErrorCppAdInterfacePtr_" << std::endl;

  if (recompileLibraries) 
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START createModels positionCppAdInterfacePtr_" << std::endl;
    positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END createModels positionCppAdInterfacePtr_" << std::endl;

    /*
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START createModels velocityCppAdInterfacePtr_" << std::endl;
    velocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END createModels velocityCppAdInterfacePtr_" << std::endl;
    */
    
    //std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] DEBUG INF" << std::endl;
    //while(1);

    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START createModels orientationErrorCppAdInterfacePtr_" << std::endl;
    orientationErrorCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END createModels orientationErrorCppAdInterfacePtr_" << std::endl;
  }
  else 
  {
    positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    //velocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    orientationErrorCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(const PinocchioEndEffectorKinematicsCppAd& rhs)
  : EndEffectorKinematics<scalar_t>(rhs),
    positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
    //velocityCppAdInterfacePtr_(new CppAdInterface(*rhs.velocityCppAdInterfacePtr_)),
    orientationErrorCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationErrorCppAdInterfacePtr_)),
    endEffectorFrameNames_(rhs.endEffectorFrameNames_),
    endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd* PinocchioEndEffectorKinematicsCppAd::clone() const 
{
  return new PinocchioEndEffectorKinematicsCppAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<std::string>& PinocchioEndEffectorKinematicsCppAd::getEndEffectorFrameNames() const 
{
  return endEffectorFrameNames_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state) 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(3)] START" << std::endl;

  //// NUA NOTE: CLEAN WHEN MULTI MODEL IS COMPLETED!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(3)] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t positions(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    const size_t frameId = endEffectorFrameIds_[i];
    positions.segment<3>(3 * i) = data.oMf[frameId].translation();
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(3)] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state,
                                                                  const ad_vector_t& fullState) 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] START" << std::endl;

  if(fullState.size() == state.size())
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] state.size(): " << state.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] fullState.size(): " << fullState.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] DEBUG INF" << std::endl;
    //while(1);
  }

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state, fullState);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] q.size(): " << q.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] DEBUG INF" << std::endl;
  //while(1);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t positions(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    const size_t frameId = endEffectorFrameIds_[i];
    positions.segment<3>(3 * i) = data.oMf[frameId].translation();
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t> 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(1)] START" << std::endl;

  //// NUA NOTE: CLEAN WHEN MULTI MODEL IS COMPLETED!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(1)] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

  std::vector<vector3_t> positions;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    positions.emplace_back(positionValues.segment<3>(3 * i));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(1)] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::getPosition(const vector_t& state, const vector_t& fullState) const -> std::vector<vector3_t> 
{
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] START" << std::endl;

  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state, fullState);

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] positionValues: " << std::endl;
  //std::cout << positionValues << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] DEBUG INF" << std::endl;
  //while(1);

  std::vector<vector3_t> positions;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    positions.emplace_back(positionValues.segment<3>(3 * i));
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(const vector_t& state) const 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(1)] START" << std::endl;

  //// NUA NOTE: CLEAN WHEN MULTI MODEL IS COMPLETED!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(1)] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

  std::vector<VectorFunctionLinearApproximation> positions;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation pos;
    pos.f = positionValues.segment<3>(3 * i);
    pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
    positions.emplace_back(std::move(pos));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(1)] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(const vector_t& state,
                                                                                                                   const vector_t& fullState) const 
{
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] START" << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] DEBUG INF" << std::endl;
  //while(1);

  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state, fullState);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state, fullState);

  /*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] positionValues: " << std::endl;
  std::cout << positionValues << std::endl << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] positionJacobian: " << std::endl;
  std::cout << positionJacobian << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] DEBUG INF" << std::endl;
  while(1);
  */

  std::vector<VectorFunctionLinearApproximation> positions;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation pos;
    pos.f = positionValues.segment<3>(3 * i);
    pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
    positions.emplace_back(std::move(pos));
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] END" << std::endl;

  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state, 
                                                                  const ad_vector_t& input) 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] START" << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd(4)] DEBUG INF" << std::endl;
  while(1);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();

  const ad_vector_t q = mapping.getPinocchioJointPosition(state);
  const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] START forwardKinematics" << std::endl;
  pinocchio::forwardKinematics(model, data, q, v);
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] END forwardKinematics" << std::endl;

  ad_vector_t velocities(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    const size_t frameId = endEffectorFrameIds_[i];
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] START getFrameVelocity" << std::endl;
    velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] END getFrameVelocity" << std::endl;
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityCppAd] END" << std::endl;

  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::getVelocity(const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocity] START" << std::endl;

  //// NUA NOTE: NOT ENTERING THIS FUNCTION!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocity] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);

  std::vector<vector3_t> velocities;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    velocities.emplace_back(velocityValues.segment<3>(3 * i));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocity] END" << std::endl;

  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation(const vector_t& state, 
                                                                                                                   const vector_t& input) const 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation] START" << std::endl;

  //// NUA NOTE: NOT ENTERING THIS FUNCTION!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);
  const matrix_t velocityJacobian = velocityCppAdInterfacePtr_->getJacobian(stateInput);

  std::vector<VectorFunctionLinearApproximation> velocities;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation vel;
    vel.f = velocityValues.segment<3>(3 * i);
    vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
    vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
    velocities.emplace_back(std::move(vel));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation] END" << std::endl;

  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::getOrientationError(const vector_t& state,
                                                              const std::vector<quaternion_t>& referenceOrientations) const
    -> std::vector<vector3_t> 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(2)] START" << std::endl;

  //// NUA NOTE: CLEAN WHEN MULTI MODEL IS COMPLETED!
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(2)] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);

  vector_t params(4 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    errors.emplace_back(errorValues.segment<3>(3 * i));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(2)] END" << std::endl;

  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::getOrientationError(const vector_t& state,
                                                              const vector_t& fullState,
                                                              const std::vector<quaternion_t>& referenceOrientations) const
    -> std::vector<vector3_t> 
{
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] START" << std::endl;

  if(fullState.size() == state.size())
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] state.size(): " << state.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] fullState.size(): " << fullState.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] DEBUG INF" << std::endl;
    //while(1);
  }

  const int stateDim = fullState.size();
  vector_t params(stateDim + 4 * endEffectorFrameIds_.size());
  
  params.head(stateDim) = fullState;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(stateDim + i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    errors.emplace_back(errorValues.segment<3>(3 * i));
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] END" << std::endl;

  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(const vector_t& state, 
                                                                                                                           const std::vector<quaternion_t>& referenceOrientations) const 
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(2)] END" << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(2)] NOT CHECKED/IMPLEMENTED INF LOOP!" << std::endl;
  while(1);
  
  vector_t params(4 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);
  const matrix_t errorJacobian = orientationErrorCppAdInterfacePtr_->getJacobian(state, params);

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation err;
    err.f = errorValues.segment<3>(3 * i);
    err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
    errors.emplace_back(std::move(err));
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(2)] END" << std::endl;

  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(const vector_t& state, 
                                                                                                                           const vector_t& fullState,
                                                                                                                           const std::vector<quaternion_t>& referenceOrientations) const 
{
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] END" << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] DEBUG INF" << std::endl;
  //while(1);

  if(fullState.size() != 9)
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] state.size(): " << state.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] fullState.size(): " << fullState.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] DEBUG INF" << std::endl;
    //while(1);
  }

  auto stateDim = fullState.size();
  vector_t params(stateDim + 4 * endEffectorFrameIds_.size());

  params.head(stateDim) = fullState;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(stateDim + i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);
  const matrix_t errorJacobian = orientationErrorCppAdInterfacePtr_->getJacobian(state, params);

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation err;
    err.f = errorValues.segment<3>(3 * i);
    err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
    errors.emplace_back(std::move(err));
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] END" << std::endl;

  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                          const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                          const ad_vector_t& state, 
                                                                          const ad_vector_t& params) 
{
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] START" << std::endl;

  using ad_quaternion_t = Eigen::Quaternion<ad_scalar_t>;

  // NUA TODO: MUST FIX!!!!!!!!!!!!!!!!!!!!!!!
  const int stateDim = robotModelInfo_.mobileBase.stateDimTmp + robotModelInfo_.robotArm.stateDim;
  //const int stateDim = robotModelInfo_.robotArm.stateDim;
  //const int stateDim = 9;

  if(params.size()-4 == state.size())
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] params.size: " << params.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] state.size: " << state.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] DEBUG INF" << std::endl;
    //while(1);
  }

  auto fullState = params.head(stateDim);
  auto pp = params.tail(4);

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  //const ad_vector_t q = mapping.getPinocchioJointPosition(state);
  const ad_vector_t q = mapping.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t errors(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    const size_t frameId = endEffectorFrameIds_[i];
    const ad_quaternion_t eeOrientation = matrixToQuaternion(data.oMf[frameId].rotation());
    ad_quaternion_t eeReferenceOrientation;
    eeReferenceOrientation.coeffs() = pp.segment<4>(i*4);
    errors.segment<3>(3 * i) = ocs2::quaternionDistance(eeOrientation, eeReferenceOrientation);
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] END" << std::endl;

  return errors;
}

}  // namespace ocs2
