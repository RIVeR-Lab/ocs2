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
                                                                         RobotModelInfo& robotModelInfo,
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
                                                                         RobotModelInfo& robotModelInfo,
                                                                         update_pinocchio_interface_callback updateCallback,
                                                                         const std::string& modelName, 
                                                                         const std::string& modelFolder, 
                                                                         bool recompileLibraries, 
                                                                         bool verbose)
  : robotModelInfo_(robotModelInfo)
{
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] START" << std::endl;

  endEffectorFrameNames_.clear();
  endEffectorFrameIds_.clear();

  if (robotModelInfo.modelMode == ModelMode::BaseMotion)
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] BaseMotion" << std::endl;
    endEffectorFrameNames_.push_back(robotModelInfo_.mobileBase.baseFrame);
  }
  else
  {
    //// NUA NOTE: DONT FORGET TO CHANGE BACK AFTER DEBUGGING!!!
    endEffectorFrameNames_.push_back(robotModelInfo_.robotArm.eeFrame);
    //endEffectorFrameNames_.push_back(robotModelInfo_.mobileBase.baseFrame);
  }

  for (const auto& bodyName : endEffectorFrameNames_) 
  {
    auto id = pinocchioInterface.getModel().getBodyId(bodyName);
    endEffectorFrameIds_.push_back(id);

    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] eeName: " << bodyName << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] id: " << id << std::endl;
  }

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] DEBUG INF" << std::endl;
  //while(1);

  // Initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // Set pinocchioInterface to mapping
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
  mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  const int stateDim = getStateDim(robotModelInfo_);
  const int modeStateDim = getModeStateDim(robotModelInfo_);
  const int modeInputDim = getModeInputDim(robotModelInfo_);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modelName: " << modelName << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modelFolder: " << modelFolder << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] stateDim: " << stateDim << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modeStateDim: " << modeStateDim << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] modeInputDim: " << modeInputDim << std::endl;

  /*
  const auto& model = pinocchioInterfaceCppAd.getModel();
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] model.nq: " << model.nq << std::endl;
  if(stateDim != model.nq)
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd] DEBUG INF " << std::endl;
    while(1);
  }
  */

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
    robotModelInfo_(rhs.robotModelInfo_),
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

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state, fullState);

  /*
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] q.size(): " << q.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] state.size(): " << state.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] fullState.size(): " << fullState.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] DEBUG INF" << std::endl;
  while(1);
  */

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t positions;
  if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionCppAd(4)] BaseMotion" << std::endl;
    positions.resize(2 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      positions.segment<2>(2 * i) = data.oMf[frameId].translation().head(2);
    }
  }
  else
  {
    positions.resize(3 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      positions.segment<3>(3 * i) = data.oMf[frameId].translation();
    }
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
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(1)] DEBUG INF" << std::endl;
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

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] BEFORE positionValues: " << std::endl;
  //std::cout << positionValues << std::endl;

  std::vector<vector3_t> positions;
  if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] BaseMotion" << std::endl;    
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      vector3_t posi;
      posi.head(2) = positionValues.segment<2>(2 * i);
      positions.emplace_back(posi);
    }
  }
  else
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] NOT BaseMotion" << std::endl;  
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      positions.emplace_back(positionValues.segment<3>(3 * i));
    }
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] AFTER positionValues size: " << positionValues.size() << std::endl;
  std::cout << positionValues << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPosition(2)] DEBUG INF" << std::endl;
  //while(1);

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
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] START" << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] DEBUG INF" << std::endl;
  //while(1);

  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state, fullState);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state, fullState);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] modeStateDim: " << robotModelInfo_.modeStateDim << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] modeInputDim: " << robotModelInfo_.modeInputDim << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] positionValues size: " << positionValues.size() << std::endl;
  std::cout << positionValues << std::endl << std::endl;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] positionJacobian rows: " << positionJacobian.rows() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] positionJacobian cols: " << positionJacobian.cols() << std::endl;
  std::cout << positionJacobian << std::endl;

  std::vector<VectorFunctionLinearApproximation> positions;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation pos;
    
    if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
    {
      std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation(2)] BaseMotion" << std::endl;
      pos.f = positionValues.segment<2>(2 * i);
      pos.dfdx = positionJacobian.block(2 * i, 0, 2, state.rows());
      positions.emplace_back(std::move(pos));
    }
    else
    {
      std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] NOT BaseMotion" << std::endl;
      pos.f = positionValues.segment<3>(3 * i);
      pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
      positions.emplace_back(std::move(pos));
    }
  }

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] dfdx rows: " << positions[0].dfdx.rows() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] dfdx cols: " << positions[0].dfdx.cols() << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getPositionLinearApproximation] DEBUG INF" << std::endl;
  //while(1);

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

  //// NUA NOTE: NOT ENTERING THIS FUNCTION!
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
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocity] DEBUG INF" << std::endl;
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
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getVelocityLinearApproximation] DEBUG INF" << std::endl;
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
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(2)] DEBUG INF" << std::endl;
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

  /*
  if(fullState.size() == state.size())
  {
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] state.size(): " << state.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] fullState.size(): " << fullState.size() << std::endl;
    std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] DEBUG INF" << std::endl;
    //while(1);
  }
  */

  const int stateDim = fullState.size();
  vector_t params(stateDim + 4 * endEffectorFrameIds_.size());
  
  params.head(stateDim) = fullState;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(stateDim + i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] errorValues size: " << errorValues.size() << std::endl;
  std::cout << errorValues << std::endl;

  /*
  double pi = 3.14159265359;
  auto yaw = 180 * errorValues(0) / pi;
  auto pitch = 180 * errorValues(1) / pi;
  auto roll = 180 * errorValues(2) / pi;

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] yaw: " << yaw << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] pitch: " << pitch << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] roll: " << roll << std::endl;
  */

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
    {
      vector3_t error;
      error.head(1) = errorValues.segment<1>(i);
      errors.emplace_back(error);
    }
    else
    {
      errors.emplace_back(errorValues.segment<3>(3 * i));
    }
  }
  
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationError(3)] DEBUG INF" << std::endl;
  //while(1);

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

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(2)] DEBUG INF" << std::endl;
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

  auto stateDim = fullState.size();
  vector_t params(stateDim + 4 * endEffectorFrameIds_.size());

  params.head(stateDim) = fullState;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    params.segment<4>(stateDim + i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);
  const matrix_t errorJacobian = orientationErrorCppAdInterfacePtr_->getJacobian(state, params);

  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] errorValues size: " << errorValues.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] errorJacobian size: " << errorJacobian.size() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] errorJacobian rows: " << errorJacobian.rows() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] errorJacobian cols: " << errorJacobian.cols() << std::endl;
  std::cout << errorJacobian << std::endl;

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
  {
    VectorFunctionLinearApproximation err;
    
    if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
    {
      err.f = errorValues.segment<1>(i);
      err.dfdx = errorJacobian.block(i, 0, 1, state.rows());
      errors.emplace_back(std::move(err));
    }
    else
    {
      err.f = errorValues.segment<3>(3 * i);
      err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
      errors.emplace_back(std::move(err));
    }
  }
  
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] dfdx rows: " << errors[0].dfdx.rows() << std::endl;
  std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] dfdx cols: " << errors[0].dfdx.cols() << std::endl;

  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorLinearApproximation(3)] DEBUG INF" << std::endl;
  //while(1);

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

  const int stateDim = getStateDim(robotModelInfo_);

  auto fullState = params.head(stateDim);
  auto pp = params.tail(4);

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  //const ad_vector_t q = mapping.getPinocchioJointPosition(state);
  const ad_vector_t q = mapping.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t errors;
  if (robotModelInfo_.modelMode == ModelMode::BaseMotion)
  {
    errors.resize((endEffectorFrameIds_.size()));
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      const ad_quaternion_t eeOrientation = matrixToQuaternion(data.oMf[frameId].rotation());
      Eigen::Matrix<ad_scalar_t, 3, 1> eeEulerZYX = getEulerAnglesZyxFromQuaternion(eeOrientation);
      
      ad_quaternion_t eeReferenceOrientation;
      eeReferenceOrientation.coeffs() = pp.segment<4>(i*4);
      Eigen::Matrix<ad_scalar_t, 3, 1> eeRefEulerZYX = getEulerAnglesZyxFromQuaternion(eeOrientation);

      auto yawDiff = eeEulerZYX - eeRefEulerZYX;
      //auto yawDiff = eeEulerZYX;
      errors.segment<1>(i) = yawDiff.segment<1>(0);
    }
  }
  else
  {
    errors.resize((3 * endEffectorFrameIds_.size()));
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      const ad_quaternion_t eeOrientation = matrixToQuaternion(data.oMf[frameId].rotation());
      ad_quaternion_t eeReferenceOrientation;
      eeReferenceOrientation.coeffs() = pp.segment<4>(i*4);
      errors.segment<3>(3 * i) = ocs2::quaternionDistance(eeOrientation, eeReferenceOrientation);

      //Eigen::Matrix<ad_scalar_t, 3, 1> eeRefEulerZYX = getEulerAnglesZyxFromQuaternion(eeOrientation);
      //errors.segment<3>(3 * i) = eeRefEulerZYX;
    }
  }
  
  //std::cout << "[PinocchioEndEffectorKinematicsCppAd::getOrientationErrorCppAd] END" << std::endl;

  return errors;
}

}  // namespace ocs2
