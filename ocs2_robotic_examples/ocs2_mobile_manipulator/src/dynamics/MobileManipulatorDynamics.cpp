// LAST UPDATE: 2022.04.10
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_mobile_manipulator/dynamics/MobileManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorDynamics::MobileManipulatorDynamics(RobotModelInfo info, 
                                                     const std::string& modelName,
                                                     const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                     bool recompileLibraries /*= true*/, bool verbose /*= true*/)
  : info_(std::move(info)) 
{
  //std::cout << "[MobileManipulatorDynamics::MobileManipulatorDynamics] START" << std::endl;

  auto stateDim = getStateDim(info);
  auto inputDim = getInputDim(info);

  if(stateDim != 9 || inputDim != 8)
  {
    std::cout << "[MobileManipulatorDynamics::MobileManipulatorDynamics] DEBUG INF" << std::endl;
    while(1);
  }

  this->initialize(stateDim, inputDim, modelName, modelFolder, recompileLibraries, verbose);

  //std::cout << "[MobileManipulatorDynamics::MobileManipulatorDynamics] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t MobileManipulatorDynamics::systemFlowMap(ad_scalar_t time, 
                                                     const ad_vector_t& state, 
                                                     const ad_vector_t& input,
                                                     const ad_vector_t&) const 
{
  //std::cout << "[MobileManipulatorDynamics::systemFlowMap] START" << std::endl;

  if(state.size() != 9 || input.size() != 8)
  {
    std::cout << "[MobileManipulatorDynamics::systemFlowMap] DEBUG INF" << std::endl;
    while(1);
  }

  auto info = info_;
  auto stateDim = getStateDim(info);
  
  ad_vector_t dxdt(stateDim);
  
  const auto theta = state(2);
  const auto v = input(0);  // forward velocity in base frame
  
  dxdt << cos(theta) * v, sin(theta) * v, input(1), input.tail(info_.robotArm.stateDim);

  //std::cout << "[MobileManipulatorDynamics::systemFlowMap] END" << std::endl;

  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
