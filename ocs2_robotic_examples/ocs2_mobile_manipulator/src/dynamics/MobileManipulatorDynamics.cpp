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
  auto stateDim = info_.mobileBase.stateDim + info_.robotArm.stateDim;
  auto inputDim = info_.mobileBase.inputDim + info_.robotArm.inputDim;

  this->initialize(stateDim, inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t MobileManipulatorDynamics::systemFlowMap(ad_scalar_t time, 
                                                     const ad_vector_t& state, 
                                                     const ad_vector_t& input,
                                                     const ad_vector_t&) const 
{
  auto stateDim = info_.mobileBase.stateDim + info_.robotArm.stateDim;
  ad_vector_t dxdt(stateDim);
  const auto theta = state(2);
  const auto v = input(0);  // forward velocity in base frame
  dxdt << cos(theta) * v, sin(theta) * v, input(1), input.tail(info_.robotArm.stateDim);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
