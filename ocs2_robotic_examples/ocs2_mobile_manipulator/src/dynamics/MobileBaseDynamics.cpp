// LAST UPDATE: 2022.03.03
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_mobile_manipulator/dynamics/MobileBaseDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileBaseDynamics::MobileBaseDynamics(RobotModelInfo info, 
                                       const std::string& modelName,
                                       const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                       bool recompileLibraries /*= true*/, 
                                       bool verbose /*= true*/)
  : info_(std::move(info))
{
  this->initialize(info_.mobileBase.stateDim, info_.mobileBase.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t MobileBaseDynamics::systemFlowMap(ad_scalar_t time, 
                                              const ad_vector_t& state, 
                                              const ad_vector_t& input,
                                              const ad_vector_t&) const 
{
  ad_vector_t dxdt(info_.mobileBase.stateDim);
  const auto theta = state(2);
  const auto v = input(0);  // forward velocity in base frame
  dxdt << cos(theta) * v, sin(theta) * v, input(1);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
