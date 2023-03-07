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

#include "ocs2_mobile_manipulator/dynamics/RobotArmDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RobotArmDynamics::RobotArmDynamics(const RobotModelInfo& info, 
                                   const std::string& modelName,
                                   const std::string& modelFolder, bool recompileLibraries /*= true*/,
                                   bool verbose /*= true*/) 
{
  this->initialize(info.robotArm.stateDim, info.robotArm.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t RobotArmDynamics::systemFlowMap(ad_scalar_t time, 
                                            const ad_vector_t& state, 
                                            const ad_vector_t& input,
                                            const ad_vector_t&) const 
{
  return input;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
