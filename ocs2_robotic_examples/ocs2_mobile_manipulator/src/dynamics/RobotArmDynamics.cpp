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
  std::cout << "[RobotArmDynamics::RobotArmDynamics] START" << std::endl;

  if(info.robotArm.stateDim != 6 || info.robotArm.inputDim != 6)
  {
    std::cout << "[RobotArmDynamics::RobotArmDynamics] DEBUG INF" << std::endl;
    while(1);
  }

  this->initialize(info.robotArm.stateDim, info.robotArm.inputDim, modelName, modelFolder, recompileLibraries, verbose);

  std::cout << "[RobotArmDynamics::RobotArmDynamics] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t RobotArmDynamics::systemFlowMap(ad_scalar_t time, 
                                            const ad_vector_t& state, 
                                            const ad_vector_t& input,
                                            const ad_vector_t&) const 
{
  std::cout << "[RobotArmDynamics::systemFlowMap] START" << std::endl;

  if(state.size() != 6 || input.size() != 6)
  {
    std::cout << "[RobotArmDynamics::RobotArmDynamics] DEBUG INF" << std::endl;
    while(1);
  }

  return input;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
