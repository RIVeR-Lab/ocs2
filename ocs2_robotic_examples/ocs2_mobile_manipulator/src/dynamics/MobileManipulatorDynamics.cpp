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
  auto modeStateDim = getModeStateDim(info_);
  auto modeInputDim = getModeInputDim(info_);

  this->initialize(modeStateDim, modeInputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t MobileManipulatorDynamics::systemFlowMap(ad_scalar_t time, 
                                                     const ad_vector_t& state, 
                                                     const ad_vector_t& input,
                                                     const ad_vector_t&) const 
{
  std::cout << "[MobileManipulatorDynamics::systemFlowMap] START" << std::endl;

  auto robotModelInfo = info_;
  auto modeStateDim = getModeStateDim(robotModelInfo);

  std::cout << "[MobileManipulatorDynamics::systemFlowMap] modelMode: " << modelModeEnumToString(robotModelInfo) << std::endl;
  std::cout << "[MobileManipulatorDynamics::systemFlowMap] modeStateDim: " << modeStateDim << std::endl;;

  switch (info_.modelMode)
  {
    case ModelMode::BaseMotion:
    {
      std::cout << "[MobileManipulatorDynamics::systemFlowMap] DEBUG INF BaseMotion" << std::endl;
      while(1);

      ad_vector_t dxdt(modeStateDim);
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      dxdt << cos(theta) * v, sin(theta) * v, input(1);
      return dxdt;
    }

    case ModelMode::ArmMotion:
    {
      std::cout << "[MobileManipulatorDynamics::systemFlowMap] DEBUG INF ArmMotion" << std::endl;
      //while(1);

      return input;

      //ad_vector_t dxdt(modeStateDim);
      //dxdt << input.tail(info_.robotArm.stateDim);
      //return dxdt;
    }

    case ModelMode::WholeBodyMotion:
    {
      std::cout << "[MobileManipulatorDynamics::systemFlowMap] DEBUG INF WholeBodyMotion" << std::endl;

      /*
      ad_vector_t dxdt(1);
      dxdt << state(4);
      */
      
      ad_vector_t dxdt(modeStateDim);
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      dxdt << cos(theta) * v, sin(theta) * v, input(1), input.tail(info_.robotArm.stateDim);
      

      return dxdt;
    }

    default:
      std::cerr << "[MobileManipulatorDynamics::systemFlowMap] ERROR: Invalid model mode!";
      break;
  }

  std::cout << "[MobileManipulatorDynamics::systemFlowMap] END" << std::endl;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
