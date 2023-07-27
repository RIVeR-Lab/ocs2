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

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
//#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
//#include <ocs2_mobile_manipulator/RobotModelInfo.h>
#include "ocs2_core/dynamics/RobotModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Implementation of a fixed arm manipulator dynamics.
 *
 * The fixed-arm manipulator has the state: (arm joints).
 * The end-effector targets are assumed to given with respect to the base frame.
 * The arm is assumed to be velocity controlled.
 */
class RobotArmDynamics final : public SystemDynamicsBaseAD 
{
  public:
    /**
     * Constructor
     *
     * @param [in] modelInfo : The manipulator information.
     * @param [in] modelName : name of the generate model library
     * @param [in] modelFolder : folder to save the model library files to
     * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
     * @param [in] verbose : Display information.
     */
    RobotArmDynamics(const RobotModelInfo& modelInfo, 
                    const std::string& modelName,
                    const std::string& modelFolder = "/tmp/ocs2", 
                    bool recompileLibraries = true, 
                    bool verbose = true);

    ~RobotArmDynamics() override = default;

    RobotArmDynamics* clone() const override 
    { 
      return new RobotArmDynamics(*this); 
    }

  private:
    RobotArmDynamics(const RobotArmDynamics& rhs) = default;

    ad_vector_t systemFlowMap(ad_scalar_t time, 
                              const ad_vector_t& state, 
                              const ad_vector_t& input,
                              const ad_vector_t& /*parameters*/) const override;
};

}  // namespace mobile_manipulator
}  // namespace ocs2