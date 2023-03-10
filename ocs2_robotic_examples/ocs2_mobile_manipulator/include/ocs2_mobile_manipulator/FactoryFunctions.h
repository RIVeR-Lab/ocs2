// LAST UPDATE: 2022.03.09
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

#include <string>
#include <vector>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include "ocs2_core/dynamics/RobotModelInfo.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * NUA TODO: UPDATE!
 * @param [in] robotUrdfPath: The robot URDF path.
 * @return PinocchioInterface
 */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const RobotModelType& robotModelType,
                                            std::string world_frame_name = "world",
                                            std::string base_frame_name = "base_link");

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * NUA TODO: UPDATE!
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (fixed-arm or wheel-based)
 * @param [in] jointNames: The joint names from URDF to make fixed/unactuated.
 * @return PinocchioInterface
 */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const RobotModelType& robotModelType,
                                            const std::vector<std::string>& jointNamesRemoved,
                                            std::string world_frame_name = "world",
                                            std::string base_frame_name = "base_link");

/** Load ManipulatorModelType for a config file */
//RobotModelType loadRobotType(const std::string& configFilePath, const std::string& fieldName = "robotModelType");

}  // namespace mobile_manipulator
}  // namespace ocs2
