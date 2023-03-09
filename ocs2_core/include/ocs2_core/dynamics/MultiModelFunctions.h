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

//#include <tf_conversions/tf_eigen.h>
//#include <tf/message_filter.h>
//#include <tf/transform_broadcaster.h>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
//#include <ocs2_pinocchio_interface/PinocchioInterface.h>

//#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
//#include "ocs2_mobile_manipulator/RobotModelInfo.h"
#include "ocs2_core/dynamics/RobotModelInfo.h"

namespace ocs2 {
//namespace mobile_manipulator {

/**
 * Create a scalar-typed RobotModelInfo.
 * NUA TODO: UPDATE!
 * @param [in] interface: Pinocchio interface
 * @param [in] baseFrame: Name of the root frame.
 * @param [in] eeFrame: Name of the end-effector frame.
 * @return RobotModelInfo
 */
RobotModelInfo createRobotModelInfo(const RobotModelType& robotModelType,
                                    const std::string& baseFrame, 
                                    const std::string& armBaseFrame,
                                    const std::string& eeFrame,
                                    const std::vector<std::string>& jointFrameNames,
                                    const std::vector<std::string>& jointNames);

/**
 * NUA TODO: UPDATE!
 */
bool updateModelMode(RobotModelInfo& robotModelInfo, size_t& modelMode);

/**
 * NUA TODO: UPDATE!
 */
size_t getModelModeInt(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDimBase(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDimArm(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDim(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getModeStateDim(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDimBase(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDimArm(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDim(RobotModelInfo& robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getModeInputDim(RobotModelInfo& robotModelInfo);

/** Load ManipulatorModelType for a config file */
RobotModelType loadRobotType(const std::string& configFilePath, const std::string& fieldName = "robotModelType");

//}  // namespace mobile_manipulator
}  // namespace ocs2
