// LAST UPDATE: 2024.02.15
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

namespace ocs2 {

/**
 * Create a scalar-typed RobotModelInfo.
 * NUA TODO: UPDATE!
 * @param [in] interface: Pinocchio interface
 * @param [in] baseFrame: Name of the root frame.
 * @param [in] eeFrame: Name of the end-effector frame.
 * @return RobotModelInfo
 */
RobotModelInfo createRobotModelInfo(const std::string robotName,
                                    const RobotModelType robotModelType,
                                    const std::string baseFrame, 
                                    const std::string armBaseFrame,
                                    const std::string eeFrame,
                                    const std::vector<std::string> jointFrameNames,
                                    const std::vector<std::string> jointNames);

/**
 * NUA TODO: UPDATE!
 */
bool updateModelMode(RobotModelInfo& robotModelInfo, size_t modelMode);

/**
 * NUA TODO: UPDATE!
 */
bool updateModelModeByState(RobotModelInfo& robotModelInfo, size_t stateDim);

/**
 * NUA TODO: UPDATE!
 */
std::string getRobotName(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getModelModeInt(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDimBase(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDimArm(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getStateDim(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getModeStateDim(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDimBase(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDimArm(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getInputDim(RobotModelInfo robotModelInfo);

/**
 * NUA TODO: UPDATE!
 */
size_t getModeInputDim(RobotModelInfo robotModelInfo);

/**
 * @brief Returns a string for a RobotModelType for retrieving data from a .info file
 * NUA TODO: UPDATE!
 */
std::string getRobotModelTypeString(RobotModelInfo robotModelInfo);

/**
 * @brief Returns a string for a ModelMode
 * NUA TODO: UPDATE!
 */
std::string getModelModeString(RobotModelInfo robotModelInfo);

/** Load ManipulatorModelType for a config file
 * NUA TODO: UPDATE!
 */
RobotModelType loadRobotType(const std::string configFilePath, const std::string fieldName = "robotModelType");

/**
 * NUA TODO: UPDATE!
 */
void printRobotModelInfo(RobotModelInfo robotModelInfo);

}  // namespace ocs2
