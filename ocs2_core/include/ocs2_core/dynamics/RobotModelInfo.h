// LAST UPDATE: 2023.07.27
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

namespace ocs2 {

/**
 * @brief Defines robot models.
 */
enum class RobotModelType 
{
  MobileBase = 0,
  RobotArm = 1,
  MobileManipulator = 2
};

/**
 * @brief Defines modes of the model.
 */
enum class ModelMode 
{
  BaseMotion = 0,
  ArmMotion = 1,
  WholeBodyMotion = 2
};

struct MobileBase
{
  size_t stateDim;                                // number of states
  size_t inputDim;                                // number of inputs
  std::string baseFrame;                          // name of the root frame of the robot
};

struct  RobotArm
{
  size_t stateDim;                                // number of states needed to define the system flow map
  size_t inputDim;                                // number of inputs needed to define the system flow map
  std::string baseFrame;                          // name of the root frame of the robot
  std::string eeFrame;                            // name of the end-effector frame of the robot
  std::vector<std::string> jointFrameNames;       // name of the actuated DOFs in the robot
  std::vector<std::string> jointNames;            // name of the actuated DOFs in the robot
};

struct RobotModelInfo
{
  std::string robotName;
  RobotModelType robotModelType;
  MobileBase mobileBase;
  RobotArm robotArm;
  ModelMode modelMode;                            // mode of the robot model
  size_t modeStateDim;
  size_t modeInputDim;
};

}  // namespace ocs2
