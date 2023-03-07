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

#pragma once

#include <string>
#include <vector>

namespace ocs2 {
namespace mobile_manipulator {

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
  RobotModelType robotModelType;
  ModelMode modelMode;                            // mode of the robot model
  MobileBase mobileBase;
  RobotArm robotArm;
};

/**
 * @brief Returns a string for a RobotModelType for retrieving data from a .info file
 */
static std::string modelTypeEnumToString(RobotModelType robotModelType) 
{
  std::string robotModelTypeString;

  switch (robotModelType) 
  {
    case RobotModelType::MobileBase: 
      robotModelTypeString = "mobileBase";
      break;

    case RobotModelType::RobotArm: 
      robotModelTypeString = "robotArm";
      break;

    case RobotModelType::MobileManipulator: 
      robotModelTypeString = "mobileManipulator";
      break;

    default:
      throw std::invalid_argument("[RobotModelInfo::modelTypeEnumToString] Error: Invalid robot model!");
      break;
  }

  return robotModelTypeString;
}

/**
 * @brief Returns a string for a ModelMode
 */
static std::string modelModeEnumToString(ModelMode modelMode) 
{
  std::string modeModelString;

  switch (modelMode) 
  {
    case ModelMode::BaseMotion: 
      modeModelString = "baseMotion";
      break;

    case ModelMode::ArmMotion: 
      modeModelString = "armMotion";
      break;

    case ModelMode::WholeBodyMotion: 
      modeModelString = "wholeBodyMotion";
      break;

    default:
      throw std::invalid_argument("[RobotModelInfo::modelModeEnumToString] Error: Invalid model mode!");
      break;
  }

  return modeModelString;
}

}  // namespace mobile_manipulator
}  // namespace ocs2