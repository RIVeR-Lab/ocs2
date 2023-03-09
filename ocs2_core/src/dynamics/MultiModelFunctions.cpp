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

//#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_core/dynamics/MultiModelFunctions.h"

//#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
//#include <pinocchio/multibody/joint/joint-composite.hpp>
//#include <pinocchio/multibody/model.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
//#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
//namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RobotModelInfo createRobotModelInfo(const RobotModelType& robotModelType,
                                    const std::string& baseFrame, 
                                    const std::string& armBaseFrame,
                                    const std::string& eeFrame, 
                                    const std::vector<std::string>& armJointFrameNames,
                                    const std::vector<std::string>& armJointNames) 
{
  //const auto& model = interface.getModel();
  RobotModelInfo info;
  info.robotModelType = robotModelType;

  auto n_armJoints = armJointNames.size();
  
  /////// NUA TODO: ADD 6 DOF BASE VERSION!  
  switch (robotModelType) 
  {
    case RobotModelType::MobileBase:
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] MobileBase" << std::endl;
      info.mobileBase.baseFrame = baseFrame;
      info.mobileBase.stateDim = 3;
      info.mobileBase.inputDim = 2;
      info.robotArm.baseFrame = "";
      info.robotArm.eeFrame = "";
      info.robotArm.stateDim = 0;
      info.robotArm.inputDim = 0;
      info.modelMode = ModelMode::BaseMotion;
      info.modeStateDim = info.mobileBase.stateDim;
      info.modeInputDim = info.mobileBase.inputDim;
      break;
    }

    case RobotModelType::RobotArm: 
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] RobotArm" << std::endl;
      info.mobileBase.baseFrame = "";
      info.mobileBase.stateDim = 0;
      info.mobileBase.inputDim = 0;
      info.robotArm.baseFrame = armBaseFrame;
      info.robotArm.eeFrame = eeFrame;
      info.robotArm.jointFrameNames = armJointFrameNames;
      info.robotArm.jointNames = armJointNames;
      //info.robotArm.jointFrameNames = model.names;
      info.robotArm.stateDim = n_armJoints;
      info.robotArm.inputDim = n_armJoints;
      info.modelMode = ModelMode::ArmMotion;
      info.modeStateDim = info.robotArm.stateDim;
      info.modeInputDim = info.robotArm.inputDim;
      break;
    }
      
    case RobotModelType::MobileManipulator: 
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] MobileManipulator" << std::endl;
      info.mobileBase.baseFrame = baseFrame;
      info.mobileBase.stateDim = 3;
      info.mobileBase.inputDim = 2;
      info.robotArm.baseFrame = armBaseFrame;
      info.robotArm.eeFrame = eeFrame;
      info.robotArm.jointFrameNames = armJointFrameNames;
      info.robotArm.jointNames = armJointNames;
      info.robotArm.stateDim = n_armJoints;
      info.robotArm.inputDim = n_armJoints;
      info.modelMode = ModelMode::WholeBodyMotion;
      info.modeStateDim = info.mobileBase.stateDim + info.robotArm.stateDim;
      info.modeInputDim = info.mobileBase.inputDim + info.robotArm.inputDim;
      break;
    }

    default:
      throw std::invalid_argument("[FactoryFunction::createRobotModelInfo] ERROR: Invalid manipulator model type!");
      break;
  }

  return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getModelModeInt(RobotModelInfo& robotModelInfo)
{
  size_t modelModeInt;

  switch (robotModelInfo.modelMode)
  {
    case ModelMode::BaseMotion:
      modelModeInt = 0;
      break;

    case ModelMode::ArmMotion:
      modelModeInt = 1;
      break;

    case ModelMode::WholeBodyMotion:
      modelModeInt = 2;
      break;

    default:
      std::cerr << "[FactoryFunction::createRobotModelInfo] ERROR: Invalid manipulator model type!";
      break;
  }
  return modelModeInt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getStateDimBase(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.mobileBase.stateDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getStateDimArm(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.robotArm.stateDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getStateDim(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.mobileBase.stateDim + robotModelInfo.robotArm.stateDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getModeStateDim(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.modeStateDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getInputDimBase(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.mobileBase.inputDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getInputDimArm(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.robotArm.inputDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getInputDim(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.mobileBase.inputDim + robotModelInfo.robotArm.inputDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t getModeInputDim(RobotModelInfo& robotModelInfo)
{
  return robotModelInfo.modeInputDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool updateModelMode(RobotModelInfo& robotModelInfo, size_t& modelMode)
{
  bool result = false;
  if (robotModelInfo.robotModelType == RobotModelType::MobileManipulator)
  {
    switch (modelMode)
    {
      case 0:
        robotModelInfo.modelMode = ModelMode::BaseMotion;
        robotModelInfo.modeStateDim = robotModelInfo.mobileBase.stateDim;
        robotModelInfo.modeInputDim = robotModelInfo.mobileBase.inputDim;
        result = true;
        break;

      case 1:
        robotModelInfo.modelMode = ModelMode::ArmMotion;
        robotModelInfo.modeStateDim = robotModelInfo.robotArm.stateDim;
        robotModelInfo.modeInputDim = robotModelInfo.robotArm.inputDim;
        result = true;
        break;

      case 2:
        robotModelInfo.modelMode = ModelMode::WholeBodyMotion;
        robotModelInfo.modeStateDim = robotModelInfo.mobileBase.stateDim + robotModelInfo.robotArm.stateDim;
        robotModelInfo.modeInputDim = robotModelInfo.mobileBase.inputDim + robotModelInfo.robotArm.inputDim;
        result = true;
        break;

      default:
        std::cerr << "[FactoryFunction::updateModelMode] ERROR: Invalid manipulator model type!";
        break;
    }
  }
  return result;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RobotModelType loadRobotType(const std::string& configFilePath, const std::string& fieldName) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<RobotModelType>(type);
}

//}  // namespace mobile_manipulator
}  // namespace ocs2