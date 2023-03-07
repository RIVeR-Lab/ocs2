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

#include "ocs2_mobile_manipulator/FactoryFunctions.h"

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const RobotModelType& robotModelType,
                                            std::string world_frame_name,
                                            std::string base_frame_name) 
{
  tf::TransformListener tflistener;
  tf::StampedTransform transform_base_wrt_world;
  pinocchio::SE3 se3_base_wrt_world = pinocchio::SE3::Identity();

  try
  {
    while(!tflistener.waitForTransform(world_frame_name, base_frame_name, ros::Time(0), ros::Duration(10.0)));
    tflistener.lookupTransform(world_frame_name, base_frame_name, ros::Time(0), transform_base_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[FactoryFunctions::createPinocchioInterface(4)] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
    while(1);
  }
  
  se3_base_wrt_world.translation().x() = transform_base_wrt_world.getOrigin().x();
  se3_base_wrt_world.translation().y() = transform_base_wrt_world.getOrigin().y();
  se3_base_wrt_world.translation().z() = transform_base_wrt_world.getOrigin().z();

  tf::matrixTFToEigen(tf::Matrix3x3(transform_base_wrt_world.getRotation()), se3_base_wrt_world.rotation());

  //// NUA TODO: NONE OF THESE FUNCTIONS ARE TESTED YET!
  switch (robotModelType) 
  {
    //// NUA TODO: THIS IS NOT TESTED YET!
    case RobotModelType::MobileBase: 
    {
      // Add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      //// NUA TODO: THIS NEEDS TO BE TESTED WITH A URDF WITH ONLY ROBOT BASE!
      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite, se3_base_wrt_world);
    }

    case RobotModelType::RobotArm: 
    {
      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, se3_base_wrt_world);
    }

    case RobotModelType::MobileManipulator: 
    {
      // Add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite, se3_base_wrt_world);
    }
    
    default:
      throw std::invalid_argument("[FactoryFunctions::createPinocchioInterface(4)] ERROR: Invalid robot model type!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath,
                                            const RobotModelType& robotModelType,
                                            const std::vector<std::string>& removeJointNames,
                                            std::string world_frame_name,
                                            std::string base_frame_name) 
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  // Parse the URDF
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  
  // Remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) 
  {
    if (std::find(removeJointNames.begin(), removeJointNames.end(), jointPair.first) != removeJointNames.end()) 
    {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }

  tf::TransformListener tflistener;
  tf::StampedTransform transform_base_wrt_world;
  pinocchio::SE3 se3_base_wrt_world = pinocchio::SE3::Identity();

  try
  {
    while(!tflistener.waitForTransform(world_frame_name, base_frame_name, ros::Time(0), ros::Duration(10.0)));
    tflistener.lookupTransform(world_frame_name, base_frame_name, ros::Time(0), transform_base_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[FactoryFunctions::createPinocchioInterface(5)] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
    while(1);
  }
  
  se3_base_wrt_world.translation().x() = transform_base_wrt_world.getOrigin().x();
  se3_base_wrt_world.translation().y() = transform_base_wrt_world.getOrigin().y();
  se3_base_wrt_world.translation().z() = transform_base_wrt_world.getOrigin().z();

  tf::matrixTFToEigen(tf::Matrix3x3(transform_base_wrt_world.getRotation()), se3_base_wrt_world.rotation());

  // resolve for the robot type
  switch (robotModelType) 
  {
    case RobotModelType::MobileBase: 
    {
      // Add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      //// NUA TODO: THIS NEEDS TO BE TESTED WITH A URDF WITH ONLY ROBOT BASE!
      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, se3_base_wrt_world);
    }

    case RobotModelType::RobotArm: 
    {
      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, se3_base_wrt_world);
    }

    case RobotModelType::MobileManipulator: 
    {
      // Add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      // Return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite, se3_base_wrt_world);
    }
    
    default:
      throw std::invalid_argument("[FactoryFunctions::createPinocchioInterface(5)] ERROR: Invalid robot model type!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RobotModelInfo createRobotModelInfo(const PinocchioInterface& interface, 
                                    const RobotModelType& robotModelType,
                                    const std::string& baseFrame, 
                                    const std::string& armBaseFrame,
                                    const std::string& eeFrame, 
                                    const std::vector<std::string>& armJointFrameNames,
                                    const std::vector<std::string>& armJointNames) 
{
  const auto& model = interface.getModel();
  RobotModelInfo info;
  info.robotModelType = robotModelType;
  
  /////// NUA TODO: ADD 6 DOF BASE VERSION!  
  switch (robotModelType) 
  {
    case RobotModelType::MobileBase:
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] MobileBase" << std::endl;

      info.modelMode = ModelMode::BaseMotion;
      info.mobileBase.baseFrame = baseFrame;
      info.mobileBase.stateDim = 3;
      info.mobileBase.inputDim = 2;
      info.robotArm.baseFrame = "";
      info.robotArm.eeFrame = "";
      info.robotArm.stateDim = 0;
      info.robotArm.inputDim = 0;
      break;
    }

    case RobotModelType::RobotArm: 
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] RobotArm" << std::endl;

      info.modelMode = ModelMode::ArmMotion;
      info.mobileBase.baseFrame = "";
      info.mobileBase.stateDim = 0;
      info.mobileBase.inputDim = 0;
      info.robotArm.baseFrame = armBaseFrame;
      info.robotArm.eeFrame = eeFrame;
      info.robotArm.jointFrameNames = armJointFrameNames;
      info.robotArm.jointNames = armJointNames;
      //info.robotArm.jointFrameNames = model.names;
      info.robotArm.stateDim = model.nq;
      info.robotArm.inputDim = model.njoints;
      break;
    }
      
    case RobotModelType::MobileManipulator: 
    {
      std::cout << "[FactoryFunction::createRobotModelInfo] MobileManipulator" << std::endl;
      info.modelMode = ModelMode::WholeBodyMotion;
      info.mobileBase.baseFrame = baseFrame;
      info.mobileBase.stateDim = 3;
      info.mobileBase.inputDim = 2;
      info.robotArm.baseFrame = armBaseFrame;
      info.robotArm.eeFrame = eeFrame;
      info.robotArm.jointFrameNames = armJointFrameNames;
      info.robotArm.jointNames = armJointNames;
      info.robotArm.stateDim = model.nq - info.mobileBase.stateDim;
      info.robotArm.inputDim = model.njoints - info.mobileBase.inputDim;
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
bool updateModelMode(RobotModelInfo& robotModelInfo, size_t& modelMode)
{
  bool result = false;
  if (robotModelInfo.robotModelType == RobotModelType::MobileManipulator)
  {
    switch (modelMode)
    {
      case 0:
        robotModelInfo.modelMode = ModelMode::BaseMotion;
        result = true;
        break;

      case 1:
        robotModelInfo.modelMode = ModelMode::ArmMotion;
        result = true;
        break;

      case 2:
        robotModelInfo.modelMode = ModelMode::WholeBodyMotion;
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

}  // namespace mobile_manipulator
}  // namespace ocs2