// LAST UPDATE: 2023.07.12
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

  while(!tflistener.waitForTransform(world_frame_name, base_frame_name, ros::Time(0), ros::Duration(5.0))){ros::spinOnce();}

  try
  {
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

  std::cout << "[FactoryFunctions::createPinocchioInterface(5)] base_frame_name: " << base_frame_name << std::endl;
  std::cout << "[FactoryFunctions::createPinocchioInterface(5)] world_frame_name: " << world_frame_name << std::endl;
  std::cout << "[FactoryFunctions::createPinocchioInterface(5)] Waiting for the transformation between base and world..." << std::endl;
  try
  {
    while(!tflistener.waitForTransform(world_frame_name, base_frame_name, ros::Time(0), ros::Duration(1.0))){ros::spinOnce();}
    tflistener.lookupTransform(world_frame_name, base_frame_name, ros::Time(0), transform_base_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[FactoryFunctions::createPinocchioInterface(5)] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
    while(1);
  }
  
  //se3_base_wrt_world.translation().x() = transform_base_wrt_world.getOrigin().x();
  se3_base_wrt_world.translation().x() = 0;
  //se3_base_wrt_world.translation().y() = transform_base_wrt_world.getOrigin().y();
  se3_base_wrt_world.translation().y() = 0;
  se3_base_wrt_world.translation().z() = transform_base_wrt_world.getOrigin().z();
  //se3_base_wrt_world.translation().z() = 0;


  //std::cout << "[FactoryFunctions::createPinocchioInterface(5)] x: " << se3_base_wrt_world.translation().x() << std::endl;
  //std::cout << "[FactoryFunctions::createPinocchioInterface(5)] y: " << se3_base_wrt_world.translation().y() << std::endl;
  //std::cout << "[FactoryFunctions::createPinocchioInterface(5)] z: " << se3_base_wrt_world.translation().z() << std::endl;
  
  //std::cout << "[FactoryFunctions::createPinocchioInterface(5)] DEBUG INF" << std::endl;
  //while(1);

  //tf::matrixTFToEigen(tf::Matrix3x3(transform_base_wrt_world.getRotation()), se3_base_wrt_world.rotation());

  se3_base_wrt_world.rotation().setIdentity();

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
      //return getPinocchioInterfaceFromUrdfModel(newModel, se3_base_wrt_world);

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
/*
RobotModelType loadRobotType(const std::string& configFilePath, const std::string& fieldName) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<RobotModelType>(type);
}
*/

}  // namespace mobile_manipulator
}  // namespace ocs2