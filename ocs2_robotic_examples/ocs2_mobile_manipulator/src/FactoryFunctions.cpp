/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type) 
{
  switch (type) 
  {
    case ManipulatorModelType::DefaultManipulator: 
    {
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
    }

    case ManipulatorModelType::FloatingArmManipulator: 
    {
      // add 6 DoF for the floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }

    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: 
    {
      // add 6 DOF for the fully-actuated free-floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }

    case ManipulatorModelType::WheelBasedMobileManipulator: 
    {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, 
                                            const ManipulatorModelType& type,
                                            const std::vector<std::string>& jointNames) 
{
  std::cout << "[FactoryFunctions::createPinocchioInterface(3)] START" << std::endl;

  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  // parse the URDF
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) 
  {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) 
    {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }

  // resolve for the robot type
  switch (type) 
  {
    case ManipulatorModelType::DefaultManipulator: 
    {
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel);
    }

    case ManipulatorModelType::FloatingArmManipulator: 
    {
      // add 6 DoF for the floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
    }

    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: 
    {
      // add 6 DOF for the free-floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      
      // return pinocchio interface
      return getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }

    case ManipulatorModelType::WheelBasedMobileManipulator: 
    {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());

      std::string world_frame_name = "world";
      std::string base_frame_name = "base_link";
      tf::TransformListener tflistener;
      tf::StampedTransform transform_base_wrt_world;
      pinocchio::SE3 se3_base_wrt_world = pinocchio::SE3::Identity();

      /*
      try
      {
        tflistener.waitForTransform(world_frame_name, base_frame_name, ros::Time(0), ros::Duration(10.0));
        tflistener.lookupTransform(world_frame_name, base_frame_name, ros::Time(0), transform_base_wrt_world);
      }
      catch (tf::TransformException ex)
      {
        ROS_INFO("[FactoryFunctions::createPinocchioInterface(3)] Couldn't get transform!");
        ROS_ERROR("%s", ex.what());

        while(1);
      }
      
      se3_base_wrt_world.translation().x() = transform_base_wrt_world.getOrigin().x();
      se3_base_wrt_world.translation().y() = transform_base_wrt_world.getOrigin().y();
      se3_base_wrt_world.translation().z() = transform_base_wrt_world.getOrigin().z();
      */

      se3_base_wrt_world.translation().z() = 0.06;

			//tf::matrixTFToEigen(tf::Matrix3x3(transform_base_wrt_world.getRotation()), se3_base_wrt_world.rotation());

      std::cout << "[FactoryFunctions::createPinocchioInterface(3)] se3_base_wrt_world:" << std::endl;
      std::cout << "translation x: " << se3_base_wrt_world.translation().x() << std::endl;
      std::cout << "translation y: " << se3_base_wrt_world.translation().y() << std::endl;
      std::cout << "translation z: " << se3_base_wrt_world.translation().z() << std::endl;
      //std::cout << "rotation x: " << se3_base_wrt_world.rotation().x() << std::endl;
      //std::cout << "rotation y: " << se3_base_wrt_world.rotation().y() << std::endl;
      //std::cout << "rotation z: " << se3_base_wrt_world.rotation().z() << std::endl;
      //std::cout << "rotation w: " << se3_base_wrt_world.rotation().w() << std::endl;

      std::cout << "[FactoryFunctions::createPinocchioInterface(3)] END WheelBasedMobileManipulator" << std::endl;

      // return pinocchio interface
      //return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
      return getPinocchioInterfaceFromUrdfModel(robotUrdfPath, jointComposite, se3_base_wrt_world, "offset_link");
    }
    

    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelInfo createManipulatorModelInfo(const PinocchioInterface& interface, 
                                                const ManipulatorModelType& type,
                                                const std::string& baseFrame, 
                                                const std::string& armBaseFrame,
                                                const std::string& eeFrame, 
                                                const std::vector<std::string>& jointParentFrameNames) 
{
  const auto& model = interface.getModel();

  ManipulatorModelInfo info;
  info.manipulatorModelType = type;
  info.stateDim = model.nq;
  
  // resolve for actuated dof based on type of robot
  switch (type) 
  {
    case ManipulatorModelType::DefaultManipulator: 
    {
      // for default arm, the state dimension and input dimensions are same.
      info.inputDim = info.stateDim;
      info.armDim = info.inputDim;
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: 
    {
      // remove the static 6-DOF base joints that are unactuated.
      info.inputDim = info.stateDim - 6;
      info.armDim = info.inputDim;
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: 
    {
      // all states are actuatable
      info.inputDim = info.stateDim;
      info.armDim = info.inputDim - 6;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: 
    {
      // for wheel-based, the input dimension is (v, omega, dq_j) while state dimension is (x, y, psi, q_j).
      info.inputDim = info.stateDim - 1;
      info.armDim = info.inputDim - 2;
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }
  
  // store frame names for using later.
  info.baseFrame = baseFrame;
  info.armBaseFrame = armBaseFrame;
  info.eeFrame = eeFrame;
  info.jointParentFrameNames = jointParentFrameNames;

  // get name of arm joints.
  const auto& jointNames = model.names;
  info.dofNames = std::vector<std::string>(jointNames.end() - info.armDim, jointNames.end());

  return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelType loadManipulatorType(const std::string& configFilePath, const std::string& fieldName) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<ManipulatorModelType>(type);
}

}  // namespace mobile_manipulator
}  // namespace ocs2