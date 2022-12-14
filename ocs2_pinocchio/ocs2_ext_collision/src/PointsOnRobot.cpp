/*
 * Copyright (c) 2020 Johannes Pankert <pankertj@ethz.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of this work nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pinocchio/fwd.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_ext_collision/PointsOnRobot.h>

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PointsOnRobot::PointsOnRobot(const PointsOnRobot::points_radii_t& points_radii)
  : points_(), radii_()
{
  std::cout << "[PointsOnRobot::PointsOnRobot] START" << std::endl;

  const auto& pointsAndRadii = points_radii;

  int numPoints = 0;
  for (const auto& segments : pointsAndRadii) 
  {
    std::vector<double> pointsOnSegment;
    for (const auto& pointRadius : segments) 
    {
      pointsOnSegment.push_back(pointRadius.first);
      numPoints++;
    }
    points_.push_back(pointsOnSegment);
  }

  int idx = 0;
  radii_ = Eigen::VectorXd(numPoints);
  for (const auto& segments : pointsAndRadii) 
  {
    for (const auto& pointRadius : segments) 
    {
      radii_(idx++) = pointRadius.second;
    }
  }

  std::cout << "[PointsOnRobot::PointsOnRobot] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_),
    cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)),
    base_link_name_(rhs.base_link_name_),
    arm_mount_link_name_(rhs.arm_mount_link_name_),
    tool_link_name_(rhs.tool_link_name_),
    ee_link_name_(rhs.ee_link_name_),
    transform_Base_wrt_World_(rhs.transform_Base_wrt_World_),
    transform_ArmMount_wrt_Base_(rhs.transform_ArmMount_wrt_Base_),
    transform_J1_wrt_ArmMount_(rhs.transform_J1_wrt_ArmMount_),
    transform_J2_wrt_J1_(rhs.transform_J2_wrt_J1_),
    transform_J3_wrt_J2_(rhs.transform_J3_wrt_J2_),
    transform_J4_wrt_J3_(rhs.transform_J4_wrt_J3_),
    transform_J5_wrt_J4_(rhs.transform_J5_wrt_J4_),
    transform_J6_wrt_J5_(rhs.transform_J6_wrt_J5_),
    transform_ToolMount_wrt_J6_(rhs.transform_ToolMount_wrt_J6_),
    dofParentLinkIds_(rhs.dofParentLinkIds_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::initialize(ocs2::PinocchioInterface& pinocchioInterface,
                               const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                               const std::string& modelName, 
                               const std::string& modelFolder, 
                               bool recompileLibraries, 
                               bool verbose,
                               const std::string& base_link_name, 
                               const std::string& arm_mount_link_name, 
                               const std::string& tool_link_name, 
                               const std::string& ee_link_name,
                               const std::vector<std::string>& dofParentLinkNames)
{
  base_link_name_ = base_link_name;
  arm_mount_link_name_ = arm_mount_link_name; 
  tool_link_name_ = tool_link_name; 
  ee_link_name_ = ee_link_name;
  dofParentLinkNames_ = dofParentLinkNames;
  dofParentLinkNames_.push_back(tool_link_name);

  // Set link Ids of joints
  for (const auto& bodyName : dofParentLinkNames_) 
  {
    dofParentLinkIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }
  dofParentLinkIds_.push_back(pinocchioInterface.getModel().getBodyId(tool_link_name));

  // Set fixed transforms
  setFixedTransforms();

  // CppAD interface
  ocs2::PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // pinocchioInterface to mapping
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  setADInterfaces(pinocchioInterfaceCppAd, *mappingCppAdPtr, modelName, modelFolder);

  if (recompileLibraries) 
  {
    createModels(verbose);
  } 
  else 
  {
    loadModelsIfAvailable(verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::getPointsPositions(ocs2::PinocchioInterface& pinocchioInterface, 
                                                  const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                  const Eigen::VectorXd& state) const
{
  std::cout << "[PointsOnRobot::getPointPosition] START" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] state:" << std::endl << state << std::endl;
  //updateTransforms(pinocchioInterface, mapping, state);
  
  const ocs2::scalar_t yaw = state[2];
  const ocs2::scalar_t pitch = 0.0;
  const ocs2::scalar_t roll = 0.0;

  Eigen::Quaternion<ocs2::scalar_t> baseOrientation = EulerToQuaternion(yaw, pitch, roll);
  Eigen::Matrix<ocs2::scalar_t, 3, 1> basePosition;
  basePosition << state[0], state[1], 0.0;

  transform_Base_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_Base_wrt_World_.topLeftCorner(3,3) = baseOrientation.toRotationMatrix();
  transform_Base_wrt_World_.topRightCorner(3,1) = basePosition;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_Base_wrt_World_: " << std::endl << transform_Base_wrt_World_ << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_ArmMount_wrt_Base_: " << std::endl << transform_ArmMount_wrt_Base_ << std::endl;

  updateTransforms();

  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = mapping.getPinocchioJointPosition(state);

  //std::cout << "[PointsOnRobot::getPointPosition] q:" << std::endl << q << std::endl;

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  Eigen::Matrix4d transform_J1_wrt_ArmMount_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J1_wrt_ArmMount_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[0]].rotation();
  transform_J1_wrt_ArmMount_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[0]].translation();

  //std::cout << "[PointsOnRobot::getPointsPositions] transform_Base_wrt_World_: " << std::endl << transform_Base_wrt_World_ << std::endl;
  //std::cout << "[PointsOnRobot::getPointsPositions] transform_ArmMount_wrt_Base_: " << std::endl << transform_ArmMount_wrt_Base_ << std::endl;
  //std::cout << "[PointsOnRobot::getPointsPositions] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] dofParentLinkIds_[0]: " << dofParentLinkNames_[0] << std::endl;
  std::cout << "[PointsOnRobot::getPointsPositions] dofParentLinkIds_[0]: " << dofParentLinkIds_[0] << std::endl;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J1_wrt_ArmMount_pin: " << std::endl << transform_J1_wrt_ArmMount_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J1_wrt_ArmMount_test = transform_Base_wrt_World_ * transform_ArmMount_wrt_Base_ * transform_J1_wrt_ArmMount_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J1_wrt_ArmMount_test: " << std::endl << transform_J1_wrt_ArmMount_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  std::cout << "[PointsOnRobot::getPointsPositions] dofParentLinkIds_[1]: " << dofParentLinkNames_[1] << std::endl;
  std::cout << "[PointsOnRobot::getPointsPositions] dofParentLinkIds_[1]: " << dofParentLinkIds_[1] << std::endl;
  Eigen::Matrix4d transform_J2_wrt_J1_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J2_wrt_J1_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[1]].rotation();
  transform_J2_wrt_J1_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[1]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J2_wrt_J1_pin: " << std::endl << transform_J2_wrt_J1_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J2_wrt_J1_test = transform_J1_wrt_ArmMount_test * transform_J2_wrt_J1_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J2_wrt_J1_test: " << std::endl << transform_J2_wrt_J1_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  Eigen::Matrix4d transform_J3_wrt_J2_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J3_wrt_J2_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[2]].rotation();
  transform_J3_wrt_J2_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[2]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J3_wrt_J2_pin: " << std::endl << transform_J3_wrt_J2_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J3_wrt_J2_test = transform_J2_wrt_J1_test * transform_J3_wrt_J2_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J3_wrt_J2_test: " << std::endl << transform_J3_wrt_J2_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  Eigen::Matrix4d transform_J4_wrt_J3_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J4_wrt_J3_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[3]].rotation();
  transform_J4_wrt_J3_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[3]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J4_wrt_J3_pin: " << std::endl << transform_J4_wrt_J3_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J4_wrt_J3_test = transform_J3_wrt_J2_test * transform_J4_wrt_J3_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J4_wrt_J3_test: " << std::endl << transform_J4_wrt_J3_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  Eigen::Matrix4d transform_J5_wrt_J4_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J5_wrt_J4_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[4]].rotation();
  transform_J5_wrt_J4_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[4]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J5_wrt_J4_pin: " << std::endl << transform_J5_wrt_J4_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J5_wrt_J4_test = transform_J4_wrt_J3_test * transform_J5_wrt_J4_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J5_wrt_J4_test: " << std::endl << transform_J5_wrt_J4_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  Eigen::Matrix4d transform_J6_wrt_J5_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J6_wrt_J5_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[5]].rotation();
  transform_J6_wrt_J5_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[5]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_J6_wrt_J5_pin: " << std::endl << transform_J6_wrt_J5_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_J6_wrt_J5_test = transform_J5_wrt_J4_test * transform_J6_wrt_J5_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_J6_wrt_J5_test: " << std::endl << transform_J6_wrt_J5_test << std::endl;
  std::cout << "==========================================" << std::endl;

  ///////////////////
  Eigen::Matrix4d transform_ToolMount_wrt_J6_pin = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ToolMount_wrt_J6_pin.topLeftCorner(3,3) = data.oMf[dofParentLinkIds_[6]].rotation();
  transform_ToolMount_wrt_J6_pin.topRightCorner(3,1) = data.oMf[dofParentLinkIds_[6]].translation();

  std::cout << "[PointsOnRobot::getPointsPositions] transform_ToolMount_wrt_World_: " << std::endl << transform_ToolMount_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  std::cout << "[PointsOnRobot::getPointsPositions] transform_ToolMount_wrt_J6_pin: " << std::endl << transform_ToolMount_wrt_J6_pin << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  Eigen::Matrix4d transform_ToolMount_wrt_J6_test = transform_J6_wrt_J5_test * transform_ToolMount_wrt_J6_;
  std::cout << "[PointsOnRobot::getPointsPositions] transform_ToolMount_wrt_J6_test: " << std::endl << transform_ToolMount_wrt_J6_test << std::endl;
  std::cout << "==========================================" << std::endl;

  /*
  std::cout << "[PointsOnRobot::updateTransforms] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_J4_wrt_J3_: " << std::endl << transform_J4_wrt_J3_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_J5_wrt_J4_: " << std::endl << transform_J5_wrt_J4_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_J6_wrt_J5_: " << std::endl << transform_J6_wrt_J5_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms] transform_ToolMount_wrt_J6_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  */

  std::cout << "[PointsOnRobot::getPointPosition] END" << std::endl;
  return cppAdInterface_->getFunctionValue(state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::MatrixXd PointsOnRobot::getPointsJacobian(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getJacobian(state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::getRadii() const 
{
  return radii_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int PointsOnRobot::getNumOfPoints() const 
{
  return radii_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int PointsOnRobot::getDimPoints() const
{
  int dim = 0;
  for (int i = 0; i < points_.size(); i++) 
  {
    for (int j = 0; j < points_[i].size(); j++) 
    {
      dim++;
    }
  }
  return dim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::MarkerArray PointsOnRobot::getVisualization(ocs2::PinocchioInterface& pinocchioInterface, 
                                                                const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                                const Eigen::VectorXd& state) const 
{
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(radii_.size());

  Eigen::VectorXd points = getPointsPositions(pinocchioInterface, mapping, state);

  for (int i = 0; i < markerArray.markers.size(); i++) 
  {
    auto& marker = markerArray.markers[i];
    marker.type = visualization_msgs::Marker::Type::SPHERE;
    marker.id = i;
    marker.action = 0;
    marker.scale.x = radii_[i] * 2;
    marker.scale.y = radii_[i] * 2;
    marker.scale.z = radii_[i] * 2;
    marker.pose.position.x = points(3 * i + 0);
    marker.pose.position.y = points(3 * i + 1);
    marker.pose.position.z = points(3 * i + 2);

    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.color.a = 0.5;
    marker.color.r = 0.5;
    marker.color.b = 0.0;
    marker.color.g = 0.0;

    marker.frame_locked = true;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
  }
  return markerArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Quaternion<ocs2::scalar_t> PointsOnRobot::EulerToQuaternion(const ocs2::scalar_t& yaw, 
                                                                   const ocs2::scalar_t& pitch, 
                                                                   const ocs2::scalar_t& roll) const
{
  Eigen::Quaternion<ocs2::scalar_t> quat;
  quat = Eigen::AngleAxis<ocs2::scalar_t>(roll, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxis<ocs2::scalar_t>(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxis<ocs2::scalar_t>(yaw, Eigen::Vector3d::UnitZ());
  
  std::cout << "[PointsOnRobot::EulerToQuaternion] Quaternion" << std::endl << quat.coeffs() << std::endl;

  return quat;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<ocs2::scalar_t, 3, 1> PointsOnRobot::QuaternionToEuler(Eigen::Quaternion<ocs2::scalar_t>& quat) const
{
  Eigen::Matrix<ocs2::scalar_t, 3, 1> euler;
  euler = quat.toRotationMatrix().eulerAngles(3,2,1);
  
  std::cout << "[PointsOnRobot::EulerToQuaternion] Euler from quaternion in yaw, pitch, roll"<< std::endl << euler << std::endl;

  return euler;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::setFixedTransforms()
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      while(!tfBuffer.canTransform(base_link_name_, arm_mount_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(base_link_name_, arm_mount_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ArmMount_wrt_Base_ = Eigen::Matrix4d::Identity();
    transform_ArmMount_wrt_Base_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ArmMount_wrt_Base_(0, 3) = transformStamped.transform.translation.x;
    transform_ArmMount_wrt_Base_(1, 3) = transformStamped.transform.translation.y;
    transform_ArmMount_wrt_Base_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransforms] baseToArmMount_: " << std::endl << transform_ArmMount_wrt_Base_ << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      while(!tfBuffer.canTransform(tool_link_name_, ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(tool_link_name_, ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransforms] wrist2ToEETransform_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  }
}

void PointsOnRobot::updateTransforms() const
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] arm_mount_link_name_: " << arm_mount_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[0]: " << dofParentLinkNames_[0] << std::endl;
      while(!tfBuffer.canTransform(arm_mount_link_name_, dofParentLinkNames_[0], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(arm_mount_link_name_, dofParentLinkNames_[0], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J1_wrt_ArmMount_ = Eigen::Matrix4d::Identity();
    transform_J1_wrt_ArmMount_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J1_wrt_ArmMount_(0, 3) = transformStamped.transform.translation.x;
    transform_J1_wrt_ArmMount_(1, 3) = transformStamped.transform.translation.y;
    transform_J1_wrt_ArmMount_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  
  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[0]: " << dofParentLinkNames_[1] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[0], dofParentLinkNames_[1], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[0], dofParentLinkNames_[1], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J2_wrt_J1_ = Eigen::Matrix4d::Identity();
    transform_J2_wrt_J1_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J2_wrt_J1_(0, 3) = transformStamped.transform.translation.x;
    transform_J2_wrt_J1_(1, 3) = transformStamped.transform.translation.y;
    transform_J2_wrt_J1_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[2]: " << dofParentLinkNames_[2] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[1], dofParentLinkNames_[2], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[1], dofParentLinkNames_[2], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J3_wrt_J2_ = Eigen::Matrix4d::Identity();
    transform_J3_wrt_J2_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J3_wrt_J2_(0, 3) = transformStamped.transform.translation.x;
    transform_J3_wrt_J2_(1, 3) = transformStamped.transform.translation.y;
    transform_J3_wrt_J2_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[3]: " << dofParentLinkNames_[3] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[2], dofParentLinkNames_[3], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[2], dofParentLinkNames_[3], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J4_wrt_J3_ = Eigen::Matrix4d::Identity();
    transform_J4_wrt_J3_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J4_wrt_J3_(0, 3) = transformStamped.transform.translation.x;
    transform_J4_wrt_J3_(1, 3) = transformStamped.transform.translation.y;
    transform_J4_wrt_J3_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J4_wrt_J3_: " << std::endl << transform_J4_wrt_J3_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[4]: " << dofParentLinkNames_[4] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[3], dofParentLinkNames_[4], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[3], dofParentLinkNames_[4], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J5_wrt_J4_ = Eigen::Matrix4d::Identity();
    transform_J5_wrt_J4_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J5_wrt_J4_(0, 3) = transformStamped.transform.translation.x;
    transform_J5_wrt_J4_(1, 3) = transformStamped.transform.translation.y;
    transform_J5_wrt_J4_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J5_wrt_J4_: " << std::endl << transform_J5_wrt_J4_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[5]: " << dofParentLinkNames_[5] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[4], dofParentLinkNames_[5], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[4], dofParentLinkNames_[5], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J6_wrt_J5_ = Eigen::Matrix4d::Identity();
    transform_J6_wrt_J5_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J6_wrt_J5_(0, 3) = transformStamped.transform.translation.x;
    transform_J6_wrt_J5_(1, 3) = transformStamped.transform.translation.y;
    transform_J6_wrt_J5_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_J6_wrt_J5_: " << std::endl << transform_J6_wrt_J5_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[6]: " << dofParentLinkNames_[6] << std::endl;
      while(!tfBuffer.canTransform(dofParentLinkNames_[5], dofParentLinkNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(dofParentLinkNames_[5], dofParentLinkNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_ToolMount_wrt_J6_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] dofParentLinkNames_[6]: " << dofParentLinkNames_[6] << std::endl;
      while(!tfBuffer.canTransform("world", dofParentLinkNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", dofParentLinkNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ToolMount_wrt_World_ = Eigen::Matrix4d::Identity();
    transform_ToolMount_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ToolMount_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    transform_ToolMount_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    transform_ToolMount_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] transform_ToolMount_wrt_World_: " << std::endl << transform_ToolMount_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::updateTransforms(ocs2::PinocchioInterface& pinocchioInterface,
                                     const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                     const Eigen::VectorXd& state) const
{
  
  std::cout << "[PointsOnRobot::updateTransforms] NOWAY START" << std::endl;
  /*
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  transform_J1_wrt_ArmMount_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J1_wrt_ArmMount_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[0]].rotation();
  transform_J1_wrt_ArmMount_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[0]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  transform_J2_wrt_J1_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J2_wrt_J1_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[1]].rotation();
  transform_J2_wrt_J1_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[1]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  
  transform_J3_wrt_J2_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J3_wrt_J2_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[2]].rotation();
  transform_J3_wrt_J2_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[2]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  transform_J4_wrt_J3_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J4_wrt_J3_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[3]].rotation();
  transform_J4_wrt_J3_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[3]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J4_wrt_J3_: " << std::endl << transform_J4_wrt_J3_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  transform_J5_wrt_J4_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J5_wrt_J4_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[4]].rotation();
  transform_J5_wrt_J4_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[4]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J5_wrt_J4_: " << std::endl << transform_J5_wrt_J4_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  transform_J6_wrt_J5_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J6_wrt_J5_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[5]].rotation();
  transform_J6_wrt_J5_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[5]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_J6_wrt_J5_: " << std::endl << transform_J6_wrt_J5_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  transform_ToolMount_wrt_J6_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ToolMount_wrt_J6_.topLeftCorner(3,3) = data.liMi[dofParentLinkIds_[6]].rotation();
  transform_ToolMount_wrt_J6_.topRightCorner(3,1) = data.liMi[dofParentLinkIds_[6]].translation();

  std::cout << "[PointsOnRobot::updateTransforms] transform_ToolMount_wrt_J6_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  */

  std::cout << "[PointsOnRobot::updateTransforms] NOWAY END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<ocs2::scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobot(ocs2::PinocchioInterface& pinocchioInterface,
                                                                               const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                                               const Eigen::Matrix<ocs2::scalar_t, -1, 1>& state) const
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

  int dim = getDimPoints();
  if (dim == 0) 
  {
    return Eigen::Matrix<ocs2::scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ocs2::scalar_t, 3, -1> points_on_robot(3, dim);
  
  updateTransforms(pinocchioInterface, mapping, state);

  /*
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const PointsOnRobot::ad_dynamic_vector_t q = mappingCppAd.getPinocchioJointPosition(stateCppAd);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  */

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ocs2::scalar_t, 4, 4> transform_tmp = transform_Base_wrt_World_;

  // Points between base and arm mount
  Eigen::Matrix<ocs2::scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.block<3,1>(0,0) = directionVector.block<3,1>(0,0) * points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).block<3,1>(0,0);
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  /*
  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transformBase_X_ArmMount_.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[0]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[0]].translation();
  next_transform = transform_tmp.transpose() * next_transform;
  for (int i = 0; i < points[linkIndex].size(); i++)
  {
    std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);

    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;
  */

  return points_on_robot;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                                                                const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                                                const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

  int dim = getDimPoints();
  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ad_scalar_t, 3, -1> points_on_robot(3, dim);
  
  //updateTransforms(pinocchioInterface, mapping, stateCppAd);

  /*
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const PointsOnRobot::ad_dynamic_vector_t q = mappingCppAd.getPinocchioJointPosition(stateCppAd);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  */

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_ArmMount_cppAd = transform_J1_wrt_ArmMount_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_J1_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_J2_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_J3_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_J4_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_J5_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_J6_cppAd = transform_ToolMount_wrt_J6_.cast<ad_scalar_t>();

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_tmp = transform_J1_wrt_ArmMount_cppAd;

  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transform_J1_wrt_ArmMount_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  /*
  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transformBase_X_ArmMount_.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[0]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[0]].translation();
  next_transform = transform_tmp.transpose() * next_transform;
  for (int i = 0; i < points[linkIndex].size(); i++)
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);

    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;
  */

  return points_on_robot;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::setADInterfaces(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                    const std::string& modelName,
                                    const std::string& modelFolder)
{
  using ad_interface = ocs2::CppAdInterface;
  using ad_dynamic_vector_t = ad_interface::ad_vector_t;
  using ad_scalar_t = ad_interface::ad_scalar_t;

  int numPoints = 0;
  for (int i = 0; i < points_.size(); i++) 
  {
    numPoints += points_[i].size();
  }
  assert(numPoints == radii_.size());

  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;

  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixPointsOnRobot = computeState2PointsOnRobotCppAd(pinocchioInterfaceAd, mappingCppAd, x);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixPointsOnRobot.data(), matrixPointsOnRobot.size());
  };

  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::createModels(bool verbose) 
{
  cppAdInterface_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::loadModelsIfAvailable(bool verbose) 
{
  cppAdInterface_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}
