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

  ros::NodeHandle nh;
  nh_ = nh;
  pointsOnRobot_visu_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("points_on_robot", 100);

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
    ee_link_name_(rhs.ee_link_name_),
    frameNames_(rhs.frameNames_),
    frameIds_(rhs.frameIds_),
    transform_Base_wrt_World_(rhs.transform_Base_wrt_World_),
    transform_ArmMount_wrt_Base_(rhs.transform_ArmMount_wrt_Base_),
    transform_J1_wrt_ArmMount_(rhs.transform_J1_wrt_ArmMount_),
    transform_J2_wrt_J1_(rhs.transform_J2_wrt_J1_),
    transform_J3_wrt_J2_(rhs.transform_J3_wrt_J2_),
    transform_J4_wrt_J3_(rhs.transform_J4_wrt_J3_),
    transform_J5_wrt_J4_(rhs.transform_J5_wrt_J4_),
    transform_J6_wrt_J5_(rhs.transform_J6_wrt_J5_),
    transform_ToolMount_wrt_J6_(rhs.transform_ToolMount_wrt_J6_){}

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
                               const std::string& ee_link_name,
                               const std::vector<std::string>& jointParentFrameNames)
{
  base_link_name_ = base_link_name;
  arm_mount_link_name_ = arm_mount_link_name; 
  ee_link_name_ = ee_link_name;

  std::cout << "[PointsOnRobot::initialize] frameNames_: " << std::endl;
  frameNames_.push_back(base_link_name);
  frameNames_.push_back(arm_mount_link_name);
  for(auto dof_name : jointParentFrameNames)
  {
    frameNames_.push_back(dof_name);
  }
  frameNames_.push_back(ee_link_name);

  for (size_t i = 0; i < frameNames_.size(); i++)
  {
    std::cout << i << ": "  << frameNames_[i] << std::endl;
  }
  
  // Set link Ids of joints
  for (const auto& bodyName : frameNames_) 
  {
    auto id =  pinocchioInterface.getModel().getBodyId(bodyName);
    frameIds_.push_back(id);

    //std::cout << bodyName << " -> " << id << std::endl;
  }

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

  //std::cout << "[PointsOnRobot::getPointPosition] state:" << std::endl << state << std::endl;
  //updateTransforms(pinocchioInterface, mapping, state);
  
  //updateTransforms();
  //updateTransformsWorld();
  updateTransforms(pinocchioInterface, mapping, state);

  /*
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_Base_wrt_World_: " << std::endl << tf2_transform_Base_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_Base_wrt_World_: " << std::endl << transform_Base_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_ArmMount_wrt_World_: " << std::endl << tf2_transform_ArmMount_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_ArmMount_wrt_World_: " << std::endl << transform_ArmMount_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J1_wrt_World_: " << std::endl << tf2_transform_J1_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J1_wrt_World_: " << std::endl << transform_J1_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J2_wrt_World_: " << std::endl << tf2_transform_J2_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J2_wrt_World_: " << std::endl << transform_J2_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J3_wrt_World_: " << std::endl << tf2_transform_J3_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J3_wrt_World_: " << std::endl << transform_J3_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J4_wrt_World_: " << std::endl << tf2_transform_J4_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J4_wrt_World_: " << std::endl << transform_J4_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J5_wrt_World_: " << std::endl << tf2_transform_J5_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J5_wrt_World_: " << std::endl << transform_J5_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J6_wrt_World_: " << std::endl << tf2_transform_J6_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J6_wrt_World_: " << std::endl << transform_J6_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_ToolMount_wrt_World_: " << std::endl << tf2_transform_ToolMount_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_ToolMount_wrt_World_: " << std::endl << transform_ToolMount_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  */

  /*
  Eigen::Matrix4d diff_transform_Base_wrt_World = tf2_transform_Base_wrt_World_ - transform_Base_wrt_World_;
  Eigen::Matrix4d diff_transform_ArmMount_wrt_World = tf2_transform_ArmMount_wrt_World_ - transform_ArmMount_wrt_World_;
  Eigen::Matrix4d diff_transform_J1_wrt_World = tf2_transform_J1_wrt_World_ - transform_J1_wrt_World_;
  Eigen::Matrix4d diff_transform_J2_wrt_World = tf2_transform_J2_wrt_World_ - transform_J2_wrt_World_;
  Eigen::Matrix4d diff_transform_J3_wrt_World = tf2_transform_J3_wrt_World_ - transform_J3_wrt_World_;
  Eigen::Matrix4d diff_transform_J4_wrt_World = tf2_transform_J4_wrt_World_ - transform_J4_wrt_World_;
  Eigen::Matrix4d diff_transform_J5_wrt_World = tf2_transform_J5_wrt_World_ - transform_J5_wrt_World_;
  Eigen::Matrix4d diff_transform_J6_wrt_World = tf2_transform_J6_wrt_World_ - transform_J6_wrt_World_;
  Eigen::Matrix4d diff_transform_ToolMount_wrt_World = tf2_transform_ToolMount_wrt_World_ - transform_ToolMount_wrt_World_;

  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_Base_wrt_World: " << std::endl << diff_transform_Base_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_ArmMount_wrt_World: " << std::endl << diff_transform_ArmMount_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J1_wrt_World: " << std::endl << diff_transform_J1_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J2_wrt_World: " << std::endl << diff_transform_J2_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J3_wrt_World: " << std::endl << diff_transform_J3_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J4_wrt_World: " << std::endl << diff_transform_J4_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J5_wrt_World: " << std::endl << diff_transform_J5_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J6_wrt_World: " << std::endl << diff_transform_J6_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_ToolMount_wrt_World: " << std::endl << diff_transform_ToolMount_wrt_World << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  */

  /*
  Eigen::Matrix4d diff_transform_ArmMount_wrt_Base = tf2_transform_ArmMount_wrt_Base_ - transform_ArmMount_wrt_Base_;
  Eigen::Matrix4d diff_transform_J2_wrt_J1 = tf2_transform_J2_wrt_J1_ - transform_J2_wrt_J1_;
  Eigen::Matrix4d diff_transform_J3_wrt_J2 = tf2_transform_J3_wrt_J2_ - transform_J3_wrt_J2_;
  Eigen::Matrix4d diff_transform_J4_wrt_J3 = tf2_transform_J4_wrt_J3_ - transform_J4_wrt_J3_;
  Eigen::Matrix4d diff_transform_J5_wrt_J4 = tf2_transform_J5_wrt_J4_ - transform_J5_wrt_J4_;
  Eigen::Matrix4d diff_transform_J6_wrt_J5 = tf2_transform_J6_wrt_J5_ - transform_J6_wrt_J5_;
  Eigen::Matrix4d diff_transform_ToolMount_wrt_J6 = tf2_transform_ToolMount_wrt_J6_ - transform_ToolMount_wrt_J6_;

  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_ArmMount_wrt_Base: " << std::endl << diff_transform_ArmMount_wrt_Base << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J2_wrt_J1: " << std::endl << diff_transform_J2_wrt_J1 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J3_wrt_J2: " << std::endl << diff_transform_J3_wrt_J2 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J4_wrt_J3: " << std::endl << diff_transform_J4_wrt_J3 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J5_wrt_J4: " << std::endl << diff_transform_J5_wrt_J4 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_J6_wrt_J5: " << std::endl << diff_transform_J6_wrt_J5 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] diff_transform_ToolMount_wrt_J6: " << std::endl << diff_transform_ToolMount_wrt_J6 << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;

  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_ArmMount_wrt_Base_: " << std::endl << tf2_transform_ArmMount_wrt_Base_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_ArmMount_wrt_Base_: " << std::endl << transform_ArmMount_wrt_Base_ << std::endl;
  //std::cout << "--------------------&&--------------------" << std::endl;
  //std::cout << "[PointsOnRobot::getPointPosition] transform_Base_wrt_World_: " << std::endl << transform_Base_wrt_World_ << std::endl;
  //std::cout << "--------------------&&--------------------" << std::endl;
  //std::cout << "[PointsOnRobot::getPointPosition] transform_ArmMount_wrt_World_: " << std::endl << transform_ArmMount_wrt_World_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J1_wrt_ArmMount_: " << std::endl << tf2_transform_J1_wrt_ArmMount_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
  //std::cout << "--------------------&&--------------------" << std::endl;
  //std::cout << "[PointsOnRobot::getPointPosition] transform_J1_wrt_World_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J1: " << frameNames_[2] << std::endl;
  std::cout << "J2: " << frameNames_[3] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J2_wrt_J1_: " << std::endl << tf2_transform_J2_wrt_J1_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J2: " << frameNames_[3] << std::endl;
  std::cout << "J3: " << frameNames_[4] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J3_wrt_J2_: " << std::endl << tf2_transform_J3_wrt_J2_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J3: " << frameNames_[4] << std::endl;
  std::cout << "J4: " << frameNames_[5] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J4_wrt_J3_: " << std::endl << tf2_transform_J4_wrt_J3_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J4_wrt_J3_: " << std::endl << transform_J4_wrt_J3_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J4: " << frameNames_[5] << std::endl;
  std::cout << "J5: " << frameNames_[6] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J5_wrt_J4_: " << std::endl << tf2_transform_J5_wrt_J4_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J5_wrt_J4_: " << std::endl << transform_J5_wrt_J4_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J5: " << frameNames_[6] << std::endl;
  std::cout << "J6: " << frameNames_[7] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_J6_wrt_J5_: " << std::endl << tf2_transform_J6_wrt_J5_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_J6_wrt_J5_: " << std::endl << transform_J6_wrt_J5_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  std::cout << "J6: " << frameNames_[7] << std::endl;
  std::cout << "ToolMount: " << frameNames_[8] << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] tf2_transform_ToolMount_wrt_J6_: " << std::endl << tf2_transform_ToolMount_wrt_J6_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::getPointPosition] transform_ToolMount_wrt_J6_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  std::cout << "//////////////////////////////////////////" << std::endl;
  */

  std::cout << "[PointsOnRobot::getPointPosition] END" << std::endl;

  Eigen::VectorXd points_on_robot_cppAd = cppAdInterface_->getFunctionValue(state);

  return computeState2PointsOnRobot(pinocchioInterface, mapping, state);
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
void PointsOnRobot::getVisualization(ocs2::PinocchioInterface& pinocchioInterface, 
                                     const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                     const Eigen::VectorXd& state) const
{
  pointsOnRobot_visu_.markers.resize(radii_.size());

  Eigen::VectorXd points = getPointsPositions(pinocchioInterface, mapping, state);

  for (int i = 0; i < pointsOnRobot_visu_.markers.size(); i++) 
  {
    auto& marker = pointsOnRobot_visu_.markers[i];
    marker.type = visualization_msgs::Marker::Type::SPHERE;
    marker.id = i;
    marker.action = 0;
    marker.scale.x = radii_[i] * 1;
    marker.scale.y = radii_[i] * 1;
    marker.scale.z = radii_[i] * 1;
    marker.pose.position.x = points(3 * i + 0);
    marker.pose.position.y = points(3 * i + 1);
    marker.pose.position.z = points(3 * i + 2);

    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.color.a = 0.3;
    marker.color.r = 0.5;
    marker.color.b = 0.0;
    marker.color.g = 0.0;

    marker.frame_locked = true;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
  }

  pointsOnRobot_visu_pub_.publish(pointsOnRobot_visu_);
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
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      //std::cout << "[PointsOnRobot::setFixedTransforms] base_link_name_: " << base_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::setFixedTransforms] arm_mount_link_name_: " << arm_mount_link_name_ << std::endl;
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

    tf2_transform_ArmMount_wrt_Base_ = Eigen::Matrix4d::Identity();
    tf2_transform_ArmMount_wrt_Base_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ArmMount_wrt_Base_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ArmMount_wrt_Base_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ArmMount_wrt_Base_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransforms] baseToArmMount_: " << std::endl << tf2_transform_ArmMount_wrt_Base_ << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::setFixedTransforms] ee_link_name_: " << ee_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::setFixedTransforms] J6: " << frameNames_[frameNames_.size()-2] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[frameNames_.size()-2], ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[frameNames_.size()-2], ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    tf2_transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransforms] wrist2ToEETransform_: " << std::endl << tf2_transform_ToolMount_wrt_J6_ << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::updateTransforms() const
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] arm_mount_link_name_: " << arm_mount_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[2]: " << frameNames_[2] << std::endl;
      while(!tfBuffer.canTransform(arm_mount_link_name_, frameNames_[2], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(arm_mount_link_name_, frameNames_[2], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J1_wrt_ArmMount_ = Eigen::Matrix4d::Identity();
    tf2_transform_J1_wrt_ArmMount_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J1_wrt_ArmMount_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J1_wrt_ArmMount_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J1_wrt_ArmMount_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J1_wrt_ArmMount_: " << std::endl << tf2_transform_J1_wrt_ArmMount_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[3]: " << frameNames_[3] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[2], frameNames_[3], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[2], frameNames_[3], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J2_wrt_J1_ = Eigen::Matrix4d::Identity();
    tf2_transform_J2_wrt_J1_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J2_wrt_J1_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J2_wrt_J1_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J2_wrt_J1_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J2_wrt_J1_: " << std::endl << tf2_transform_J2_wrt_J1_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[4]: " << frameNames_[4] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[3], frameNames_[4], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[3], frameNames_[4], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J3_wrt_J2_ = Eigen::Matrix4d::Identity();
    tf2_transform_J3_wrt_J2_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J3_wrt_J2_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J3_wrt_J2_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J3_wrt_J2_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J3_wrt_J2_: " << std::endl << tf2_transform_J3_wrt_J2_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[5]: " << frameNames_[5] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[4], frameNames_[5], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[4], frameNames_[5], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J4_wrt_J3_ = Eigen::Matrix4d::Identity();
    tf2_transform_J4_wrt_J3_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J4_wrt_J3_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J4_wrt_J3_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J4_wrt_J3_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J4_wrt_J3_: " << std::endl << tf2_transform_J4_wrt_J3_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[6]: " << dofParentLinkNames_[4] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[5], frameNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[5], frameNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J5_wrt_J4_ = Eigen::Matrix4d::Identity();
    tf2_transform_J5_wrt_J4_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J5_wrt_J4_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J5_wrt_J4_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J5_wrt_J4_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J5_wrt_J4_: " << std::endl << tf2_transform_J5_wrt_J4_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[7]: " << dofParentLinkNames_[5] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[6], frameNames_[7], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[6], frameNames_[7], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J6_wrt_J5_ = Eigen::Matrix4d::Identity();
    tf2_transform_J6_wrt_J5_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J6_wrt_J5_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J6_wrt_J5_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J6_wrt_J5_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J6_wrt_J5_: " << std::endl << tf2_transform_J6_wrt_J5_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] ee_link_name_: " << ee_link_name_ << std::endl;
      while(!tfBuffer.canTransform(frameNames_[7], ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[7], ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    tf2_transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_ToolMount_wrt_J6_: " << std::endl << tf2_transform_ToolMount_wrt_J6_ << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::updateTransformsWorld() const
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[0], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[0], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_Base_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_Base_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_Base_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_Base_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_Base_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_Base_wrt_World_: " << std::endl << tf2_transform_Base_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[1], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[1], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ArmMount_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_ArmMount_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ArmMount_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ArmMount_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ArmMount_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_ArmMount_wrt_World_: " << std::endl << tf2_transform_ArmMount_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[2], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[2], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J1_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J1_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J1_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J1_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J1_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J1_wrt_World_: " << std::endl << tf2_transform_J1_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[3]: " << frameNames_[3] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[3], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[3], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J2_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J2_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J2_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J2_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J2_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J2_wrt_World_: " << std::endl << tf2_transform_J2_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[4]: " << frameNames_[4] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[4], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[4], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J3_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J3_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J3_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J3_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J3_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J3_wrt_World_: " << std::endl << tf2_transform_J3_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[5]: " << frameNames_[5] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[5], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[5], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J4_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J4_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J4_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J4_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J4_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J4_wrt_World_: " << std::endl << tf2_transform_J4_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[6]: " << dofParentLinkNames_[4] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J5_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J5_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J5_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J5_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J5_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J5_wrt_World_: " << std::endl << tf2_transform_J5_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] frameNames_[7]: " << dofParentLinkNames_[5] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[7], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[7], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J6_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J6_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J6_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J6_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J6_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_J6_wrt_World_: " << std::endl << tf2_transform_J6_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransforms] ee_link_name_: " << ee_link_name_ << std::endl;
      while(!tfBuffer.canTransform("world", ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ToolMount_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_ToolMount_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ToolMount_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ToolMount_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ToolMount_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_ToolMount_wrt_World_: " << std::endl << tf2_transform_ToolMount_wrt_World_ << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::updateTransforms(ocs2::PinocchioInterface& pinocchioInterface,
                                     const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                     const Eigen::VectorXd& state) const
{
  //std::cout << "[PointsOnRobot::updateTransforms(3)] START" << std::endl;

  transform_Base_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ArmMount_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J1_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J2_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J3_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J4_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J5_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J6_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ToolMount_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();

  /*
  // Base wrt world
  const ocs2::scalar_t yaw = state[2];
  const ocs2::scalar_t pitch = 0.0;
  const ocs2::scalar_t roll = 0.0;

  Eigen::Quaternion<ocs2::scalar_t> baseOrientation = EulerToQuaternion(yaw, pitch, roll);
  Eigen::Matrix<ocs2::scalar_t, 3, 1> basePosition;
  basePosition << state[0], state[1], 0.0;

  transform_Base_wrt_World_.topLeftCorner(3,3) = baseOrientation.toRotationMatrix();
  transform_Base_wrt_World_.topRightCorner(3,1) = basePosition;
  */

  // Update Pinocchio
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[0]].rotation();
  transform_Base_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[0]].translation();

  // Arm Base wrt World
  transform_ArmMount_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[1]].rotation();
  transform_ArmMount_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[1]].translation();

  // J1 wrt World
  transform_J1_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[2]].rotation();
  transform_J1_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[2]].translation();

  // J2 wrt World
  transform_J2_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[3]].rotation();
  transform_J2_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[3]].translation();

  // J3 wrt World
  transform_J3_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[4]].rotation();
  transform_J3_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[4]].translation();

  // J4 wrt World
  transform_J4_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[5]].rotation();
  transform_J4_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[5]].translation();

  // J5 wrt World
  transform_J5_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[6]].rotation();
  transform_J5_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[6]].translation();

  // J6 wrt World
  transform_J6_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[7]].rotation();
  transform_J6_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[7]].translation();

  // Tool Mount wrt World
  transform_ToolMount_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[8]].rotation();
  transform_ToolMount_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[8]].translation();

  // Arm Base wrt Base
  transform_ArmMount_wrt_Base_ = transform_Base_wrt_World_.inverse() * transform_ArmMount_wrt_World_;

  // J1 wrt Arm Base
  transform_J1_wrt_ArmMount_ = transform_ArmMount_wrt_World_.inverse() * transform_J1_wrt_World_;

  // J2 wrt J1
  transform_J2_wrt_J1_ = transform_J1_wrt_World_.inverse() * transform_J2_wrt_World_;

  // J3 wrt J2
  transform_J3_wrt_J2_ = transform_J2_wrt_World_.inverse() * transform_J3_wrt_World_;

  // J4 wrt J3
  transform_J4_wrt_J3_ = transform_J3_wrt_World_.inverse() * transform_J4_wrt_World_;

  // J5 wrt J4
  transform_J5_wrt_J4_ = transform_J4_wrt_World_.inverse() * transform_J5_wrt_World_;

  // J6 wrt J5
  transform_J6_wrt_J5_ = transform_J5_wrt_World_.inverse() * transform_J6_wrt_World_;

  // Tool Mount wrt J6
  transform_ToolMount_wrt_J6_ = transform_J6_wrt_World_.inverse() * transform_ToolMount_wrt_World_;

  /*
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_ArmMount_wrt_World_: " << std::endl << transform_ArmMount_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J1_wrt_World_: " << std::endl << transform_J1_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J2_wrt_World_: " << std::endl << transform_J2_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J3_wrt_World_: " << std::endl << transform_J3_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J4_wrt_World_: " << std::endl << transform_J4_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J5_wrt_World_: " << std::endl << transform_J5_wrt_World_ << std::endl;
  std::cout << "--------------------&&--------------------" << std::endl;
  std::cout << "[PointsOnRobot::updateTransforms(3)] transform_J6_wrt_World_: " << std::endl << transform_J6_wrt_World_ << std::endl;
  */

  //std::cout << "[PointsOnRobot::updateTransforms(3)] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::computeState2PointsOnRobot(ocs2::PinocchioInterface& pinocchioInterface,
                                                          const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                          const Eigen::Matrix<ocs2::scalar_t, -1, 1>& state) const
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

  int dim = getDimPoints();
  if (dim == 0) 
  {
    return Eigen::Matrix<ocs2::scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ocs2::scalar_t, 3, -1> pointsOnRobot_matrix(3, dim);
  Eigen::VectorXd pointsOnRobot_vec;

  std::cout << "[PointsOnRobot::computeState2PointsOnRobot] dim: " << dim << std::endl;
  
  updateTransforms(pinocchioInterface, mapping, state);

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ocs2::scalar_t, 4, 4> transform_tmp = transform_Base_wrt_World_;

  // Points between base and arm mount
  Eigen::Matrix<ocs2::scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = transform_J1_wrt_ArmMount_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 1 and joint 2
  next_transform = transform_J2_wrt_J1_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 2 and joint 3
  next_transform = transform_J3_wrt_J2_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 3 and joint 4
  next_transform = transform_J4_wrt_J3_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 4 and joint 5
  next_transform = transform_J5_wrt_J4_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 5 and joint 6
  next_transform = transform_J6_wrt_J5_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 6 and tool mount
  next_transform = transform_ToolMount_wrt_J6_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  pointsOnRobot_vec = Eigen::Map<Eigen::Matrix<ocs2::scalar_t, -1, 1>>(pointsOnRobot_matrix.data(), pointsOnRobot_matrix.size());

  return pointsOnRobot_vec;
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

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_Base_cppAd = transform_ArmMount_wrt_Base_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_ArmMount_cppAd = transform_J1_wrt_ArmMount_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_J1_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_J2_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_J3_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_J4_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_J5_cppAd = transform_J2_wrt_J1_.cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_J6_cppAd = transform_ToolMount_wrt_J6_.cast<ad_scalar_t>();

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_tmp = transform_ArmMount_wrt_Base_cppAd;

  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = transform_J1_wrt_ArmMount_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 1 and joint 2
  next_transform = transform_J2_wrt_J1_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 2 and joint 3
  next_transform = transform_J3_wrt_J2_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 3 and joint 4
  next_transform = transform_J4_wrt_J3_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 4 and joint 5
  next_transform = transform_J5_wrt_J4_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 5 and joint 6
  next_transform = transform_J6_wrt_J5_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points_[linkIndex][i];
    points_on_robot.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 6 and tool mount
  next_transform = transform_ToolMount_wrt_J6_cppAd;
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
