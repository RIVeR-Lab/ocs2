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
PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_),
    cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)){}

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
void PointsOnRobot::initialize(const ocs2::PinocchioInterface& pinocchioInterface,
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
  setTransforms(base_link_name, arm_mount_link_name, tool_link_name, ee_link_name);

  for (const auto& bodyName : dofParentLinkNames) 
  {
    dofParentLinkIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }
  dofParentLinkIds_.push_back(pinocchioInterface.getModel().getBodyId(tool_link_name));

  // CppAD interface
  ocs2::PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // pinocchioInterface to mapping
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  setADInterfaces(pinocchioInterfaceCppAd, *mappingCppAdPtr, modelName, modelFolder);

  if (recompileLibraries) 
  {
    std::cout << "[PointsOnRobot::initialize] recompileLibraries TRUE " << std::endl;
    createModels(verbose);
  } 
  else 
  {
    std::cout << "[PointsOnRobot::initialize] recompileLibraries FALSE " << std::endl;
    loadModelsIfAvailable(verbose);
  }


}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::getPoints(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getFunctionValue(state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::MatrixXd PointsOnRobot::getJacobian(const Eigen::VectorXd& state) const 
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
int PointsOnRobot::numOfPoints() const 
{
  return radii_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::MarkerArray PointsOnRobot::getVisualization(const Eigen::VectorXd& state) const 
{
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(radii_.size());

  Eigen::VectorXd points = getPoints(state);

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
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
  }
  return markerArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Quaternion<PointsOnRobot::ad_scalar_t> PointsOnRobot::EulerToQuaternion(ad_scalar_t& yaw, ad_scalar_t& pitch, ad_scalar_t& roll) const
{
  Eigen::Quaternion<ad_scalar_t> quat;
  quat = Eigen::AngleAxis<ad_scalar_t>(roll, Eigen::Vector3d::UnitX().cast<ad_scalar_t>())
          * Eigen::AngleAxis<ad_scalar_t>(pitch, Eigen::Vector3d::UnitY().cast<ad_scalar_t>())
          * Eigen::AngleAxis<ad_scalar_t>(yaw, Eigen::Vector3d::UnitZ().cast<ad_scalar_t>());
  
  //std::cout << "[PointsOnRobot::EulerToQuaternion] Quaternion" << std::endl << quat.coeffs() << std::endl;

  return quat;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, 1> PointsOnRobot::QuaternionToEuler(Eigen::Quaternion<ad_scalar_t>& quat) const
{
  Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, 1> euler;
  euler = quat.toRotationMatrix().eulerAngles(3,2,1);
  
  //std::cout << "[PointsOnRobot::EulerToQuaternion] Euler from quaternion in yaw, pitch, roll"<< std::endl << euler << std::endl;

  return euler;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2MultiplePointsOnRobot(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                                                                   const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                                                   const Eigen::Matrix<ad_scalar_t, -1, 1>& state,  
                                                                                                   const std::vector<std::vector<double>>& points) const 
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

  /*
  if (state.size() != 13) 
  {
    std::stringstream ss;
    ss << "Error: state.size()=" << state.size() << "!=13";
    std::string errorMessage = ss.str();
    std::cerr << std::endl << errorMessage << std::endl << std::endl;
    throw std::runtime_error(ss.str());
  }
  */

  int dim = 0;
  for (int i = 0; i < points.size(); i++) 
  {
    for (int j = 0; j < points[i].size(); j++) 
    {
      dim++;
    }
  }
  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }

  std::cout << "[PointsOnRobot::computeState2MultiplePointsOnRobot] state" << std::endl;
  for (size_t i = 0; i < state.rows(); i++)
  {
    for (size_t j = 0; j < state.cols(); j++)
    {
      std::cout << i << ": " << state(i,j) << std::endl;
    } 
  }

  std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] points" << std::endl;
  for (size_t i = 0; i < points.size(); i++)
  {
    for (size_t j = 0; j < points[i].size(); j++)
    {
      std::cout << i << ", " << j << ": " << points[i][j] << std::endl; 
    }
  }
  
  Eigen::Quaternion<ad_scalar_t> baseOrientation;
  Eigen::Matrix<ad_scalar_t, 1, 1> yaw;
  Eigen::Matrix<ad_scalar_t, 1, 1> pitch;
  Eigen::Matrix<ad_scalar_t, 1, 1> roll;

  yaw = state.template head<3>().template tail<1>();
  pitch = pitch.template head<1>() * (ad_scalar_t) 0.0;
  roll = roll.template head<1>() * (ad_scalar_t) 0.0;

  baseOrientation = EulerToQuaternion(yaw(0), pitch(0), roll(0));

  Eigen::Matrix<ad_scalar_t, 3, 1> basePosition = state.template head<3>();
  basePosition.template head<3>().template tail<1>() = basePosition.template head<3>().template tail<1>() * (ad_scalar_t) 0;

  Eigen::Matrix<ad_scalar_t, 4, 4> worldXFrBase = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  worldXFrBase.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  worldXFrBase.template topRightCorner<3, 1>() = basePosition;

  return computeArmState2MultiplePointsOnRobot(pinocchioInterfaceAd,
                                               mapping,
                                               state, 
                                               points, 
                                               worldXFrBase);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeArmState2MultiplePointsOnRobot(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                                                      const Eigen::Matrix<ad_scalar_t, -1, 1>& state,
                                                                                                      const std::vector<std::vector<double>>& points,
                                                                                                      const Eigen::Matrix<ad_scalar_t, 4, 4>& transformWorld_X_Base) const 
{
  // NUA TODO: The function is specific to a robot model! Generalize it!
  //assert(points.size() == 8);

  int pointIndex = 0;
  int linkIndex = 0;
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_tmp = transformWorld_X_Base;
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_dynamic_vector_t q = mapping.getPinocchioJointPosition(state);

  int dim = 0;
  for (int i = 0; i < points.size(); i++) 
  {
    for (int j = 0; j < points[i].size(); j++) 
    {
      dim++;
    }
  }

  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ad_scalar_t, 3, -1> points_on_robot(3, dim);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] dofParentLinkIds: " << std::endl;
  for (const auto& id : dofParentLinkIds_) 
  {
    std::cout << id << std::endl;
  }

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

  // Points between arm mount and joint 2
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[1]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[1]].translation();
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

  //std::cout << "[PointsOnRobot::computeArmState2MultiplePointsOnRobot] BEFORE INF LOOP: " << std::endl;
  //while(1){;}

  // Points between arm mount and joint 3
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[2]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[2]].translation();
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

  // Points between arm mount and joint 4
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[3]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[3]].translation();
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

  // Points between arm mount and joint 5
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[4]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[4]].translation();
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

  // Points between arm mount and joint 6
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[5]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[5]].translation();
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

  // Points between joint 7 (end effector) and tool mount
  next_transform = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  next_transform.template topLeftCorner<3, 3>() = data.oMf[dofParentLinkIds_[6]].rotation();
  next_transform.template topRightCorner<3, 1>() = data.oMf[dofParentLinkIds_[6]].translation();
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

  return points_on_robot;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::setTransforms(std::string base_link_name, 
                                  std::string arm_mount_link_name, 
                                  std::string tool_link_name, 
                                  std::string ee_link_name)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  {
    geometry_msgs::TransformStamped transformStamped;
    try 
    {
      transformStamped = tfBuffer.lookupTransform(base_link_name, arm_mount_link_name, ros::Time(0), ros::Duration(10.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transformBase_X_ArmMount_ = Eigen::Matrix4d::Identity();
    transformBase_X_ArmMount_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transformBase_X_ArmMount_(0, 3) = transformStamped.transform.translation.x;
    transformBase_X_ArmMount_(1, 3) = transformStamped.transform.translation.y;
    transformBase_X_ArmMount_(2, 3) = transformStamped.transform.translation.z;

    std::cout << "[PointsOnRobot::setTransforms] baseToArmMount_: " << std::endl << transformBase_X_ArmMount_ << std::endl;
  }

  {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform(tool_link_name, ee_link_name, ros::Time(0), ros::Duration(10.0));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transformToolMount_X_Endeffector_ = Eigen::Matrix4d::Identity();
    transformToolMount_X_Endeffector_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transformToolMount_X_Endeffector_(0, 3) = transformStamped.transform.translation.x;
    transformToolMount_X_Endeffector_(1, 3) = transformStamped.transform.translation.y;
    transformToolMount_X_Endeffector_(2, 3) = transformStamped.transform.translation.z;

    std::cout << "[PointsOnRobot::setTransforms] wrist2ToEETransform_: " << std::endl << transformToolMount_X_Endeffector_ << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::setADInterfaces(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mapping,
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
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixResult = computeState2MultiplePointsOnRobot(pinocchioInterfaceAd, mapping, x, points_);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
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
