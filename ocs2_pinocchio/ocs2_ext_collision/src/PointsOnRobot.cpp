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

#include <ocs2_ext_collision/PointsOnRobot.h>

PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_),
    cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)) {}
    //kinematics_(rhs.kinematics_) {}

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

void PointsOnRobot::initialize(const ocs2::PinocchioInterface& pinocchioInterface,
                               const std::string& modelName, 
                               const std::string& modelFolder, 
                               bool recompileLibraries, 
                               bool verbose)
{
  ocs2::PinocchioInterfaceCppAd pinocchioInterfaceAd = pinocchioInterface.toCppAd();

  setADInterfaces(pinocchioInterfaceAd, modelName, modelFolder);

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

Eigen::VectorXd PointsOnRobot::getPoints(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getFunctionValue(state);
}

Eigen::MatrixXd PointsOnRobot::getJacobian(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getJacobian(state);
}

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

/*
void PointsOnRobot::setADInterfaces(const std::string& modelName, const std::string& modelFolder) 
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

  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixResult = kinematics_->computeState2MultiplePointsOnRobot(x, points_);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 Definitions::STATE_DIM_, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));
}
*/

void PointsOnRobot::setADInterfaces(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
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
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixResult = computeState2MultiplePointsOnRobot(x, points_);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };

  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));

  /*
  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;
  const size_t numDistanceResults = pinocchioGeometryInterface_.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };

  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, 
                                                              stateDim,
                                                              numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", 
                                                              modelFolder));

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = computeLinkPointsAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, modelName + "_links_intermediate",
                                                     modelFolder));
  */
}

void PointsOnRobot::createModels(bool verbose) 
{
  cppAdInterface_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

void PointsOnRobot::loadModelsIfAvailable(bool verbose) 
{
  cppAdInterface_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

Eigen::VectorXd PointsOnRobot::getRadii() const 
{
  return radii_;
}

int PointsOnRobot::numOfPoints() const 
{
  return radii_.size();
}

Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, -1, 1>& state,  
                                                                                                   const std::vector<std::vector<double>>& points) const 
{
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
    for (int j = 0; j < points[i].size(); j++) {
      dim++;
    }
  }
  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }

  Eigen::Matrix<ad_scalar_t, 3, -1> result(3, dim);
  int resultIndex = 0;
  int linkIndex;

  Eigen::Quaternion<ad_scalar_t> baseOrientation;
  baseOrientation.coeffs() = state.template head<7>().template head<4>();
  const Eigen::Matrix<ad_scalar_t, 3, 1>& basePosition = state.template head<7>().template tail<3>();
  const Eigen::Matrix<ad_scalar_t, 6, 1>& armState = state.template tail<6>();

  Eigen::Matrix<ad_scalar_t, 4, 4> worldXFrBase = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  worldXFrBase.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  worldXFrBase.template topRightCorner<3, 1>() = basePosition;

  Eigen::Matrix4d transformBase_X_ArmMount;
  Eigen::Matrix4d transformToolMount_X_Endeffector;

  return computeArmState2MultiplePointsOnRobot(armState, 
                                               points, 
                                               transformBase_X_ArmMount, 
                                               transformToolMount_X_Endeffector,
                                               worldXFrBase);
}

Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeArmState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, 6, 1>& state,  
                                                                                                      const std::vector<std::vector<double>>& points,
                                                                                                      const Eigen::Matrix4d& transformBase_X_ArmBase, 
                                                                                                      const Eigen::Matrix4d& transformToolMount_X_Endeffector,
                                                                                                      const Eigen::Matrix<ad_scalar_t, 4, 4>& transformWorld_X_Base) const 
{
  assert(points.size() == 8);

  int dim = 0;
  for (int i = 0; i < points.size(); i++) {
    for (int j = 0; j < points[i].size(); j++) {
      dim++;
    }
  }

  //if (dim == 0) {
  //  return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  //}
  Eigen::Matrix<ad_scalar_t, 3, -1> result(3, dim);
  int resultIndex = 0;
  int linkIndex;

  Eigen::Matrix<ad_scalar_t, 4, 4> transformWorld_X_Endeffector = transformWorld_X_Base;

  /*
  typedef typename iit::rbd::tpl::TraitSelector<ad_scalar_t>::Trait trait_t;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_arm_mount_X_fr_SHOULDER armMountXFrShoulderXFrShoulder;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_SHOULDER_X_fr_ARM shoulderXFrArm;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_ARM_X_fr_ELBOW armXFrElbow;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_ELBOW_X_fr_FOREARM elbowXFrForearm;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_FOREARM_X_fr_WRIST_1 forearmXFrWrist1;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_WRIST_1_X_fr_WRIST_2 wrist1XFrWrist2;

  Eigen::Matrix<ad_scalar_t, 4, 4> nextStep = transformBase_X_ArmBase.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = armMountXFrShoulderXFrShoulder.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = shoulderXFrArm.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = armXFrElbow.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = elbowXFrForearm.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = forearmXFrWrist1.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = wrist1XFrWrist2.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = transformToolMount_X_Endeffector.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;
  */

  return result;
}
