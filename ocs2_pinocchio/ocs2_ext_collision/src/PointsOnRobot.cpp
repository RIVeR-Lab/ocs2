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

//using namespace ocs2;
//using namespace ocs2_ext_collision;

ocs2_ext_collision::PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_) {}
    //cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)),
    //kinematics_(rhs.kinematics_) {}

ocs2_ext_collision::PointsOnRobot::PointsOnRobot(const ocs2_ext_collision::PointsOnRobot::points_radii_t& points_radii)
  : points_(), radii_()
{
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
}

void ocs2_ext_collision::PointsOnRobot::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) 
{
  setADInterfaces(modelName, modelFolder);

  if (recompileLibraries) 
  {
    createModels(verbose);
  } 
  else 
  {
    loadModelsIfAvailable(verbose);
  }
}

Eigen::VectorXd ocs2_ext_collision::PointsOnRobot::getPoints(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getFunctionValue(state);
}

Eigen::MatrixXd ocs2_ext_collision::PointsOnRobot::getJacobian(const Eigen::VectorXd& state) const {
  return cppAdInterface_->getJacobian(state);
}

visualization_msgs::MarkerArray ocs2_ext_collision::PointsOnRobot::getVisualization(const Eigen::VectorXd& state) const 
{
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(radii_.size());

  Eigen::VectorXd points = getPoints(state);

  for (int i = 0; i < markerArray.markers.size(); i++) {
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

void ocs2_ext_collision::PointsOnRobot::setADInterfaces(const std::string& modelName, const std::string& modelFolder) 
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

void SelfCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                         const std::string& modelName,
                                         const std::string& modelFolder)
{
  
}

void ocs2_ext_collision::PointsOnRobot::createModels(bool verbose) {
  cppAdInterface_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

void ocs2_ext_collision::PointsOnRobot::loadModelsIfAvailable(bool verbose) 
{
  cppAdInterface_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}
Eigen::VectorXd ocs2_ext_collision::PointsOnRobot::getRadii() const {
  return radii_;
}
int ocs2_ext_collision::PointsOnRobot::numOfPoints() const {
  return radii_.size();
}
