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

#pragma once

#include <assert.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>

#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ocs2_ext_collision/ExtCollisionPinocchioGeometryInterface.h"
//#include <ocs2_ext_collision/KinematicsInterface.hpp>
#include "ocs2_ext_collision/Definitions.h"

//namespace ocs2 {
//template <typename ad_scalar_t>
//class CppAdInterface;
//}

//namespace ocs2_ext_collision {

/*
struct PointsOnRobotConfig 
{
  using points_radii_t = std::vector<std::vector<std::pair<double, double>>>;
  points_radii_t pointsAndRadii;
  std::shared_ptr<const KinematicsInterface<CppAD::AD<CppAD::cg::CG<double>>>> kinematics;
};
*/

class PointsOnRobot 
{
  using ad_interface = ocs2::CppAdInterface;
  using ad_dynamic_vector_t = ad_interface::ad_vector_t;
  using ad_scalar_t = ad_interface::ad_scalar_t;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using points_radii_t = std::vector<std::vector<std::pair<double, double>>>;

    explicit PointsOnRobot(const points_radii_t& points_radii);

    PointsOnRobot(const PointsOnRobot& rhs);

    void initialize(const ocs2::PinocchioInterface& pinocchioInterface, 
                    ocs2::ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                    const std::string& modelName, 
                    const std::string& modelFolder, 
                    bool recompileLibraries, 
                    bool verbose);

    Eigen::VectorXd getPoints(const Eigen::VectorXd& state) const;

    Eigen::MatrixXd getJacobian(const Eigen::VectorXd& state) const;

    Eigen::VectorXd getRadii() const;

    int numOfPoints() const;

    visualization_msgs::MarkerArray getVisualization(const Eigen::VectorXd& state) const;

    //void computeState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, -1, 1>& state,
    Eigen::Matrix<ad_scalar_t, 3, -1> computeState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, -1, 1>& state,
                                                                   const std::vector<std::vector<double>>& points) const;
    
    //void computeArmState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, 6, 1>& state,
    Eigen::Matrix<ad_scalar_t, 3, -1> computeArmState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, 6, 1>& state, 
                                                                      const std::vector<std::vector<double>>& points,
                                                                      const Eigen::Matrix4d& transformBase_X_ArmBase, 
                                                                      const Eigen::Matrix4d& transformToolMount_X_Endeffector,
                                                                      const Eigen::Matrix<ad_scalar_t, 4, 4>& transformWorld_X_Base) const;

  private:
    void setADInterfaces(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                         const std::string& modelName,
                         const std::string& modelFolder);

    void createModels(bool verbose);
    
    void loadModelsIfAvailable(bool verbose);

    std::shared_ptr<ocs2::CppAdInterface> cppAdInterface_;
    //std::shared_ptr<const ocs2_ext_collision::KinematicsInterface<CppAD::AD<CppAD::cg::CG<double>>>> kinematics_;

    std::vector<std::vector<double>> points_;
    Eigen::VectorXd radii_;
};
//}  // namespace ocs2_ext_collision