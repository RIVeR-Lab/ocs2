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
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "ocs2_ext_collision/Definitions.h"

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

    void initialize(ocs2::PinocchioInterface& pinocchioInterface,
                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                    const std::string& modelName, 
                    const std::string& modelFolder, 
                    bool recompileLibraries, 
                    bool verbose,
                    const std::string& base_link_name, 
                    const std::string& arm_mount_link_name, 
                    const std::string& ee_link_name,
                    const std::vector<std::string>& jointParentFrameNames);

    Eigen::VectorXd getPointsPositions(ocs2::PinocchioInterface& pinocchioInterface, 
                                       const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                       const Eigen::VectorXd& state) const;

    Eigen::MatrixXd getPointsJacobian(const Eigen::VectorXd& state) const;

    Eigen::VectorXd getRadii() const;

    int getNumOfPoints() const;

    int getDimPoints() const;

    void getVisualization(ocs2::PinocchioInterface& pinocchioInterface, 
                          const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                          const Eigen::VectorXd& state) const;

    Eigen::Quaternion<ocs2::scalar_t> EulerToQuaternion(const ocs2::scalar_t& yaw, const ocs2::scalar_t& pitch, const ocs2::scalar_t& roll) const;

    Eigen::Matrix<ocs2::scalar_t, 3, 1> QuaternionToEuler(Eigen::Quaternion<ocs2::scalar_t>& quat) const;

    void setFixedTransforms();

    void updateTransforms() const;

    void updateTransformsWorld() const;

    void updateTransforms(ocs2::PinocchioInterface& pinocchioInterface,
                          const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                          const Eigen::VectorXd& state) const;

    Eigen::VectorXd computeState2PointsOnRobot(ocs2::PinocchioInterface& pinocchioInterface,
                                               const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                               const Eigen::Matrix<ocs2::scalar_t, -1, 1>& state) const;

    Eigen::Matrix<ad_scalar_t, 3, -1> computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                      const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const;
    
    /*
    Eigen::Matrix<ad_scalar_t, 3, -1> computeArmState2MultiplePointsOnRobot(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                            const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                            const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const;
    */

  private:
    void setADInterfaces(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                         const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                         const std::string& modelName,
                         const std::string& modelFolder);

    void createModels(bool verbose);
    
    void loadModelsIfAvailable(bool verbose);

    std::vector<std::vector<double>> points_;
    Eigen::VectorXd radii_;

    //std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
    std::shared_ptr<ocs2::CppAdInterface> cppAdInterface_;

    std::string base_link_name_;
    std::string arm_mount_link_name_;
    std::string ee_link_name_;

    std::vector<std::string> frameNames_;
    std::vector<size_t> frameIds_;
    
    mutable Eigen::Matrix4d tf2_transform_ArmMount_wrt_Base_;
    mutable Eigen::Matrix4d tf2_transform_J1_wrt_ArmMount_;
    mutable Eigen::Matrix4d tf2_transform_J2_wrt_J1_;
    mutable Eigen::Matrix4d tf2_transform_J3_wrt_J2_;
    mutable Eigen::Matrix4d tf2_transform_J4_wrt_J3_;
    mutable Eigen::Matrix4d tf2_transform_J5_wrt_J4_;
    mutable Eigen::Matrix4d tf2_transform_J6_wrt_J5_;
    mutable Eigen::Matrix4d tf2_transform_ToolMount_wrt_J6_;

    mutable Eigen::Matrix4d tf2_transform_Base_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_ArmMount_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J1_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J2_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J3_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J4_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J5_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_J6_wrt_World_;
    mutable Eigen::Matrix4d tf2_transform_ToolMount_wrt_World_;

    mutable Eigen::Matrix4d transform_Base_wrt_World_;
    mutable Eigen::Matrix4d transform_ArmMount_wrt_Base_;
    mutable Eigen::Matrix4d transform_J1_wrt_ArmMount_;
    mutable Eigen::Matrix4d transform_J2_wrt_J1_;
    mutable Eigen::Matrix4d transform_J3_wrt_J2_;
    mutable Eigen::Matrix4d transform_J4_wrt_J3_;
    mutable Eigen::Matrix4d transform_J5_wrt_J4_;
    mutable Eigen::Matrix4d transform_J6_wrt_J5_;
    mutable Eigen::Matrix4d transform_ToolMount_wrt_J6_;

    mutable Eigen::Matrix4d transform_ArmMount_wrt_World_;
    mutable Eigen::Matrix4d transform_J1_wrt_World_;
    mutable Eigen::Matrix4d transform_J2_wrt_World_;
    mutable Eigen::Matrix4d transform_J3_wrt_World_;
    mutable Eigen::Matrix4d transform_J4_wrt_World_;
    mutable Eigen::Matrix4d transform_J5_wrt_World_;
    mutable Eigen::Matrix4d transform_J6_wrt_World_;
    mutable Eigen::Matrix4d transform_ToolMount_wrt_World_;

    ros::NodeHandle nh_;
    mutable visualization_msgs::MarkerArray pointsOnRobot_visu_;
    ros::Publisher pointsOnRobot_visu_pub_;
};