// LAST UPDATE: 2024.02.15
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
//

#pragma once

#include <assert.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <pinocchio/multibody/geometry.hpp>

#include "ocs2_core/dynamics/MultiModelFunctions.h"
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

    // DESCRIPTION: TODO...
    explicit PointsOnRobot(const points_radii_t& points_radii);

    // DESCRIPTION: TODO...
    PointsOnRobot(const PointsOnRobot& rhs);

    // DESCRIPTION: TODO...
    void initialize(ocs2::PinocchioInterface& pinocchioInterface,
                    const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                    const ocs2::RobotModelInfo robotModelInfo,
                    const std::string& modelName, 
                    const std::string& modelFolder, 
                    bool recompileLibraries, 
                    bool verbose);
    
    // DESCRIPTION: TODO...
    Eigen::VectorXd getPointsPositionCppAd(const Eigen::VectorXd& state) const;

    // DESCRIPTION: TODO...
    Eigen::VectorXd getPointsPositionCppAd(const Eigen::VectorXd& state, const Eigen::VectorXd& fullState) const;

    // DESCRIPTION: TODO...
    Eigen::MatrixXd getPointsJacobianCppAd(const Eigen::VectorXd& state) const;

    // DESCRIPTION: TODO...
    void getPointsEigenToGeometryMsgsVec(Eigen::VectorXd& pointsEigen, std::vector<geometry_msgs::Point>& pointsVec);

    // DESCRIPTION: TODO...
    ocs2::RobotModelInfo getRobotModelInfo() const;

    // DESCRIPTION: TODO...
    Eigen::VectorXd getRadii() const;

    // DESCRIPTION: TODO...
    int getNumOfPoints() const;

    // DESCRIPTION: TODO...
    int getDimPoints() const;

    // DESCRIPTION: TODO...
    std::string getBaseFrameName() const;

    // DESCRIPTION: TODO...
    std::vector<size_t> getFrameIds() const;

    // DESCRIPTION: TODO...
    void setNodeHandle(ros::NodeHandle& nh);

    // DESCRIPTION: TODO...
    void setPointsOnRobotVisu();

    // DESCRIPTION: TODO...
    void setTimerPointsOnRobotVisu(double dt);

    // DESCRIPTION: TODO...
    void fillPointsOnRobotVisu() const;

    // DESCRIPTION: TODO...
    void publishPointsOnRobotVisu();

    void publishPointsOnRobotVisu(const ros::TimerEvent& e);

    // DESCRIPTION: TODO...
    Eigen::Quaternion<ocs2::scalar_t> EulerToQuaternion(const ocs2::scalar_t& yaw, const ocs2::scalar_t& pitch, const ocs2::scalar_t& roll) const;

    // DESCRIPTION: TODO...
    Eigen::Matrix<ocs2::scalar_t, 3, 1> QuaternionToEuler(Eigen::Quaternion<ocs2::scalar_t>& quat) const;

    // DESCRIPTION: TODO...
    Eigen::Matrix<ad_scalar_t, 3, -1> computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                      const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const;

    // DESCRIPTION: TODO...
    Eigen::Matrix<ad_scalar_t, 3, -1> computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                      const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd,
                                                                      const Eigen::Matrix<ad_scalar_t, -1, 1>& fullStateCppAd) const;

  private:
    void setADInterfaces(ocs2::PinocchioInterface& pinocchioInterface,
                         ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                         const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                         const std::string& modelName,
                         const std::string& modelFolder);

    void createModels(bool verbose);
    
    void loadModelsIfAvailable(bool verbose);

    ros::NodeHandle nh_;

    ros::Timer timer_;

    std::vector<std::vector<double>> points_;
    Eigen::VectorXd radii_;

    //std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
    std::shared_ptr<ocs2::CppAdInterface> cppAdInterface_;

    ocs2::RobotModelInfo robotModelInfo_;

    //std::string base_link_name_;
    //std::string arm_mount_link_name_;
    //std::string ee_link_name_;

    std::vector<std::string> frameNames_;
    std::vector<size_t> frameIds_;

    mutable Eigen::VectorXd points_on_robot_;

    
    mutable visualization_msgs::MarkerArray pointsOnRobot_visu_;
    ros::Publisher pointsOnRobot_visu_pub_;
    
    /*
    mutable Eigen::Matrix4d tf2_transform_ArmMount_wrt_Base_;
    mutable Eigen::Matrix4d tf2_transform_J1_wrt_ArmMount_;
    mutable Eigen::Matrix4d tf2_transform_J2_wrt_J1_;
    mutable Eigen::Matrix4d tf2_transform_J3_wrt_J2_;
    mutable Eigen::Matrix4d tf2_transform_J4_wrt_J3_;
    mutable Eigen::Matrix4d tf2_transform_J5_wrt_J4_;
    mutable Eigen::Matrix4d tf2_transform_J6_wrt_J5_;
    mutable Eigen::Matrix4d tf2_transform_ToolMount_wrt_J6_;
    */

    /*
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

    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_Base_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_Base_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_ArmMount_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_J1_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_J2_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_J3_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_J4_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_J5_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_J6_cppAd_;

    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_World_cppAd_;
    mutable Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_World_cppAd_;
    */
};