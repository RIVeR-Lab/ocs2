// LAST UPDATE: 2023.08.18
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#pragma once

//#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/dynamics/MultiModelFunctions.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorVisualization final : public DummyObserver 
{
  public:
    MobileManipulatorVisualization(ros::NodeHandle& nodeHandle, 
                                   const PinocchioInterface& pinocchioInterface,
                                   const std::string& worldFrameName,
                                   const std::string& baseFrameName,
                                   const std::string& urdfFile,
                                   const std::string& jointStateMsgName,
                                   const RobotModelInfo& robotModelInfo,
                                   const bool& selfCollisionFlag,
                                   const bool& extCollisionFlag,
                                   const std::vector<std::string>& removeJointNames,
                                   std::vector<std::pair<size_t, size_t>>& collisionObjectPairs,
                                   std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                   std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                   std::shared_ptr<ExtMapUtility> emuPtr);

    ~MobileManipulatorVisualization() override = default;

    void updateModelMode(size_t modelModeInt);

    void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

    void publishDistances(const ocs2::vector_t& state, const ocs2::vector_t& fullState, const RobotModelInfo& robotModelInfo);

    void getPoint(std::string& baseFrameName, std::string& pointFrameName, geometry_msgs::Point p);

    void updateStateIndexMap();

    void updateModelState();

    void updateState();

    void updateDistances(bool normalize_flag);

    void updateOctCallback(const ros::TimerEvent& event);

    void updateDistancesCallback(const ros::TimerEvent& event);

  private:
    void launchVisualizerNode(ros::NodeHandle& nodeHandle);

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    //void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);
    
    void publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);
    
    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy, vector_t fullState);

    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;

    const std::string urdfFile_;
    std::string worldFrameName_;
    std::string baseFrameName_;
    
    RobotModelInfo robotModelInfo_;

    double maxDistance_;
    Eigen::Matrix<scalar_t, -1, 1> distances_;
    vector<geometry_msgs::Point> p0_vec_;
    vector<geometry_msgs::Point> p1_vec_;
    std::vector<std::string> removeJointNames_;
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs_;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs_;
    std::vector<int> stateIndexMap_;
    std::vector<double> statePositionBase_;
    std::vector<double> statePositionArm_;
    vector_t state_;
    vector_t fullState_;
    std::vector<geometry_msgs::Point> jointPos_;

    bool printOutFlag_ = false;
    bool initTFTransformFlag_ = false;
    bool initJointStateFlag_ = false;
    bool selfCollisionFlag_; 
    bool extCollisionFlag_; 

    PinocchioInterface pinocchioInterface_;
    PinocchioInterface pinocchioInterfaceInternal_;
    std::shared_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;
    //PinocchioGeometryInterface geometryInterface_;
    std::unique_ptr<GeometryInterfaceVisualization> visualizationInterfacePtr_;
    std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_;
    std::shared_ptr<ExtMapUtility> emuPtr_;

    std::string jointStateMsgName_;
    tf::StampedTransform tf_robot_wrt_world_;
    sensor_msgs::JointState jointStateMsg_;

    //std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    
    ocs2::benchmark::RepeatedTimer timer0_;
    ocs2::benchmark::RepeatedTimer timer1_;
    ocs2::benchmark::RepeatedTimer timer2_;
    ocs2::benchmark::RepeatedTimer timer3_;
    ocs2::benchmark::RepeatedTimer timer4_;

    ros::Subscriber jointStatesSub_;
    ros::Subscriber tfSub_;

    ros::Publisher markerPublisher_;
    ros::Publisher stateOptimizedPublisher_;
    ros::Publisher stateOptimizedPosePublisher_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
