// LAST UPDATE: 2023.07.13
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

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"
//#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorVisualization final : public DummyObserver 
{
  public:
    MobileManipulatorVisualization(ros::NodeHandle& nodeHandle, 
                                          const PinocchioInterface& pinocchioInterface,
                                          const RobotModelInfo& robotModelInfo,
                                          const std::string& urdfFile,
                                          const std::string& taskFile)
      : pinocchioInterface_(pinocchioInterface), 
        robotModelInfo_(robotModelInfo),
        urdfFile_(urdfFile),
        taskFile_(taskFile),
        markerPublisher_(nodeHandle.advertise<visualization_msgs::MarkerArray>("distance_markers", 1, true))
    {
      std::cout << "[MobileManipulatorVisualization::MobileManipulatorVisualization] START" << std::endl;
      launchVisualizerNode(nodeHandle);
      std::cout << "[MobileManipulatorVisualization::MobileManipulatorVisualization] END" << std::endl;
    }

    ~MobileManipulatorVisualization() override = default;

    void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

    void publishDistances(const ocs2::vector_t& state, const ocs2::vector_t& fullState, const RobotModelInfo& robotModelInfo);

  private:
    void launchVisualizerNode(ros::NodeHandle& nodeHandle);

    void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);
    
    void publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);
    
    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy, vector_t fullState);

    const std::string urdfFile_;
    const std::string taskFile_;
    const RobotModelInfo robotModelInfo_;

    PinocchioInterface pinocchioInterface_;
    
    std::vector<std::string> removeJointNames_;

    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;

    ros::Publisher markerPublisher_;
    ros::Publisher stateOptimizedPublisher_;
    ros::Publisher stateOptimizedPosePublisher_;

    std::unique_ptr<GeometryInterfaceVisualization> geometryVisualization_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
