// LAST UPDATE: 2024.02.23
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
#include <ocs2_msgs/collision_info.h>
#include "ocs2_msgs/MobimanGoalObservation.h"
#include "ocs2_msgs/MobimanObservationTraj.h"
#include <ocs2_msgs/MobimanOccupancyObservation.h>
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
                                   const std::string& ns,
                                   const std::string& baseFrameName,
                                   const std::string& urdfFile,
                                   const std::string& jointStateMsgName,
                                   const RobotModelInfo robotModelInfo,
                                   const bool& selfCollisionFlag,
                                   const bool& extCollisionFlag,
                                   const std::vector<std::string>& removeJointNames,
                                   std::vector<std::pair<size_t, size_t>>& collisionObjectPairs,
                                   std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                   const std::string& selfCollisionMsg,
                                   std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                   std::shared_ptr<ExtMapUtility> emuPtr,
                                   double maxDistance);

    ~MobileManipulatorVisualization() override = default;

    void setObjOctomapNames(std::vector<std::string>& objOctomapNames);

    void setGoalFrameName(std::string goalFrameName);

    void setGoalTrajectoryFrameName(std::string goalTrajectoryFrameName);

    void setGoalTrajectoryQueueDt(double goalTrajectoryQueueDt);

    void setMobimanOccObsNames(std::vector<std::string>& mobimanOccObsNames);

    void setOccupancyInfoFrameName(std::string occupancyInfoFrameName);

    void setOccupancyInfoQueueDt(double occupancyInfoQueueDt);

    bool isTrue(std::vector<bool>& bvec);

    void updateModelMode(size_t modelModeInt);

    void update(const SystemObservation& observation, const PrimalSolution& policy);

    void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command, RobotModelInfo robotModelInfo) override;

    void publishSelfCollisionDistances();

    void publishSelfCollisionDistances(const ocs2::vector_t& state, const ocs2::vector_t& fullState, const RobotModelInfo& robotModelInfo);

    void getPoint(std::string& baseFrameName, std::string& pointFrameName, geometry_msgs::Point p);

    void updateStateIndexMap();

    void updateModelState();

    void updateState();

    void updateMobimanGoalObs(bool onlyPosFlag=true);

    void updateMobimanOccupancyObs(bool onlyPosFlag=true);

    void updateExtCollisionDistances(bool normalize_flag);

    void updateOctCallback(const ros::TimerEvent& event);

    void distanceVisualizationCallback(const ros::TimerEvent& event);

    void mobimanObservationTimerCallback(const ros::TimerEvent& event);

    void launchVisualizerNode(ros::NodeHandle& nodeHandle);

    void launchSubscriberMobimanGoalObsTraj(ros::NodeHandle& nodeHandle);

    void launchSubscriberMobimanOccupancyObsTraj(ros::NodeHandle& nodeHandle);

  private:

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void mobimanGoalTrajCallback(const ocs2_msgs::MobimanObservationTraj::ConstPtr& msg);

    void mobimanOccupancyTrajCallback(const ocs2_msgs::MobimanObservationTraj::ConstPtr& msg);

    bool getTransform(std::string frame_from, std::string frame_to, tf::StampedTransform& stf);

    void transformPoint(std::string& frame_from,
                        std::string& frame_to,
                        geometry_msgs::Point& p_from_to);

    // DESCRIPTION: TODO...
    void transformPointFromWorldtoBase(vector<geometry_msgs::Point>& p_from, vector<geometry_msgs::Point>& p_to);

    // DESCRIPTION: TODO...
    void fillSelfCollisionInfo(vector<bool>& col_status,
                               vector<double>& dist, 
                               vector<geometry_msgs::Point>& p0_vec, 
                               vector<geometry_msgs::Point>& p1_vec,
                               vector<double>& dist_threshold) const;
    
    void publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);
    
    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

    void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy, vector_t fullState);

    // DESCRIPTION: TODO...
    void publishSelfCollisionInfo();

    // DESCRIPTION: TODO...
    void publishSelfCollisionInfo(vector<bool>& col_status, 
                                  vector<double>& dist, 
                                  vector<geometry_msgs::Point>& p0_vec, 
                                  vector<geometry_msgs::Point>& p1_vec,
                                  vector<double>& dist_threshold);

    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;

    const std::string urdfFile_;
    std::string worldFrameName_;
    std::string ns_;
    std::string baseFrameName_;
    std::string baseFrameName_withNS_;
    std::string occupancyInfoFrameName_;

    std::vector<std::string> objOctomapNames_;
    
    RobotModelInfo robotModelInfo_;

    double maxDistance_;
    double radius_base_ = 0.35;
    vector<bool> col_status_base_;
    vector<bool> col_status_arm_;
    vector<double> col_dist_thresh_base_;
    vector<double> col_dist_thresh_arm_;
    //Eigen::Matrix<scalar_t, -1, 1> distances_;
    vector<geometry_msgs::Point> p0_vec_;
    vector<geometry_msgs::Point> p1_vec_;
    vector<geometry_msgs::Point> p0_vec_wrt_base_;
    vector<geometry_msgs::Point> p1_vec_wrt_base_;
    vector<geometry_msgs::Point> p0_vec2_;
    vector<geometry_msgs::Point> p1_vec2_;
    vector<geometry_msgs::Point> p0_vec2_wrt_base_;
    vector<geometry_msgs::Point> p1_vec2_wrt_base_;
    vector<double> dist_;
    vector<double> dist2_;
    std::vector<std::string> removeJointNames_;
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs_;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs_;
    std::vector<int> stateIndexMap_;
    std::vector<double> statePositionBase_;
    std::vector<double> statePositionArm_;
    vector_t state_;
    vector_t fullState_;
    std::vector<geometry_msgs::Point> jointPos_;

    ocs2_msgs::MobimanObservationTraj mobimanGoalTrajMsg_;
    std::string goalFrameName_;
    std::string goalTrajectoryFrameName_;
    std::string mobimanGoalTrajMsgName_;
    std::string mobimanGoalObsMsgName_;
    tf::StampedTransform mobimanGoalTF_;
    std::deque<std::vector<double>> goalTrajectoryQueue_;
    int goalTrajectoryQueueSize_ = 100;
    double goalTrajectoryQueueDt_ = 0;
    int mobimanGoalObsTrajSampleNum_ = 1;
    int mobimanGoalObsTrajSampleFreq_ = 5;
    int mobimanObsSeq_ = 0;
    int mobimanGoalObsSeq_ = 0;
    ros::Subscriber mobimanGoalObsTrajSubscriber_;
    ros::Publisher mobimanGoalObsTrajPublisher_;
    ros::Publisher mobimanGoalObsPublisher_;

    ocs2_msgs::MobimanObservationTraj mobimanOccupancyTrajMsg_;
    std::vector<std::string> mobimanOccObsNames_;
    std::string mobimanOccupancyTrajMsgName_;
    std::string mobimanOccupancyObsMsgName_;
    std::vector<tf::StampedTransform> occupancyInfoTFs_;
    std::vector<std::deque<std::vector<double>>> occupancyInfoQueue_;
    int occupancyInfoQueueSize_ = 100;
    double occupancyInfoQueueDt_ = 0;
    int mobimanOccupancyObsSampleNum_ = 1;
    int mobimanOccupancyObsSampleFreq_ = 5;
    int mobimanOccupancyObsSeq_ = 0;
    ros::Subscriber mobimanOccupancyObsTrajSubscriber_;
    ros::Publisher mobimanOccupancyObsTrajPublisher_;
    ros::Publisher mobimanOccupancyObsPublisher_;

    bool printOutFlag_ = false;
    bool initTFTransformFlag_ = false;
    bool initJointStateFlag_ = false;
    bool selfCollisionFlag_;
    bool extCollisionFlag_;
    bool goalTFInitFlag_ = false;
    std::vector<bool> occupancyInfoTFInitFlags_;

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

    double selfCollisionRangeMin_ = 0.05;
    vector<bool> self_col_status_;
    vector<double> self_col_dist_;
    vector<geometry_msgs::Point> self_p0_;
    vector<geometry_msgs::Point> self_p1_;
    vector<double> self_col_dist_thresh_;
    std::string selfCollisionMsg_;
    mutable ocs2_msgs::collision_info selfCollisionInfo_;

    std::string optimizedStateTrajectoryMsgName_;
    std::string optimizedPoseTrajectoryMsgName_;

    //std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    
    std::chrono::steady_clock::time_point startTime_;

    ocs2::benchmark::RepeatedTimer timer0_;
    ocs2::benchmark::RepeatedTimer timer1_;
    ocs2::benchmark::RepeatedTimer timer2_;
    ocs2::benchmark::RepeatedTimer timer3_;
    ocs2::benchmark::RepeatedTimer timer4_;
    ocs2::benchmark::RepeatedTimer timer5_;

    ocs2::benchmark::RepeatedTimer timer6_;
    ocs2::benchmark::RepeatedTimer timer7_;
    ocs2::benchmark::RepeatedTimer timer8_;
    ocs2::benchmark::RepeatedTimer timer9_;
    ocs2::benchmark::RepeatedTimer timer10_;
    ocs2::benchmark::RepeatedTimer timer11_;
    ocs2::benchmark::RepeatedTimer timer12_;

    ros::Subscriber jointStatesSub_;
    ros::Subscriber tfSub_;

    ros::Publisher pubSelfCollisionInfo_;
    ros::Publisher markerPublisher_;
    ros::Publisher stateOptimizedPublisher_;
    ros::Publisher stateOptimizedPosePublisher_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
