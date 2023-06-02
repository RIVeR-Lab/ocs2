// LAST UPDATE: 2022.05.30
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2 {

/**
 * This class lets the user to command robot form interactive marker.
 */
class TargetTrajectoriesGazebo final 
{
  public:
    using GoalPoseToTargetTrajectories = std::function<TargetTrajectories(const Eigen::Vector3d& position, 
                                                                          const Eigen::Quaterniond& orientation, 
                                                                          const SystemObservation& observation)>;

    /**
     * Constructor
     *
     * @param [in] nodeHandle: ROS node handle.
     * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
     * observation is be expected on "topicPrefix_mpc_observation" topic.
     * @param [in] goalPoseToTargetTrajectories: A function which transforms the commanded pose to TargetTrajectories.
     */
    TargetTrajectoriesGazebo(ros::NodeHandle& nodeHandle,
                             const std::string& topicPrefix,
                             const std::string& gazeboModelMsgName,
                             std::string robotName,
                             std::vector<std::string>& targetNames,
                             GoalPoseToTargetTrajectories goalPoseToTargetTrajectories);

    // DESCRIPTION: TODO...
    ~TargetTrajectoriesGazebo();

    // DESCRIPTION: TODO...
    TargetTrajectoriesGazebo(const TargetTrajectoriesGazebo& ttg);

    // DESCRIPTION: TODO...
  	TargetTrajectoriesGazebo& operator=(const TargetTrajectoriesGazebo& ttg);

    // DESCRIPTION: TODO...
    void updateObservationAndTarget();

    // DESCRIPTION: TODO...
    void initializeInteractiveMarker();

  private:
    /// FUNCTIONS:
    // DESCRIPTION: TODO...
    void transformPose(std::string& frame_from,
                       std::string& frame_to,
                       geometry_msgs::Pose& p_from,
                       geometry_msgs::Pose& p_to);

    // DESCRIPTION: TODO...
    void rotateQuaternion(Eigen::Quaterniond& quat, Eigen::Vector3d& rpy_rot, Eigen::Quaterniond& quat_new);

    // DESCRIPTION: TODO...
    void print(geometry_msgs::Point po);

    // DESCRIPTION: TODO...
    int isIn(std::vector<std::string>& vec, std::string& s);

    // DESCRIPTION: TODO...
    double calculateEuclideanDistance(Eigen::Vector3d& p1, geometry_msgs::Pose& p2);

    // DESCRIPTION: TODO...
    std::pair<double, int> findClosestDistance(std::vector<Eigen::Vector3d>& pos_vec, geometry_msgs::Pose& query_p);

    // DESCRIPTION: TODO...
    void gazeboModelStatesCallback(const gazebo_msgs::ModelStatesPtr& feedback);

    // DESCRIPTION: TODO...
    void updateTarget();

    // DESCRIPTION: TODO...
    void updateGraspPose();

    // DESCRIPTION: TODO...
    void fillTargetVisu();

    // DESCRIPTION: TODO...
    void publishTargetVisu();

    // DESCRIPTION: TODO...
    void publishGraspFrame();

    // DESCRIPTION: TODO...
    visualization_msgs::InteractiveMarker createInteractiveMarker() const;

    // DESCRIPTION: TODO...
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    /// VARIABLES:
    bool initFlag_ = false;

    int ctr_ = 0;

    std::string worldFrameName_ = "world";
    std::string robotFrameName_ = "base_link";

    std::vector<std::string> targetNames_;
    std::string robotName_;
    geometry_msgs::Pose robotPose_;

    std::string currentTargetName_;
    Eigen::Vector3d currentTargetPosition_;
    Eigen::Quaterniond currentTargetOrientation_;
    
    Eigen::Vector3d graspPosOffset_;
    Eigen::Vector3d currentGraspPosition_;
    Eigen::Quaterniond currentGraspOrientation_;
    
    std::vector<std::string> currentTargetNames_;
    std::vector<Eigen::Vector3d> currentTargetPositions_;
    std::vector<Eigen::Quaterniond> currentTargetOrientations_;

    visualization_msgs::MarkerArray targetMarkerArray_;
    ros::Publisher targetMarkerArrayPublisher_;

    GoalPoseToTargetTrajectories goalPoseToTargetTrajectories_;

    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

    tf::TransformListener* tflistenerPtr_;

    interactive_markers::MenuHandler menuHandler_;
    interactive_markers::InteractiveMarkerServer server_;

    ros::Subscriber observationSubscriber_;
    ros::Subscriber gazeboModelStatesSubscriber_;

    mutable std::mutex latestObservationMutex_;
    
    SystemObservation latestObservation_;
};

}  // namespace ocs2
