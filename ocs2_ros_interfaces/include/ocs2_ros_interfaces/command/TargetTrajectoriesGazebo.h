// LAST UPDATE: 2024.02.15
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
#include <queue>
#include <deque>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "ocs2_msgs/setBool.h"
#include "ocs2_msgs/setInt.h"
#include "ocs2_msgs/setTask.h"
#include "ocs2_msgs/setSystemObservation.h"
//#include "ocs2_msgs/MobimanGoalObservation.h"
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
    TargetTrajectoriesGazebo(const std::string& ns,
                             const std::string& topicPrefix,
                             const std::string& gazeboModelMsgName,
                             std::string robotName,
                             std::vector<std::string>& targetNames,
                             std::string dropTargetName,
                             GoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
                             bool drlFlag=false);

    // DESCRIPTION: TODO...
    ~TargetTrajectoriesGazebo();

    // DESCRIPTION: TODO...
    TargetTrajectoriesGazebo(const TargetTrajectoriesGazebo& ttg);

    // DESCRIPTION: TODO...
  	TargetTrajectoriesGazebo& operator=(const TargetTrajectoriesGazebo& ttg);

    // DESCRIPTION: TODO...
    //void setGoalTrajectoryFrameName(std::string goalTrajectoryFrameName);

    // DESCRIPTION: TODO...
    //void setGoalTrajectoryQueueDt(double goalTrajectoryQueueDt);

    // DESCRIPTION: TODO...
    void setWorldFrameName(std::string worldFrameName);

    // DESCRIPTION: TODO...
    void setRobotFrameName(std::string robotFrameName);

    // DESCRIPTION: TODO...
    void setGoalFrameName(std::string goalFrameName);

    // DESCRIPTION: TODO...
    void setEEFrameName(std::string eeFrameName);

    // DESCRIPTION: TODO...
    void setGraspFrameName(std::string graspFrameName);

    // DESCRIPTION: TODO...
    void setDropFrameName(std::string dropFrameName);

    // DESCRIPTION: TODO...
    void setTaskMode(int taskMode);

    // DESCRIPTION: TODO...
    void launchNode(ros::NodeHandle& nodeHandle);

    // DESCRIPTION: TODO...
    void updateDummyGoal(double x, double y, double z, double roll, double pitch, double yaw);

    // DESCRIPTION: TODO...
    void updateObservationAndTarget();

    // DESCRIPTION: TODO...
    void updateGoal(bool autoFlag=false);

    // DESCRIPTION: TODO...
    void updateTarget(bool autoFlag=false);

    // DESCRIPTION: TODO...
    void setTargetToEEPose();

    // DESCRIPTION: TODO...
    void updateEEPose(Eigen::Vector3d& eePos, Eigen::Quaterniond& eeOri);

    // DESCRIPTION: TODO...
    void initializeInteractiveMarkerTarget();

    // DESCRIPTION: TODO...
    void initializeInteractiveMarkerAutoTarget();

    // DESCRIPTION: TODO...
    //void initializeInteractiveMarkerDropTarget();

    // DESCRIPTION: TODO...
    void initializeInteractiveMarkerModelMode();

    // DESCRIPTION: TODO...
    void publishGoalVisu();

    // DESCRIPTION: TODO...
    void publishGoalFrame();

    // DESCRIPTION: TODO...
    void publishTargetVisu();

    // DESCRIPTION: TODO...
    void publishGraspFrame();

    // DESCRIPTION: TODO...
    void publishDropFrame();

    // DESCRIPTION: TODO...
    void publishTargetTrajectories();

    // DESCRIPTION: TODO...
    void publishTargetTrajectories(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    // DESCRIPTION: TODO...
    //void publishMobimanGoalObs(bool onlyPosFlag=true);

    // DESCRIPTION: TODO...
    void updateCallback(const ros::TimerEvent& event);

    // DESCRIPTION: TODO...
    //void goalTrajectoryTimerCallback(const ros::TimerEvent& event);

  private:
    /// FUNCTIONS

    // DESCRIPTION: TODO...
    bool getTransform(std::string frame_from, std::string frame_to, tf::StampedTransform& stf);

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
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void statusModelModeMPCCallback(const std_msgs::Bool::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void statusModelModeMRTCallback(const std_msgs::Bool::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void updateGoal(const Eigen::Vector3d& goalPos, const Eigen::Quaterniond& goalOri);

    // DESCRIPTION: TODO...
    void updateTargetInfo();

    // DESCRIPTION: TODO...
    void updateTarget(const Eigen::Vector3d& targetPos, const Eigen::Quaterniond& targetOri);

    // DESCRIPTION: TODO...
    void updateGraspPose();

    // DESCRIPTION: TODO...
    void updateGraspPose(const Eigen::Vector3d& graspPos, const Eigen::Quaterniond& graspOri);

    // DESCRIPTION: TODO...
    void updateDropPose();

    // DESCRIPTION: TODO...
    void updateDropPose(const Eigen::Vector3d& targetPos, const Eigen::Quaterniond& targetOri);

    // DESCRIPTION: TODO...
    void fillGoalVisu();

    // DESCRIPTION: TODO...
    void fillTargetVisu();

    // DESCRIPTION: TODO...
    visualization_msgs::InteractiveMarker createInteractiveMarkerTarget() const;

    // DESCRIPTION: TODO...
    visualization_msgs::InteractiveMarker createInteractiveMarkerAutoTarget() const;

    // DESCRIPTION: TODO...
    //visualization_msgs::InteractiveMarker createInteractiveMarkerDropTarget() const;

    // DESCRIPTION: TODO...
    visualization_msgs::InteractiveMarker createInteractiveMarkerModelMode() const;

    // DESCRIPTION: TODO...
    void createMenuModelMode();

    // DESCRIPTION: TODO...
    void processFeedbackTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // DESCRIPTION: TODO...
    void processFeedbackAutoTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // DESCRIPTION: TODO...
    //void processFeedbackDropTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // DESCRIPTION: TODO...
    void processFeedbackModelMode(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // DESCRIPTION: TODO...
    //bool setTaskMode(int val);

    // DESCRIPTION: TODO...
    bool setTask(int taskMode, geometry_msgs::Pose target, double time_horizon=0.0);

    // DESCRIPTION: TODO...
    //bool setTaskModeSrv(ocs2_msgs::setInt::Request &req, 
    //                    ocs2_msgs::setInt::Response &res);

    // DESCRIPTION: TODO...
    bool setPickedFlagSrv(ocs2_msgs::setBool::Request& req, 
                          ocs2_msgs::setBool::Response& res);

    // DESCRIPTION: TODO...
    bool setSystemObservationSrv(ocs2_msgs::setSystemObservation::Request &req, 
                                 ocs2_msgs::setSystemObservation::Response &res);

    // DESCRIPTION: TODO...
    bool setTargetDRLSrv(ocs2_msgs::setTask::Request &req, 
                         ocs2_msgs::setTask::Response &res);

    /// VARIABLES:
    bool initCallbackFlag_ = false;
    bool initTFCallbackFlag_ = false;
    bool initMenuModelModeFlag_ = false;

    std::string ns_;

    bool drlFlag_;

    //bool graspFrameReadyFlag_ = false;
    //bool dropFrameReadyFlag_ = false;

    // 0: Go
    // 1: Go & Pick
    // 2: Go & Drop
    int taskMode_ = 0;

    bool pickedFlag_ = false;

    std::string worldFrameName_;
    std::string robotFrameName_; 
    std::string goalFrameName_; 
    std::string graspFrameName_; 
    std::string dropFrameName_;
    std::string eeFrameName_;

    std::string gazeboModelMsgName_;
    std::string modelModeMsgName_;
    std::string goalVisuMsgName_;
    std::string targetVisuMsgName_;
    std::string setTaskSrvName_;
    std::string setPickedFlagSrvName_;
    std::string setSystemObservationSrvName_;
    std::string setTargetDRLSrvName_;
    
    std::string mpcObservationMsgName_;
    std::string modelModeMPCStatusMsgName_;
    std::string modelModeMRTStatusMsgName_;
    std::string targetTrajectoriesMsgName_;
    //std::string mobimanGoalObsMsgName_;

    gazebo_msgs::ModelStates modelStatesMsg_;

    tf::StampedTransform tf_ee_wrt_world_;

    std::vector<std::string> targetNames_;
    std::string dropTargetName_;
    std::string robotName_;
    geometry_msgs::Pose robotPose_;

    bool targetIsGoalFlag_ = false;
    Eigen::Vector3d goalPosition_;
    Eigen::Quaterniond goalOrientation_;
    //Eigen::Vector3d goalPositionWrtRobot_;
    //Eigen::Quaterniond goalOrientationWrtRobot_;
    visualization_msgs::MarkerArray goalMarkerArray_;
    ros::Publisher goalMarkerArrayPublisher_;

    Eigen::Vector3d dummyGoalPosition_;
    Eigen::Quaterniond dummyGoalOrientation_;

    bool targetReadyFlag_ = false;
    std::string currentTargetName_;
    Eigen::Vector3d currentTargetPosition_;
    Eigen::Quaterniond currentTargetOrientation_;
    //Eigen::Vector3d currentTargetPositionWrtRobot_;
    //Eigen::Quaterniond currentTargetOrientationWrtRobot_;
    visualization_msgs::MarkerArray targetMarkerArray_;
    ros::Publisher targetMarkerArrayPublisher_;
    
    bool graspReadyFlag_ = false;
    Eigen::Vector3d graspPositionOffset_;
    Eigen::Matrix3d graspOrientationOffsetMatrix_;
    Eigen::Vector3d currentGraspPosition_;
    Eigen::Quaterniond currentGraspOrientation_;
    Eigen::Vector3d graspPositionEstimated_;

    bool dropReadyFlag_ = false;
    Eigen::Vector3d dropPositionOffset_;
    Eigen::Matrix3d dropOrientationOffsetMatrix_;
    Eigen::Vector3d currentDropPosition_;
    Eigen::Quaterniond currentDropOrientation_;
    
    std::vector<std::string> currentTargetNames_;
    std::vector<Eigen::Vector3d> currentTargetPositions_;
    std::vector<Eigen::Quaterniond> currentTargetOrientations_;
    //std::vector<Eigen::Vector3d> currentTargetPositionsWrtRobot_;
    //std::vector<Eigen::Quaterniond> currentTargetOrientationsWrtRobot_;

    GoalPoseToTargetTrajectories goalPoseToTargetTrajectories_;

    bool statusModelModeMPC_ = false;
    bool statusModelModeMRT_ = false;
    ros::Subscriber statusModelModeMPCSubscriber_;
    ros::Subscriber statusModelModeMRTSubscriber_;

    //std::string goalTrajectoryFrameName_;
    //int goalTrajectoryQueueSize_ = 10000;
    //double goalTrajectoryQueueDt_ = 0;
    //std::deque<std::vector<double>> goalTrajectoryQueue_;
    //int mobimanGoalObsTrajSampleNum_ = 1;
    //int mobimanGoalObsTrajSampleFreq_ = 5;
    //int mobimanGoalObsSeq_ = 0;
    //ros::Publisher mobimanGoalObsPublisher_;

    std::chrono::steady_clock::time_point startTime_;

    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    ros::Publisher modelModePublisher_;

    tf::TransformListener* tflistenerPtr_;

    interactive_markers::MenuHandler::EntryHandle h_mode_last_;
    //interactive_markers::MenuHandler::EntryHandle h_first_entry_;

    interactive_markers::MenuHandler menuHandlerTarget_;
    interactive_markers::MenuHandler menuHandlerAutoTarget_;
    interactive_markers::MenuHandler menuHandlerDropTarget_;
    interactive_markers::MenuHandler menuHandlerModelMode_;
    interactive_markers::InteractiveMarkerServer targetServer_;
    interactive_markers::InteractiveMarkerServer autoTargetServer_;
    interactive_markers::InteractiveMarkerServer dropTargetServer_;
    interactive_markers::InteractiveMarkerServer modelModeServer_;

    ros::Subscriber observationSubscriber_;
    ros::Subscriber gazeboModelStatesSubscriber_;
    ros::Subscriber tfSubscriber_;

    ros::ServiceClient setTaskClient_;
    ros::ServiceServer setPickedFlagService_;
    ros::ServiceServer setSystemObservationService_;
    ros::ServiceServer setTargetDRLService_;

    bool policyReceivedFlag_ = false;
    mutable std::mutex latestObservationMutex_;
    SystemObservation latestObservation_;
};

}  // namespace ocs2
