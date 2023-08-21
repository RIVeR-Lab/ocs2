// LAST UPDATE: 2023.08.03
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

#include <math.h>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <ros/package.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/JointControllerState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

#include "ocs2_msgs/setBool.h"
#include "ocs2_msgs/setInt.h"
#include "ocs2_msgs/setTask.h"
#include "ocs2_msgs/setSystemObservation.h"
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

#include "gazebo_ros_link_attacher/Attach.h"

namespace ocs2 {

/**
 * This class implements a loop to test MPC-MRT communication interface using ROS.
 */
class MRT_ROS_Gazebo_Loop 
{
  public:
    /**
     * Constructor. NUA TODO: Update!
     *
     * @param [in] mrt: The underlying MRT class to be used. If MRT contains a rollout object, the dummy will roll out
     * the received controller using the MRT::rolloutPolicy() method instead of just sending back a planned state.
     * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
     * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
     * will be simulated to run by this frequency. Note that this might not be the MPC's real-time frequency.
     */
    MRT_ROS_Gazebo_Loop(ros::NodeHandle& nh,
                        MRT_ROS_Interface& mrt,
                        std::string worldFrameName,
                        std::string targetMsg,
                        std::string baseStateMsg,
                        std::string armStateMsg,
                        std::string baseControlMsg,
                        std::string armControlMsg,
                        double err_threshold_pos_,
                        double err_threshold_ori_,
                        scalar_t mrtDesiredFrequency,
                        scalar_t mpcDesiredFrequency = -1,
                        std::string logSavePath="");

    /**
     * Destructor.
     */
    virtual ~MRT_ROS_Gazebo_Loop() = default;

    void setStateIndexMap(std::vector<int>& stateIndexMap);

    /** NUA TODO: Add description */
    bool isArmStateInitialized();

    /** NUA TODO: Add description */
    bool isStateInitialized();

    /**
       * Runs the MRT loop.
       *
       * @param [in] initTargetTrajectories: The initial TargetTrajectories.
       */
    //void run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);
    //void run(const TargetTrajectories& initTargetTrajectories);
    void run(vector_t initTarget);

    /**
     * Subscribe a set of observers to the dummy loop. Observers are updated in the provided order at the end of each timestep.
     * The previous list of observers is overwritten.
     *
     * @param observers : vector of observers.
     */
    void subscribeObservers(const std::vector<std::shared_ptr<DummyObserver>>& observers) { observers_ = observers; }

    void getInitTarget(vector_t& initTarget);

    void getCurrentState(vector_t& currentState);

  private:

    /** Forward simulates the system from current observation*/
    SystemObservation forwardSimulation(const SystemObservation& currentObservation);

    /** NUA TODO: Add decription. */
    void mrtLoop();

    /** NUA TODO: Add description */
    void updateStateIndexMap(std::string& armStateMsg, bool updateStateIndexMapFlag);

    /** NUA TODO: Add description */
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

    /** NUA TODO: Add description */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /** NUA TODO: Add description */
    void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

    /** NUA TODO: Add description */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void updateFullModelState();

    /** NUA TODO: Add description */
    SystemObservation getCurrentObservation(bool initFlag=false);

    /** NUA TODO: Add description */
    bool isPickDropPoseReached(int taskMode);

    /** NUA TODO: Add description */
    //bool setTaskMode(int val);

    /** NUA TODO: Add description */
    bool setPickedFlag(bool val);

    /** NUA TODO: Add description */
    bool setSystemObservation(const SystemObservation& currentObservation);

    // DESCRIPTION: TODO...
    //bool setTaskModeSrv(ocs2_msgs::setInt::Request &req, 
    //                    ocs2_msgs::setInt::Response &res);

    // DESCRIPTION: TODO...
    bool setTaskSrv(ocs2_msgs::setTask::Request &req, 
                    ocs2_msgs::setTask::Response &res);

    /** NUA TODO: Add description */
    //void publishCommand(const PrimalSolution& primalSolution);
    void publishCommand(const PrimalSolution& currentPolicy, 
                        const SystemObservation& currentObservation,
                        std::vector<double>& currentStateVelocityBase);

    /** NUA TODO: Add description */
    const std::string getDateTime();

    /** NUA TODO: Add description */
    void writeData(bool endFlag=false);

    std::string dataPathReL_;
    std::vector<std::vector<double>> dataStatePosition_;
    std::vector<std::vector<double>> dataStateVelocityBase_;
    std::vector<std::vector<double>> dataCommand_;
    std::vector<std::vector<double>> dataMPCTimeTrajectory_;
    std::vector< std::vector<std::vector<double>> > dataMPCStateTrajectory_;
    std::vector< std::vector<std::vector<double>> > dataMPCInputTrajectory_;
    double dataTimeStart_ = 0;
    double dataTimeEnd_ = 0;
    double dataWriteFreq_ = 10;
    double dataWriteLastTime_ = 0;

    std::string worldFrameName_;
    std::string baseFrameName_;
    std::string graspFrameName_;

    bool dataCollectionFlag_ = true;

    double initPolicyCtrMax_ = 5000;

    double err_threshold_pos_;
    double err_threshold_ori_;

    RobotModelInfo robotModelInfo_;

    bool tfFlag_ = false;
    bool initFlagBaseState_ = false;
    bool initFlagArmState_ = false;
    bool initFlagArmState2_ = false;
    bool targetReceivedFlag_ = false;

    bool updateIndexMapFlag_ = true;
    std::vector<int> stateIndexMap_;

    scalar_t mrtDesiredFrequency_;
    scalar_t mpcDesiredFrequency_;

    MRT_ROS_Interface& mrt_;
    scalar_t dt_;
    scalar_t time_;

    std::string currentTargetName_;
    std::string currentTargetAttachLinkName_;
    vector_t currentTarget_;
    vector_t currentInput_;

    std::vector<double> statePositionBase_;
    std::vector<double> statePositionArm_;

    std::vector<double> stateVelocityBase_;
    std::vector<double> inputArm_;

    std::vector<double> mpcTimeTrajectory_;
    std::vector<std::vector<double>> mpcStateTrajectory_;
    std::vector<std::vector<double>> mpcInputTrajectory_;

    tf::TransformListener tfListener_;

    std::vector<std::shared_ptr<DummyObserver>> observers_;

    tf::StampedTransform tf_robot_wrt_world_;
    tf::StampedTransform tf_ee_wrt_world_;
    //tf::StampedTransform tf_grasp_wrt_world_;

    nav_msgs::Odometry odometryMsg_;
    geometry_msgs::Pose robotBasePoseMsg_;
    geometry_msgs::Twist robotBaseTwistMsg_;
    sensor_msgs::JointState jointStateMsg_;
    control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg_;

    bool shutDownFlag_ = false;
    std::string mrtShutDownFlag_;

    // 0: Go
    // 1: Go & Pick
    // 2: Go & Drop
    int taskMode_ = 0;

    bool pickedFlag_ = false;
    bool taskEndFlag_ = true;

    benchmark::RepeatedTimer timer1_;
    benchmark::RepeatedTimer timer2_;

    ros::Subscriber tfSub_;
    ros::Subscriber odometrySub_;
    ros::Subscriber linkStateSub_;
    ros::Subscriber jointStateSub_;
    ros::Subscriber jointTrajectoryControllerStateSub_;
    ros::Subscriber targetTrajectoriesSubscriber_;

    ros::Publisher baseTwistPub_;
    ros::Publisher armJointTrajectoryPub_;

    ros::ServiceClient attachClient_;
    ros::ServiceClient detachClient_;
    //ros::ServiceClient setTaskModeClient_;
    ros::ServiceClient setPickedFlagClient_;
    ros::ServiceClient setSystemObservationClient_;

    //ros::ServiceServer setTaskModeService_;
    ros::ServiceServer setTaskService_;
};

}  // namespace ocs2
