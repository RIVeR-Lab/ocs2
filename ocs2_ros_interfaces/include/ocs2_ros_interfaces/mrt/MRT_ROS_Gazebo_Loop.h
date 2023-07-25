// LAST UPDATE: 2023.07.24
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
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/JointControllerState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

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
                        std::string graspFrameName,
                        std::string baseStateMsg,
                        std::string armStateMsg,
                        std::string baseControlMsg,
                        std::string armControlMsg,
                        double err_threshold_pos_,
                        double err_threshold_ori_,
                        scalar_t mrtDesiredFrequency,
                        scalar_t mpcDesiredFrequency = -1,
                        bool updateIndexMapFlag=false);

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
    bool isGraspPoseReached();

    /** NUA TODO: Add description */
    //void publishCommand(const PrimalSolution& primalSolution);
    void publishCommand(const PrimalSolution& currentPolicy);

    std::string worldFrameName_;
    std::string baseFrameName_;
    std::string graspFrameName_;

    double err_threshold_pos_;
    double err_threshold_ori_;

    RobotModelInfo robotModelInfo_;

    bool tfFlag_ = false;
    bool initFlagBaseState_ = false;
    bool initFlagArmState_ = false;

    std::vector<int> stateIndexMap_;

    scalar_t mrtDesiredFrequency_;
    scalar_t mpcDesiredFrequency_;

    MRT_ROS_Interface& mrt_;
    scalar_t dt_;
    scalar_t time_;

    vector_t currentInput_;

    std::vector<double> stateBase_;
    std::vector<double> stateArm_;

    std::vector<double> inputBase_;
    std::vector<double> inputArm_;

    tf::TransformListener tfListener_;

    std::vector<std::shared_ptr<DummyObserver>> observers_;

    tf::StampedTransform tf_robot_wrt_world_;
    tf::StampedTransform tf_ee_wrt_world_;
    tf::StampedTransform tf_grasp_wrt_world_;
    nav_msgs::Odometry odometryMsg_;
    geometry_msgs::Pose robotBasePoseMsg_;
    geometry_msgs::Twist robotBaseTwistMsg_;
    sensor_msgs::JointState jointStateMsg_;
    control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg_;

    bool shutDownFlag_ = false;
    std::string mrtShutDownFlag_;

    // 0: No object attached
    // 1: Object picked
    int taskMode_ = 0;

    benchmark::RepeatedTimer timer1_;
    benchmark::RepeatedTimer timer2_;

    ros::Subscriber tfSub_;
    ros::Subscriber odometrySub_;
    ros::Subscriber linkStateSub_;
    ros::Subscriber jointStateSub_;
    ros::Subscriber jointTrajectoryPControllerStateSub_;

    ros::Publisher baseTwistPub_;
    ros::Publisher armJointTrajectoryPub_;

    ros::ServiceClient attachClient_;
    ros::ServiceClient detachClient_;
};

}  // namespace ocs2
