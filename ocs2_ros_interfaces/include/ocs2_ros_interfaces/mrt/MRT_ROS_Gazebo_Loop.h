/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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

#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"
//#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

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
                        std::string baseStateMsg,
                        std::string armStateMsg,
                        std::string baseControlMsg,
                        std::string armControlMsg,
                        scalar_t mrtDesiredFrequency,
                        scalar_t mpcDesiredFrequency = -1);

    /**
     * Destructor.
     */
    virtual ~MRT_ROS_Gazebo_Loop() = default;

    /** NUA TODO: Add description */
    //void setRobotModelType(std::string robotModelType);

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

  protected:
    /**
     * A user-defined function which modifies the observation before publishing.
     *
     * @param [in] observation: The current observation.
     */
    virtual void modifyObservation(SystemObservation& observation) {}

  private:

    /** Forward simulates the system from current observation*/
    SystemObservation forwardSimulation(const SystemObservation& currentObservation);

    /** NUA TODO: Add decription. */
    void mrtLoop();

    /** NUA TODO: Add description */
    void setStateIndexMap(std::string& armStateMsg);

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
    void joint1ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void joint2ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void joint3ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void joint4ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void joint5ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void joint6ControllerStateCallback(const control_msgs::JointControllerState::ConstPtr& msg);

    /** NUA TODO: Add description */
    void updateFullModelState();

    /** NUA TODO: Add description */
    SystemObservation getCurrentObservation(bool initFlag=false);

    /** NUA TODO: Add description */
    //void publishCommand(const PrimalSolution& primalSolution);
    void publishCommand();

    std::string worldFrameName_;
    std::string baseFrameName_;

    RobotModelInfo robotModelInfo_;

    bool initFlagBaseState_ = false;
    bool initFlagArmState_ = false;
    bool initFlagArm1State_ = false;
    bool initFlagArm2State_ = false;
    bool initFlagArm3State_ = false;
    bool initFlagArm4State_ = false;
    bool initFlagArm5State_ = false;
    bool initFlagArm6State_ = false;

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
    nav_msgs::Odometry odometryMsg_;
    geometry_msgs::Pose robotBasePoseMsg_;
    geometry_msgs::Twist robotBaseTwistMsg_;
    sensor_msgs::JointState jointStateMsg_;
    control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg_;
    control_msgs::JointControllerState joint1ControllerStateMsg_;
    control_msgs::JointControllerState joint2ControllerStateMsg_;
    control_msgs::JointControllerState joint3ControllerStateMsg_;
    control_msgs::JointControllerState joint4ControllerStateMsg_;
    control_msgs::JointControllerState joint5ControllerStateMsg_;
    control_msgs::JointControllerState joint6ControllerStateMsg_;

    ros::Subscriber tfSub_;
    ros::Subscriber odometrySub_;
    ros::Subscriber linkStateSub_;
    ros::Subscriber jointStateSub_;
    ros::Subscriber jointTrajectoryPControllerStateSub_;
    /*
    ros::Subscriber joint1PControllerStateSub_;
    ros::Subscriber joint2PControllerStateSub_;
    ros::Subscriber joint3PControllerStateSub_;
    ros::Subscriber joint4PControllerStateSub_;
    ros::Subscriber joint5PControllerStateSub_;
    ros::Subscriber joint6PControllerStateSub_;
    */

    ros::Publisher baseTwistPub_;
    ros::Publisher armJointTrajectoryPub_;
    /*
    ros::Publisher armJoint1TrajectoryPub_;
    ros::Publisher armJoint2TrajectoryPub_;
    ros::Publisher armJoint3TrajectoryPub_;
    ros::Publisher armJoint4TrajectoryPub_;
    ros::Publisher armJoint5TrajectoryPub_;
    ros::Publisher armJoint6TrajectoryPub_;
    */
};

}  // namespace ocs2
