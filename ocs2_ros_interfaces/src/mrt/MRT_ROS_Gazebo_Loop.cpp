/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h"

namespace ocs2 {

MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop(ros::NodeHandle& nh,
                                         MRT_ROS_Interface& mrt,
                                         std::string worldFrameName,
                                         std::string baseFrameName,
                                         size_t stateDim,
                                         size_t inputDim,
                                         std::vector<std::string> dofNames,
                                         scalar_t mrtDesiredFrequency,
                                         scalar_t mpcDesiredFrequency)
  : mrt_(mrt), 
    worldFrameName_(worldFrameName),
    baseFrameName_(baseFrameName),
    stateDim_(stateDim), 
    inputDim_(inputDim), 
    dofNames_(dofNames), 
    mrtDesiredFrequency_(mrtDesiredFrequency), 
    mpcDesiredFrequency_(mpcDesiredFrequency)
{
  
  std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] mrtDesiredFrequency_: " << mrtDesiredFrequency_ << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] mpcDesiredFrequency_: " << mpcDesiredFrequency_ << std::endl;
  
  dt_ = 1.0 / mrtDesiredFrequency_;

  setStateIndexMap();
  
  // SUBSCRIBE TO STATE INFO
  // NUA TODO: Consider localization error
  //odometrySub_ = nh.subscribe("/jackal_velocity_controller/odom", 10, &MRT_ROS_Gazebo_Loop::odometryCallback, this);
  //linkStateSub_ = nh.subscribe("/gazebo/link_states", 10, &MRT_ROS_Gazebo_Loop::linkStateCallback, this);
  tfSub_ = nh.subscribe("/tf", 10, &MRT_ROS_Gazebo_Loop::tfCallback, this);
  
  //jointStateSub_ = nh.subscribe("/joint_states", 10, &MRT_ROS_Gazebo_Loop::jointStateCallback, this);
  jointTrajectoryPControllerStateSub_ = nh.subscribe("/arm_controller/state", 10, &MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback, this);
  
  // Publish control inputs (base and/or arm)
  baseTwistPub_ = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 100);
  armJointTrajectoryPub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 100);

  if (mrtDesiredFrequency_ < 0) 
  {
    throw std::runtime_error("[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] Error: MRT loop frequency should be a positive number!");
  }

  if (mpcDesiredFrequency_ > 0) 
  {
    ROS_WARN_STREAM("[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] Warning: MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void MRT_ROS_Gazebo_Loop::setRobotModelType(std::string robotModelType)
{
  robotModelType_ = robotModelType;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRT_ROS_Gazebo_Loop::isArmStateInitialized()
{
  return initFlagArmState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRT_ROS_Gazebo_Loop::isStateInitialized()
{
  return initFlagBaseState_ && initFlagArmState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::run(const TargetTrajectories& initTargetTrajectories) 
{
  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Waiting for the initial policy ...");
  SystemObservation initObservation;

  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial state and policy
  while (!isStateInitialized() && !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
  {
    mrt_.spinMRT();

    // Get initial observation
    initObservation = getCurrentObservation(true);

    //std::cout << "OCS2_MRT_Loop::run -> initObservation:" << std::endl;
    //std::cout << initObservation << std::endl;

    mrt_.setCurrentObservation(initObservation);
    ros::Rate(mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Initial policy has been received.");

  currentInput_ = initObservation.input;

  mrtLoop();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Gazebo_Loop::forwardSimulation(const SystemObservation& currentObservation) 
{
  std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] START" << std::endl;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt_;
  
  if (mrt_.isRolloutSet()) 
  {  // If available, use the provided rollout as to integrate the dynamics.
    std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] INTEGRATION" << std::endl;
    
    mrt_.rolloutPolicy(currentObservation.time, 
                       currentObservation.state, 
                       dt_, 
                       nextObservation.state, 
                       nextObservation.input,
                       nextObservation.mode);
  } 
  else 
  {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
    std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] INTERPOLATION" << std::endl;
    
    mrt_.evaluatePolicy(currentObservation.time + dt_, 
                        currentObservation.state, 
                        nextObservation.state, 
                        nextObservation.input,
                        nextObservation.mode);
  }

  std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] END" << std::endl;

  return nextObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::mrtLoop() 
{
  // Loop variables
  SystemObservation currentObservation;
  SystemObservation targetObservation;
  PrimalSolution currentPolicy;

  // Update the policy
  mrt_.updatePolicy();

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) 
  {
    //std::cout << "---------------" << std::endl;
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] START" << std::endl;

    mrt_.reset();

    while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
    {
      mrt_.spinMRT();

      // Get current observation
      currentObservation = getCurrentObservation(false);

      // Set current observation
      mrt_.setCurrentObservation(currentObservation);
    }

    // Update the policy if a new one was received
    mrt_.updatePolicy();

    currentPolicy = mrt_.getPolicy();
    currentInput_ = currentPolicy.getDesiredInput(time_);

    //std::cout << "[OCS2_MRT_Loop::mrtLoop] currentInput size: " << currentInput_.size() << std::endl;

    // Update observers for visualization
    for (auto& observer : observers_) 
    {
      observer -> update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    // NUA NOTE: Instead used interpolation in publishCommand, which provides much stable commands.
    //targetObservation = forwardSimulation(currentObservation);

    // Publish the control command 
    publishCommand();

    time_ += dt_;

    /*
    if (time_ > 3*dt_)
    {
      std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] BEFORE INF" << std::endl;
      while(1);
    }
    */

    //std::cout << "[OCS2_MRT_Loop::mrtLoop] END" << std::endl;
    //std::cout << "---------------" << std::endl << std::endl;

    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::setStateIndexMap()
{
  boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> jointTrajectoryControllerStatePtrMsg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/arm_controller/state");
  
  int n_joints = dofNames_.size();
  stateIndexMap_.clear();
  int c;

  if (dofNames_.size() != jointTrajectoryControllerStatePtrMsg->joint_names.size())
  {
    throw std::runtime_error("[MRT_ROS_Gazebo_Loop::setStateIndexMap] Error: State dimension mismatch!");
  }
  
  for (int i = 0; i < n_joints; ++i)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::setStateIndexMap] jointTrajectoryControllerStatePtrMsg " << i << ": " << jointTrajectoryControllerStatePtrMsg -> joint_names[i] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::setStateIndexMap] dofNames_ " << i << ": " << dofNames_[i] << std::endl;
    //std::cout << "" << std::endl;

    c = 0;
    while (jointTrajectoryControllerStatePtrMsg -> joint_names[c] != dofNames_[i] && c < n_joints)
    {
      c++;
    }

    if (jointTrajectoryControllerStatePtrMsg -> joint_names[c] == dofNames_[i])
    {
      stateIndexMap_.push_back(c);
    }
  }

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::setStateIndexMap] stateIndexMap_:" << std::endl;
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    std::cout << i << " -> " << stateIndexMap_[i] << std::endl;
  }
  */
}

void MRT_ROS_Gazebo_Loop::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //tf2_msgs::TFMessage tf_msg = *msg;

  tfListener_.lookupTransform(worldFrameName_, baseFrameName_, ros::Time(0), tf_robot_wrt_world_);

  initFlagBaseState_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometryMsg_ = *msg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  for (int i = 0; i < msg -> name.size(); ++i)
  {
    if (msg -> name[i] == "mobiman::base_link")
    {
      robotBasePoseMsg_ = msg -> pose[i];
    }
  }

  initFlagBaseState_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointStateMsg_ = *msg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  jointTrajectoryControllerStateMsg_ = *msg;

  initFlagArmState_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::updateFullModelState()
{
  tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg = jointTrajectoryControllerStateMsg_;

  baseState_.clear();
  armState_.clear();

  // Set mobile base states
  if (mrt_.getRobotModelInfo().mobileBase.stateDim != 3)
  {
    std::cerr << "[MRT_ROS_Gazebo_Loop::updateFullModelState] ERROR: Base state dimension mismatch!" << std::endl;
  }
  else
  {
    tf::Quaternion quat_robot_wrt_world = tf_robot_wrt_world.getRotation();
    tf::Matrix3x3 matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
    double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
    matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);

    baseState_.push_back(tf_robot_wrt_world.getOrigin().x());
    baseState_.push_back(tf_robot_wrt_world.getOrigin().y());
    baseState_.push_back(yaw_robot_wrt_world);
  }

  //std::cerr << "[MRT_ROS_Gazebo_Loop::updateFullModelState] mrt_.getArmStateDim(): " << mrt_.getArmStateDim() << std::endl;

  // Set arm states
  if (mrt_.getRobotModelInfo().robotArm.stateDim != jointTrajectoryControllerStateMsg.joint_names.size())
  {
    std::cerr << "[MRT_ROS_Gazebo_Loop::updateFullModelState] ERROR: Arm state dimension mismatch!" << std::endl;
  }
  else
  {
    for (int i = 0; i < jointTrajectoryControllerStateMsg.joint_names.size(); ++i)
    {
      armState_.push_back(jointTrajectoryControllerStateMsg.actual.positions[stateIndexMap_[i]]);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Gazebo_Loop::getCurrentObservation(bool initFlag)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] START" << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] inputDim_: " << inputDim_ << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] stateDim_: " << stateDim_ << std::endl;
  
  SystemObservation currentObservation;
  currentObservation.mode = 0;
  currentObservation.time = time_;
  currentObservation.input.setZero(inputDim_);
  currentObservation.state.setZero(stateDim_);

  // Set current observation input
  if (!initFlag)
  {
    currentObservation.input = currentInput_;
  }

  updateFullModelState();

  // Set mobile base states
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    currentObservation.state[0] = baseState_[0];
    currentObservation.state[1] = baseState_[1];
    currentObservation.state[2] = baseState_[2];
  }

  // Set arm states
  int baseOffset = mrt_.getRobotModelInfo().mobileBase.stateDim;
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] baseOffset: " << baseOffset << std::endl;

    for (size_t i = 0; i < armState_.size(); i++)
    {
      currentObservation.state[baseOffset + i] = armState_[i];
    }
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] END" << std::endl << std::endl;

  return currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::publishCommand()
{
  //std::cout << "OCS2_MRT_Loop::publishCommand -> START" << std::endl;

  geometry_msgs::Twist baseTwistMsg;
  trajectory_msgs::JointTrajectory armJointTrajectoryMsg;

  // Set mobile base command
  int baseOffset = mrt_.getRobotModelInfo().mobileBase.stateDim;
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistMsg.linear.x = currentInput_[0];
    baseTwistMsg.angular.z = currentInput_[1];
  }

  // Set arm command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    int n_joints = dofNames_.size();
    armJointTrajectoryMsg.joint_names.resize(n_joints);

    PrimalSolution primalSolution = mrt_.getPolicy();
    auto nextState = primalSolution.getDesiredState(time_ + dt_);
    
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(n_joints);
    jtp.time_from_start = ros::Duration(dt_);

    for (int i = 0; i < n_joints; ++i)
    {
      armJointTrajectoryMsg.joint_names[i] = dofNames_[i];
      jtp.positions[i] = nextState[baseOffset + i];
    }
    armJointTrajectoryMsg.points.push_back(jtp);
  }

  // Publish command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistPub_.publish(baseTwistMsg);
  }

  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    armJointTrajectoryPub_.publish(armJointTrajectoryMsg);
  }

  //std::cout << "OCS2_MRT_Loop::publishCommand -> END" << std::endl;
}

}  // namespace ocs2
