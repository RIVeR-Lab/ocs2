// LAST UPDATE: 2022.04.10
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h"

namespace ocs2 {

MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop(ros::NodeHandle& nh,
                                         MRT_ROS_Interface& mrt,
                                         std::string worldFrameName,
                                         scalar_t mrtDesiredFrequency,
                                         scalar_t mpcDesiredFrequency)
  : mrt_(mrt), 
    worldFrameName_(worldFrameName),
    mrtDesiredFrequency_(mrtDesiredFrequency), 
    mpcDesiredFrequency_(mpcDesiredFrequency),
    robotModelInfo_(mrt.getRobotModelInfo())
{
  dt_ = 1.0 / mrtDesiredFrequency_;

  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
      baseFrameName_ = robotModelInfo_.mobileBase.baseFrame;
      break;

    case RobotModelType::RobotArm:
      baseFrameName_ = robotModelInfo_.robotArm.baseFrame;
      break;

    case RobotModelType::MobileManipulator:
      baseFrameName_ = robotModelInfo_.mobileBase.baseFrame;
      break;
    
    default:
      baseFrameName_ = robotModelInfo_.mobileBase.baseFrame;
      std::cerr << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] ERROR: Invalid robot model type!";
      break;
  }

  setStateIndexMap();
  
  // SUBSCRIBE TO STATE INFO
  //// NUA TODO: Consider localization error
  //odometrySub_ = nh.subscribe("/jackal_velocity_controller/odom", 10, &MRT_ROS_Gazebo_Loop::odometryCallback, this);
  linkStateSub_ = nh.subscribe("/gazebo/link_states", 10, &MRT_ROS_Gazebo_Loop::linkStateCallback, this);
  //tfSub_ = nh.subscribe("/tf", 10, &MRT_ROS_Gazebo_Loop::tfCallback, this);
  
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
bool MRT_ROS_Gazebo_Loop::isArmStateInitialized()
{
  return initFlagArmState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRT_ROS_Gazebo_Loop::isStateInitialized()
{
  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {  
      //std::cout << "[MRT_ROS_Gazebo_Loop::isStateInitialized] RobotModelType::MobileBase" << std::endl;
      return initFlagBaseState_;
    }

    case RobotModelType::RobotArm:
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::isStateInitialized] RobotModelType::RobotArm" << std::endl;
      return initFlagArmState_;
    }

    case RobotModelType::MobileManipulator:
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::isStateInitialized] RobotModelType::MobileManipulator" << std::endl;
      return initFlagBaseState_ && initFlagArmState_;
    }

    default:
      std::cerr << "[MRT_ROS_Gazebo_Loop::isStateInitialized] ERROR: Invalid robot model type!";
      return initFlagBaseState_ && initFlagArmState_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::run(vector_t initTarget) 
{
  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Waiting for the initial policy ...");
  
  while(!isStateInitialized())
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] WARNING: State not initialized!" << std::endl;
    ros::spinOnce();
  }

  SystemObservation initObservation = getCurrentObservation(true);
  const TargetTrajectories initTargetTrajectories({0}, {initTarget}, {initObservation.input});

  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial state and policy
  while ( (!isStateInitialized() || !mrt_.initialPolicyReceived()) && ros::ok() && ros::master::check() ) 
  {
    mrt_.spinMRT();

    // Get initial observation
    initObservation = getCurrentObservation(true);

    mrt_.setCurrentObservation(initObservation);
    ros::Rate(mrtDesiredFrequency_).sleep();

    ros::spinOnce();
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

  std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] DEBUG INF" << std::endl;
  while(1);

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

    // Update observers for visualization
    for (auto& observer : observers_) 
    {
      observer -> update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    // Publish the control command 
    publishCommand();

    time_ += dt_;

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
  
  auto jointNames = mrt_.getRobotModelInfo().robotArm.jointNames;
  int n_joints = jointNames.size();
  stateIndexMap_.clear();
  int c;

  if (n_joints != jointTrajectoryControllerStatePtrMsg->joint_names.size())
  {
    throw std::runtime_error("[MRT_ROS_Gazebo_Loop::setStateIndexMap] Error: State dimension mismatch!");
  }
  
  for (int i = 0; i < n_joints; ++i)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::setStateIndexMap] jointTrajectoryControllerStatePtrMsg " << i << ": " << jointTrajectoryControllerStatePtrMsg -> joint_names[i] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::setStateIndexMap] jointNames " << i << ": " << jointNames[i] << std::endl;
    //std::cout << "" << std::endl;

    c = 0;
    while (jointTrajectoryControllerStatePtrMsg -> joint_names[c] != jointNames[i] && c < n_joints)
    {
      c++;
    }

    if (jointTrajectoryControllerStatePtrMsg -> joint_names[c] == jointNames[i])
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
  //std::cerr << "[MRT_ROS_Gazebo_Loop::tfCallback] START " << std::endl;
  //tf2_msgs::TFMessage tf_msg = *msg;

  tfListener_.lookupTransform(worldFrameName_, baseFrameName_, ros::Time(0), tf_robot_wrt_world_);

  initFlagBaseState_ = true;

  //std::cerr << "[MRT_ROS_Gazebo_Loop::tfCallback] START " << std::endl;
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
      robotBasePoseMsg_ = msg->pose[i];
      robotBaseTwistMsg_ = msg->twist[i];
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
  //std::cerr << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] START " << std::endl;

  jointTrajectoryControllerStateMsg_ = *msg;

  initFlagArmState_ = true;

  //std::cerr << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] END " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::updateFullModelState()
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] START " << std::endl;

  stateBase_.clear();
  inputBase_.clear();
  stateArm_.clear();

  //while(!initFlagBaseState_);
  //tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  geometry_msgs::Pose robotBasePoseMsg = robotBasePoseMsg_;
  geometry_msgs::Twist robotBaseTwistMsg = robotBaseTwistMsg_;

  //while(!initFlagArmState_);
  control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg = jointTrajectoryControllerStateMsg_;

  

  // Set mobile base state
  tf::Quaternion quat_robot_wrt_world(robotBasePoseMsg.orientation.x, 
                                      robotBasePoseMsg.orientation.y, 
                                      robotBasePoseMsg.orientation.z, 
                                      robotBasePoseMsg.orientation.w);
  tf::Matrix3x3 matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);

  stateBase_.push_back(robotBasePoseMsg.position.x);
  stateBase_.push_back(robotBasePoseMsg.position.y);
  stateBase_.push_back(yaw_robot_wrt_world);

  // Set mobile base input
  inputBase_.push_back(robotBaseTwistMsg_.linear.x);
  inputBase_.push_back(robotBaseTwistMsg_.angular.z);

  //std::cerr << "[MRT_ROS_Gazebo_Loop::updateFullModelState] mrt_.getArmStateDim(): " << mrt_.getArmStateDim() << std::endl;

  // Set arm state
  for (int i = 0; i < jointTrajectoryControllerStateMsg.joint_names.size(); ++i)
  {
    stateArm_.push_back(jointTrajectoryControllerStateMsg.actual.positions[stateIndexMap_[i]]);
  }

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] stateBase_ size: " << stateBase_.size() << std::endl;
  for (size_t i = 0; i < stateBase_.size(); i++)
  {
    std::cout << i << " -> " << stateBase_[i] << std::endl;
  }
  
  std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] stateArm_ size: " << stateArm_.size() << std::endl;
  for (size_t i = 0; i < stateArm_.size(); i++)
  {
    std::cout << i << " -> " << stateArm_[i] << std::endl;
  }
  */

  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] END " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Gazebo_Loop::getCurrentObservation(bool initFlag)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] START" << std::endl;
  
  auto stateDimBase = getStateDimBase(robotModelInfo_);
  auto stateDim = getStateDim(robotModelInfo_);
  auto inputDimBase = getInputDimBase(robotModelInfo_);
  auto modeStateDim = getModeStateDim(robotModelInfo_);
  auto modeInputDim = getModeInputDim(robotModelInfo_);

  SystemObservation currentObservation;
  currentObservation.mode = 0;
  currentObservation.time = time_;
  currentObservation.state.setZero(modeStateDim);
  currentObservation.full_state.setZero(stateDim);
  currentObservation.input.setZero(modeInputDim);

  // Set current observation input
  if (!initFlag)
  {
    currentObservation.input = currentInput_;
  }

  updateFullModelState();

  auto stateBase = stateBase_;
  auto inputBase = inputBase_;
  auto stateArm = stateArm_;

  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {  
      // Set state
      currentObservation.state[0] = stateBase[0];
      currentObservation.state[1] = stateBase[1];
      currentObservation.state[2] = stateBase[2];

      // Set input
      if (initFlag)
      {
        currentObservation.input[0] = inputBase[0];
        currentObservation.input[1] = inputBase[1];
      }

      // Set full state
      currentObservation.full_state = currentObservation.state;
      break;
    }

    case RobotModelType::RobotArm:
    { 
      // Set state and input
      for (size_t i = 0; i < stateArm.size(); i++)
      {
        currentObservation.state[i] = stateArm[i];

        if(initFlag)
        {
          currentObservation.input[i] = stateArm[i];
        }
      }

      // Set full state
      currentObservation.full_state = currentObservation.state;
      break;
    }

    case RobotModelType::MobileManipulator:
    {
      switch (robotModelInfo_.modelMode)
      {
        case ModelMode::BaseMotion:
        {
          // Set state
          currentObservation.state[0] = stateBase[0];
          currentObservation.state[1] = stateBase[1];
          currentObservation.state[2] = stateBase[2];

          // Set input
          if (initFlag)
          {
            currentObservation.input[0] = inputBase[0];
            currentObservation.input[1] = inputBase[1];
          }

          // Set full state
          currentObservation.full_state[0] = stateBase[0];
          currentObservation.full_state[1] = stateBase[1];
          currentObservation.full_state[2] = stateBase[2];

          for (size_t i = 0; i < stateArm.size(); i++)
          {
            currentObservation.full_state[stateDimBase + i] = stateArm[i];
          }

          break;
        }

        case ModelMode::ArmMotion:
        {
          // Set full state base
          currentObservation.full_state[0] = stateBase[0];
          currentObservation.full_state[1] = stateBase[1];
          currentObservation.full_state[2] = stateBase[2];

          for (size_t i = 0; i < stateArm.size(); i++)
          {
            // Set state
            currentObservation.state[i] = stateArm[i];

            if(initFlag)
            {
              currentObservation.input[i] = stateArm[i];
            }

            // Set full state arm
            currentObservation.full_state[stateDimBase + i] = stateArm[i];
          }
          break;
        }

        case ModelMode::WholeBodyMotion:
        {
          // Set state
          currentObservation.state[0] = stateBase[0];
          currentObservation.state[1] = stateBase[1];
          currentObservation.state[2] = stateBase[2];

          // Set input
          if (initFlag)
          {
            currentObservation.input[0] = inputBase[0];
            currentObservation.input[1] = inputBase[1];
          }

          for (size_t i = 0; i < stateArm.size(); i++)
          {
            currentObservation.state[stateDimBase + i] = stateArm[i];

            if(initFlag)
            {
              currentObservation.input[inputDimBase + i] = stateArm[i];
            }
          }

          // Set full state
          currentObservation.full_state = currentObservation.state;
          break;
        }

        default:
          std::cerr << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] ERROR: Invalid model mode!";
          while(1);
          break;
      }
      break;
    }

    default:
    {
      std::cerr << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] ERROR: Invalid robot model type!";
      while(1);
      break;
    }
  }

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] currentObservation.state size: " << currentObservation.state.size() << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;

  std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] currentObservation.input size: " << currentObservation.input.size() << std::endl;
  std::cout << currentObservation.input << std::endl << std::endl;

  std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;
  std::cout << currentObservation.full_state << std::endl << std::endl;
  */

  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] END" << std::endl;

  return currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Gazebo_Loop::publishCommand()
{
  //std::cout << "[OCS2_MRT_Loop::publishCommand] START" << std::endl;

  geometry_msgs::Twist baseTwistMsg;
  trajectory_msgs::JointTrajectory armJointTrajectoryMsg;

  // Set mobile base command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistMsg.linear.x = currentInput_[0];
    baseTwistMsg.angular.z = currentInput_[1];
  }

  // Set arm command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    int baseOffset = 0;
    if (mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
    {
      baseOffset = mrt_.getRobotModelInfo().mobileBase.stateDim;
    }
    int n_joints = mrt_.getRobotModelInfo().robotArm.jointNames.size();
    armJointTrajectoryMsg.joint_names.resize(n_joints);

    PrimalSolution primalSolution = mrt_.getPolicy();
    auto nextState = primalSolution.getDesiredState(time_ + dt_);
    
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(n_joints);
    jtp.time_from_start = ros::Duration(dt_);

    for (int i = 0; i < n_joints; ++i)
    {
      armJointTrajectoryMsg.joint_names[i] = mrt_.getRobotModelInfo().robotArm.jointNames[i];
      jtp.positions[i] = nextState[baseOffset + i];
    }
    armJointTrajectoryMsg.points.push_back(jtp);
  }

  // Publish command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistPub_.publish(baseTwistMsg);
  }

  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    armJointTrajectoryPub_.publish(armJointTrajectoryMsg);
  }

  //std::cout << "[OCS2_MRT_Loop::publishCommand] END" << std::endl;
}

}  // namespace ocs2
