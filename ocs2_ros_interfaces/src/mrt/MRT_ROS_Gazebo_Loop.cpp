// LAST UPDATE: 2023.09.07
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
                                         std::string targetMsg,
                                         std::string baseStateMsg,
                                         std::string armStateMsg,
                                         std::string baseControlMsg,
                                         std::string armControlMsg,
                                         double err_threshold_pos,
                                         double err_threshold_ori,
                                         scalar_t mrtDesiredFrequency,
                                         scalar_t mpcDesiredFrequency,
                                         bool drlFlag,
                                         std::string logSavePathRel)
  : mrt_(mrt), 
    worldFrameName_(worldFrameName),
    err_threshold_pos_(err_threshold_pos),
    err_threshold_ori_(err_threshold_ori),
    mrtDesiredFrequency_(mrtDesiredFrequency), 
    mpcDesiredFrequency_(mpcDesiredFrequency),
    drlFlag_(drlFlag),
    robotModelInfo_(mrt.getRobotModelInfo()),
    dataPathReL_(logSavePathRel)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] START" << std::endl;

  timer1_.startTimer();

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
      std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] ERROR: Invalid robot model type!" << std::endl;
      break;
  }

  dataCollectionFlag_ = (dataPathReL_ == "") ? false : true;

  timer2_.startTimer();
  updateStateIndexMap(armStateMsg, updateIndexMapFlag_);
  timer2_.endTimer();
  
  // Subscribers
  //// NUA TODO: Consider localization error
  //odometrySub_ = nh.subscribe("/jackal_velocity_controller/odom", 5, &MRT_ROS_Gazebo_Loop::odometryCallback, this);
  if (baseStateMsg == "")
  {
    std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] DEFAULT: Base state info is acquired by /tf msg." << std::endl;
    tfFlag_ = true;
    tfSub_ = nh.subscribe("/tf", 5, &MRT_ROS_Gazebo_Loop::tfCallback, this);
  }
  else
  {
    linkStateSub_ = nh.subscribe(baseStateMsg, 5, &MRT_ROS_Gazebo_Loop::linkStateCallback, this);
  }
  
  std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] armStateMsg: " << armStateMsg << std::endl;
  jointStateSub_ = nh.subscribe(armStateMsg, 5, &MRT_ROS_Gazebo_Loop::jointStateCallback, this);
  //jointTrajectoryControllerStateSub_ = nh.subscribe(armStateMsg, 5, &MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback, this);

  if (drlFlag_)
  {
    selfCollisionDistanceSub_ = nh.subscribe(selfCollisionDistanceMsgName_, 5, &MRT_ROS_Gazebo_Loop::selfCollisionDistanceCallback, this);
    extCollisionDistanceSub_ = nh.subscribe(extCollisionDistanceMsgName_, 5, &MRT_ROS_Gazebo_Loop::extCollisionDistanceCallback, this);
    pointsOnRobotSub_ = nh.subscribe(pointsOnRobotMsgName_, 5, &MRT_ROS_Gazebo_Loop::pointsOnRobotCallback, this);
  }

  currentTarget_.resize(7);

  // Publishers
  baseTwistPub_ = nh.advertise<geometry_msgs::Twist>(baseControlMsg, 1);
  armJointTrajectoryPub_ = nh.advertise<trajectory_msgs::JointTrajectory>(armControlMsg, 1);
  armJointVelocityPub_ = nh.advertise<kinova_msgs::JointVelocity>("/arm_controller/velocity", 1);

  /// Clients
  attachClient_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  detachClient_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
  //setTaskModeClient_ = nh.serviceClient<ocs2_msgs::setInt>("/set_task_mode");
  setPickedFlagClient_ = nh.serviceClient<ocs2_msgs::setBool>("/set_picked_flag");
  setSystemObservationClient_ = nh.serviceClient<ocs2_msgs::setSystemObservation>("/set_system_observation");

  /// Services
  //setTaskModeService_ = nh.advertiseService("set_task_mode", &MRT_ROS_Gazebo_Loop::setTaskModeSrv, this);
  setTaskService_ = nh.advertiseService("set_task", &MRT_ROS_Gazebo_Loop::setTaskSrv, this);

  if (mrtDesiredFrequency_ < 0) 
  {
    throw std::runtime_error("[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] Error: MRT loop frequency should be a positive number!");
  }

  if (mpcDesiredFrequency_ > 0) 
  {
    ROS_WARN_STREAM("[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] Warning: MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }

  timer1_.endTimer();

  std::cout << "\n### [MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] MRT_ROS Benchmarking";
  std::cout << "\n### timer1_";
  std::cout << "\n###   Maximum : " << timer1_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << timer1_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << timer1_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### timer2_";
  std::cout << "\n###   Maximum : " << timer2_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << timer2_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << timer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

  std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::isArmStateInitialized()
{
  //return initFlagArmState_ && initFlagArmState2_;
  return initFlagArmState_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
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
      return isArmStateInitialized();
    }

    case RobotModelType::MobileManipulator:
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::isStateInitialized] RobotModelType::MobileManipulator" << std::endl;
      return initFlagBaseState_ && isArmStateInitialized();
    }

    default:
      std::cerr << "[MRT_ROS_Gazebo_Loop::isStateInitialized] ERROR: Invalid robot model type!";
      return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::run(vector_t initTarget) 
{
  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Waiting for the initial policy...");
  
  //currentTarget_ = initTarget;
  taskEndFlag_ = true;
  shutDownFlag_ = false;
  currentTarget_ = initTarget;

  std::cout << "[MRT_ROS_Gazebo_Loop::run] Waiting for the state to be initialized..." << std::endl;
  while(!isStateInitialized()){ros::spinOnce();}
  
  if (drlFlag_)
  {
    std::cout << "[MRT_ROS_Gazebo_Loop::run] Waiting for the initFlagSelfCollision_, initFlagExtCollision_, initFlagPointsOnRobot_ to be initialized..." << std::endl;
    while(!initFlagSelfCollision_ && !initFlagExtCollision_ && !initFlagPointsOnRobot_){ros::spinOnce();}
  }

  //ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] BEFORE getCurrentObservation");
  SystemObservation initObservation = getCurrentObservation(true);

  setSystemObservation(initObservation);

  std::cout << "[MRT_ROS_Gazebo_Loop::run] Waiting for the target to be received..." << std::endl;
  while(!targetReceivedFlag_ && !shutDownFlag_)
  {
    mrtShutDownFlag_ = getenv("mrtShutDownFlag");
    if (mrtShutDownFlag_ == "true")
    {
      shutDownFlag_ = true;
    }
    ros::spinOnce();
  }
  vector_t currentTarget = currentTarget_;
  //currentTarget_ = initTarget;

  std::cout << "[MRT_ROS_Gazebo_Loop::run] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

  if (!shutDownFlag_)
  {
    /*
    std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE initTarget size: " << initTarget.size() << std::endl;
    for (size_t i = 0; i < initTarget.size(); i++)
    {
      std::cout << i << " -> " << initTarget[i] << std::endl;
    }
    std::cout << "------------" << std::endl;

    std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE currentTarget size: " << currentTarget.size() << std::endl;
    for (size_t i = 0; i < currentTarget.size(); i++)
    {
      std::cout << i << " -> " << currentTarget[i] << std::endl;
    }
    std::cout << "------------" << std::endl;

    std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE initObservation.input size: " << initObservation.input.size() << std::endl;
    for (size_t i = 0; i < initObservation.input.size(); i++)
    {
      std::cout << i << " -> " << initObservation.input[i] << std::endl;
    }
    std::cout << "------------" << std::endl;
    */

    const TargetTrajectories initTargetTrajectories({0}, {currentTarget}, {initObservation.input});

    // Reset MPC node
    mrt_.resetMpcNode(initTargetTrajectories);

    // Wait for the initial state and policy
    while ( !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check() ) 
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::run] START INIT WHILE" << std::endl;
      mrt_.spinMRT();

      // Get initial observation
      initObservation = getCurrentObservation(true);

      /*std::cout << "[MRT_ROS_Gazebo_Loop::run] initObservation: " << initObservation.state.size() << std::endl;
      for (size_t i = 0; i < initObservation.state.size(); i++)
      {
        std::cout << i << " -> " << initObservation.state[i] << std::endl;
      }
      */

      //std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE setCurrentObservation" << std::endl;
      mrt_.setCurrentObservation(initObservation);
      //std::cout << "[MRT_ROS_Gazebo_Loop::run] AFTER setCurrentObservation" << std::endl;

      ros::Rate(mrtDesiredFrequency_).sleep();

      ros::spinOnce();
      //std::cout << "[MRT_ROS_Gazebo_Loop::run] END INIT WHILE" << std::endl;
    }
    ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Initial policy has been received.");

    currentInput_ = initObservation.input;

    //std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE mrtLoop" << std::endl;
    mrtLoop();
    //std::cout << "[MRT_ROS_Gazebo_Loop::run] AFTER mrtLoop" << std::endl;
  }

  std::cout << "[MRT_ROS_Gazebo_Loop::run] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::getInitTarget(vector_t& initTarget)
{
  std::string eeFrame = robotModelInfo_.robotArm.eeFrame;
  if (robotModelInfo_.robotModelType == RobotModelType::MobileBase)
  {
    eeFrame = robotModelInfo_.mobileBase.baseFrame;
  }

  tf::StampedTransform tf_ee_wrt_world;
  try
  {
    tfListener_.waitForTransform(worldFrameName_, eeFrame, ros::Time::now(), ros::Duration(1.0));
    tfListener_.lookupTransform(worldFrameName_, eeFrame, ros::Time(0), tf_ee_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MRT_ROS_Gazebo_Loop::getInitTarget] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  initTarget.resize(7);
  initTarget.head(3) << tf_ee_wrt_world.getOrigin().x(), 
                        tf_ee_wrt_world.getOrigin().y(), 
                        tf_ee_wrt_world.getOrigin().z();
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(tf_ee_wrt_world.getRotation().w(), 
                                                    tf_ee_wrt_world.getRotation().x(), 
                                                    tf_ee_wrt_world.getRotation().y(), 
                                                    tf_ee_wrt_world.getRotation().z()).coeffs();
  
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] eeFrame: " << eeFrame << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.x: " << initTarget(0) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.y: " << initTarget(1) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.z: " << initTarget(2) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.qx: " << initTarget(3) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.qy: " << initTarget(4) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.qz: " << initTarget(5) << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::getInitTarget] initTarget.qw: " << initTarget(6) << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::getCurrentState(vector_t& currentState)
{
  updateFullModelState();

  std::vector<double> statePositionBase = statePositionBase_;
  std::vector<double> statePositionArm = statePositionArm_;

  int stateSize = getStateDim(robotModelInfo_);
  currentState.resize(stateSize);

  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {
      for (size_t i = 0; i < statePositionBase.size(); i++)
      {
        currentState(i) = statePositionBase[i];
      }
      break;
    }

    case RobotModelType::RobotArm:
    {
      for (size_t i = 0; i < statePositionArm.size(); i++)
      {
        currentState(i) = statePositionArm[i];
      }
      break;
    }

    case RobotModelType::MobileManipulator:
    {
      int offset = statePositionBase.size();
      for (size_t i = 0; i < statePositionBase.size(); i++)
      {
        currentState(i) = statePositionBase[i];
      }

      for (size_t i = 0; i < statePositionArm.size(); i++)
      {
        currentState(offset + i) = statePositionArm[i];
      }
      
      break;
    }
    
    default:
      std::cerr << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] ERROR: Invalid robot model type!";
      break;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
SystemObservation MRT_ROS_Gazebo_Loop::forwardSimulation(const SystemObservation& currentObservation) 
{
  std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] START" << std::endl;

  std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] DEBUG INF" << std::endl;
  while(1);

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt_;
  
  if (mrt_.isRolloutSet()) 
  {  
    // If available, use the provided rollout as to integrate the dynamics.
    std::cout << "[MRT_ROS_Gazebo_Loop::forwardSimulation] INTEGRATION" << std::endl;
    
    mrt_.rolloutPolicy(currentObservation.time, 
                       currentObservation.state, 
                       dt_, 
                       nextObservation.state, 
                       nextObservation.input,
                       nextObservation.mode);
  } 
  else 
  {  
    // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
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

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::mrtLoop() 
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START" << std::endl;

  // Loop variables
  SystemObservation currentObservation;
  std::vector<double> currentStateVelocityBase;
  //SystemObservation targetObservation;
  PrimalSolution currentPolicy;

  shutDownFlag_ = false;
  drlActionResult_ = -1;

  // Update the policy
  mrt_.updatePolicy();

  ros::Rate simRate(mrtDesiredFrequency_);

  while (!shutDownFlag_ && ros::ok() && ros::master::check()) 
  {
    //mrtExitFlag_ = false;
    //publishMRTExitFlag();
    
    //setenv("mrtExitFlag", "false", 1);
    //std::cout << "[GLOBAL ENVIRONMENT] MRT_FLAG: " << getenv("mpcProblemReadyFlag") << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] shutDownFlag: " << shutDownFlag_ << std::endl;

    //robotModelInfo_ = mrt_.getRobotModelInfo();
    //std::cout << "---------------" << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START while" << std::endl;

    mrt_.reset();
    int initPolicyCtr = 0;

    mrt_.spinMRT();

    // Get current observation
    currentObservation = getCurrentObservation(false);
    currentStateVelocityBase = stateVelocityBase_;

    // Set current observation
    mrt_.setCurrentObservation(currentObservation);

    mrtShutDownFlag_ = getenv("mrtShutDownFlag");
    //std::cout << "[MPC_ROS_Interface::spin] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

    if (mrtShutDownFlag_ == "true")
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::spin] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
      shutDownFlag_ = true;
    }

    while (!shutDownFlag_ && !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
    {
      if (initPolicyCtr > initPolicyCtrMax_)
      {
        std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] THATS ENOUGH WAITING FOR THE POLICY!" << std::endl;
        shutDownFlag_ = true;
      }

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START spinMRT" << std::endl;
      mrt_.spinMRT();

      // Get current observation
      currentObservation = getCurrentObservation(false);
      currentStateVelocityBase = stateVelocityBase_;

      // Set current observation
      mrt_.setCurrentObservation(currentObservation);

      mrtShutDownFlag_ = getenv("mrtShutDownFlag");
      //std::cout << "[MPC_ROS_Interface::spin] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

      if (mrtShutDownFlag_ == "true")
      {
        //std::cout << "[MRT_ROS_Gazebo_Loop::spin] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
        shutDownFlag_ = true;
      }

      //shutDownFlag_ = mrt_.getShutDownFlag();

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] END spinMRT" << std::endl;

      initPolicyCtr++;
    }

    mrt_.updatePolicy();

    int taskMode = taskMode_;
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] taskMode: " << taskMode << std::endl;

    if (isPickDropPoseReached(taskMode))
    {
      std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START PICK/DROP" << std::endl;
      
      gazebo_ros_link_attacher::Attach srv;
      srv.request.model_name_1 = robotModelInfo_.robotName;
      srv.request.link_name_1 = robotModelInfo_.robotArm.jointFrameNames.back();
      srv.request.model_name_2 = currentTargetName_;
      srv.request.link_name_2 = currentTargetAttachLinkName_;

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] taskMode: " << taskMode << std::endl;
      if (taskMode == 1)
      {
        if (attachClient_.call(srv))
        {
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] response: " << srv.response.ok << std::endl;
          taskMode = 2;
          pickedFlag_ = true;
          taskEndFlag_ = true;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[MRT_ROS_Gazebo_Loop::mrtLoop] ERROR: Failed to call service!");
        }
      }
      else if (taskMode == 2)
      {
        std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] DROP" << std::endl;
        if (detachClient_.call(srv))
        {
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] response: " << srv.response.ok << std::endl;
          taskMode = 1;
          pickedFlag_ = false;
          taskEndFlag_ = true;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[MRT_ROS_Gazebo_Loop::mrtLoop] ERROR: Failed to call service!");
        }
      }

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] END PICK/DROP" << std::endl;
    }
    else
    {
      if (!shutDownFlag_)
      {
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START updatePolicy" << std::endl;
        // Update the policy if a new one was received
        mrt_.updatePolicy();
        currentPolicy = mrt_.getPolicy();
        currentInput_ = currentPolicy.getDesiredInput(time_);

        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START observer" << std::endl;
        // Update observers for visualization
        for (auto& observer : observers_) 
        {
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] OBSERVING..." << std::endl;
          observer->update(currentObservation, currentPolicy, mrt_.getCommand());
        }

        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] START publishCommand" << std::endl;
        // Publish the control command 
        publishCommand(currentPolicy, currentObservation, currentStateVelocityBase);

        if (drlFlag_)
        {
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] BEFORE checkCollision" << std::endl;
          checkCollision();

          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] BEFORE checkRollover" << std::endl;
          checkRollover();

          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] OUT time_: " << time_ << std::endl;
          if (time_ > drlActionTimeHorizon_)
          {
            shutDownFlag_ = true;
            drlActionResult_ = 0;
            //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] IN time_: " << time_ << std::endl;
          }
        }
        
        writeData();

        time_ += dt_;

        //shutDownFlag_ = mrt_.getShutDownFlag();

        ros::spinOnce();
        simRate.sleep();
      }
    }
    
    mrtShutDownFlag_ = getenv("mrtShutDownFlag");
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

    if (mrtShutDownFlag_ == "true")
    {
      shutDownFlag_ = true;
    }
    
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] END while" << std::endl;
    //std::cout << "---------------" << std::endl << std::endl;
  }

  writeData(true);

  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int MRT_ROS_Gazebo_Loop::getDRLActionResult()
{
  return drlActionResult_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::setStateIndexMap(std::vector<int>& stateIndexMap)
{
  stateIndexMap_ = stateIndexMap;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::setTargetReceivedFlag(bool targetReceivedFlag)
{
  targetReceivedFlag_ = targetReceivedFlag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::setTaskMode(int taskMode)
{
  taskMode_ = taskMode;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::setDRLFlag(bool drlFlag)
{
  drlFlag_ = drlFlag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::setDRLActionTimeHorizon(double drlActionTimeHorizon)
{
  drlActionTimeHorizon_ = drlActionTimeHorizon;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::updateStateIndexMap(std::string& armStateMsg, bool updateStateIndexMapFlag)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] START" << std::endl;
  
  auto jointNames = mrt_.getRobotModelInfo().robotArm.jointNames;
  int n_joints = jointNames.size();
  stateIndexMap_.clear();
  int c;

  if (updateStateIndexMapFlag)
  {
    //boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> jointTrajectoryControllerStatePtrMsg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(armStateMsg);
    //if (n_joints != jointTrajectoryControllerStatePtrMsg->joint_names.size())
    boost::shared_ptr<sensor_msgs::JointState const> jointStatePtrMsg = ros::topic::waitForMessage<sensor_msgs::JointState>(armStateMsg);

    /*
    std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] jointStatePtrMsg->name.size(): " << jointStatePtrMsg->name.size() << std::endl; 
    for (size_t i = 0; i < jointStatePtrMsg->name.size(); i++)
    {
      std::cout << jointStatePtrMsg->name[i] << std::endl;
    }
    */

    for (size_t i = 0; i < n_joints; i++)
    {
      auto it = find(jointStatePtrMsg->name.begin(), jointStatePtrMsg->name.end(), jointNames[i]);
  
      // If element was found
      if (it != jointStatePtrMsg->name.end()) 
      {
          // calculating the index
          int index = it - jointStatePtrMsg->name.begin();
          stateIndexMap_.push_back(index);
      }
      else
      {
        throw std::runtime_error("[MRT_ROS_Gazebo_Loop::updateStateIndexMap] Error: Joint " + jointNames[i] + " not found!");
      }
    }
  }
  else
  {
    for (int i = 0; i < n_joints; ++i)
    {
      stateIndexMap_.push_back(i);
    }
  }

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] stateIndexMap_ size: " << stateIndexMap_.size() << std::endl;
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    std::cout << i << " -> " << stateIndexMap_[i] << std::endl;
  }
  */

  //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] END" << std::endl;
}

void MRT_ROS_Gazebo_Loop::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //std::cerr << "[MRT_ROS_Gazebo_Loop::tfCallback] START " << std::endl;
  //tf2_msgs::TFMessage tf_msg = *msg;

  try
  {
    tfListener_.waitForTransform(worldFrameName_, baseFrameName_, ros::Time::now(), ros::Duration(5.0));
    tfListener_.lookupTransform(worldFrameName_, baseFrameName_, ros::Time(0), tf_robot_wrt_world_);

    tfListener_.waitForTransform(worldFrameName_, robotModelInfo_.robotArm.eeFrame, ros::Time::now(), ros::Duration(5.0));
    tfListener_.lookupTransform(worldFrameName_, robotModelInfo_.robotArm.eeFrame, ros::Time(0), tf_ee_wrt_world_);

    //tfListener_.waitForTransform(worldFrameName_, graspFrameName_, ros::Time::now(), ros::Duration(1.0));
    //tfListener_.lookupTransform(worldFrameName_, graspFrameName_, ros::Time(0), tf_grasp_wrt_world_);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MRT_ROS_Gazebo_Loop::tfCallback] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  initFlagBaseState_ = true;

  //std::cerr << "[MRT_ROS_Gazebo_Loop::tfCallback] START " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometryMsg_ = *msg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::selfCollisionDistanceCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  selfCollisionDistanceMsg_ = *msg;
  initFlagSelfCollision_  = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::extCollisionDistanceCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  extCollisionDistanceMsg_ = *msg;
  initFlagExtCollision_  = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::pointsOnRobotCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  pointsOnRobotMsg_ = *msg;
  initFlagPointsOnRobot_  = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointStateMsg_ = *msg;
  initFlagArmState_ = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] START " << std::endl;

  jointTrajectoryControllerStateMsg_ = *msg;
  initFlagArmState2_ = true;

  //std::cout << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::updateFullModelState()
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] START " << std::endl;

  statePositionBase_.clear();
  stateVelocityBase_.clear();
  statePositionArm_.clear();
  //statePositionArmTmp_.clear();

  //while(!initFlagBaseState_){ros::spinOnce();}
  tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  geometry_msgs::Pose robotBasePoseMsg = robotBasePoseMsg_;
  geometry_msgs::Twist robotBaseTwistMsg = robotBaseTwistMsg_;

  //while(!initFlagArmState_){ros::spinOnce();}
  //control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg = jointTrajectoryControllerStateMsg_;
  sensor_msgs::JointState jointStateMsg = jointStateMsg_;

  // Set mobile base state
  tf::Matrix3x3 matrix_robot_wrt_world;
  if (tfFlag_)
  {
    statePositionBase_.push_back(tf_robot_wrt_world.getOrigin().x());
    statePositionBase_.push_back(tf_robot_wrt_world.getOrigin().y());\

    matrix_robot_wrt_world = tf::Matrix3x3(tf_robot_wrt_world.getRotation());
  }
  else
  {
    statePositionBase_.push_back(robotBasePoseMsg.position.x);
    statePositionBase_.push_back(robotBasePoseMsg.position.y);

    tf::Quaternion quat_robot_wrt_world(robotBasePoseMsg.orientation.x, 
                                        robotBasePoseMsg.orientation.y, 
                                        robotBasePoseMsg.orientation.z, 
                                        robotBasePoseMsg.orientation.w);
    matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
  }
  
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);
  statePositionBase_.push_back(yaw_robot_wrt_world);

  // Set mobile base input
  stateVelocityBase_.push_back(robotBaseTwistMsg_.linear.x);
  stateVelocityBase_.push_back(robotBaseTwistMsg_.angular.z);

  //std::cerr << "[MRT_ROS_Gazebo_Loop::updateFullModelState] mrt_.getArmStateDim(): " << mrt_.getArmStateDim() << std::endl;

  // Set arm state
  //for (int i = 0; i < jointTrajectoryControllerStateMsg.joint_names.size(); ++i)
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] jointStateMsg.name size: " << jointStateMsg.name.size() << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] jointStateMsg.position size: " << jointStateMsg.position.size() << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] jointTrajectoryControllerStateMsg.actual.positions size: " << jointTrajectoryControllerStateMsg.actual.positions.size() << std::endl;
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    //statePositionArm_.push_back(jointTrajectoryControllerStateMsg.actual.positions[stateIndexMap_[i]]);
    statePositionArm_.push_back(jointStateMsg.position[stateIndexMap_[i]]);
  }

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] statePositionBase_ size: " << statePositionBase_.size() << std::endl;
  for (size_t i = 0; i < statePositionBase_.size(); i++)
  {
    std::cout << i << " -> " << statePositionBase_[i] << std::endl;
  }
  
  std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] statePositionArm_ size: " << statePositionArm_.size() << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] statePositionArmTmp_ size: " << statePositionArmTmp_.size() << std::endl;
  for (size_t i = 0; i < statePositionArm_.size(); i++)
  {
    std::cout << i << " -> " << statePositionArm_[i] << std::endl;
    std::cout << i << " -> " << statePositionArmTmp_[i] << std::endl;
  }
  */

  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] END " << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
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

  auto statePositionBase = statePositionBase_;
  auto stateVelocityBase = stateVelocityBase_;
  auto statePositionArm = statePositionArm_;

  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {  
      // Set state
      currentObservation.state[0] = statePositionBase[0];
      currentObservation.state[1] = statePositionBase[1];
      currentObservation.state[2] = statePositionBase[2];

      // Set input
      if (initFlag)
      {
        currentObservation.input[0] = stateVelocityBase[0];
        currentObservation.input[1] = stateVelocityBase[1];
      }

      // Set full state
      currentObservation.full_state = currentObservation.state;
      break;
    }

    case RobotModelType::RobotArm:
    { 
      // Set state and input
      for (size_t i = 0; i < statePositionArm.size(); i++)
      {
        currentObservation.state[i] = statePositionArm[i];

        if(initFlag)
        {
          currentObservation.input[i] = statePositionArm[i];
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
          currentObservation.state[0] = statePositionBase[0];
          currentObservation.state[1] = statePositionBase[1];
          currentObservation.state[2] = statePositionBase[2];

          // Set input
          if (initFlag)
          {
            currentObservation.input[0] = stateVelocityBase[0];
            currentObservation.input[1] = stateVelocityBase[1];
          }

          // Set full state
          currentObservation.full_state[0] = statePositionBase[0];
          currentObservation.full_state[1] = statePositionBase[1];
          currentObservation.full_state[2] = statePositionBase[2];

          for (size_t i = 0; i < statePositionArm.size(); i++)
          {
            currentObservation.full_state[stateDimBase + i] = statePositionArm[i];
          }

          break;
        }

        case ModelMode::ArmMotion:
        {
          // Set full state base
          currentObservation.full_state[0] = statePositionBase[0];
          currentObservation.full_state[1] = statePositionBase[1];
          currentObservation.full_state[2] = statePositionBase[2];

          for (size_t i = 0; i < statePositionArm.size(); i++)
          {
            // Set state
            currentObservation.state[i] = statePositionArm[i];

            if(initFlag)
            {
              currentObservation.input[i] = statePositionArm[i];
            }

            // Set full state arm
            currentObservation.full_state[stateDimBase + i] = statePositionArm[i];
          }
          break;
        }

        case ModelMode::WholeBodyMotion:
        {
          // Set state
          currentObservation.state[0] = statePositionBase[0];
          currentObservation.state[1] = statePositionBase[1];
          currentObservation.state[2] = statePositionBase[2];

          // Set input
          if (initFlag)
          {
            currentObservation.input[0] = stateVelocityBase[0];
            currentObservation.input[1] = stateVelocityBase[1];
          }

          for (size_t i = 0; i < statePositionArm.size(); i++)
          {
            currentObservation.state[stateDimBase + i] = statePositionArm[i];

            if(initFlag)
            {
              currentObservation.input[inputDimBase + i] = statePositionArm[i];
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
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::isPickDropPoseReached(int taskMode)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] START" << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] taskMode: " << taskMode << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] err_threshold_pos_: " << err_threshold_pos_ << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] err_threshold_ori_: " << err_threshold_ori_ << std::endl;

  if (!shutDownFlag_ && !taskEndFlag_ && taskMode != 0)
  {
    vector_t currentTarget = currentTarget_;

    tf::StampedTransform tf_ee_wrt_world = tf_ee_wrt_world_;

    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] currentTarget(0): " << currentTarget(0) << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] currentTarget(1): " << currentTarget(1) << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] currentTarget(2): " << currentTarget(2) << std::endl;

    vector_t dist_pos(3);
    dist_pos << abs(tf_ee_wrt_world.getOrigin().x() - currentTarget(0)), 
                abs(tf_ee_wrt_world.getOrigin().y() - currentTarget(1)), 
                abs(tf_ee_wrt_world.getOrigin().z() - currentTarget(2));

    double err_pos = dist_pos.norm();

    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_pos x: " << dist_pos[0] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_pos y: " << dist_pos[1] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_pos z: " << dist_pos[2] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] err_pos: " << err_pos << std::endl;

    Eigen::Quaterniond quat_ee(tf_ee_wrt_world.getRotation().w(), tf_ee_wrt_world.getRotation().x(), tf_ee_wrt_world.getRotation().y(), tf_ee_wrt_world.getRotation().z());
    Eigen::Quaterniond quat_target(currentTarget(6), currentTarget(3), currentTarget(4), currentTarget(5));

    vector_t dist_quat = quaternionDistance(quat_ee, quat_target);
    double err_ori = dist_quat.norm();

    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_quat r: " << dist_quat[0] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_quat p: " << dist_quat[1] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] dist_quat y: " << dist_quat[2] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] err_ori: " << err_ori << std::endl;

    //std::cout << "[MRT_ROS_Gazebo_Loop::isPickDropPoseReached] END" << std::endl << std::endl;

    return (err_pos < err_threshold_pos_) && (err_ori < err_threshold_ori_);
  }

  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
bool MRT_ROS_Gazebo_Loop::setTaskMode(int val)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskMode] START" << std::endl;

  bool success = false;
  ocs2_msgs::setInt srv;
  srv.request.val = val;
  if (setTaskModeClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MRT_ROS_Gazebo_Loop::setTaskMode] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskMode] END" << std::endl;
  
  return success;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::setPickedFlag(bool val)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::setPickedFlag] START" << std::endl;

  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = val;
  if (setPickedFlagClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MRT_ROS_Gazebo_Loop::setPickedFlag] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[MRT_ROS_Gazebo_Loop::setPickedFlag] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::setSystemObservation(const SystemObservation& currentObservation)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::setSystemObservation] START" << std::endl;

  bool success = false;
  ocs2_msgs::setSystemObservation srv;
  srv.request.obs = ros_msg_conversions::createObservationMsg(currentObservation);
  if (setSystemObservationClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MRT_ROS_Gazebo_Loop::setSystemObservation] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[MRT_ROS_Gazebo_Loop::setSystemObservation] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
bool MRT_ROS_Gazebo_Loop::setTaskModeSrv(ocs2_msgs::setInt::Request &req, 
                                         ocs2_msgs::setInt::Response &res)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] START" << std::endl;
  taskMode_ = req.val;
  res.success = true;

  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] taskMode_: " << taskMode_ << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] END" << std::endl;
  return res.success;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::setTaskSrv(ocs2_msgs::setTask::Request &req, 
                                     ocs2_msgs::setTask::Response &res)
{
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] START" << std::endl;
  taskMode_ = req.taskMode;
  currentTargetName_ = req.targetName;
  currentTargetAttachLinkName_ = req.targetAttachLinkName;
  currentTarget_(0) = req.targetPose.position.x;
  currentTarget_(1) = req.targetPose.position.y;
  currentTarget_(2) = req.targetPose.position.z;
  currentTarget_(3) = req.targetPose.orientation.x;
  currentTarget_(4) = req.targetPose.orientation.y;
  currentTarget_(5) = req.targetPose.orientation.z;
  currentTarget_(6) = req.targetPose.orientation.w;
  res.success = true;

  taskEndFlag_ = false;
  targetReceivedFlag_ = true;

  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] taskMode_: " << taskMode_ << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::publishCommand(const PrimalSolution& currentPolicy, 
                                         const SystemObservation& currentObservation,
                                         std::vector<double>& currentStateVelocityBase)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] START time_: " << time_ << std::endl;

  mpcTimeTrajectory_ = currentPolicy.timeTrajectory_;
  dataMPCTimeTrajectory_.push_back(mpcTimeTrajectory_);

  mpcStateTrajectory_.clear();
  for (auto traj : currentPolicy.stateTrajectory_)
  {
    std::vector<double> traj_tmp;
    for (size_t i = 0; i < traj.size(); i++)
    {
      traj_tmp.push_back(traj[i]);
    }
    mpcStateTrajectory_.push_back(traj_tmp);
  }
  dataMPCStateTrajectory_.push_back(mpcStateTrajectory_);

  mpcInputTrajectory_.clear();
  for (auto traj : currentPolicy.inputTrajectory_)
  {
    std::vector<double> traj_tmp;
    for (size_t i = 0; i < traj.size(); i++)
    {
      traj_tmp.push_back(traj[i]);
    }
    mpcInputTrajectory_.push_back(traj_tmp);
  }
  dataMPCInputTrajectory_.push_back(mpcInputTrajectory_);

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] mpcTimeTrajectory_ size: " << mpcTimeTrajectory_.size() << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] mpcStateTrajectory_ size: " << mpcStateTrajectory_.size() << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] mpcInputTrajectory_ size: " << mpcInputTrajectory_.size() << std::endl;

  if (mpcStateTrajectory_.size() > 0)
  {
    std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] mpcStateTrajectory_[0] size: " << mpcInputTrajectory_[0].size() << std::endl;
  }

  if (mpcInputTrajectory_.size() > 0)
  {
    std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] mpcInputTrajectory_[0] size: " << mpcInputTrajectory_[0].size() << std::endl;
  }
  */

  std::vector<double> state_pos;
  for (size_t i = 0; i < currentObservation.full_state.size(); i++)
  {
    state_pos.push_back(currentObservation.full_state[i]);
  }

  std::vector<double> cmd;

  geometry_msgs::Twist baseTwistMsg;
  trajectory_msgs::JointTrajectory armJointTrajectoryMsg;
  kinova_msgs::JointVelocity armJointVelocityMsg;
  // Set mobile base command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistMsg.linear.x = currentInput_[0];
    baseTwistMsg.angular.z = currentInput_[1];
    // Cutoff Frequency
    if(abs(abs(baseTwistMsg.linear.x) - abs(prev_lin_x)) > lin_x_cutoff) {
      baseTwistMsg.linear.x = prev_lin_x+copysign(1, baseTwistMsg.linear.x-prev_lin_x)*lin_x_cutoff;
    }
    if(abs(abs(baseTwistMsg.angular.z) - abs(prev_ang_z)) > ang_z_cutoff) {
      baseTwistMsg.angular.z = prev_ang_z+copysign(1, baseTwistMsg.angular.z-prev_ang_z)*ang_z_cutoff;
    }
    prev_lin_x = baseTwistMsg.linear.x;
    prev_ang_z = baseTwistMsg.angular.z;
    
    
    cmd.push_back(baseTwistMsg.linear.x);
    cmd.push_back(baseTwistMsg.angular.z);
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

    //PrimalSolution primalSolution = mrt_.getPolicy();
    //PrimalSolution primalSolution = currentPolicy;
    auto nextState = currentPolicy.getDesiredState(time_ + dt_);

    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(n_joints);
    jtp.time_from_start = ros::Duration(dt_);

    for (int i = 0; i < n_joints; ++i)
    {
      armJointTrajectoryMsg.joint_names[i] = mrt_.getRobotModelInfo().robotArm.jointNames[i];
      jtp.positions[i] = nextState[baseOffset + i];

      cmd.push_back(jtp.positions[i]);
    }
    armJointTrajectoryMsg.points.push_back(jtp);
  }

  // Publish command
  if (mrt_.getRobotModelInfo().modelMode == ModelMode::BaseMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] BASE PUB" << std::endl;
    baseTwistPub_.publish(baseTwistMsg);
  }

  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    armJointVelocityMsg.joint1 = currentInput_[2] * (180.0/M_PIf32);
    armJointVelocityMsg.joint2 = currentInput_[3] * (180.0/M_PIf32);
    armJointVelocityMsg.joint3 = currentInput_[4] * (180.0/M_PIf32);
    armJointVelocityMsg.joint4 = currentInput_[5] * (180.0/M_PIf32);
    armJointVelocityMsg.joint5 = currentInput_[6] * (180.0/M_PIf32);
    armJointVelocityMsg.joint6 = currentInput_[7] * (180.0/M_PIf32);
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] ARM PUB" << std::endl;
    armJointTrajectoryPub_.publish(armJointTrajectoryMsg);
    armJointVelocityPub_.publish(armJointVelocityMsg);

  }

  dataStatePosition_.push_back(state_pos);
  dataStateVelocityBase_.push_back(currentStateVelocityBase);
  dataCommand_.push_back(cmd);
  
  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::checkCollision()
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] START" << std::endl << std::endl;

  visualization_msgs::MarkerArray selfCollisionDistanceMsg = selfCollisionDistanceMsg_;
  visualization_msgs::MarkerArray extCollisionDistanceMsg = extCollisionDistanceMsg_;
  visualization_msgs::MarkerArray pointsOnRobotMsg = pointsOnRobotMsg_;
  
  //std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] BEFORE self" << std::endl;
  // Check self collision
  int n_selfColDist = selfCollisionDistanceMsg.markers.size() / selfColDistance_n_coeff_;
  std::vector<double> selfCollisionDistances;
  for (size_t i = 0; i < n_selfColDist; i++)
  {
    Eigen::VectorXd p1(3);
    Eigen::VectorXd p2(3);

    p1 << selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[0].x, 
          selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[0].y,
          selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[0].z;
    p2 << selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[1].x, 
          selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[1].y,
          selfCollisionDistanceMsg.markers[i*selfColDistance_n_coeff_].points[1].z;

    double dist = (p1 - p2).norm();

    if (dist < selfCollisionRangeMin_)
    {
      drlActionResult_ = 1;
      shutDownFlag_ = true;
      std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] SELF COLLISION!" << std::endl;
      return true;
    }
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] BEFORE external" << std::endl;
  // Check external collision
  for (size_t i = 0; i < extCollisionDistanceMsg.markers.size(); i++)
  {
    Eigen::VectorXd p1(3);
    Eigen::VectorXd p2(3);

    p1 << extCollisionDistanceMsg.markers[i].points[0].x, 
          extCollisionDistanceMsg.markers[i].points[0].y,
          extCollisionDistanceMsg.markers[i].points[0].z;
    p2 << extCollisionDistanceMsg.markers[i].points[1].x, 
          extCollisionDistanceMsg.markers[i].points[1].y,
          extCollisionDistanceMsg.markers[i].points[1].z;

    double dist = (p1 - p2).norm();

    if (dist < extCollisionRangeMin_)
    {
      drlActionResult_ = 1;
      shutDownFlag_ = true;
      std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] EXT COLLISION!" << std::endl;
      return true;
    }
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] BEFORE ground" << std::endl;
  // Check ground collision
  for (size_t i = 1; i < pointsOnRobotMsg.markers.size(); i++)
  {
    if (pointsOnRobotMsg.markers[i].pose.position.z < extCollisionRangeMin_)
    {
      drlActionResult_ = 1;
      shutDownFlag_ = true;
      std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] GROUND COLLISION!" << std::endl;
      return true;
    }
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::checkCollision] END" << std::endl;

  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::checkRollover()
{
  tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  geometry_msgs::Pose robotBasePoseMsg = robotBasePoseMsg_;

  tf::Matrix3x3 matrix_robot_wrt_world;
  if (tfFlag_)
  {
    matrix_robot_wrt_world = tf::Matrix3x3(tf_robot_wrt_world.getRotation());
  }
  else
  {
    tf::Quaternion quat_robot_wrt_world(robotBasePoseMsg.orientation.x, 
                                        robotBasePoseMsg.orientation.y, 
                                        robotBasePoseMsg.orientation.z, 
                                        robotBasePoseMsg.orientation.w);
    matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
  }
  
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);

  if (roll_robot_wrt_world > rolloverRollThreshold_)
  {
    drlActionResult_ = 2;
    shutDownFlag_ = true;
    std::cout << "[MRT_ROS_Gazebo_Loop::checkRollover] ROLL ROLLOVER!" << std::endl;
    return true;
  }

  if (pitch_robot_wrt_world > rolloverPitchThreshold_)
  {
    drlActionResult_ = 2;
    shutDownFlag_ = true;
    std::cout << "[MRT_ROS_Gazebo_Loop::checkRollover] PITCH ROLLOVER!" << std::endl;
    return true;
  }

  return false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
const std::string MRT_ROS_Gazebo_Loop::getDateTime() 
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);

  strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tstruct);

  return buf;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::writeData(bool endFlag)
{
  if (dataCollectionFlag_)
  {
    if ( (time_ - dataWriteLastTime_) > dataWriteFreq_ || shutDownFlag_)
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::writeData] START" << std::endl;

      dataTimeEnd_ = time_;

      /*
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataTimeStart_: " << dataTimeStart_ << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataTimeEnd_: " << dataTimeEnd_ << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataPathReL_: " << dataPathReL_ << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataStatePosition_ size: " << dataStatePosition_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataStateVelocityBase_ size: " << dataStateVelocityBase_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataCommand_ size: " << dataCommand_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCTimeTrajectory_ size: " << dataMPCTimeTrajectory_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCStateTrajectory_ size: " << dataMPCStateTrajectory_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCStateTrajectory_[0] size: " << dataMPCStateTrajectory_[0].size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCStateTrajectory_[0][0] size: " << dataMPCStateTrajectory_[0][0].size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCInputTrajectory_ size: " << dataMPCInputTrajectory_.size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCInputTrajectory_[0] size: " << dataMPCInputTrajectory_[0].size() << std::endl;
      std::cout << "[MRT_ROS_Gazebo_Loop::writeData] dataMPCInputTrajectory_[0][0] size: " << dataMPCInputTrajectory_[0][0].size() << std::endl;
      */

      dataTimeEnd_ = time_;

      nlohmann::json j;
      j["dt"] = dt_;
      j["time_start"] = dataTimeStart_;
      j["time_end"] = dataTimeEnd_;
      j["end_flag"] = endFlag;
      j["model_mode"] = getModelModeInt(robotModelInfo_);
      j["state_arm_offset"] = robotModelInfo_.mobileBase.stateDim;
      j["input_arm_offset"] = robotModelInfo_.mobileBase.inputDim;
      j["state_position"] = dataStatePosition_;
      j["state_velocity_base"] = dataStateVelocityBase_;
      j["command"] = dataCommand_;
      j["mpc_time_trajectory"] = dataMPCTimeTrajectory_;
      j["mpc_state_trajectory"] = dataMPCStateTrajectory_;
      j["mpc_input_trajectory"] = dataMPCInputTrajectory_;

      std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
      std::string dataPath = pkg_dir + dataPathReL_;

      boost::filesystem::create_directories(pkg_dir + dataPathReL_);
      std::string filename = getDateTime() + ".json";
      std::ofstream o(dataPath + filename);
      o << std::setw(4) << j << std::endl;

      dataTimeStart_ = dataTimeEnd_;
      dataStatePosition_.clear();
      dataStateVelocityBase_.clear();
      dataCommand_.clear();
      dataMPCTimeTrajectory_.clear();
      dataMPCStateTrajectory_.clear();
      dataMPCInputTrajectory_.clear();
      dataWriteLastTime_ = time_;

      //std::cout << "[MRT_ROS_Gazebo_Loop::writeData] END" << std::endl << std::endl;
    }
  }
}

}  // namespace ocs2
