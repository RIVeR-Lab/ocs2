// LAST UPDATE: 2023.07.27
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
                                         bool updateIndexMapFlag)
  : mrt_(mrt), 
    worldFrameName_(worldFrameName),
    err_threshold_pos_(err_threshold_pos),
    err_threshold_ori_(err_threshold_ori),
    mrtDesiredFrequency_(mrtDesiredFrequency), 
    mpcDesiredFrequency_(mpcDesiredFrequency),
    robotModelInfo_(mrt.getRobotModelInfo())
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] START" << std::endl;

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
      std::cerr << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] ERROR: Invalid robot model type!";
      break;
  }

  timer2_.startTimer();
  updateStateIndexMap(armStateMsg, updateIndexMapFlag);
  timer2_.endTimer();
  
  // Subscribers
  //// NUA TODO: Consider localization error
  //odometrySub_ = nh.subscribe("/jackal_velocity_controller/odom", 10, &MRT_ROS_Gazebo_Loop::odometryCallback, this);
  if (baseStateMsg != "")
  {
    linkStateSub_ = nh.subscribe(baseStateMsg, 5, &MRT_ROS_Gazebo_Loop::linkStateCallback, this);
  }
  else
  {
    tfFlag_ = true;
    tfSub_ = nh.subscribe("/tf", 5, &MRT_ROS_Gazebo_Loop::tfCallback, this);
  }
  
  //jointStateSub_ = nh.subscribe("/joint_states", 10, &MRT_ROS_Gazebo_Loop::jointStateCallback, this);
  //jointTrajectoryPControllerStateSub_ = nh.subscribe("/arm_controller/state", 10, &MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback, this);
  jointTrajectoryPControllerStateSub_ = nh.subscribe(armStateMsg, 5, &MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback, this);

  currentTarget_.resize(7);

  // Publishers
  baseTwistPub_ = nh.advertise<geometry_msgs::Twist>(baseControlMsg, 1);
  armJointTrajectoryPub_ = nh.advertise<trajectory_msgs::JointTrajectory>(armControlMsg, 1);

  /// Clients
  attachClient_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  detachClient_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
  //setTaskModeClient_ = nh.serviceClient<ocs2_msgs::setInt>("/set_task_mode");
  setPickedFlagClient_ = nh.serviceClient<ocs2_msgs::setBool>("/set_picked_flag");

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
  std::cout << "[MRT_ROS_Gazebo_Loop::run] START" << std::endl;

  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Waiting for the initial policy...");
  
  currentTarget_ = initTarget;
  taskEndFlag_ = true;

  while(!isStateInitialized())
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::run] WARNING: State not initialized!" << std::endl;
    ros::spinOnce();
  }

  updateFullModelState();

  std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE getCurrentObservation" << std::endl;
  SystemObservation initObservation = getCurrentObservation(true);

  /*
  std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE currentTarget size: " << initTarget.size() << std::endl;
  for (size_t i = 0; i < initTarget.size(); i++)
  {
    std::cout << i << " -> " << initTarget[i] << std::endl;
  }
  std::cout << "------------" << std::endl;
  */

  const TargetTrajectories initTargetTrajectories({0}, {initTarget}, {initObservation.input});

  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial state and policy
  while ( (!isStateInitialized() || !mrt_.initialPolicyReceived()) && ros::ok() && ros::master::check() ) 
  {
    mrt_.spinMRT();

    // Get initial observation
    //initObservation = getCurrentObservation(true);

    mrt_.setCurrentObservation(initObservation);
    //ros::Rate(mrtDesiredFrequency_).sleep();

    ros::spinOnce();
  }
  ROS_INFO_STREAM("[MRT_ROS_Gazebo_Loop::run] Initial policy has been received.");

  std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE currentInput_" << std::endl;
  currentInput_ = initObservation.input;
  std::cout << "[MRT_ROS_Gazebo_Loop::run] AFTER currentInput_" << std::endl;

  std::cout << "[MRT_ROS_Gazebo_Loop::run] BEFORE mrtLoop" << std::endl;
  //mrtLoop();
  mrtLoop2();
  std::cout << "[MRT_ROS_Gazebo_Loop::run] AFTER mrtLoop" << std::endl;

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
/*
void MRT_ROS_Gazebo_Loop::getCurrentState(vector_t& currentState)
{
  updateFullModelState();

  std::vector<double> stateBase = stateBase_;
  std::vector<double> stateArm = stateArm_;

  int stateSize = getStateDim(robotModelInfo_);
  currentState.resize(stateSize);

  switch (robotModelInfo_.robotModelType)
  {
    case RobotModelType::MobileBase:
    {
      for (size_t i = 0; i < stateBase.size(); i++)
      {
        currentState(i) = stateBase[i];
      }
      break;
    }

    case RobotModelType::RobotArm:
    {
      for (size_t i = 0; i < stateArm.size(); i++)
      {
        currentState(i) = stateArm[i];
      }
      break;
    }

    case RobotModelType::MobileManipulator:
    {
      int offset = stateBase.size();
      for (size_t i = 0; i < stateBase.size(); i++)
      {
        currentState(i) = stateBase[i];
      }

      for (size_t i = 0; i < stateArm.size(); i++)
      {
        currentState(offset + i) = stateArm[i];
      }
      
      break;
    }
    
    default:
      std::cerr << "[MRT_ROS_Gazebo_Loop::MRT_ROS_Gazebo_Loop] ERROR: Invalid robot model type!";
      break;
  }
}
*/

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
  //std::cout << "[OCS2_MRT_Loop::mrtLoop] START" << std::endl;

  // Loop variables
  SystemObservation currentObservation;
  SystemObservation targetObservation;
  PrimalSolution currentPolicy;

  shutDownFlag_ = false;

  // Update the policy
  mrt_.updatePolicy();

  ros::Rate simRate(mrtDesiredFrequency_);

  while (!shutDownFlag_ && ros::ok() && ros::master::check()) 
  {
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] START while" << std::endl;

    mrt_.reset();

    while (!shutDownFlag_ && !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
    {
      //std::cout << "[OCS2_MRT_Loop::mrtLoop] START spinMRT" << std::endl;
      mrt_.spinMRT();

      // Get current observation
      currentObservation = getCurrentObservation(false);

      // Set current observation
      mrt_.setCurrentObservation(currentObservation);

      mrtShutDownFlag_ = getenv("mrtShutDownFlag");
      //std::cout << "[MPC_ROS_Interface::spin] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

      if (mrtShutDownFlag_ == "true")
      {
        //std::cout << "[OCS2_MRT_Loop::spin] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
        shutDownFlag_ = true;
      }
      //std::cout << "[OCS2_MRT_Loop::mrtLoop] END spinMRT" << std::endl;
    }

    int taskMode = taskMode_;
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] taskMode: " << taskMode << std::endl;

    if (isTargetReached(taskMode))
    {
      std::cout << "[OCS2_MRT_Loop::mrtLoop] START PICK/DROP" << std::endl;
      
      gazebo_ros_link_attacher::Attach srv;
      srv.request.model_name_1 = robotModelInfo_.robotName;
      srv.request.link_name_1 = robotModelInfo_.robotArm.jointFrameNames.back();
      srv.request.model_name_2 = currentTargetName_;
      srv.request.link_name_2 = currentTargetAttachLinkName_;

      //std::cout << "[OCS2_MRT_Loop::mrtLoop] taskMode: " << taskMode << std::endl;
      if (taskMode == 1)
      {
        if (attachClient_.call(srv))
        {
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] response: " << srv.response.ok << std::endl;
          taskMode = 2;
          pickedFlag_ = true;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[OCS2_MRT_Loop::mrtLoop] ERROR: Failed to call service!");
        }
      }
      else if (taskMode == 2)
      {
        std::cout << "[OCS2_MRT_Loop::mrtLoop] DROP" << std::endl;
        if (detachClient_.call(srv))
        {
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] response: " << srv.response.ok << std::endl;
          taskMode = 1;
          pickedFlag_ = false;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[OCS2_MRT_Loop::mrtLoop] ERROR: Failed to call service!");
        }
      }

      taskEndFlag_ = true;
      
      //std::cout << "[OCS2_MRT_Loop::mrtLoop] END PICK/DROP" << std::endl;
    }
    else
    {
      if (!shutDownFlag_)
      {
        //std::cout << "[OCS2_MRT_Loop::mrtLoop] START updatePolicy" << std::endl;
        // Update the policy if a new one was received
        mrt_.updatePolicy();
        currentPolicy = mrt_.getPolicy();
        currentInput_ = currentPolicy.getDesiredInput(time_);

        //std::cout << "[OCS2_MRT_Loop::mrtLoop] START observer" << std::endl;
        // Update observers for visualization
        //for (auto& observer : observers_) 
        //{
        //  observer->update(currentObservation, currentPolicy, mrt_.getCommand());
        //}

        //std::cout << "[OCS2_MRT_Loop::mrtLoop] START publishCommand" << std::endl;
        // Publish the control command 
        publishCommand(currentPolicy);

        time_ += dt_;

        //shutDownFlag_ = mrt_.getShutDownFlag();

        ros::spinOnce();
        simRate.sleep();
      }
    }
    
    mrtShutDownFlag_ = getenv("mrtShutDownFlag");
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

    if (mrtShutDownFlag_ == "true")
    {
      //std::cout << "[OCS2_MRT_Loop::mrtLoop] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
      shutDownFlag_ = true;
    }
    
    /*
    if (mpcProblemReadyFlag_)
    {
      shutDownFlag_ = true;
      //mrt_.setShutDownFlag(true);
    }
    */
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] END while" << std::endl;
    //std::cout << "---------------" << std::endl << std::endl;
  }

  //std::cout << "[OCS2_MRT_Loop::mrtLoop] DEBUG INF" << std::endl;
  //while(1);

  //mrt_.shutdownNodes();
  
  //mrtExitFlag_ = true;
  //publishMRTExitFlag();

  //std::cout << "[OCS2_MRT_Loop::mrtLoop] DEBUG INF" << std::endl;
  //while(1);

  //setenv("mrtExitFlag", "true", 1);
  //linkStateSub_.shutdown();
  //tfSub_.shutdown();
  std::cout << "[OCS2_MRT_Loop::mrtLoop] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::mrtLoop2() 
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START" << std::endl;

  // Loop variables
  SystemObservation currentObservation;
  SystemObservation targetObservation;
  PrimalSolution currentPolicy;

  shutDownFlag_ = false;

  // Update the policy
  mrt_.updatePolicy();
  mrt_.reset();

  ros::Rate simRate(mrtDesiredFrequency_);

  int ctr1 = 0;
  int ctr2 = 0;

  while (!checkShutDownFlag() && ros::ok() && ros::master::check()) 
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START while" << std::endl;
  
    /////////////////////////////////// CHECK THE TASK /////////// START
    int taskMode = taskMode_;
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskMode: " << taskMode << std::endl;

    if (!checkShutDownFlag() && isTargetReached(taskMode))
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START PICK/DROP" << std::endl;
      
      gazebo_ros_link_attacher::Attach srv;
      srv.request.model_name_1 = robotModelInfo_.robotName;
      srv.request.link_name_1 = robotModelInfo_.robotArm.jointFrameNames.back();
      srv.request.model_name_2 = currentTargetName_;
      srv.request.link_name_2 = currentTargetAttachLinkName_;

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] srv.request.link_name_1: " << srv.request.link_name_1 << std::endl;

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskMode: " << taskMode << std::endl;
      if (taskMode == 1)
      {
        if (attachClient_.call(srv))
        {
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] response: " << srv.response.ok << std::endl;
          taskMode = 2;
          pickedFlag_ = true;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[MRT_ROS_Gazebo_Loop::mrtLoop2] ERROR: Failed to call service!");
        }
      }
      else if (taskMode == 2)
      {
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] DROP" << std::endl;
        if (detachClient_.call(srv))
        {
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] response: " << srv.response.ok << std::endl;
          taskMode = 1;
          pickedFlag_ = false;
          bool taskModeSuccess = setPickedFlag(pickedFlag_);
          //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] taskModeSuccess: " << taskModeSuccess << std::endl;
          ros::spinOnce();
        }
        else
        {
          ROS_ERROR("[MRT_ROS_Gazebo_Loop::mrtLoop2] ERROR: Failed to call service!");
        }
      }

      taskEndFlag_ = true;

      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] END PICK/DROP" << std::endl;
    }
    /////////////////////////////////// CHECK THE TASK /////////// END

    if (!checkShutDownFlag() && !taskEndFlag_)
    {
      updateFullModelState();

      mrt_.reset();
      mrt_.spinMRT();

      // Get current observation
      currentObservation = getCurrentObservation(false);
      mrt_.setCurrentObservation(currentObservation);

      ctr2 = 0;
      while (!checkShutDownFlag() && !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
      {
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START spinMRT ctr2: " << ctr2 << std::endl;
        mrt_.spinMRT();
        //ros::spinOnce();

        // Set current observation
        //mrt_.setCurrentObservation(currentObservation);

        if (ctr2 > 100)
        {
          mrt_.reset();
          currentObservation = getCurrentObservation(true);
          mrt_.setCurrentObservation(currentObservation);
          currentInput_ = currentObservation.input;
          ctr2 = 0;
        }

        ctr2++;

        //ros::spinOnce();
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] END spinMRT" << std::endl << std::endl;
      }

      if (!checkShutDownFlag())
      {
        mrt_.updatePolicy();
        currentPolicy = mrt_.getPolicy();
        currentInput_ = currentPolicy.getDesiredInput(time_);

        publishCommand(currentPolicy);

        time_ += dt_;
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] time_: " << time_ << std::endl;
      }
    }

    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] shutDownFlag_: " << shutDownFlag_ << std::endl;

    ros::spinOnce();

    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] AFTER spinOnce" << std::endl;
    simRate.sleep();
    

    /*
    while (!shutDownFlag_ && !mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) 
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START spinMRT" << std::endl;
      mrt_.spinMRT();

      // Set current observation
      mrt_.setCurrentObservation(currentObservation);

      mrtShutDownFlag_ = getenv("mrtShutDownFlag");
      //std::cout << "[MPC_ROS_Interface::spin] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

      if (mrtShutDownFlag_ == "true")
      {
        //std::cout << "[MRT_ROS_Gazebo_Loop::spin] CMOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOON: " << std::endl;
        //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] DEBUG INF" << std::endl;
        //while(1);
        shutDownFlag_ = true;
      }

      //shutDownFlag_ = mrt_.getShutDownFlag();

      ctr2++;
      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] END spinMRT" << std::endl;
    }
    */

    
    
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START updatePolicy" << std::endl;
    // Update the policy if a new one was received
    /*
    if (!shutDownFlag_)
    {
      mrt_.updatePolicy();
      currentPolicy = mrt_.getPolicy();
      currentInput_ = currentPolicy.getDesiredInput(time_);
    }
    */

    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START observer" << std::endl;
    // Update observers for visualization
    //for (auto& observer : observers_) 
    //{
    //  observer->update(currentObservation, currentPolicy, mrt_.getCommand());
    //}

    /*
    if (!shutDownFlag_ && !taskEndFlag_)
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] START publishCommand" << std::endl;
      // Publish the control command 
      publishCommand(currentPolicy);
    }

    time_ += dt_;

    mrtShutDownFlag_ = getenv("mrtShutDownFlag");
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

    if (mrtShutDownFlag_ == "true")
    {
      shutDownFlag_ = true;
    }

    ros::spinOnce();
    simRate.sleep();
    */
    
    //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] END while" << std::endl;
    //std::cout << "---------------" << std::endl << std::endl;
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] DEBUG INF" << std::endl;
  //while(1);

  //mrt_.shutdownNodes();
  
  //mrtExitFlag_ = true;
  //publishMRTExitFlag();

  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] DEBUG INF" << std::endl;
  //while(1);

  //setenv("mrtExitFlag", "true", 1);
  //linkStateSub_.shutdown();
  //tfSub_.shutdown();
  std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop2] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::checkShutDownFlag()
{
  mrtShutDownFlag_ = getenv("mrtShutDownFlag");
  //std::cout << "[MRT_ROS_Gazebo_Loop::mrtLoop] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

  shutDownFlag_ = false;
  if (mrtShutDownFlag_ == "true")
  {
    shutDownFlag_ = true;
  }
  
  return shutDownFlag_;
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
void MRT_ROS_Gazebo_Loop::updateStateIndexMap(std::string& armStateMsg, bool updateStateIndexMapFlag)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] START" << std::endl;  
  auto jointNames = mrt_.getRobotModelInfo().robotArm.jointNames;
  int n_joints = jointNames.size();
  stateIndexMap_.clear();
  int c;

  if (updateStateIndexMapFlag)
  {
    boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> jointTrajectoryControllerStatePtrMsg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(armStateMsg);
    if (n_joints != jointTrajectoryControllerStatePtrMsg->joint_names.size())
    {
      throw std::runtime_error("[MRT_ROS_Gazebo_Loop::updateStateIndexMap] Error: State dimension mismatch!");
    }
    
    for (int i = 0; i < n_joints; ++i)
    {
      //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] jointTrajectoryControllerStatePtrMsg " << i << ": " << jointTrajectoryControllerStatePtrMsg -> joint_names[i] << std::endl;
      //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] jointNames " << i << ": " << jointNames[i] << std::endl;
      //std::cout << "" << std::endl;

      c = 0;
      while (jointTrajectoryControllerStatePtrMsg->joint_names[c] != jointNames[i] && c < n_joints)
      {
        c++;
      }

      if (jointTrajectoryControllerStatePtrMsg->joint_names[c] == jointNames[i])
      {
        stateIndexMap_.push_back(c);
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
  std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] stateIndexMap_:" << std::endl;
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    std::cout << i << " -> " << stateIndexMap_[i] << std::endl;
  }
  */

  //std::cout << "[MRT_ROS_Gazebo_Loop::updateStateIndexMap] END" << std::endl;
}

void MRT_ROS_Gazebo_Loop::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //std::cerr << "[MRT_ROS_Gazebo_Loop::tfCallback] START " << std::endl;
  //tf2_msgs::TFMessage tf_msg = *msg;

  try
  {
    tfListener_.waitForTransform(worldFrameName_, baseFrameName_, ros::Time::now(), ros::Duration(0.5));
    tfListener_.lookupTransform(worldFrameName_, baseFrameName_, ros::Time(0), tf_robot_wrt_world_);

    tfListener_.waitForTransform(worldFrameName_, robotModelInfo_.robotArm.eeFrame, ros::Time::now(), ros::Duration(0.5));
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
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] START " << std::endl;

  jointTrajectoryControllerStateMsg_ = *msg;

  initFlagArmState_ = true;

  //std::cout << "[MRT_ROS_Gazebo_Loop::jointTrajectoryControllerStateCallback] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::updateFullModelState()
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::updateFullModelState] START " << std::endl;

  stateBase_.clear();
  inputBase_.clear();
  stateArm_.clear();

  //while(!initFlagBaseState_);
  tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  geometry_msgs::Pose robotBasePoseMsg = robotBasePoseMsg_;
  geometry_msgs::Twist robotBaseTwistMsg = robotBaseTwistMsg_;

  //while(!initFlagArmState_);
  control_msgs::JointTrajectoryControllerState jointTrajectoryControllerStateMsg = jointTrajectoryControllerStateMsg_;

  // Set mobile base state
  tf::Matrix3x3 matrix_robot_wrt_world;
  if (tfFlag_)
  {
    stateBase_.push_back(tf_robot_wrt_world.getOrigin().x());
    stateBase_.push_back(tf_robot_wrt_world.getOrigin().y());\

    matrix_robot_wrt_world = tf::Matrix3x3(tf_robot_wrt_world.getRotation());
  }
  else
  {
    stateBase_.push_back(robotBasePoseMsg.position.x);
    stateBase_.push_back(robotBasePoseMsg.position.y);

    tf::Quaternion quat_robot_wrt_world(robotBasePoseMsg.orientation.x, 
                                        robotBasePoseMsg.orientation.y, 
                                        robotBasePoseMsg.orientation.z, 
                                        robotBasePoseMsg.orientation.w);
    matrix_robot_wrt_world = tf::Matrix3x3(quat_robot_wrt_world);
  }
  
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);
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

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
SystemObservation MRT_ROS_Gazebo_Loop::getCurrentObservation(bool initFlag)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::getCurrentObservation] START" << std::endl;
  
  RobotModelInfo robotModelInfo = robotModelInfo_;

  auto stateDimBase = getStateDimBase(robotModelInfo);
  auto stateDim = getStateDim(robotModelInfo);
  auto inputDimBase = getInputDimBase(robotModelInfo);
  auto modeStateDim = getModeStateDim(robotModelInfo);
  auto modeInputDim = getModeInputDim(robotModelInfo);

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

  auto stateBase = stateBase_;
  auto inputBase = inputBase_;
  auto stateArm = stateArm_;

  switch (robotModelInfo.robotModelType)
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
      switch (robotModelInfo.modelMode)
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
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Gazebo_Loop::isTargetReached(int taskMode)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] START" << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] taskMode: " << taskMode << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] err_threshold_pos_: " << err_threshold_pos_ << std::endl;
  //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] err_threshold_ori_: " << err_threshold_ori_ << std::endl;

  if (!taskEndFlag_)
  {
    vector_t currentTarget = currentTarget_;

    tf::StampedTransform tf_ee_wrt_world = tf_ee_wrt_world_;

    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] currentTarget(0): " << currentTarget(0) << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] currentTarget(1): " << currentTarget(1) << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] currentTarget(2): " << currentTarget(2) << std::endl;

    vector_t dist_pos(3);
    dist_pos << abs(tf_ee_wrt_world.getOrigin().x() - currentTarget(0)), 
                abs(tf_ee_wrt_world.getOrigin().y() - currentTarget(1)), 
                abs(tf_ee_wrt_world.getOrigin().z() - currentTarget(2));

    double err_pos = dist_pos.norm();

    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_pos x: " << dist_pos[0] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_pos y: " << dist_pos[1] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_pos z: " << dist_pos[2] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] err_pos: " << err_pos << std::endl;

    Eigen::Quaterniond quat_ee(tf_ee_wrt_world.getRotation().w(), tf_ee_wrt_world.getRotation().x(), tf_ee_wrt_world.getRotation().y(), tf_ee_wrt_world.getRotation().z());
    Eigen::Quaterniond quat_target(currentTarget(6), currentTarget(3), currentTarget(4), currentTarget(5));

    vector_t dist_quat = quaternionDistance(quat_ee, quat_target);
    double err_ori = dist_quat.norm();

    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_quat r: " << dist_quat[0] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_quat p: " << dist_quat[1] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] dist_quat y: " << dist_quat[2] << std::endl;
    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] err_ori: " << err_ori << std::endl;

    //std::cout << "[MRT_ROS_Gazebo_Loop::isTargetReached] END" << std::endl << std::endl;

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

  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] taskMode_: " << taskMode_ << std::endl;
  std::cout << "[MRT_ROS_Gazebo_Loop::setTaskModeSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Gazebo_Loop::publishCommand(const PrimalSolution& currentPolicy)
{
  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] START" << std::endl;

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

    //PrimalSolution primalSolution = mrt_.getPolicy();
    //PrimalSolution primalSolution = currentPolicy;
    auto nextState = currentPolicy.getDesiredState(time_ + dt_);
    //auto nextState = currentPolicy.getDesiredState(time_);

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
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] BASE PUB" << std::endl;
    baseTwistPub_.publish(baseTwistMsg);
  }

  if (mrt_.getRobotModelInfo().modelMode == ModelMode::ArmMotion || 
      mrt_.getRobotModelInfo().modelMode == ModelMode::WholeBodyMotion)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] ARM PUB" << std::endl;
    armJointTrajectoryPub_.publish(armJointTrajectoryMsg);
    
    /*
    armJoint1TrajectoryPub_.publish(armJoint1TrajectoryMsg);
    armJoint2TrajectoryPub_.publish(armJoint2TrajectoryMsg);
    armJoint3TrajectoryPub_.publish(armJoint3TrajectoryMsg);
    armJoint4TrajectoryPub_.publish(armJoint4TrajectoryMsg);
    armJoint5TrajectoryPub_.publish(armJoint5TrajectoryMsg);
    armJoint6TrajectoryPub_.publish(armJoint6TrajectoryMsg);
    */
  }

  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] END" << std::endl;
}

}  // namespace ocs2
