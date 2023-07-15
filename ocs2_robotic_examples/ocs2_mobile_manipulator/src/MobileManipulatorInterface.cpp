// LAST UPDATE: 2023.07.14
//
// AUTHOR: Neset Unver Akmandor  (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

namespace ocs2 {
namespace mobile_manipulator {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile, 
                                                       const std::string& libraryFolder,
                                                       const std::string& urdfFile,
                                                       PointsOnRobot::points_radii_t pointsAndRadii,
                                                       int initModelModeInt)
  : taskFile_(taskFile), libraryFolder_(libraryFolder), urdfFile_(urdfFile), pointsAndRadii_(pointsAndRadii), initModelModeInt_(initModelModeInt)
{
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

  mpcTimer1_.reset();
  mpcTimer2_.reset();
  mpcTimer3_.reset();
  mpcTimer4_.reset();
  mpcTimer5_.reset();
  mpcTimer6_.reset();

  mrtTimer1_.reset();
  mrtTimer2_.reset();
  mrtTimer3_.reset();
  mrtTimer4_.reset();
  mrtTimer5_.reset();
  mrtTimer6_.reset();

  /// Check that task file exists
  boost::filesystem::path taskFilePath(taskFile_);
  if (boost::filesystem::exists(taskFilePath)) 
  {
    std::cerr << "[MobileManipulatorInterface::MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
  } 
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
  }

  /// Check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile_);
  if (boost::filesystem::exists(urdfFilePath)) 
  {
    std::cerr << "[MobileManipulatorInterface::MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  }
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }

  /// Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder_);
  boost::filesystem::create_directories(libraryFolderPath);
  //std::cerr << "[MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  /// Read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  
  /// Resolve meta-information about the model
  // Read robot type
  RobotModelType robotModelType = loadRobotType(taskFile_, "model_information.robotModelType");

  // Read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.removeJoints", removeJointNames, printOutFlag_);
  
  // Read the link names of joints
  std::vector<std::string> armJointFrameNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointFrameNames", armJointFrameNames, printOutFlag_);

  // Read the names of joints
  std::vector<std::string> armJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointNames", armJointNames, printOutFlag_);

  // Read the frame names
  std::string baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg_, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg_, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg_, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg_, "model_information.armControlMsg", printOutFlag_);

  if (printOutFlag_)
  {
    std::cerr << "\n #### Model Information:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### model_information.robotModelType: " << static_cast<int>(robotModelType);
    std::cerr << "\n #### model_information.removeJoints: ";
    for (const auto& name : removeJointNames) 
    {
      std::cerr << "\"" << name << "\" ";
    }
    std::cerr << "\n #### model_information.armJointFrameNames: ";
    for (const auto& name : armJointFrameNames) 
    {
      std::cerr << "\"" << name << "\" ";
    }
    std::cerr << "\n #### model_information.jointNames: ";
    for (const auto& name : armJointNames) 
    {
      std::cerr << "\"" << name << "\" ";
    }
    std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
    std::cerr << "\n #### model_information.armBaseFrame: \"" << armBaseFrame << "\"";
    std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"";
    std::cerr << "\n #### model_information.baseStateMsg: \"" << baseStateMsg_ << "\"";
    std::cerr << "\n #### model_information.armStateMsg: \"" << armStateMsg_ << "\"";
    std::cerr << "\n #### model_information.baseControlMsg: \"" << baseControlMsg_ << "\"";
    std::cerr << "\n #### model_information.armControlMsg: \"" << armControlMsg_ << "\"";
    std::cerr << " #### =============================================================================" << std::endl;
  }

  // Create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames)));
  //std::cerr << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  robotModelInfo_ = createRobotModelInfo(robotModelType,
                                         baseFrame, 
                                         armBaseFrame, 
                                         eeFrame,
                                         armJointFrameNames,
                                         armJointNames);

  // Set Model Settings
  //std::cerr << "\n #### Model Settings:";
  //std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
  loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
  loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
  loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  //std::cerr << " #### =============================================================================\n";

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  // Set MPC Problem
  //modelModeInt = 2;
  //setMPCProblem(modelModeInt, pointsAndRadii);

  mpcTimer2_.startTimer();
  launchNodes(nodeHandle_);
  mpcTimer2_.endTimer();

  mpcTimer3_.startTimer();
  setMPCProblem(initModelModeInt_, pointsAndRadii_);
  mpcTimer3_.startTimer();

  std::cout << "\n### MPC_ROS Benchmarking mpcTimer2_";
  std::cout << "\n###   Maximum : " << mpcTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer2_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPCProblem(size_t modelModeInt, PointsOnRobot::points_radii_t& pointsAndRadii, bool nextFlag)
{
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  ocp_.costPtr->clear();
  ocp_.softConstraintPtr->clear();
  ocp_.stateSoftConstraintPtr->clear();
  ocp_.finalSoftConstraintPtr->clear();

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] modelModeInt: " << modelModeInt << std::endl;
  bool isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  //printRobotModelInfo(robotModelInfo_);

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //// Optimal control problem
  /// Cost
  ocp_.costPtr->add("inputCost", getQuadraticInputCost());
  //std::cout << "" << std::endl;

  /// Constraints
  // Joint limits constraint
  ocp_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint());
  //std::cout << "" << std::endl;

  // Mobile base or End-effector state constraint
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getEndEffectorConstraint" << std::endl;
  ocp_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint("endEffector"));
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] AFTER getEndEffectorConstraint" << std::endl;
  ocp_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint("finalEndEffector"));
  //std::cout << "" << std::endl;

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getSelfCollisionConstraint" << std::endl;
  // Self-collision avoidance constraint
  if (activateSelfCollision_) 
  {
    if (robotModelInfo_.modelMode == ModelMode::ArmMotion || robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
    {
      ocp_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint("selfCollision"));
    }
  }
  //std::cout << "" << std::endl;

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getExtCollisionConstraint" << std::endl;
  // External-collision avoidance constraint
  activateExtCollision_ = false;
  if (activateExtCollision_) 
  {
    createPointsOnRobotPtr(pointsAndRadii);
    
    if (pointsOnRobotPtr_->getNumOfPoints() > 0) 
    {
      //esdfCachingServerPtr_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
      //voxbloxInterpolatorPtr_ = esdfCachingServerPtr_->getInterpolator();

      ///////// NUA TODO: ADAPT TO MULTI MODAL!
      pointsOnRobotPtr_->initialize(*pinocchioInterfacePtr_,
                                    MobileManipulatorPinocchioMapping(robotModelInfo_),
                                    MobileManipulatorPinocchioMappingCppAd(robotModelInfo_),
                                    robotModelInfo_,
                                    "points_on_robot",
                                    libraryFolder_,
                                    recompileLibraries_,
                                    false);
    }
    else
    {
      pointsOnRobotPtr_ = nullptr;
    }

    emuPtr_.reset(new ExtMapUtility());
    
    //// NUA TODO: Set these parameters in taskfile!
    std::string world_frame_name = "world";
    std::string pub_name_oct_dist_visu = "occ_dist";
    std::string pub_name_oct_dist_array_visu = "occ_dist_array";

    emuPtr_->setWorldFrameName(world_frame_name);
    emuPtr_->setPubOccDistVisu(pub_name_oct_dist_visu);
    emuPtr_->setPubOccDistArrayVisu(pub_name_oct_dist_array_visu);
    
    ocp_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint("extCollision"));
  }
  //std::cout << "" << std::endl;

  // Dynamics
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE Dynamics" << std::endl;
  switch (robotModelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      //std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Base" << std::endl;
      ocp_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        printOutFlag_));
      break;
    }
    
    case ModelMode::ArmMotion:
    {
      //std::cout << "[MobileManipulatorInterface::setMPCProblem] Robotics Arm" << std::endl;
      ocp_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
      break;
    }
    
    case ModelMode::WholeBodyMotion: 
    {
      //std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Manipulator" << std::endl;
      ocp_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               printOutFlag_));
      break;
    }

    default:
      throw std::invalid_argument("[MobileManipulatorInterface::setMPCProblem] ERROR: Invalid model mode!");
  }
  //std::cout << "" << std::endl;

  /*
   * Pre-computation
   */
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Pre-computation" << std::endl;
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
    while(1);
    ocp_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }
  //std::cout << "" << std::endl;

  // Rollout
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Rollout" << std::endl;
  rolloutSettings_ = rollout::loadSettings(taskFile_, "rollout", printOutFlag_);
  if (!nextFlag)
  {
    rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
  }

  // Initialization
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  if (!nextFlag)
  {
    initializerPtr_.reset(new DefaultInitializer(modeInputDim));
  }

  //mpcProblemNextFlag_ = true;

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void MobileManipulatorInterface::modelModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  std::cout << "[MobileManipulatorInterface::modelModeCallback] START" << std::endl;

  size_t modelModeInt = msg->data;
  std::cout << "[MobileManipulatorInterface::modelModeCallback] modelModeInt: " << modelModeInt << std::endl;
  //updateModelMode(robotModelInfo_, modelModeInt);

  std::cout << "[MobileManipulatorInterface::modelModeCallback] END" << std::endl << std::endl;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchNodes(ros::NodeHandle& nodeHandle)
{
  std::string oct_msg_name = "octomap_scan";
  std::string tf_msg_name = "tf";
  std::string model_mode_msg_name = "mobile_manipulator_model_mode";

  double dt = 0.1;

  if (emuPtr_)
  {
    // Octomap Subscriber
    emuPtr_->setNodeHandle(nodeHandle);
    emuPtr_->updateOct(oct_msg_name);
  }

  if (pointsOnRobotPtr_)
  {
    pointsOnRobotPtr_->setNodeHandle(nodeHandle);
    pointsOnRobotPtr_->publishPointsOnRobotVisu(dt);
  }

  //subModelModeMsg_ = nodeHandle.subscribe(model_mode_msg_name, 10, &MobileManipulatorInterface::modelModeCallback, this);

  // Subscribe Model Mode
  auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
  {
    std::cout << "[MobileManipulatorInterface::launchNodes] START" << std::endl;

    size_t modelModeInt = msg->data;
    std::cout << "[MobileManipulatorInterface::launchNodes] modelModeInt: " << modelModeInt << std::endl;
    //updateModelMode(robotModelInfo_, modelModeInt);

    std::cout << "[MobileManipulatorInterface::launchNodes] END" << std::endl;
    std::cout << "" << std::endl;

    //setMPCProblem(modelModeInt, pointsAndRadii_, true);

    //mpcProblemNextFlag_ = true;
    //shutdownNodes();
    //shutDownFlag_ = true;
  };
  modelModeSubscriber_ = nodeHandle.subscribe<std_msgs::UInt8>(model_mode_msg_name, 1, modelModeCallback);

  //spin();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMPC()
{
  std::cout << "[MobileManipulatorInterface::runMPC] START" << std::endl;

  RobotModelInfo robotModelInfo = robotModelInfo_;
  ddp::Settings ddpSettings = ddpSettings_;
  mpc::Settings mpcSettings = mpcSettings_;
  OptimalControlProblem ocp = ocp_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr = referenceManagerPtr_;

  size_t modelModeInt = initModelModeInt_;
  int iter = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "=====================================================" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "[MobileManipulatorInterface::runMRT] START ITERATION: " << iter << std::endl;
    //std::cout << "[MobileManipulatorInterface::runMRT] modelModeInt: " << modelModeInt << std::endl;

    mpcTimer1_.startTimer();
    
    // Robot interface
    //std::cout << "[MobileManipulatorInterface::runMPC] START launchNodes" << std::endl;
    //mpcTimer2_.startTimer();
    //launchNodes(nodeHandle_);
    //mpcTimer2_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMPC] END launchNodes" << std::endl;

    //mpcTimer3_.startTimer();
    //setMPCProblem(modelModeInt, pointsAndRadii_);
    //mpcTimer3_.endTimer();
    //printRobotModelInfo(robotModelInfo_);

    mpcTimer4_.startTimer();
    if (mpcProblemNextFlag_)
    {
      robotModelInfo = robotModelInfo_;
      ddpSettings = ddpSettings_;
      mpcSettings = mpcSettings_;
      ocp = ocp_;
      referenceManagerPtr = referenceManagerPtr_;
      rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
      initializerPtr_.reset(new DefaultInitializer(robotModelInfo.modeInputDim));
    }
    mpcTimer4_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMPC] DEBUG INF" << std::endl;
    //while(1);

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr" << std::endl;
    // ROS ReferenceManager
    mpcTimer5_.startTimer();
    rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(robotModelName_, referenceManagerPtr, robotModelInfo));
    mpcTimer5_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr subscribe" << std::endl;
    mpcTimer6_.startTimer();
    rosReferenceManagerPtr_->subscribe(nodeHandle_);
    mpcTimer6_.endTimer();

    // MPC
    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc" << std::endl;
    mpcTimer7_.startTimer();
    ocs2::GaussNewtonDDP_MPC mpc(mpcSettings_, 
                                ddpSettings_, 
                                *rolloutPtr_, 
                                ocp_, 
                                *initializerPtr_);
    mpcTimer7_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc setReferenceManager" << std::endl;
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

    // Launch MPC ROS node
    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode" << std::endl;
    mpcTimer8_.startTimer();
    MPC_ROS_Interface mpcNode(mpc, robotModelName_);
    mpcTimer8_.endTimer();

    mpcTimer1_.endTimer();

    std::cout << '\n';
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer1_";
    std::cout << "\n###   Maximum : " << mpcTimer1_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer1_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer1_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    /*
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer2_";
    std::cout << "\n###   Maximum : " << mpcTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer2_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
    std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    */
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer4_";
    std::cout << "\n###   Maximum : " << mpcTimer4_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer4_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer4_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer5_";
    std::cout << "\n###   Maximum : " << mpcTimer5_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer5_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer5_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer6_";
    std::cout << "\n###   Maximum : " << mpcTimer6_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer6_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer6_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer7_";
    std::cout << "\n###   Maximum : " << mpcTimer7_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer7_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer7_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer8_";
    std::cout << "\n###   Maximum : " << mpcTimer8_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer8_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer8_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode launchNodes" << std::endl;
    mpcNode.launchNodes(nodeHandle_);

    modelModeInt = mpcNode.getModelModeInt();

    iter++;
 
    std::cout << "[MobileManipulatorInterface::runMRT] END ITERATION: " << iter << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  std::cout << "[MobileManipulatorInterface::runMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMRT()
{
  std::cout << "[MobileManipulatorInterface::runMRT] START" << std::endl;

  RobotModelInfo robotModelInfo = robotModelInfo_;
  ddp::Settings ddpSettings = ddpSettings_;
  mpc::Settings mpcSettings = mpcSettings_;
  OptimalControlProblem ocp = ocp_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr = referenceManagerPtr_;

  size_t modelModeInt = initModelModeInt_;
  int iter = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "=====================================================" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "[MobileManipulatorInterface::runMRT] START ITERATION: " << iter << std::endl;
    std::cout << "[MobileManipulatorInterface::runMRT] modelModeInt: " << modelModeInt << std::endl;

    mrtTimer1_.startTimer();

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE setMPCProblem" << std::endl;
    //mrtTimer2_.startTimer();
    //setMPCProblem(modelModeInt, pointsAndRadii_);
    //mrtTimer2_.endTimer();
    //printRobotModelInfo(robotModelInfo_);
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER setMPCProblem" << std::endl;

    mrtTimer2_.startTimer();
    if (mpcProblemNextFlag_)
    {
      robotModelInfo = robotModelInfo_;
      ddpSettings = ddpSettings_;
      mpcSettings = mpcSettings_;
      ocp = ocp_;
      referenceManagerPtr = referenceManagerPtr_;
      rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
      initializerPtr_.reset(new DefaultInitializer(robotModelInfo.modeInputDim));
    }
    mrtTimer2_.endTimer();

    // MRT
    mrtTimer3_.startTimer();
    MRT_ROS_Interface mrt(robotModelInfo, robotModelName_);
    mrtTimer3_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE initRollout" << std::endl;
    mrtTimer4_.startTimer();
    mrt.initRollout(&*rolloutPtr_);
    mrtTimer4_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER initRollout" << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE launchNodes" << std::endl;
    mrtTimer5_.startTimer();
    mrt.launchNodes(nodeHandle_);
    mrtTimer5_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER launchNodes" << std::endl;

    // Visualization
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE ocs2_mm_visu" << std::endl;
    //std::shared_ptr<ocs2::mobile_manipulator::OCS2_Mobile_Manipulator_Visualization> ocs2_mm_visu(new ocs2::mobile_manipulator::OCS2_Mobile_Manipulator_Visualization(nodeHandle_,
    //                                                                                                                                                                  *pinocchioInterfacePtr_,
    //                                                                                                                                                                 urdfFile_,
    //                                                                                                                                                                  taskFile_));
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER ocs2_mm_visu" << std::endl;

    // MRT loop
    std::string worldFrameName = "world";
    std::cout << "[MobileManipulatorInterface::runMRT] BEFORE mrt_loop" << std::endl;
    mrtTimer6_.startTimer();
    MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle_, 
                                mrt, 
                                worldFrameName,
                                baseStateMsg_,
                                armStateMsg_,
                                baseControlMsg_,
                                armControlMsg_,
                                mpcSettings_.mrtDesiredFrequency_, 
                                mpcSettings_.mpcDesiredFrequency_);
    mrtTimer6_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER mrt_loop" << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE subscribeObservers" << std::endl;
    //mrt_loop.subscribeObservers({ocs2_mm_visu});
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER subscribeObservers" << std::endl;

    // initial command
    mrtTimer7_.startTimer();
    vector_t initTarget(7);
    //initTarget.head(3) << -0.2, 0, 1.0;
    //initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    mrt_loop.getInitTarget(initTarget);   
    mrtTimer7_.endTimer();

    // Run mrt_loop
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE run" << std::endl;
    mrtTimer1_.endTimer();

    std::cout << '\n';
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer1_";
    std::cout << "\n###   Maximum : " << mrtTimer1_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer1_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer1_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer2_";
    std::cout << "\n###   Maximum : " << mrtTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer2_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer3_";
    std::cout << "\n###   Maximum : " << mrtTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer3_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer4_";
    std::cout << "\n###   Maximum : " << mrtTimer4_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer4_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer4_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer5_";
    std::cout << "\n###   Maximum : " << mrtTimer5_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer5_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer5_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer6_";
    std::cout << "\n###   Maximum : " << mrtTimer6_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer6_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer6_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MRT_ROS Benchmarking mrtTimer7_";
    std::cout << "\n###   Maximum : " << mrtTimer7_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mrtTimer7_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mrtTimer7_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

    mrt_loop.run(initTarget);
    
    modelModeInt = mrt.getModelModeInt();

    iter++;
    std::cout << "[MobileManipulatorInterface::runMRT] END ITERATION: " << iter << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  std::cout << "[MobileManipulatorInterface::runMRT] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost() 
{
  //const size_t modeStateDim = getStateDimTmp(robotModelInfo_);
  const size_t modeStateDim = getModeStateDim(robotModelInfo_);
  
  //const size_t modeInputDim = getInputDim(robotModelInfo_);
  const size_t modeInputDim = getModeInputDim(robotModelInfo_);
  
  auto modelMode = getModelModeInt(robotModelInfo_);

  //std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modelMode: " << modelMode << std::endl;
  //std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modeInputDim: " << modeInputDim << std::endl;

  matrix_t R = matrix_t::Zero(modeInputDim, modeInputDim);

  // Input cost of mobile base
  if (modelMode == 0 || modelMode == 2) 
  {
    const size_t inputDimBase = getInputDimBase(robotModelInfo_);

    //std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] inputDimBase: " << inputDimBase << std::endl;

    matrix_t R_base = matrix_t::Zero(inputDimBase, inputDimBase);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.base", R_base, printOutFlag_);
    R.topLeftCorner(inputDimBase, inputDimBase) = R_base;
  }

  // Input cost of arm
  if (modelMode == 1 || modelMode == 2) 
  {
    const size_t inputDimArm = getInputDimArm(robotModelInfo_);

    //std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] inputDimArm: " << inputDimArm << std::endl;

    matrix_t R_arm = matrix_t::Zero(inputDimArm, inputDimArm);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.arm", R_arm, printOutFlag_);
    R.bottomRightCorner(inputDimArm, inputDimArm) = R_arm;
  }

  if (printOutFlag_)
  {
    std::cerr << "\n #### Input Cost Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "inputCost.R:  \n" << R << '\n';
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), modeStateDim));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint() 
{
  auto modelMode = getModelModeInt(robotModelInfo_);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", printOutFlag_);

  //const size_t modeStateDim = getStateDimTmp(robotModelInfo_);
  const size_t modeStateDim = getModeStateDim(robotModelInfo_);
  
  //const size_t modeInputDim = getInputDim(robotModelInfo_);
  const size_t modeInputDim = getModeInputDim(robotModelInfo_);

  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] modeInputDim: " << modeInputDim << std::endl;

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit & modelMode != 0) 
  {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const size_t stateDimArm = getStateDimArm(robotModelInfo_);

    // Arm joint DOF limits from the parsed URDF      
    const vector_t lowerBound = model.lowerPositionLimit.tail(stateDimArm);
    const vector_t upperBound = model.upperPositionLimit.tail(stateDimArm);

    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;

    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    //std::cerr << "\n #### ArmJointPositionLimits Settings: ";
    //std::cerr << "\n #### =============================================================================\n";
    //std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    //std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", printOutFlag_);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", printOutFlag_);
    //std::cerr << " #### =============================================================================\n";
    
    stateLimits.reserve(modeStateDim);
    const size_t stateOffset = modeStateDim - stateDimArm;

    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateOffset: " << stateOffset << std::endl;

    for (int i = 0; i < stateDimArm; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = stateOffset + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }

    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;

    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits.size(): " << stateLimits.size() << std::endl;
  }

  // Load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(modeInputDim);
    vector_t upperBound = vector_t::Zero(modeInputDim);

    if (modelMode == 0 || modelMode == 2)
    {
      const size_t inputDimBase = getInputDimBase(robotModelInfo_);

      // Mobile base joint DOFs velocity limits
      vector_t lowerBoundBase = vector_t::Zero(inputDimBase);
      vector_t upperBoundBase = vector_t::Zero(inputDimBase);
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.base", lowerBoundBase, printOutFlag_);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.base", upperBoundBase, printOutFlag_);
      
      lowerBound.head(inputDimBase) = lowerBoundBase;
      upperBound.head(inputDimBase) = upperBoundBase;
    }

    if (modelMode == 1 || modelMode == 2)
    {
      const size_t inputDimArm = getInputDimArm(robotModelInfo_);

      // Arm joint DOFs velocity limits
      vector_t lowerBoundArm = vector_t::Zero(inputDimArm);
      vector_t upperBoundArm = vector_t::Zero(inputDimArm);
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.arm", lowerBoundArm, printOutFlag_);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.arm", upperBoundArm, printOutFlag_);
      
      lowerBound.tail(inputDimArm) = lowerBoundArm;
      upperBound.tail(inputDimArm) = upperBoundArm;
    }

    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;
    //std::cerr << "\n #### JointVelocityLimits Settings: ";
    //std::cerr << "\n #### =============================================================================\n";
    //std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    //std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", printOutFlag_);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", printOutFlag_);
    //std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(modeInputDim);
    for (int i = 0; i < modeInputDim; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }

    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;
  }

  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits size: " << stateLimits.size() << std::endl;
  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] inputLimits size: " << inputLimits.size() << std::endl;

  auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));

  boxConstraints->initializeOffset(0.0, vector_t::Zero(modeStateDim), vector_t::Zero(modeInputDim));

  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] DEBUG INF" << std::endl;
  //while(1);

  return boxConstraints;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const std::string& prefix) 
{
  //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] START prefix: " << prefix << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  //const int stateDim = getStateDim(robotModelInfo_);
  //const int modeStateDim = getModeStateDim(robotModelInfo_);
  //const int modeInputDim = getModeInputDim(robotModelInfo_);

  std::string modelName = getModelModeString(robotModelInfo_) + "_end_effector_kinematics";
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  //std::cerr << "\n #### " << prefix << " Settings: ";
  //std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", printOutFlag_);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", printOutFlag_);
  //std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) 
  {
    throw std::runtime_error("[MobileManipulatorInterface::getEndEffectorConstraint] ERROR: referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
    while(1);

    MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(*pinocchioInterfacePtr_, pinocchioMapping, {robotModelInfo_.robotArm.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } 
  else 
  {
    //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] BEFORE pinocchioMappingCppAd" << std::endl;
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(robotModelInfo_);

    //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] BEFORE eeKinematics" << std::endl;
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_,
                                                     pinocchioMappingCppAd, 
                                                     robotModelInfo_,
                                                     modelName, 
                                                     libraryFolder_, 
                                                     recompileLibraries_, 
                                                     false);
    //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] AFTER eeKinematics" << std::endl;
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, robotModelInfo_));
  }

  //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
  //while(1);

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const std::string& prefix) 
{
  //std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.2;
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;

  //std::cerr << "\n #### SelfCollision Settings: ";
  //std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", printOutFlag_);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", printOutFlag_);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionObjectPairs", collisionObjectPairs, printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionLinkPairs", collisionLinkPairs, printOutFlag_);
  //std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr_, collisionLinkPairs, collisionObjectPairs);
  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  //std::cerr << "[MobileManipulatorInterface::getSelfCollisionConstraint] Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] DEBUG INF" << std::endl;
    while(1);

    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorSelfCollisionConstraint(MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                                std::move(geometryInterface), 
                                                                                                minimumDistance));
  } 
  else 
  {
    constraint = std::unique_ptr<StateConstraint>(new SelfCollisionConstraintCppAd(*pinocchioInterfacePtr_, 
                                                                                    MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                    MobileManipulatorPinocchioMappingCppAd(robotModelInfo_),
                                                                                    std::move(geometryInterface), 
                                                                                    robotModelInfo_,
                                                                                    minimumDistance,
                                                                                    "self_collision", 
                                                                                    libraryFolder_, 
                                                                                    recompileLibraries_, 
                                                                                    false));
  }

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({mu, delta}));

  //std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getExtCollisionConstraint(const std::string& prefix) 
{
  std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  const int modelStateDim = getStateDim(robotModelInfo_);
  auto modelModeInt = getModelModeInt(robotModelInfo_);

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t maxDistance = 10;
  std::cerr << "\n #### ExtCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, maxDistance, prefix + ".maxDistance", true);
  std::cerr << " #### =============================================================================\n";

  ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface(*pinocchioInterfacePtr_);
  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ true" << std::endl;

    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] DEBUG INF" << std::endl;
    while(1);

    ///////// NUA TODO: NOT FUNCTIONAL AND COMPLETED YET!
    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorExtCollisionConstraint(MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                              std::move(extCollisionPinocchioGeometryInterface), 
                                                                                              pointsOnRobotPtr_,
                                                                                              maxDistance,
                                                                                              emuPtr_,
                                                                                              modelModeInt,
                                                                                              modelStateDim));
  }
  else
  {
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ false" << std::endl;

    constraint = std::unique_ptr<StateConstraint>(new ExtCollisionConstraintCppAd(*pinocchioInterfacePtr_, 
                                                                                  MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                  MobileManipulatorPinocchioMappingCppAd(robotModelInfo_),
                                                                                  pointsOnRobotPtr_,
                                                                                  maxDistance,
                                                                                  emuPtr_,
                                                                                  "ext_collision", 
                                                                                  libraryFolder_, 
                                                                                  recompileLibraries_, 
                                                                                  false));
  }

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({mu, delta}));

  std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

}  // namespace mobile_manipulator
}  // namespace ocs2
