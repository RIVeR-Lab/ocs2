// LAST UPDATE: 2023.07.31
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
  : taskFile_(taskFile), 
    libraryFolder_(libraryFolder), 
    urdfFile_(urdfFile), 
    pointsAndRadii_(pointsAndRadii), 
    initModelModeInt_(initModelModeInt), 
    modelModeIntQuery_(initModelModeInt)
{
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

  mpcTimer0_.reset();
  mpcTimer1_.reset();
  mpcTimer2_.reset();
  mpcTimer3_.reset();
  mpcTimer4_.reset();
  mpcTimer5_.reset();
  mpcTimer6_.reset();
  mpcTimer7_.reset();
  mpcTimer8_.reset();
  mpcTimer9_.reset();
  mpcTimer10_.reset();
  mpcTimer11_.reset();

  mrtTimer1_.reset();
  mrtTimer2_.reset();
  mrtTimer3_.reset();
  mrtTimer4_.reset();
  mrtTimer5_.reset();
  mrtTimer6_.reset();
  mrtTimer7_.reset();

  currentTarget_.resize(7);

  // Setting up the environment variable

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
  std::string robotName, baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg_, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg_, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg_, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg_, "model_information.armControlMsg", printOutFlag_);

  if (printOutFlag_)
  {
    std::cout << "\n #### Model Information:" << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
    std::cout << "#### model_information.robotName: " << robotName << std::endl;
    std::cout << "#### model_information.robotModelType: " << static_cast<int>(robotModelType) << std::endl;
    std::cout << "#### model_information.removeJoints: " << std::endl;
    for (const auto& name : removeJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.armJointFrameNames: ";
    for (const auto& name : armJointFrameNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.jointNames: ";
    for (const auto& name : armJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.baseFrame: " << baseFrame << std::endl;
    std::cout << "#### model_information.armBaseFrame: " << armBaseFrame << std::endl;
    std::cout << "#### model_information.eeFrame: " << eeFrame << std::endl;
    std::cout << "#### model_information.baseStateMsg: " << baseStateMsg_ << std::endl;
    std::cout << "#### model_information.armStateMsg: " << armStateMsg_ << std::endl;
    std::cout << "#### model_information.baseControlMsg: " << baseControlMsg_ << std::endl;
    std::cout << "#### model_information.armControlMsg: " << armControlMsg_ << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
  }

  // Create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames)));
  //std::cerr << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  robotModelInfo_ = createRobotModelInfo(robotName,
                                         robotModelType,
                                         baseFrame, 
                                         armBaseFrame, 
                                         eeFrame,
                                         armJointFrameNames,
                                         armJointNames);

  // Set Model Settings
  if (printOutFlag_)
  {
    std::cerr << "\n #### Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }
    loadData::loadPtreeValue(pt, drlFlag_, "model_settings.drlFlag", printOutFlag_);
    loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
    loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
    loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
    loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  if (printOutFlag_)
  {
    std::cerr << " #### =============================================================================\n";
  }

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  //// NUA NOTE: Depricated...some initialization that requires node handle is omitted!
  std::cerr << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  while(1);

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MobileManipulatorInterface::MobileManipulatorInterface(ros::NodeHandle& nodeHandle,
                                                       const std::string& taskFile, 
                                                       const std::string& libraryFolder, 
                                                       const std::string& urdfFile,
                                                       PointsOnRobot::points_radii_t pointsAndRadii,
                                                       int initModelModeInt)
  : nodeHandle_(nodeHandle), 
    taskFile_(taskFile), 
    libraryFolder_(libraryFolder), 
    urdfFile_(urdfFile), 
    pointsAndRadii_(pointsAndRadii), 
    initModelModeInt_(initModelModeInt),
    modelModeIntQuery_(initModelModeInt)
{
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] START" << std::endl;

  mpcTimer0_.reset();
  mpcTimer1_.reset();
  mpcTimer2_.reset();
  mpcTimer3_.reset();
  mpcTimer4_.reset();
  mpcTimer5_.reset();
  mpcTimer6_.reset();
  mpcTimer7_.reset();
  mpcTimer8_.reset();
  mpcTimer9_.reset();
  mpcTimer10_.reset();
  mpcTimer11_.reset();

  mrtTimer1_.reset();
  mrtTimer2_.reset();
  mrtTimer3_.reset();
  mrtTimer4_.reset();
  mrtTimer5_.reset();
  mrtTimer6_.reset();
  mrtTimer7_.reset();

  currentTarget_.resize(7);

  /// Check that task file exists
  boost::filesystem::path taskFilePath(taskFile_);
  if (boost::filesystem::exists(taskFilePath)) 
  {
    std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] Loading task file: " << taskFilePath << std::endl;
  } 
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface(6)] Task file not found: " + taskFilePath.string());
  }

  /// Check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile_);
  if (boost::filesystem::exists(urdfFilePath)) 
  {
    std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  }
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface(6)] URDF file not found: " + urdfFilePath.string());
  }

  /// Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder_);
  boost::filesystem::create_directories(libraryFolderPath);
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

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
  std::string robotName, baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg_, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg_, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg_, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg_, "model_information.armControlMsg", printOutFlag_);

  if (printOutFlag_)
  {
    std::cout << "\n #### Model Information:" << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
    std::cout << "#### model_information.robotName: " << robotName << std::endl;
    std::cout << "#### model_information.robotModelType: " << static_cast<int>(robotModelType) << std::endl;
    std::cout << "#### model_information.removeJoints: " << std::endl;
    for (const auto& name : removeJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.armJointFrameNames:" << std::endl;
    for (const auto& name : armJointFrameNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.jointNames: ";
    for (const auto& name : armJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "#### model_information.baseFrame: " << baseFrame << std::endl;
    std::cout << "#### model_information.armBaseFrame: " << armBaseFrame << std::endl;
    std::cout << "#### model_information.eeFrame: " << eeFrame << std::endl;
    std::cout << "#### model_information.baseStateMsg: " << baseStateMsg_ << std::endl;
    std::cout << "#### model_information.armStateMsg: " << armStateMsg_ << std::endl;
    std::cout << "#### model_information.baseControlMsg: " << baseControlMsg_ << std::endl;
    std::cout << "#### model_information.armControlMsg: " << armControlMsg_ << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
  }

  // Create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames)));
  //std::cerr << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  robotModelInfo_ = createRobotModelInfo(robotName,
                                         robotModelType,
                                         baseFrame, 
                                         armBaseFrame, 
                                         eeFrame,
                                         armJointFrameNames,
                                         armJointNames);

  // Set Model Settings
  if (printOutFlag_)
  {
    std::cerr << "\n #### Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }
    loadData::loadPtreeValue(pt, drlFlag_, "model_settings.drlFlag", printOutFlag_);
    loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
    loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
    loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
    loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  if (printOutFlag_)
  {
    std::cerr << " #### =============================================================================\n";
  }

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  // Set ROS Reference Manager
  rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(robotModelName_, referenceManagerPtr_));
  rosReferenceManagerPtr_->subscribe(nodeHandle_);

  // Set Rollout Settings
  rolloutSettings_ = rollout::loadSettings(taskFile_, "rollout", printOutFlag_);

  /*
  // Set PointsOnRobot
  initializePointsOnRobotPtr(pointsAndRadii_);

  // Set ExtMapUtility
  emuPtr_.reset(new ExtMapUtility());
  
  //// NUA TODO: Set these parameters in taskfile!
  std::string pub_name_oct_dist_visu = "occ_dist";
  std::string pub_name_oct_dist_array_visu = "occ_dist_array";

  emuPtr_->setWorldFrameName(worldFrameName_);
  emuPtr_->setPubOccDistVisu(pub_name_oct_dist_visu);
  emuPtr_->setPubOccDistArrayVisu(pub_name_oct_dist_array_visu);
  */

  // Create costs/constraints
  size_t modelModeInt;
  bool isModeUpdated;

  // Mode 0
  modelModeInt = 0;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);

  quadraticInputCostPtr_mode0_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode0_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode0_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode0_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode0_ = getSelfCollisionConstraint("selfCollision");
  
  //// NUA TODO: DEBUG AND TEST IS REQUIRED!
  //initializePointsOnRobotPtr(pointsAndRadii_);
  //extCollisionConstraintPtr_mode0_= getExtCollisionConstraint("extCollision");
  
  dynamicsPtr_mode0_.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                  "MobileBaseDynamics", 
                                                  libraryFolder_, 
                                                  recompileLibraries_, 
                                                  printOutFlag_));

  // Mode 1
  modelModeInt = 1;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);

  quadraticInputCostPtr_mode1_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode1_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode1_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode1_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode1_ = getSelfCollisionConstraint("selfCollision");
  
  //// NUA TODO: DEBUG AND TEST IS REQUIRED!
  //initializePointsOnRobotPtr(pointsAndRadii_);
  //extCollisionConstraintPtr_mode1_= getExtCollisionConstraint("extCollision");
  
  dynamicsPtr_mode1_.reset(new RobotArmDynamics(robotModelInfo_, 
                                                "MobileBaseDynamics", 
                                                libraryFolder_, 
                                                recompileLibraries_, 
                                                printOutFlag_));
  
  // Mode 2
  modelModeInt = 2;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);

  quadraticInputCostPtr_mode2_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode2_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode2_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode2_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode2_ = getSelfCollisionConstraint("selfCollision");
  
  //// NUA TODO: DEBUG AND TEST IS REQUIRED!
  //initializePointsOnRobotPtr(pointsAndRadii_);
  //extCollisionConstraintPtr_mode2_= getExtCollisionConstraint("extCollision");
  
  dynamicsPtr_mode2_.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                         "MobileBaseDynamics", 
                                                         libraryFolder_, 
                                                         recompileLibraries_, 
                                                         printOutFlag_));

  // Set MPC Problem
  //mpcTimer2_.startTimer();
  launchNodes(nodeHandle_);
  //mpcTimer2_.endTimer();

  //mpcTimer3_.startTimer();
  //setMPCProblem();
  //mpcTimer3_.endTimer();

  if (printOutFlag_)
  {
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer2_";
    std::cout << "\n###   Maximum : " << mpcTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer2_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
    std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
    std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] END" << std::endl;
}

///////// NUA TODO: NOT FUNCTIONAL, NEED DEBUG AND TESTING!!!
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializePointsOnRobotPtr(PointsOnRobot::points_radii_t& pointsAndRadii) 
{
  pointsOnRobotPtr_.reset(new PointsOnRobot(pointsAndRadii));
  if (pointsOnRobotPtr_->getNumOfPoints() > 0) 
  {
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

  size_t modelModeInt = getModelModeInt(robotModelInfo_);
  if (modelModeInt == 0)
  {
    pointsOnRobotPtr_mode0_.reset(new PointsOnRobot(pointsAndRadii));
    if (pointsOnRobotPtr_mode0_->getNumOfPoints() > 0) 
    {
      ///////// NUA TODO: ADAPT TO MULTI MODAL!
      pointsOnRobotPtr_mode0_->initialize(*pinocchioInterfacePtr_,
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
      pointsOnRobotPtr_mode0_ = nullptr;
    }
  }

  else if (modelModeInt == 1)
  {
    pointsOnRobotPtr_mode1_.reset(new PointsOnRobot(pointsAndRadii));
    if (pointsOnRobotPtr_mode1_->getNumOfPoints() > 0) 
    {
      ///////// NUA TODO: ADAPT TO MULTI MODAL!
      pointsOnRobotPtr_mode1_->initialize(*pinocchioInterfacePtr_,
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
      pointsOnRobotPtr_mode1_ = nullptr;
    }
  }

  else if (modelModeInt == 2)
  {
    pointsOnRobotPtr_mode2_.reset(new PointsOnRobot(pointsAndRadii));
    if (pointsOnRobotPtr_mode2_->getNumOfPoints() > 0) 
    {
      ///////// NUA TODO: ADAPT TO MULTI MODAL!
      pointsOnRobotPtr_mode2_->initialize(*pinocchioInterfacePtr_,
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
      pointsOnRobotPtr_mode2_ = nullptr;
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPCProblem(bool iterFlag)
{
  std::cout << "[MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  /*
  while(!mpcExitFlag_)
  {
    //std::cout << "[MobileManipulatorInterface::setMPCProblem] WAITING..." << std::endl;
    spinOnce();
  }
  */

  mpcTimer0_.startTimer();

  mpcTimer1_.startTimer();

  ocp_.costPtr->clear();
  ocp_.stateCostPtr->clear();
  ocp_.preJumpCostPtr->clear();
  ocp_.finalCostPtr->clear();

  ocp_.softConstraintPtr->clear();
  ocp_.stateSoftConstraintPtr->clear();
  ocp_.preJumpSoftConstraintPtr->clear();
  ocp_.finalSoftConstraintPtr->clear();

  ocp_.equalityConstraintPtr->clear();
  ocp_.stateEqualityConstraintPtr->clear();
  ocp_.preJumpEqualityConstraintPtr->clear();
  ocp_.finalEqualityConstraintPtr->clear();

  ocp_.equalityLagrangianPtr->clear();
  ocp_.stateEqualityLagrangianPtr->clear();
  ocp_.inequalityLagrangianPtr->clear();
  ocp_.stateInequalityLagrangianPtr->clear();
  ocp_.preJumpEqualityLagrangianPtr->clear();
  ocp_.preJumpInequalityLagrangianPtr->clear();
  ocp_.finalEqualityLagrangianPtr->clear();
  ocp_.finalInequalityLagrangianPtr->clear();

  /*
  ocp1_.costPtr->clear();
  ocp1_.stateCostPtr->clear();
  ocp1_.preJumpCostPtr->clear();
  ocp1_.finalCostPtr->clear();

  ocp1_.softConstraintPtr->clear();
  ocp1_.stateSoftConstraintPtr->clear();
  ocp1_.preJumpSoftConstraintPtr->clear();
  ocp1_.finalSoftConstraintPtr->clear();

  ocp1_.equalityConstraintPtr->clear();
  ocp1_.stateEqualityConstraintPtr->clear();
  ocp1_.preJumpEqualityConstraintPtr->clear();
  ocp1_.finalEqualityConstraintPtr->clear();

  ocp1_.equalityLagrangianPtr->clear();
  ocp1_.stateEqualityLagrangianPtr->clear();
  ocp1_.inequalityLagrangianPtr->clear();
  ocp1_.stateInequalityLagrangianPtr->clear();
  ocp1_.preJumpEqualityLagrangianPtr->clear();
  ocp1_.preJumpInequalityLagrangianPtr->clear();
  ocp1_.finalEqualityLagrangianPtr->clear();
  ocp1_.finalInequalityLagrangianPtr->clear();

  // ---------------------------------------

  ocp2_.costPtr->clear();
  ocp2_.stateCostPtr->clear();
  ocp2_.preJumpCostPtr->clear();
  ocp2_.finalCostPtr->clear();

  ocp2_.softConstraintPtr->clear();
  ocp2_.stateSoftConstraintPtr->clear();
  ocp2_.preJumpSoftConstraintPtr->clear();
  ocp2_.finalSoftConstraintPtr->clear();

  ocp2_.equalityConstraintPtr->clear();
  ocp2_.stateEqualityConstraintPtr->clear();
  ocp2_.preJumpEqualityConstraintPtr->clear();
  ocp2_.finalEqualityConstraintPtr->clear();

  ocp2_.equalityLagrangianPtr->clear();
  ocp2_.stateEqualityLagrangianPtr->clear();
  ocp2_.inequalityLagrangianPtr->clear();
  ocp2_.stateInequalityLagrangianPtr->clear();
  ocp2_.preJumpEqualityLagrangianPtr->clear();
  ocp2_.preJumpInequalityLagrangianPtr->clear();
  ocp2_.finalEqualityLagrangianPtr->clear();
  ocp2_.finalInequalityLagrangianPtr->clear();
  */

  mpcTimer1_.endTimer();

  /*
  int iter = mpcIter_;
  if (iterFlag)
  {
    iter++;
  }
  */

  // Set MPC Problem Settings
  size_t modelModeInt = modelModeIntQuery_;
  if (drlFlag_)
  {
    std::cout << "[MobileManipulatorInterface::setMPCProblem] DRL IS ON" << std::endl;
    modelModeInt = mpcProblemSettings_.modelMode;
  }

  mpcTimer2_.startTimer();
  std::cout << "[MobileManipulatorInterface::setMPCProblem] modelModeInt: " << modelModeInt << std::endl;
  bool isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  std::cout << "[MobileManipulatorInterface::setMPCProblem] isModeUpdated: " << isModeUpdated << std::endl;
  //printRobotModelInfo(robotModelInfo_);
  mpcTimer2_.endTimer();

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //// Optimal control problem
  if (modelModeInt == 0)
  {
    mpcTimer3_.startTimer();
    ocp_.costPtr->add("inputCost", quadraticInputCostPtr_mode0_);
    mpcTimer3_.endTimer();

    mpcTimer4_.startTimer();
    ocp_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode0_);
    mpcTimer4_.endTimer();

    mpcTimer5_.startTimer();
    ocp_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode0_);
    mpcTimer5_.endTimer();
    
    mpcTimer6_.startTimer();
    ocp_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode0_);
    mpcTimer6_.endTimer();

    mpcTimer7_.startTimer();
    if (activateSelfCollision_) 
    {
      //ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode0_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision_) 
    {
      //ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode0_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //ocp_.dynamicsPtr = dynamicsPtr_mode0_;
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileBaseDynamics" << std::endl;
    ocp_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                      "MobileBaseDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
    mpcTimer9_.endTimer();
  }

  else if (modelModeInt == 1)
  {
    mpcTimer3_.startTimer();
    ocp_.costPtr->add("inputCost", quadraticInputCostPtr_mode1_);
    mpcTimer3_.endTimer();

    mpcTimer4_.startTimer();
    ocp_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode1_);
    mpcTimer4_.endTimer();

    mpcTimer5_.startTimer();
    ocp_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode1_);
    mpcTimer5_.endTimer();
    
    mpcTimer6_.startTimer();
    ocp_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode1_);
    mpcTimer6_.endTimer();

    mpcTimer7_.startTimer();
    if (activateSelfCollision_) 
    {
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode1_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision_) 
    {
      //ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode1_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //ocp_.dynamicsPtr = dynamicsPtr_mode1_;
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: RobotArmDynamics" << std::endl;
    ocp_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                    "RobotArmDynamics", 
                                                    libraryFolder_, 
                                                    recompileLibraries_, 
                                                    printOutFlag_));
    mpcTimer9_.endTimer();
  }

  else if (modelModeInt == 2)
  {
    mpcTimer3_.startTimer();
    ocp_.costPtr->add("inputCost", quadraticInputCostPtr_mode2_);
    mpcTimer3_.endTimer();

    mpcTimer4_.startTimer();
    ocp_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode2_);
    mpcTimer4_.endTimer();

    mpcTimer5_.startTimer();
    ocp_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode2_);
    mpcTimer5_.endTimer();
    
    mpcTimer6_.startTimer();
    ocp_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode2_);
    mpcTimer6_.endTimer();

    mpcTimer7_.startTimer();
    if (activateSelfCollision_) 
    {
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode2_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision_) 
    {
      //ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //ocp_.dynamicsPtr = dynamicsPtr_mode2_;
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileManipulatorDynamics" << std::endl;
    ocp_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                              "MobileManipulatorDynamics", 
                                                              libraryFolder_, 
                                                              recompileLibraries_, 
                                                              printOutFlag_));
    mpcTimer9_.endTimer();
  }

  /*
  if (iter % 2 == 0)
  {
    std::cout << "[MobileManipulatorInterface::setMPCProblem] ocp1_" << std::endl;

    if (modelModeInt == 0)
    {
      mpcTimer3_.startTimer();
      ocp1_.costPtr->add("inputCost", quadraticInputCostPtr_mode0_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp1_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode0_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp1_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode0_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp1_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode0_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        //ocp1_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode0_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp1_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode0_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp1_.dynamicsPtr = dynamicsPtr_mode0_;
      ocp1_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        printOutFlag_));
      mpcTimer9_.endTimer();
    }

    else if (modelModeInt == 1)
    {
      mpcTimer3_.startTimer();
      ocp1_.costPtr->add("inputCost", quadraticInputCostPtr_mode1_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp1_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode1_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp1_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode1_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp1_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode1_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        ocp1_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode1_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp1_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode1_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp1_.dynamicsPtr = dynamicsPtr_mode1_;
      ocp1_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
      mpcTimer9_.endTimer();
    }

    else if (modelModeInt == 2)
    {
      mpcTimer3_.startTimer();
      ocp1_.costPtr->add("inputCost", quadraticInputCostPtr_mode2_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp1_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode2_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp1_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode2_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp1_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode2_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        ocp1_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode2_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp1_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp1_.dynamicsPtr = dynamicsPtr_mode2_;
      ocp1_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               printOutFlag_));
      mpcTimer9_.endTimer();
    }
  }
  else
  {
    std::cout << "[MobileManipulatorInterface::setMPCProblem] ocp2_" << std::endl;

    if (modelModeInt == 0)
    {
      mpcTimer3_.startTimer();
      ocp2_.costPtr->add("inputCost", quadraticInputCostPtr_mode0_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp2_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode0_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp2_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode0_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp2_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode0_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        //ocp2_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode0_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp2_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode0_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp2_.dynamicsPtr = dynamicsPtr_mode0_;
      ocp2_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        printOutFlag_));
      mpcTimer9_.endTimer();
    }

    else if (modelModeInt == 1)
    {
      mpcTimer3_.startTimer();
      ocp2_.costPtr->add("inputCost", quadraticInputCostPtr_mode1_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp2_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode1_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp2_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode1_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp2_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode1_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        ocp2_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode1_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp2_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode1_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp2_.dynamicsPtr = dynamicsPtr_mode1_;
      ocp2_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
      mpcTimer9_.endTimer();
    }

    else if (modelModeInt == 2)
    {
      mpcTimer3_.startTimer();
      ocp2_.costPtr->add("inputCost", quadraticInputCostPtr_mode2_);
      mpcTimer3_.endTimer();

      mpcTimer4_.startTimer();
      ocp2_.softConstraintPtr->add("jointLimits", jointLimitSoftConstraintPtr_mode2_);
      mpcTimer4_.endTimer();

      mpcTimer5_.startTimer();
      ocp2_.stateSoftConstraintPtr->add("endEffector", endEffectorIntermediateConstraintPtr_mode2_);
      mpcTimer5_.endTimer();
      
      mpcTimer6_.startTimer();
      ocp2_.finalSoftConstraintPtr->add("finalEndEffector", endEffectorFinalConstraintPtr_mode2_);
      mpcTimer6_.endTimer();

      mpcTimer7_.startTimer();
      if (activateSelfCollision_) 
      {
        ocp2_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode2_);
      }
      mpcTimer7_.endTimer();

      mpcTimer8_.startTimer();
      if (activateExtCollision_) 
      {
        //ocp2_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
      }
      mpcTimer8_.endTimer();

      mpcTimer9_.startTimer();
      //ocp2_.dynamicsPtr = dynamicsPtr_mode2_;
      ocp2_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               printOutFlag_));
      mpcTimer9_.endTimer();
    }    
  }
  */

  /*
  /// Cost
  ////ocp_.costPtr->add("inputCost", getQuadraticInputCost());
  //std::cout << "" << std::endl;

  /// Constraints
  // Joint limits constraint
  ////ocp_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint());
  //std::cout << "" << std::endl;

  // Mobile base or End-effector state constraint
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getEndEffectorConstraint" << std::endl;
  ////ocp_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint("endEffector"));
  //std::cout << "[MobileManipulatorInterface::setMPCProblem] AFTER getEndEffectorConstraint" << std::endl;
  ////ocp_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint("finalEndEffector"));
  //std::cout << "" << std::endl;

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getSelfCollisionConstraint" << std::endl;
  // Self-collision avoidance constraint
  mpcTimer7_.startTimer();
  if (activateSelfCollision_) 
  {
    if (robotModelInfo_.modelMode == ModelMode::ArmMotion || robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
    {
      if (iter % 2 == 0)
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] activateSelfCollision_ ocp1_" << std::endl;
        ocp1_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint("selfCollision"));
      }
      else
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] activateSelfCollision_ ocp2_" << std::endl;
        ocp2_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint("selfCollision"));
      }
    }
  }
  mpcTimer7_.endTimer();
  //std::cout << "" << std::endl;

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getExtCollisionConstraint" << std::endl;
  // External-collision avoidance constraint
  mpcTimer8_.startTimer();
  activateExtCollision_ = false;
  if (activateExtCollision_) 
  {
    initializePointsOnRobotPtr(pointsAndRadii_);
    
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
    
    if (iter % 2 == 0)
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] activateExtCollision_ ocp1_" << std::endl;
      ocp1_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint("extCollision"));
    }
    else
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] activateExtCollision_ ocp2_" << std::endl;
      ocp2_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint("extCollision"));
    }
  }
  mpcTimer8_.endTimer();
  //std::cout << "" << std::endl;

  // Dynamics
  mpcTimer9_.startTimer();
  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE Dynamics" << std::endl;
  switch (robotModelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Base" << std::endl;
      if (iter % 2 == 0)
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp1_" << std::endl;
        ocp1_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        printOutFlag_));
      }
      else
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp2_" << std::endl;
        ocp2_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        printOutFlag_));
      }
      break;
    }
    
    case ModelMode::ArmMotion:
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Robotics Arm" << std::endl;
      if (iter % 2 == 0)
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp1_" << std::endl;
        ocp1_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
      }
      else
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp2_" << std::endl;
        ocp2_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      printOutFlag_));
      }
      break;
    }
    
    case ModelMode::WholeBodyMotion: 
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Manipulator" << std::endl;
      if (iter % 2 == 0)
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp1_" << std::endl;
        ocp1_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               printOutFlag_));
      }
      else
      {
        std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr ocp2_" << std::endl;
        ocp2_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               printOutFlag_));
      }
      break;
    }

    default:
      throw std::invalid_argument("[MobileManipulatorInterface::setMPCProblem] ERROR: Invalid model mode!");
  }
  mpcTimer9_.endTimer();
  //std::cout << "" << std::endl;
  */

  /*
   * Pre-computation
   */
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Pre-computation" << std::endl;
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
    while(1);
    //ocp_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }
  //std::cout << "" << std::endl;

  // Rollout
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Rollout" << std::endl;
  mpcTimer10_.startTimer();
  rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
  /*
  if (!iterFlag)
  {
    if (iter % 2 == 0)
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] rolloutSettings_ ocp1_" << std::endl;
      rolloutPtr_.reset(new TimeTriggeredRollout(*ocp1_.dynamicsPtr, rolloutSettings_));
    }
    else
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] rolloutSettings_ ocp2_" << std::endl;
      rolloutPtr_.reset(new TimeTriggeredRollout(*ocp2_.dynamicsPtr, rolloutSettings_));
    }
  }
  */
  mpcTimer10_.endTimer();

  // Initialization
  mpcTimer11_.startTimer();
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Initialization" << std::endl;
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] modeInputDim: " << modeInputDim << std::endl;
  initializerPtr_.reset(new DefaultInitializer(modeInputDim));
  /*
  if (!iterFlag)
  {
    initializerPtr_.reset(new DefaultInitializer(modeInputDim));
  }
  */
  mpcTimer11_.endTimer();

  mpcTimer0_.endTimer();

  std::cout << "\n### MPC_ROS Benchmarking mpcTimer0_: TOTAL";
  std::cout << "\n###   Maximum : " << mpcTimer0_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer0_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer0_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer1_: clear";
  std::cout << "\n###   Maximum : " << mpcTimer1_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer1_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer1_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer2_: updateModelMode";
  std::cout << "\n###   Maximum : " << mpcTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer2_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_: getQuadraticInputCost";
  std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer4_: getJointLimitSoftConstraint";
  std::cout << "\n###   Maximum : " << mpcTimer4_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer4_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer4_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer5_: INTER getEndEffectorConstraint";
  std::cout << "\n###   Maximum : " << mpcTimer5_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer5_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer5_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer6_: FINAL getEndEffectorConstraint";
  std::cout << "\n###   Maximum : " << mpcTimer6_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer6_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer6_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer7_: getSelfCollisionConstraint";
  std::cout << "\n###   Maximum : " << mpcTimer7_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer7_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer7_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer8_: getExtCollisionConstraint";
  std::cout << "\n###   Maximum : " << mpcTimer8_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer8_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer8_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer9_: dynamicsPtr";
  std::cout << "\n###   Maximum : " << mpcTimer9_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer9_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer9_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer10_: rolloutPtr_";
  std::cout << "\n###   Maximum : " << mpcTimer10_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer10_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer10_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer11_: initializerPtr_";
  std::cout << "\n###   Maximum : " << mpcTimer11_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer11_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer11_.getLastIntervalInMilliseconds() << "[ms]." << std::endl << std::endl;

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorInterface::setMPCProblem] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchNodes(ros::NodeHandle& nodeHandle)
{
  std::string model_mode_msg_name = "mobile_manipulator_model_mode";
  std::string target_msg_name = "mobile_manipulator_mpc_target";

  double dt = 0.1;

  if (emuPtr_)
  {
    std::string oct_msg_name = "octomap_scan";
    // Octomap Subscriber
    emuPtr_->setNodeHandle(nodeHandle_);
    emuPtr_->updateOct(oct_msg_name);
  }

  // NUA NOTE: Visualize somewhere else!
  /*
  if (pointsOnRobotPtr_mode0_)
  {
    pointsOnRobotPtr_mode0_->setNodeHandle(nodeHandle_);
    pointsOnRobotPtr_mode0_->publishPointsOnRobotVisu(dt);
  }

  if (pointsOnRobotPtr_mode0_)
  {
    pointsOnRobotPtr_mode0_->setNodeHandle(nodeHandle_);
    pointsOnRobotPtr_mode0_->publishPointsOnRobotVisu(dt);
  }

  if (pointsOnRobotPtr_mode0_)
  {
    pointsOnRobotPtr_mode0_->setNodeHandle(nodeHandle_);
    pointsOnRobotPtr_mode0_->publishPointsOnRobotVisu(dt);
  }
  */

  if (drlFlag_)
  {
    setActionDRLService_ = nodeHandle_.advertiseService("set_action_drl", &MobileManipulatorInterface::setActionDRLSrv, this);
  }
  else
  {
    // Subscribe Model Mode
    auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
    {
      std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] START" << std::endl;

      // Shutdown MRT
      std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] mrtShutDownFlag true"  << std::endl;
      mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

      modelModeIntQuery_ = msg->data;
      std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] modelModeIntQuery_: " << modelModeIntQuery_ << std::endl;

      //mpcTimer3_.startTimer();
      //setMPCProblem(true);
      //mpcTimer3_.endTimer();

      //mpcProblemReadyFlag_ = true;
      //std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] mrtShutDownFlag true"  << std::endl;
      //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

      //std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
      //std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
      //std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
      //std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

      std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] END" << std::endl;
      std::cout << "" << std::endl;
    };
    modelModeSubscriber_ = nodeHandle_.subscribe<std_msgs::UInt8>(model_mode_msg_name, 1, modelModeCallback);
  }

  //modelModeSubscriber_ = nodeHandle_.subscribe(model_mode_msg_name, 5, &MobileManipulatorInterface::modelModeCallback, this);

  // TargetTrajectories
  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) 
  {
    //std::cout << "[MobileManipulatorInterface::targetTrajectoriesCallback] START" << std::endl;
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

    if (currentTarget_.size() == targetTrajectories.stateTrajectory[0].size())
    {
      currentTarget_ = targetTrajectories.stateTrajectory[0]; 
    }
    else
    {
      std::cout << "[MobileManipulatorInterface::targetTrajectoriesCallback] ERROR: Size mismatch!" << std::endl;
    }

    targetReadyFlag_ = true;
    //std::cout << "[MobileManipulatorInterface::targetTrajectoriesCallback] END" << std::endl;
  };
  targetTrajectoriesSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(target_msg_name, 5, targetTrajectoriesCallback);

  //spin();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::getEEPose(vector_t& eePose)
{
  //std::cout << "[MobileManipulatorInterface::getEEPose] START" << std::endl;

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
    ROS_INFO("[MobileManipulatorInterface::getEEPose] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  eePose.resize(7);
  eePose(0) = tf_ee_wrt_world.getOrigin().x();
  eePose(1) = tf_ee_wrt_world.getOrigin().y();
  eePose(2) = tf_ee_wrt_world.getOrigin().z();
  eePose(3) = tf_ee_wrt_world.getRotation().x();
  eePose(4) = tf_ee_wrt_world.getRotation().y();
  eePose(5) = tf_ee_wrt_world.getRotation().z();
  eePose(6) = tf_ee_wrt_world.getRotation().w();

  //std::cout << "[MobileManipulatorInterface::getEEPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void MobileManipulatorInterface::getGraspPose(vector_t& targetPose)
{
  //std::cout << "[MobileManipulatorInterface::getGraspPose] START" << std::endl;

  tf::StampedTransform tf_grasp_wrt_world;
  try
  {
    tfListener_.waitForTransform(worldFrameName_, graspFrameName_, ros::Time::now(), ros::Duration(1.0));
    tfListener_.lookupTransform(worldFrameName_, graspFrameName_, ros::Time(0), tf_grasp_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MobileManipulatorInterface::getGraspPose] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  targetPose.resize(7);
  targetPose(0) = tf_grasp_wrt_world.getOrigin().x();
  targetPose(1) = tf_grasp_wrt_world.getOrigin().y();
  targetPose(2) = tf_grasp_wrt_world.getOrigin().z();
  targetPose(3) = tf_grasp_wrt_world.getRotation().x();
  targetPose(4) = tf_grasp_wrt_world.getRotation().y();
  targetPose(5) = tf_grasp_wrt_world.getRotation().z();
  targetPose(6) = tf_grasp_wrt_world.getRotation().w();
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setActionDRLSrv(ocs2_msgs::setActionDRL::Request &req, 
                                                 ocs2_msgs::setActionDRL::Response &res)
{
  std::cout << "[MobileManipulatorInterface::setActionDRLSrv] START" << std::endl;
  drlAction_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  res.success = true;

  mapDRLAction(drlAction_);

  mpcTimer3_.startTimer();
  setMPCProblem(true);
  mpcTimer3_.endTimer();

  //mpcProblemReadyFlag_ = true;
  std::cout << "[MobileManipulatorInterface::setActionDRLSrv] mrtShutDownFlag true"  << std::endl;
  mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

  std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

  std::cout << "[MobileManipulatorInterface::setActionDRLSrv] END" << std::endl;
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::setActionDRLSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMPC()
{
  //std::cout << "[MobileManipulatorInterface::runMPC] START" << std::endl;

  bool mpcPrintOutFlag = true;

  RobotModelInfo robotModelInfo;

  mpcIter_ = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "[MobileManipulatorInterface::runMPC] mpcIter_: " << mpcIter_ << std::endl;
    std::cout << "[MobileManipulatorInterface::runMPC] mrtIter_: " << mrtIter_ << std::endl;

    /*
    if (mpcIter_ > 0)
    {
      std::cout << "[MobileManipulatorInterface::runMPC] DEBUG INF " << std::endl;
      while(1);
    }
    */

    // Wait for sync mpc and mrt
    while(mpcIter_ != mrtIter_);

    if (mpcPrintOutFlag)
    {
      std::cout << "=====================================================" << std::endl;
      std::cout << "=====================================================" << std::endl;
      std::cout << "[MobileManipulatorInterface::runMPC] START ITERATION: " << mpcIter_ << std::endl;
    }

    //mpcTimer1_.startTimer();

    // Robot interface
    //std::cout << "[MobileManipulatorInterface::runMPC] START launchNodes" << std::endl;
    //mpcTimer2_.startTimer();
    //launchNodes(nodeHandle_);
    //mpcTimer2_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMPC] END launchNodes" << std::endl;

    //ros::Duration(3.0).sleep();

    while(!mrtExitFlag_){spinOnce();}
    setenv("mpcShutDownFlag", "false", 1);

    std::string mrtExitEnvStatus = getenv("mrtExitFlag");
    //std::cout << "[OCS2_MRT_Loop::mrtLoop] mrtShutDownFlag_: " << mrtShutDownFlag_ << std::endl;

    while(mrtExitEnvStatus == "false")
    {
      std::cout << "[OCS2_MRT_Loop::mrtLoop] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
      mrtExitEnvStatus = getenv("mrtExitFlag");
    }

    //mpcTimer3_.startTimer();
    setMPCProblem();
    //mpcTimer3_.endTimer();
    //printRobotModelInfo(robotModelInfo_);

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE Setting MPC Parameters " << std::endl;
    //mpcTimer4_.startTimer();
    robotModelInfo = robotModelInfo_;
    //OptimalControlProblem ocp;
    //ocp.swap(ocp_);
    /*
    if (mpcIter_ % 2 == 0)
    {
      std::cout << "[MobileManipulatorInterface::runMPC] ocp1_" << std::endl;
      ocp.swap(ocp1_);
    }
    else
    {
      std::cout << "[MobileManipulatorInterface::runMPC] ocp2_" << std::endl;
      ocp.swap(ocp2_);
    }
    */
    //rolloutPtr_.reset(new TimeTriggeredRollout(*ocp.dynamicsPtr, rolloutSettings_));
    //initializerPtr_.reset(new DefaultInitializer(robotModelInfo.modeInputDim));
    mpcProblemReadyFlag_ = true;
    //mpcTimer4_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMPC] AFTER Setting MPC Parameters " << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr" << std::endl;
    // ROS ReferenceManager
    //mpcTimer5_.startTimer();
    //rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(robotModelName_, referenceManagerPtr_));
    //mpcTimer5_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr subscribe" << std::endl;
    //mpcTimer6_.startTimer();
    //rosReferenceManagerPtr_->subscribe(nodeHandle_);
    //mpcTimer6_.endTimer();

    // MPC
    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc" << std::endl;
    //mpcTimer7_.startTimer();
    ocs2::GaussNewtonDDP_MPC mpc(mpcSettings_, 
                                 ddpSettings_, 
                                 *rolloutPtr_, 
                                 ocp_, 
                                 *initializerPtr_);
    //mpcTimer7_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc setReferenceManager" << std::endl;
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

    // Launch MPC ROS node
    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode" << std::endl;
    //mpcTimer8_.startTimer();
    MPC_ROS_Interface mpcNode(mpc, robotModelName_);
    mpcNode.setModelModeInt(getModelModeInt(robotModelInfo));
    //mpcTimer8_.endTimer();

    //mpcTimer1_.endTimer();

    if (false)
    {
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
    }

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    mpcLaunchReadyFlag_ = true;
    //std::cout << "[MobileManipulatorInterface::runMPC] AFTER mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;

    std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode launchNodes" << std::endl;
    //mpcExitFlag_ = false;
    printRobotModelInfo(robotModelInfo);
    mpcNode.launchNodes(nodeHandle_);
    //mpcExitFlag_ = true;
    mpcLaunchReadyFlag_ = false;
    std::cout << "[MobileManipulatorInterface::runMPC] AFTER mpc mpcNode launchNodes" << std::endl;
 
    if (mpcPrintOutFlag)
    {
      std::cout << "[MobileManipulatorInterface::runMPC] END ITERATION: " << mpcIter_ << std::endl;
      std::cout << "=====================================================" << std::endl;
      std::cout << "=====================================================" << std::endl;
    }

    //std::cout << "[MobileManipulatorInterface::runMPC] DEBUG INF" << std::endl;
    //while(1);

    mpcIter_++;
  }

  //std::cout << "[MobileManipulatorInterface::runMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMRT()
{
  //std::cout << "[MobileManipulatorInterface::runMRT] START" << std::endl;

  bool mrtPrintOutFlag = true;

  RobotModelInfo robotModelInfo;
  //OptimalControlProblem ocp;

  getEEPose(currentTarget_);

  mrtIter_ = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "[MobileManipulatorInterface::runMRT] mpcIter_: " << mpcIter_ << std::endl;
    std::cout << "[MobileManipulatorInterface::runMRT] mrtIter_: " << mrtIter_ << std::endl;

    // Wait for sync mpc and mrt
    while(mpcIter_ != mrtIter_){spinOnce();}
    
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    //while(!mpcLaunchReadyFlag_){spinOnce();}
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    //mpcLaunchReadyFlag_ = false;

    if (mrtPrintOutFlag)
    {
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      std::cout << "[MobileManipulatorInterface::runMRT] START ITERATION: " << mrtIter_ << std::endl;
    }

    //mrtTimer1_.startTimer();

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE setMPCProblem" << std::endl;
    //mrtTimer2_.startTimer();
    while(!mpcProblemReadyFlag_){spinOnce();}
    mpcProblemReadyFlag_ = false;
    //setMPCProblem();
    robotModelInfo = robotModelInfo_;
    printRobotModelInfo(robotModelInfo);
    //mrtTimer2_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER setMPCProblem" << std::endl;

    // MRT
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE mrt" << std::endl;
    //mrtTimer3_.startTimer();
    MRT_ROS_Interface mrt(robotModelInfo, robotModelName_);
    //mrtTimer3_.endTimer();

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE initRollout" << std::endl;
    //mrtTimer4_.startTimer();
    mrt.initRollout(&*rolloutPtr_);
    //mrtTimer4_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER initRollout" << std::endl;
    
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE launchNodes" << std::endl;
    //mrtTimer5_.startTimer();
    mrt.launchNodes(nodeHandle_);
    //mrtTimer5_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER launchNodes" << std::endl;

    // Visualization
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE ocs2_mm_visu" << std::endl;
    //std::shared_ptr<ocs2::mobile_manipulator::OCS2_Mobile_Manipulator_Visualization> ocs2_mm_visu(new ocs2::mobile_manipulator::OCS2_Mobile_Manipulator_Visualization(nodeHandle_,
    //                                                                                                                                                                  *pinocchioInterfacePtr_,
    //                                                                                                                                                                 urdfFile_,
    //                                                                                                                                                                  taskFile_));
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER ocs2_mm_visu" << std::endl;

    // MRT loop
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE mrt_loop" << std::endl;
    //mrtTimer6_.startTimer();
    MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle_, 
                                 mrt, 
                                 worldFrameName_,
                                 "mobile_manipulator_mpc_target",
                                 baseStateMsg_,
                                 armStateMsg_,
                                 baseControlMsg_,
                                 armControlMsg_,
                                 err_threshold_pos_,
                                 err_threshold_ori_,
                                 mpcSettings_.mrtDesiredFrequency_, 
                                 mpcSettings_.mpcDesiredFrequency_);
    //mrtTimer6_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER mrt_loop" << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE subscribeObservers" << std::endl;
    //mrt_loop.subscribeObservers({ocs2_mm_visu});
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER subscribeObservers" << std::endl;

    // initial command
    //mrtTimer7_.startTimer();
    vector_t currentTarget;
    spinOnce();
    currentTarget = currentTarget_;
    //mrtTimer7_.endTimer();

    // Run mrt_loop
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE run" << std::endl;
    //mrtTimer1_.endTimer();

    if (false)
    {
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
    }
    
    setenv("mrtExitFlag", "false", 1);
    mrtExitFlag_ = false;

    printRobotModelInfo(robotModelInfo);

    mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);

    mrt_loop.run(currentTarget);
    mrtExitFlag_ = true;
    setenv("mpcShutDownFlag", "true", 1);

    if (mrtPrintOutFlag)
    {
      std::cout << "[MobileManipulatorInterface::runMRT] AFTER currentTarget size: " << currentTarget.size() << std::endl;
      for (size_t i = 0; i < currentTarget.size(); i++)
      {
        std::cout << i << " -> " << currentTarget[i] << std::endl;
      }
      std::cout << "------------" << std::endl;
    }

    if (mrtPrintOutFlag)
    {
      std::cout << "[MobileManipulatorInterface::runMRT] END ITERATION: " << mrtIter_ << std::endl;
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    }

    //std::cout << "[MobileManipulatorInterface::runMRT] DEBUG INF" << std::endl;
    //while(1);

    mrtIter_++;
  }

  //std::cout << "[MobileManipulatorInterface::runMRT] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mpcCallback(const ros::TimerEvent& event)
{
  //std::cout << "[MobileManipulatorInterface::mpcCallback] START" << std::endl;

  runMPC();

  //std::cout << "[MobileManipulatorInterface::mpcCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mrtCallback(const ros::TimerEvent& event)
{
  //std::cout << "[MobileManipulatorInterface::mrtCallback] START" << std::endl;
  
  runMRT();
  
  //std::cout << "[MobileManipulatorInterface::mrtCallback] END" << std::endl;

  //std::cout << "[MobileManipulatorInterface::mrtCallback] DEBUG INF" << std::endl;
  //while(1);
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
  scalar_t muPosition;
  scalar_t muOrientation;
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

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapDRLAction(int action)
{
  std::cout << "[MobileManipulatorInterface::mapDRLAction] START" << std::endl;

  // MPC Settings Variables -------
  int n_mode = 3;
  int n_bool_var = mpcProblemSettings_.binarySettingNames.size();
  // ------------------------------

  int n_settings_per_mode = pow(2, n_bool_var);

  mpcProblemSettings_.modelMode = action / n_settings_per_mode;
  
  int dec = action % n_settings_per_mode;
  while (dec >= 1)
  {
    mpcProblemSettings_.binarySettingValues.push_back(dec % 2);
    dec = dec / 2;
  }
  mpcProblemSettings_.binarySettingValues.push_back(dec);

  std::string tmp_name;
  bool tmp_val;
  for (size_t i = 0; i < mpcProblemSettings_.binarySettingNames.size(); i++)
  {
    tmp_name = mpcProblemSettings_.binarySettingNames[i];
    tmp_val = mpcProblemSettings_.binarySettingValues[i];
    std::cout << "[MobileManipulatorInterface::mapDRLAction] " << tmp_name << ": " << tmp_val << std::endl;
  }

  std::cout << "[MobileManipulatorInterface::mapDRLAction] END" << std::endl;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
