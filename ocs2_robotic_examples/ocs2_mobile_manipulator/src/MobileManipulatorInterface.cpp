// LAST UPDATE: 2023.08.24
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
//#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

namespace ocs2 {
namespace mobile_manipulator {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile, 
                                                       const std::string& libraryFolder,
                                                       const std::string& urdfFile,
                                                       int initModelModeInt)
  : taskFile_(taskFile), 
    libraryFolder_(libraryFolder), 
    urdfFile_(urdfFile),
    initModelModeInt_(initModelModeInt), 
    modelModeIntQuery_(initModelModeInt)
{
  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

  /// NUA NOTE: DEPRICATED! NEED REVIEW AND UPDATE!
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;
  while(1);

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
                                                       int initModelModeInt)
  : nodeHandle_(nodeHandle), 
    taskFile_(taskFile), 
    libraryFolder_(libraryFolder), 
    urdfFile_(urdfFile),
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
  std::string robotName, baseFrame, armBaseFrame, eeFrame, collisionConstraintPoints, collisionCheckPoints;
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, worldFrameName_, "model_information.worldFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg_, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg_, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg_, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg_, "model_information.armControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceMsg_, "model_information.occupancyDistanceMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, octomapMsg_, "model_information.octomapMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, modelModeMsgName_, "model_information.modelModeMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, targetMsgName_, "model_information.targetMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionConstraintPoints, "model_information.collisionConstraintPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionCheckPoints, "model_information.collisionCheckPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, logSavePathRel_, "model_information.logSavePathRel", printOutFlag_);

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
    std::cout << "#### model_information.occupancyDistanceMsg: " << occupancyDistanceMsg_ << std::endl;
    std::cout << "#### model_information.octomapMsg: " << octomapMsg_ << std::endl;
    std::cout << "#### model_information.modelModeMsg: " << modelModeMsgName_ << std::endl;
    std::cout << "#### model_information.targetMsg: " << targetMsgName_ << std::endl;
    std::cout << "#### model_information.logSavePathRel: " << logSavePathRel_ << std::endl;
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
    loadData::loadPtreeValue(pt, drlActionType_, "model_settings.drlActionType", printOutFlag_);
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

  // Set PointsOnRobot
  initializePointsOnRobotPtr(collisionConstraintPoints);

  // Set ExtMapUtility
  emuPtr_.reset(new ExtMapUtility());
  emuPtr_->setWorldFrameName(worldFrameName_);
  emuPtr_->setPubOccDistArrayVisu(occupancyDistanceMsg_);
  emuPtr_->setNodeHandle(nodeHandle_);
  emuPtr_->subscribeOctMsg(octomapMsg_);

  // Create costs/constraints
  size_t modelModeInt;
  bool isModeUpdated;
  std::string modelName;

  // Mode 0
  modelModeInt = 0;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  modelName = getModelModeString(robotModelInfo_);

  quadraticInputCostPtr_mode0_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode0_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode0_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode0_ = getEndEffectorConstraint("finalEndEffector");
  //selfCollisionConstraintPtr_mode0_ = getSelfCollisionConstraint("selfCollision");
  //extCollisionConstraintPtr_mode0_= getExtCollisionConstraint("extCollision");

  dynamicsPtr_mode0_.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                  "MobileBaseDynamics", 
                                                  libraryFolder_, 
                                                  recompileLibraries_, 
                                                  printOutFlag_));

  // Mode 1
  modelModeInt = 1;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  modelName = getModelModeString(robotModelInfo_);

  quadraticInputCostPtr_mode1_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode1_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode1_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode1_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode1_ = getSelfCollisionConstraint(modelName);
  //extCollisionConstraintPtr_mode1_= getExtCollisionConstraint("extCollision");
  
  dynamicsPtr_mode1_.reset(new RobotArmDynamics(robotModelInfo_, 
                                                "RobotArmDynamics", 
                                                libraryFolder_, 
                                                recompileLibraries_, 
                                                printOutFlag_));
  
  // Mode 2
  modelModeInt = 2;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  modelName = getModelModeString(robotModelInfo_);

  quadraticInputCostPtr_mode2_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode2_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode2_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode2_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode2_ = getSelfCollisionConstraint(modelName);
  extCollisionConstraintPtr_mode2_ = getExtCollisionConstraint("extCollision");

  dynamicsPtr_mode2_.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                         "MobileManipulatorDynamics", 
                                                         libraryFolder_, 
                                                         recompileLibraries_, 
                                                         printOutFlag_));
  
  // Visualization
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] BEFORE mobileManipulatorVisu_" << std::endl;
  mobileManipulatorVisu_.reset(new ocs2::mobile_manipulator::MobileManipulatorVisualization(nodeHandle_, 
                                                                                            *pinocchioInterfacePtr_,
                                                                                            worldFrameName_,
                                                                                            baseFrame,
                                                                                            urdfFile,
                                                                                            armStateMsg_,
                                                                                            robotModelInfo_,
                                                                                            activateSelfCollision_,
                                                                                            activateExtCollision_,
                                                                                            removeJointNames,
                                                                                            collisionObjectPairs_,
                                                                                            collisionLinkPairs_,
                                                                                            pointsOnRobotPtr_,
                                                                                            emuPtr_,
                                                                                            maxDistance_));
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] AFTER mobileManipulatorVisu_" << std::endl;

  launchNodes(nodeHandle_);

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface(6)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializePointsOnRobotPtr(std::string& collisionPointsName) 
{
  std::cout << "[MobileManipulatorInterface::initializePointsOnRobotPtr] START" << std::endl;
  //PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>();

  std::cout << "[MobileManipulatorInterface::initializePointsOnRobotPtr] collisionPointsName: " << collisionPointsName << std::endl;

  // Get points on robot parameters
  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nodeHandle_.hasParam(collisionPointsName)) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle_.getParam(collisionPointsName, collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorNode::main] collision_points parameter is not of type array.");
    }
    
    //// NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    std::cout << "[MobileManipulatorNode::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("[MobileManipulatorNode::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorNode::main] ERROR: collision_points is not defined!" << std::endl;
  }

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

  std::cout << "[MobileManipulatorInterface::initializePointsOnRobotPtr] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPCProblem()
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

  mpcTimer1_.endTimer();

  // Set MPC Problem Settings
  size_t modelModeInt = modelModeIntQuery_;
  
  /*
  if (drlFlag_)
  {
    std::cout << "[MobileManipulatorInterface::setMPCProblem] DRL IS ON" << std::endl;
    modelModeInt = mpcProblemSettings_.modelMode;
  }
  */

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
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileBaseDynamics" << std::endl;

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
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileBaseDynamics" << std::endl;
    ocp_.dynamicsPtr = dynamicsPtr_mode0_;
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
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: RobotArmDynamics" << std::endl;
    ocp_.dynamicsPtr = dynamicsPtr_mode1_;
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
      std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE extCollisionConstraintPtr_mode2_" << std::endl;
      ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    std::cout << "[MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileManipulatorDynamics" << std::endl;
    ocp_.dynamicsPtr = dynamicsPtr_mode2_;
    mpcTimer9_.endTimer();
  }

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
  mpcTimer10_.endTimer();

  // Initialization
  mpcTimer11_.startTimer();
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Initialization" << std::endl;
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] modeInputDim: " << modeInputDim << std::endl;
  initializerPtr_.reset(new DefaultInitializer(modeInputDim));
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
  if (drlFlag_)
  {
    if (drlActionType_ == 0)
    {
      std::cout << "[MobileManipulatorInterface::launchNodes] DISCRETE ACTION" << std::endl;
      setActionDRLService_ = nodeHandle_.advertiseService("set_action_drl", &MobileManipulatorInterface::setDiscreteActionDRLSrv, this);
    }
    else
    {
      std::cout << "[MobileManipulatorInterface::launchNodes] CONTINUOUS ACTION" << std::endl;
      setActionDRLService_ = nodeHandle_.advertiseService("set_action_drl", &MobileManipulatorInterface::setContinuousActionDRLSrv, this);
    }

    setTargetDRLClient_ = nodeHandle_.serviceClient<ocs2_msgs::setTask>("/set_target_drl");
    setMRTReadyClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>("/set_mrt_ready");
    setMPCActionResultClient_ = nodeHandle_.serviceClient<ocs2_msgs::setMPCActionResult>("/set_mpc_action_result");
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
      //setMPCProblem();
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
    modelModeSubscriber_ = nodeHandle_.subscribe<std_msgs::UInt8>(modelModeMsgName_, 1, modelModeCallback);
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

    /*
    std::cout << "[MobileManipulatorInterface::targetTrajectoriesCallback] currentTarget_.size(): " << currentTarget_.size() << std::endl;
    for (size_t i = 0; i < currentTarget_.size(); i++)
    {
      std::cout << i << " -> " << currentTarget_[i] << std::endl;
    }
    */
    
    //targetReadyFlag_ = true;
    //std::cout << "[MobileManipulatorInterface::targetTrajectoriesCallback] END" << std::endl;
  };
  targetTrajectoriesSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(targetMsgName_, 5, targetTrajectoriesCallback);

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
bool MobileManipulatorInterface::setDiscreteActionDRLSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                                         ocs2_msgs::setDiscreteActionDRL::Response &res)
{
  std::cout << "[MobileManipulatorInterface::setDiscreteActionDRLSrv] START" << std::endl;

  drlActionDiscrete_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  res.success = true;

  mapDiscreteActionDRL(drlActionDiscrete_);

  //mpcTimer3_.startTimer();
  //setMPCProblem();
  //mpcTimer3_.endTimer();

  //mpcProblemReadyFlag_ = true;
  //std::cout << "[MobileManipulatorInterface::setDiscreteActionDRLSrv] mrtShutDownFlag true"  << std::endl;
  //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

  //std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  //std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  //std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  //std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

  std::cout << "[MobileManipulatorInterface::setDiscreteActionDRLSrv] DEBUG INF" << std::endl;
  while(1);

  std::cout << "[MobileManipulatorInterface::setDiscreteActionDRLSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setContinuousActionDRLSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                                           ocs2_msgs::setContinuousActionDRL::Response &res)
{
  std::cout << "[MobileManipulatorInterface::setContinuousActionDRLSrv] START" << std::endl;
  drlActionContinuous_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  drlActionLastStepFlag_ = req.last_step_flag;
  drlActionLastStepDistanceThreshold_ = req.last_step_distance_threshold;
  res.success = true;

  mapContinuousActionDRL(drlActionContinuous_);

  //mpcTimer3_.startTimer();
  //setMPCProblem();
  //mpcTimer3_.endTimer();

  //mpcProblemReadyFlag_ = true;
  //std::cout << "[MobileManipulatorInterface::setContinuousActionDRLSrv] mrtShutDownFlag true"  << std::endl;
  //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

  /*
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  */

  targetReceivedFlag_ = true;
  //std::cout << "[MobileManipulatorInterface::setContinuousActionDRLSrv] END" << std::endl;
  //std::cout << "" << std::endl;
  
  //std::cout << "[MobileManipulatorInterface::setContinuousActionDRLSrv] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorInterface::setContinuousActionDRLSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMPC()
{
  //std::cout << "[MobileManipulatorInterface::runMPC] START" << std::endl;

  bool mpcPrintOutFlag = false;

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
    while(mpcIter_ != mrtIter_){spinOnce();}

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
    //std::cout << "[MobileManipulatorInterface::runMPC] mrtExitEnvStatus: " << mrtExitEnvStatus << std::endl;

    while(mrtExitEnvStatus == "false")
    {
      //std::cout << "[MobileManipulatorInterface::runMPC] CMOOOOOOOOOOOOOOOOOON: " << std::endl;
      mrtExitEnvStatus = getenv("mrtExitFlag");
    }
    //std::cout << "[MobileManipulatorInterface::runMPC] MRT EXIT HAPPENED" << std::endl;

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

    //std::cout << "[MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode launchNodes" << std::endl;
    //mpcExitFlag_ = false;
    //printRobotModelInfo(robotModelInfo);
    mpcNode.launchNodes(nodeHandle_);
    //mpcExitFlag_ = true;
    mpcLaunchReadyFlag_ = false;
    //std::cout << "[MobileManipulatorInterface::runMPC] AFTER mpc mpcNode launchNodes" << std::endl;
 
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

    while(!mpcProblemReadyFlag_){spinOnce();}
    mpcProblemReadyFlag_ = false;

    if (mrtPrintOutFlag)
    {
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
      std::cout << "[MobileManipulatorInterface::runMRT] START ITERATION: " << mrtIter_ << std::endl;
    }

    //mrtTimer1_.startTimer();

    std::cout << "[MobileManipulatorInterface::runMRT] BEFORE setMPCProblem" << std::endl;
    //mrtTimer2_.startTimer();
    //setMPCProblem();
    //OptimalControlProblem ocp;
    //ocp.swap(ocp_);
    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE TimeTriggeredRollout" << std::endl;
    //mrtRolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
    robotModelInfo = robotModelInfo_;
    printRobotModelInfo(robotModelInfo); 
    //mrtTimer2_.endTimer();
    std::cout << "[MobileManipulatorInterface::runMRT] AFTER setMPCProblem" << std::endl;

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
                                 err_threshold_ori_yaw_,
                                 err_threshold_ori_quat_,
                                 mpcSettings_.mrtDesiredFrequency_, 
                                 mpcSettings_.mpcDesiredFrequency_,
                                 drlFlag_,
                                 logSavePathRel_);

    //mrtTimer6_.endTimer();
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER mrt_loop" << std::endl;

    //std::cout << "[MobileManipulatorInterface::runMRT] BEFORE subscribeObservers" << std::endl;
    mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo));
    mrt_loop.subscribeObservers({mobileManipulatorVisu_});
    //std::cout << "[MobileManipulatorInterface::runMRT] AFTER subscribeObservers" << std::endl;

    if (drlFlag_)
    {
      std::cout << "[MobileManipulatorInterface::runMRT] Waiting for targetReceivedFlag_..." << std::endl;
      //setMRTReady();
      while(!targetReceivedFlag_)
      {
        setMRTReady();
        spinOnce();
      }
      //mrt_loop.setDRLFlag(drlFlag_);
      mrt_loop.setTargetReceivedFlag(targetReceivedFlag_);
      mrt_loop.setTaskMode(taskMode_);
      mrt_loop.setDRLActionTimeHorizon(drlActionTimeHorizon_);
      mrt_loop.setDRLActionLastStepFlag(drlActionLastStepFlag_);
      mrt_loop.setDRLActionLastStepDistanceThreshold(drlActionLastStepDistanceThreshold_);
      targetReceivedFlag_ = false;
    }

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
    mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);

    mrt_loop.run(currentTarget);
    
    mrtExitFlag_ = true;
    setenv("mpcShutDownFlag", "true", 1);

    if (drlFlag_)
    {
      setMPCActionResult(mrt_loop.getDRLActionResult());
    }

    /*
    if (false)
    {
      std::cout << "[MobileManipulatorInterface::runMRT] AFTER currentTarget size: " << currentTarget.size() << std::endl;
      for (size_t i = 0; i < currentTarget.size(); i++)
      {
        std::cout << i << " -> " << currentTarget[i] << std::endl;
      }
      std::cout << "------------" << std::endl;
    }
    */

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
  //std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  //std::vector<std::pair<std::string, std::string>> collisionLinkPairs;

  //std::cerr << "\n #### SelfCollision Settings: ";
  //std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, "selfCollision.mu", printOutFlag_);
  loadData::loadPtreeValue(pt, delta, "selfCollision.delta", printOutFlag_);
  loadData::loadPtreeValue(pt, minimumDistance, "selfCollision.minimumDistance", printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, "selfCollision.collisionObjectPairs", collisionObjectPairs_, printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, "selfCollision.collisionLinkPairs", collisionLinkPairs_, printOutFlag_);
  //std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr_, collisionLinkPairs_, collisionObjectPairs_);
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
                                                                                    prefix + "_self_collision", 
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
  //scalar_t maxDistance = 10;
  std::cerr << "\n #### ExtCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, maxDistance_, prefix + ".maxDistance", true);
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
                                                                                              maxDistance_,
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
                                                                                  maxDistance_,
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
bool MobileManipulatorInterface::setTargetDRL(double x, double y, double z, double roll, double pitch, double yaw)
{
  std::cout << "[MobileManipulatorInterface::setTargetDRL] START" << std::endl;

  bool success = false;
  ocs2_msgs::setTask srv;
  srv.request.targetPose.position.x = x;
  srv.request.targetPose.position.y = y;
  srv.request.targetPose.position.z = z;

  // Convert RPY to Quaternion
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  srv.request.targetPose.orientation.x = quat.x();
  srv.request.targetPose.orientation.y = quat.y();
  srv.request.targetPose.orientation.z = quat.z();
  srv.request.targetPose.orientation.w = quat.w();

  if (setTargetDRLClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setTargetDRL] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[MobileManipulatorInterface::setTargetDRL] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapDiscreteActionDRL(int action)
{
  std::cout << "[MobileManipulatorInterface::mapDiscreteActionDRL] START" << std::endl;

  /// NUA TODO: NEEDS DEBUGGING!

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
    std::cout << "[MobileManipulatorInterface::mapDiscreteActionDRL] " << tmp_name << ": " << tmp_val << std::endl;
  }

  std::cout << "[MobileManipulatorInterface::mapDiscreteActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapContinuousActionDRL(std::vector<double>& action)
{
  std::cout << "[MobileManipulatorInterface::mapContinuousActionDRL] START" << std::endl;

  double modelModeProb = action[0];
  double constraintProb = action[1];
  double target_x = action[2];
  double target_y = action[3];
  double target_z = action[4];
  double target_roll = action[5];
  double target_pitch = action[6];
  double target_yaw = action[7];
  
  // Set Model Mode
  if (modelModeProb <= 0.3)
  {
    target_roll = 0.0;
    target_pitch = 0.0;
    modelModeIntQuery_ = 0;
  }
  else if (modelModeProb > 0.6)
  {
    modelModeIntQuery_ = 2;
  }
  else
  {
    modelModeIntQuery_ = 1;
  }

  // Set Constraint Flags
  if (constraintProb <= 0.5)
  {
    activateSelfCollision_ = false;
  }
  else
  {
    activateSelfCollision_ = true;
  }

  // Set Target
  setTargetDRL(target_x, target_y, target_z, target_roll, target_pitch, target_yaw);

  std::cout << "[MobileManipulatorInterface::mapContinuousActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMRTReady()
{
  //std::cout << "[MobileManipulatorInterface::setMRTReady] START" << std::endl;
  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = true;

  //std::cout << "[MobileManipulatorInterface::setMRTReady] Waiting for the service..." << std::endl;
  ros::service::waitForService("/set_mrt_ready");
  if (setMRTReadyClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMRTReady] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[MobileManipulatorInterface::setMRTReady] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCActionResult(int drlActionResult)
{
  std::cout << "[MobileManipulatorInterface::setMPCActionResult] START" << std::endl;

  bool success = false;
  ocs2_msgs::setMPCActionResult srv;
  srv.request.action_result = drlActionResult;
  srv.request.model_mode = getModelModeInt(robotModelInfo_);

  if (setMPCActionResultClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMPCActionResult] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[MobileManipulatorInterface::setMPCActionResult] END" << std::endl;
  
  return success;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
