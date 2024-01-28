// LAST UPDATE: 2024.01.20
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
MobileManipulatorInterface::MobileManipulatorInterface(ros::NodeHandle& nodeHandle,
                                                       const std::string& taskFile, 
                                                       const std::string& libraryFolder, 
                                                       const std::string& urdfFile,
                                                       int initModelModeInt,
                                                       std::string interfaceName,
                                                       bool printOutFlag)
  : nodeHandle_(nodeHandle), 
    taskFile_(taskFile), 
    libraryFolder_(libraryFolder), 
    urdfFile_(urdfFile),
    initModelModeInt_(initModelModeInt),
    modelModeInt_(initModelModeInt),
    interfaceName_(interfaceName),
    printOutFlag_(printOutFlag)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

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
    if (printOutFlag_)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
    }
  } 
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
  }

  /// Check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile_);
  if (boost::filesystem::exists(urdfFilePath)) 
  {
    if (printOutFlag_)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
    }
  }
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }

  /// Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder_);
  boost::filesystem::create_directories(libraryFolderPath);
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  /// Read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  
  //// Resolve meta-information about the model
  /// Read robot type
  RobotModelType robotModelType = loadRobotType(taskFile_, "model_information.robotModelType");

  /// Read the joints to make fixed
  loadData::loadStdVector<std::string>(taskFile_, "model_information.removeJoints", removeJointNames_, printOutFlag_);
  
  /// Read the link names of joints
  std::vector<std::string> armJointFrameNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointFrameNames", armJointFrameNames, printOutFlag_);

  /// Read the names of joints
  std::vector<std::string> armJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointNames", armJointNames, printOutFlag_);
  
  /// Read the frame names
  std::string robotName, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, sim_, "model_information.sim", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, worldFrameName_, "model_information.worldFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrameName_, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, odomMsgName_, "model_information.odomMsgName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg_, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg_, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg_, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg_, "model_information.armControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, selfCollisionMsg_, "model_information.selfCollisionMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceBaseMsg_, "model_information.occupancyDistanceBaseMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceArmMsg_, "model_information.occupancyDistanceArmMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, pointsOnRobotMsgName_, "model_information.pointsOnRobotMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, octomapMsg_, "model_information.octomapMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, modelModeMsgName_, "model_information.modelModeMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, mpcTargetMsgName_, "model_information.mpcTargetMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, targetMsgName_, "model_information.targetMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, goalFrameName_, "model_information.goalFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionConstraintPoints_, "model_information.collisionConstraintPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionCheckPoints_, "model_information.collisionCheckPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, logSavePathRel_, "model_information.logSavePathRel", printOutFlag_);

  /// NUA TODO: SET THEM IN THE CONFIG!
  /// NUA TODO: DO WE NEED SEPERATE NAMES, SERVICES, CLIENTS FOR DISCRETE AND CONTINUOUS CASES?
  setDiscreteActionDRLMPCServiceName_ = "/set_discrete_action_drl_mpc";
  setDiscreteActionDRLMRTServiceName_ = "/set_discrete_action_drl_mrt";
  setContinuousActionDRLMPCServiceName_ = "/set_continuous_action_drl_mpc";
  setContinuousActionDRLMRTServiceName_ = "/set_continuous_action_drl_mrt";
  setTargetDRLServiceName_ = "/set_target_drl";
  //calculateMPCTrajectoryServiceName_ = "/calculate_mpc_trajectory";
  //computeCommandServiceName_ = "/compute_command";
  setMPCActionResultServiceName_ = "/set_mpc_action_result";

  setStopMPCFlagSrvName_ = "/set_stop_mpc_flag";
  setMPCWaitingFlagSrvName_ = "/set_mpc_waiting_flag";
  setMPCReadyFlagSrvName_ = "/set_mpc_ready";
  setMRTReadyFlagSrvName_ = "/set_mrt_ready";

  if (printOutFlag_)
  {
    std::cout << "\n #### Model Information:" << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
    std::cout << "#### model_information.robotName: " << robotName << std::endl;
    std::cout << "#### model_information.robotModelType: " << static_cast<int>(robotModelType) << std::endl;
    std::cout << "#### model_information.removeJoints: " << std::endl;
    for (const auto& name : removeJointNames_) 
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
    std::cout << "#### model_information.sim: " << sim_ << std::endl;
    std::cout << "#### model_information.baseFrame: " << baseFrameName_ << std::endl;
    std::cout << "#### model_information.armBaseFrame: " << armBaseFrame << std::endl;
    std::cout << "#### model_information.eeFrame: " << eeFrame << std::endl;
    std::cout << "#### model_information.odomMsgName: " << odomMsgName_ << std::endl;
    std::cout << "#### model_information.baseStateMsg: " << baseStateMsg_ << std::endl;
    std::cout << "#### model_information.armStateMsg: " << armStateMsg_ << std::endl;
    std::cout << "#### model_information.baseControlMsg: " << baseControlMsg_ << std::endl;
    std::cout << "#### model_information.armControlMsg: " << armControlMsg_ << std::endl;
    std::cout << "#### model_information.selfCollisionMsg: " << selfCollisionMsg_ << std::endl;
    std::cout << "#### model_information.occupancyDistanceBaseMsg: " << occupancyDistanceBaseMsg_ << std::endl;
    std::cout << "#### model_information.occupancyDistanceArmMsg: " << occupancyDistanceArmMsg_ << std::endl;
    std::cout << "#### model_information.pointsOnRobotMsg: " << pointsOnRobotMsgName_ << std::endl;
    std::cout << "#### model_information.octomapMsg: " << octomapMsg_ << std::endl;
    std::cout << "#### model_information.modelModeMsg: " << modelModeMsgName_ << std::endl;
    std::cout << "#### model_information.mpcTargetMsg: " << mpcTargetMsgName_ << std::endl;
    std::cout << "#### model_information.targetMsg: " << targetMsgName_ << std::endl;
    std::cout << "#### model_information.goalFrame: " << goalFrameName_ << std::endl;
    std::cout << "#### model_information.logSavePathRel: " << logSavePathRel_ << std::endl;
    std::cout << "#### =============================================================================" << std::endl;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] BEFORE getNamespace" << std::endl;
  /// Add namespace
  ns_ = nodeHandle.getNamespace();

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] AFTER getNamespace" << std::endl;

  armJointFrameNames_withNS_ = armJointFrameNames;
  baseFrame_withNS_ = baseFrameName_;
  armBaseFrame_withNS_ = armBaseFrame;
  eeFrame_withNS_ = eeFrame;
  if (ns_ != "/")
  {
    baseFrame_withNS_ = ns_ + "/" + baseFrameName_;
    armBaseFrame_withNS_ = ns_ + "/" + armBaseFrame;
    eeFrame_withNS_ = ns_ + "/" + eeFrame;

    if (printOutFlag_)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] ns: " << ns_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armJointFrameNames_withNS_: " << std::endl;
    }
    for (auto& name : armJointFrameNames_withNS_) 
    {
      name = ns_ + "/" + name;
      if (printOutFlag_)
      {
        std::cout << name << std::endl;
      }
    }

    odomMsgName_ = ns_ + odomMsgName_;
    armStateMsg_ = ns_ + armStateMsg_;
    baseControlMsg_ = ns_ + baseControlMsg_;
    armControlMsg_ = ns_ + armControlMsg_;
    selfCollisionMsg_ = ns_ + selfCollisionMsg_;
    occupancyDistanceBaseMsg_ = ns_ + occupancyDistanceBaseMsg_;
    occupancyDistanceArmMsg_ = ns_ + occupancyDistanceArmMsg_;
    pointsOnRobotMsgName_ = ns_ + pointsOnRobotMsgName_;
    octomapMsg_ = ns_ + octomapMsg_;
    modelModeMsgName_ = ns_ + modelModeMsgName_;
    mpcTargetMsgName_ = ns_ + mpcTargetMsgName_;
    targetMsgName_ = ns_ + targetMsgName_;
    goalFrameName_ = ns_ + goalFrameName_;
    setDiscreteActionDRLMPCServiceName_ = ns_ + setDiscreteActionDRLMPCServiceName_;
    setDiscreteActionDRLMRTServiceName_ = ns_ + setDiscreteActionDRLMRTServiceName_;
    setContinuousActionDRLMPCServiceName_ = ns_ + setContinuousActionDRLMPCServiceName_;
    setContinuousActionDRLMRTServiceName_ = ns_ + setContinuousActionDRLMRTServiceName_;
    setTargetDRLServiceName_ = ns_ + setTargetDRLServiceName_;
    //calculateMPCTrajectoryServiceName_ = ns_ + calculateMPCTrajectoryServiceName_;
    //computeCommandServiceName_ = ns_ + computeCommandServiceName_;
    setMPCActionResultServiceName_ = ns_ + setMPCActionResultServiceName_;
    setStopMPCFlagSrvName_ = ns_ + setStopMPCFlagSrvName_;
    setMPCWaitingFlagSrvName_ = ns_ + setMPCWaitingFlagSrvName_;
    setMPCReadyFlagSrvName_ = ns_ + setMPCReadyFlagSrvName_;
    setMRTReadyFlagSrvName_ = ns_ + setMRTReadyFlagSrvName_;

    if (printOutFlag_)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] baseFrame_withNS_: " << baseFrame_withNS_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armBaseFrame_withNS_: " << armBaseFrame_withNS_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] eeFrame_withNS_: " << eeFrame_withNS_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] odomMsgName_: " << odomMsgName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armStateMsg_: " << armStateMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] baseControlMsg_: " << baseControlMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armControlMsg_: " << armControlMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] selfCollisionMsg_: " << selfCollisionMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] occupancyDistanceBaseMsg_: " << occupancyDistanceBaseMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] occupancyDistanceArmMsg_: " << occupancyDistanceArmMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] pointsOnRobotMsgName_: " << pointsOnRobotMsgName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] octomapMsg_: " << octomapMsg_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] modelModeMsgName_: " << modelModeMsgName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] mpcTargetMsgName_: " << mpcTargetMsgName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] targetMsgName_: " << targetMsgName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] goalFrameName_: " << goalFrameName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setDiscreteActionDRLMPCServiceName_: " << setDiscreteActionDRLMPCServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setDiscreteActionDRLMRTServiceName_: " << setDiscreteActionDRLMRTServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setContinuousActionDRLMPCServiceName_: " << setContinuousActionDRLMPCServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setContinuousActionDRLMRTServiceName_: " << setContinuousActionDRLMRTServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setTargetDRLServiceName_: " << setTargetDRLServiceName_ << std::endl;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] calculateMPCTrajectoryServiceName_: " << calculateMPCTrajectoryServiceName_ << std::endl;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] computeCommandServiceName_: " << computeCommandServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMPCActionResultServiceName_: " << setMPCActionResultServiceName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setStopMPCFlagSrvName_: " << setStopMPCFlagSrvName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMPCWaitingFlagSrvName_: " << setMPCWaitingFlagSrvName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMPCReadyFlagSrvName_: " << setMPCReadyFlagSrvName_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMRTReadyFlagSrvName_: " << setMRTReadyFlagSrvName_ << std::endl;
    }
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  //while(1);

  /// Create pinocchio interface
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createPinocchioInterface" << std::endl;
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames_, worldFrameName_, baseFrame_withNS_)));
  //std::cout << *pinocchioInterfacePtr_;

  /// Set Robot Model Info
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createRobotModelInfo" << std::endl;
  robotModelInfo_ = createRobotModelInfo(robotName,
                                         robotModelType,
                                         baseFrameName_, 
                                         armBaseFrame, 
                                         eeFrame,
                                         armJointFrameNames,
                                         armJointNames);

  /// Set Model Settings
  if (printOutFlag_)
  {
    std::cout << "\n #### Model Settings:";
    std::cout << "\n #### =============================================================================\n";
  }
  //loadData::loadPtreeValue(pt, drlFlag_, "model_settings.drlFlag", printOutFlag_);
  //loadData::loadPtreeValue(pt, drlActionType_, "model_settings.drlActionType", printOutFlag_);
  loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
  loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
  loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
  loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  initActivateSelfCollision_ = activateSelfCollision_;
  initActivateExtCollision_ = activateExtCollision_;
  if (printOutFlag_)
  {
    std::cout << " #### =============================================================================\n";
  }

  /// Get ROS Parameters
  nodeHandle_.getParam("/flag_drl", drlFlag_);
  nodeHandle_.getParam("/drl_action_type", drlActionType_);

  if (printOutFlag_)
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] drlFlag_: " << drlFlag_ << std::endl;
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] drlActionType_: " << drlActionType_ << std::endl;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] DEBUG_INF" << std::endl;
  //while(1);

  /// Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  /// Set Reference Manager
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START referenceManagerPtr_" << std::endl;
  referenceManagerPtr_.reset(new ReferenceManager);

  /// Set ROS Reference Manager
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START rosReferenceManagerPtr_" << std::endl;
  std::string topicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    topicPrefix = ns_ + "/";
  }
  rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(topicPrefix, referenceManagerPtr_));
  rosReferenceManagerPtr_->subscribe(nodeHandle_);

  /// Set Rollout Settings
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START rolloutSettings_" << std::endl;
  
  rolloutSettings_ = rollout::loadSettings(taskFile_, "rollout", printOutFlag_);

  /// Set PointsOnRobot
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START initializePointsOnRobotPtr" << std::endl;
  initializePointsOnRobotPtr(collisionConstraintPoints_);


  /// Set ExtMapUtility
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START emuPtr_" << std::endl;
  emuPtr_.reset(new ExtMapUtility());
  emuPtr_->setWorldFrameName(worldFrameName_);
  emuPtr_->setNodeHandle(nodeHandle_);
  emuPtr_->subscribeOctMsg(octomapMsg_);

  /// Create costs/constraints
  size_t modelModeInt;
  bool isModeUpdated;
  std::string modelName;

  // Mode 0
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 0" << std::endl;
  
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
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 1" << std::endl;
  
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
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 2" << std::endl;

  modelModeInt = 2;
  isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  modelName = getModelModeString(robotModelInfo_);

  quadraticInputCostPtr_mode2_ = getQuadraticInputCost();
  jointLimitSoftConstraintPtr_mode2_ = getJointLimitSoftConstraint();
  endEffectorIntermediateConstraintPtr_mode2_ = getEndEffectorConstraint("endEffector");
  endEffectorFinalConstraintPtr_mode2_ = getEndEffectorConstraint("finalEndEffector");
  selfCollisionConstraintPtr_mode2_ = getSelfCollisionConstraint(modelName);
  extCollisionConstraintPtr_mode2_ = getExtCollisionConstraint("extCollision");

  /// Set MobileManipulatorDynamics
  dynamicsPtr_mode2_.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                         "MobileManipulatorDynamics", 
                                                         libraryFolder_, 
                                                         recompileLibraries_, 
                                                         printOutFlag_));

  // Set initial target
  currentTarget_.resize(7);
  currentTarget_ << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  launchNodes();

  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setNodeHandle(ros::NodeHandle& nodeHandle) 
{ 
  nodeHandle_ = nodeHandle;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setPrintOutFlag(bool printOutFlag)
{
  printOutFlag_ = printOutFlag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializeMPC()
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] activateSelfCollision_: " << activateSelfCollision_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] activateExtCollision_: " << activateExtCollision_ << std::endl;

  /// Set MPC Problem
  setMPCProblem(modelModeInt_, activateSelfCollision_, activateExtCollision_, false, false);

  /// Set MPC BASE
  mpc_.reset(new ocs2::GaussNewtonDDP_MPC(mpcSettings_, 
                                          ddpSettings_, 
                                          *rolloutPtr_, 
                                          ocp_, 
                                          *initializerPtr_));

  /// Set Reference Manager of MPC BASE
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

  /// Set MPC ROS INTERFACE
  std::string topicPrefix = "mobile_manipulator_";
  mpcNode_.reset(new ocs2::MPC_ROS_Interface(mpc_, topicPrefix));
  
  /// Launch nodes of MPC ROS INTERFACE
  mpcNode_->launchNodes(nodeHandle_);

  /// Subscribers
  // TargetTrajectories
  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) 
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC::targetTrajectoriesCallback] START" << std::endl;
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

    if (currentTarget_.size() == targetTrajectories.stateTrajectory[0].size())
    {
      mpcNode_->setTargetTrajectories(targetTrajectories);

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC::targetTrajectoriesCallback] currentTarget_.size(): " << currentTarget_.size() << std::endl;
      for (size_t i = 0; i < currentTarget_.size(); i++)
      {
        //std::cout << i << " -> " << currentTarget_[i] << std::endl;
        currentTarget_[i] = targetTrajectories.stateTrajectory[0][i];
      }
    }
    else
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC::targetTrajectoriesCallback] ERROR: Size mismatch!" << std::endl;
      getEEPose(currentTarget_);
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC::targetTrajectoriesCallback] currentTarget_ IS EE !!!" << std::endl;
    }

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] END" << std::endl;
  };
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] mpcTargetMsgName_: " << mpcTargetMsgName_ << std::endl;
  targetTrajectoriesSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(mpcTargetMsgName_, 5, targetTrajectoriesCallback);

  // Clients
  setMPCWaitingFlagClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>(setMPCWaitingFlagSrvName_);
  setMPCReadyFlagClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>(setMPCReadyFlagSrvName_);

  /// Services
  if (drlFlag_)
  {
    if (drlActionType_ == 0)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] DISCRETE ACTION" << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] setDiscreteActionDRLMPCServiceName_: " << setDiscreteActionDRLMPCServiceName_ << std::endl;
      setDiscreteActionDRLMPCService_ = nodeHandle_.advertiseService(setDiscreteActionDRLMPCServiceName_, &MobileManipulatorInterface::setDiscreteActionDRLMPCSrv, this);
    }
    else
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] CONTINUOUS ACTION" << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] setContinuousActionDRLMPCServiceName_: " << setContinuousActionDRLMPCServiceName_ << std::endl;
      setContinuousActionDRLMPCService_ = nodeHandle_.advertiseService(setContinuousActionDRLMPCServiceName_, &MobileManipulatorInterface::setContinuousActionDRLMPCSrv, this);
    }
  }
  else
  {
    setStopMPCFlagService_ = nodeHandle_.advertiseService(setStopMPCFlagSrvName_, &MobileManipulatorInterface::setStopMPCFlagSrv, this);
    
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] BEFORE updateStatusModelModeMPC" << std::endl;
    mpcNode_->updateStatusModelModeMPC(true);
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializeMRT()
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] activateSelfCollision_: " << activateSelfCollision_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] activateExtCollision_: " << activateExtCollision_ << std::endl;

  /// Set MPC Problem
  setMPCProblem(modelModeInt_, activateSelfCollision_, activateExtCollision_, false, false);

  // Set MRT ROS INTERFACE
  std::string topicPrefix = "mobile_manipulator_";
  mrt_.reset(new MRT_ROS_Interface(topicPrefix));

  // Set Rollout
  mrt_->initRollout(&*rolloutPtr_);

  // Launch nodes of MRT ROS INTERFACE
  mrt_->launchNodes(nodeHandle_);

  // Launch nodes of MRT ROS Gazebo Loop
  mrt_loop_.reset(new MRT_ROS_Gazebo_Loop(nodeHandle_, 
                                          *mrt_, 
                                          robotModelInfo_,
                                          worldFrameName_,
                                          ns_,
                                          targetMsgName_,
                                          goalFrameName_,
                                          baseStateMsg_,
                                          armStateMsg_,
                                          baseControlMsg_,
                                          armControlMsg_,
                                          selfCollisionMsg_,
                                          occupancyDistanceBaseMsg_,
                                          occupancyDistanceArmMsg_,
                                          pointsOnRobotMsgName_,
                                          err_threshold_pos_,
                                          err_threshold_ori_yaw_,
                                          err_threshold_ori_quat_,
                                          mpcSettings_.mrtDesiredFrequency_, 
                                          mpcSettings_.mpcDesiredFrequency_,
                                          drlFlag_,
                                          logSavePathRel_));

  // Set MobileManipulatorVisualization
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] BEFORE mobileManipulatorVisu_" << std::endl;
  mobileManipulatorVisu_.reset(new ocs2::mobile_manipulator::MobileManipulatorVisualization(nodeHandle_, 
                                                                                            *pinocchioInterfacePtr_,
                                                                                            worldFrameName_,
                                                                                            ns_,
                                                                                            baseFrameName_,
                                                                                            urdfFile_,
                                                                                            armStateMsg_,
                                                                                            robotModelInfo_,
                                                                                            activateSelfCollision_,
                                                                                            activateExtCollision_,
                                                                                            removeJointNames_,
                                                                                            collisionObjectPairs_,
                                                                                            collisionLinkPairs_,
                                                                                            selfCollisionMsg_,
                                                                                            pointsOnRobotPtr_,
                                                                                            emuPtr_,
                                                                                            maxDistance_));
  if (printOutFlag_)
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] AFTER mobileManipulatorVisu_" << std::endl;

  // Set model mode of MobileManipulatorVisualization
  mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo_));
  
  // Set observers of MRT ROS Gazebo Loop
  mrt_loop_->subscribeObservers({mobileManipulatorVisu_});  
  //mrt_loop_->setTargetReceivedFlag(true);

  // Clients
  if (drlFlag_)
  {
    if (drlActionType_ == 0)
    {
      setDiscreteActionDRLMPCClient_ = nodeHandle_.serviceClient<ocs2_msgs::setDiscreteActionDRL>(setDiscreteActionDRLMPCServiceName_);
    }
    else
    {
      setContinuousActionDRLMPCClient_ = nodeHandle_.serviceClient<ocs2_msgs::setContinuousActionDRL>(setContinuousActionDRLMPCServiceName_);
    }
    
    setTargetDRLClient_ = nodeHandle_.serviceClient<ocs2_msgs::setTask>(setTargetDRLServiceName_);
    setMPCActionResultClient_ = nodeHandle_.serviceClient<ocs2_msgs::setMPCActionResult>(setMPCActionResultServiceName_);
  }
  else
  {
    setStopMPCFlagClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>(setStopMPCFlagSrvName_);
  }
  setMRTReadyFlagClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>(setMRTReadyFlagSrvName_);

  /// Services
  setMPCWaitingFlagService_ = nodeHandle_.advertiseService(setMPCWaitingFlagSrvName_, &MobileManipulatorInterface::setMPCWaitingFlagSrv, this);
  setMPCReadyFlagService_ = nodeHandle_.advertiseService(setMPCReadyFlagSrvName_, &MobileManipulatorInterface::setMPCReadyFlagSrv, this);

  if (drlFlag_)
  {
    if (drlActionType_ == 0)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] DISCRETE ACTION" << std::endl;
      setDiscreteActionDRLMRTService_ = nodeHandle_.advertiseService(setDiscreteActionDRLMRTServiceName_, &MobileManipulatorInterface::setDiscreteActionDRLMRTSrv, this);
    }
    else
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] CONTINUOUS ACTION" << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] setContinuousActionDRLMRTServiceName_: " << setContinuousActionDRLMRTServiceName_ << std::endl;

      setContinuousActionDRLMRTService_ = nodeHandle_.advertiseService(setContinuousActionDRLMRTServiceName_, &MobileManipulatorInterface::setContinuousActionDRLMRTSrv, this);
    }

    setMRTReadyFlag(true);
  }
  else
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] BEFORE updateStatusModelModeMRT" << std::endl;
    mrt_->updateStatusModelModeMRT(true);
  }

  newMPCProblemFlag_ = true;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializeMRT] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializePointsOnRobotPtr(std::string& collisionPointsName) 
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] START" << std::endl;
  //PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>();

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] collisionPointsName: " << collisionPointsName << std::endl;

  // Get points on robot parameters
  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nodeHandle_.hasParam(collisionPointsName)) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle_.getParam(collisionPointsName, collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorInterface::initializePointsOnRobotPtr] collision_points parameter is not of type array.");
    }
    
    //// NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorInterface::initializePointsOnRobotPtr] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorInterface::initializePointsOnRobotPtr] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorInterface::initializePointsOnRobotPtr] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius << std::endl;
      }
    }
  }
  else
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] ERROR: collision_points is not defined!" << std::endl;
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

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::updateFullModelState(std::vector<double>& statePositionBase, 
                                                      std::vector<double>& statePositionArm,
                                                      std::vector<double>& stateVelocityBase)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] START " << std::endl;

  nav_msgs::Odometry odomMsg = odomMsg_;
  sensor_msgs::JointState jointStateMsg = jointStateMsg_;

  statePositionBase.clear();
  statePositionArm.clear();
  stateVelocityBase.clear();

  // Set mobile base state
  statePositionBase.push_back(odomMsg.pose.pose.position.x);
  statePositionBase.push_back(odomMsg.pose.pose.position.y);

  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  tf::Quaternion quatBase(odomMsg.pose.pose.orientation.x, odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z, odomMsg.pose.pose.orientation.w);
  tf::Matrix3x3 matBase(quatBase);
  matBase.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);
  statePositionBase.push_back(yaw_robot_wrt_world);

  // Set mobile base input
  stateVelocityBase.push_back(odomMsg.twist.twist.linear.x);
  stateVelocityBase.push_back(odomMsg.twist.twist.angular.z);

  // Set arm state
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    statePositionArm.push_back(jointStateMsg.position[stateIndexMap_[i]]);
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] statePositionBase:" << std::endl;
  for (size_t i = 0; i < statePositionBase.size(); i++)
  {
    std::cout << i << ": " << statePositionBase[i] << std::endl;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] statePositionArm:" << std::endl;
  for (size_t i = 0; i < statePositionArm.size(); i++)
  {
    std::cout << i << ": " << statePositionArm[i] << std::endl;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] stateVelocityBase:" << std::endl;
  for (size_t i = 0; i < stateVelocityBase.size(); i++)
  {
    std::cout << i << ": " << stateVelocityBase[i] << std::endl;
  }
  
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
SystemObservation MobileManipulatorInterface::getCurrentObservation(vector_t& currentInput, scalar_t time)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] START" << std::endl;
  
  RobotModelInfo robotModelInfo = robotModelInfo_;

  auto stateDimBase = getStateDimBase(robotModelInfo);
  auto stateDim = getStateDim(robotModelInfo);
  auto inputDimBase = getInputDimBase(robotModelInfo);
  auto modeStateDim = getModeStateDim(robotModelInfo);
  auto modeInputDim = getModeInputDim(robotModelInfo);

  SystemObservation currentObservation;
  currentObservation.mode = 0;
  currentObservation.time = time;
  currentObservation.state.setZero(modeStateDim);
  currentObservation.full_state.setZero(stateDim);
  currentObservation.input.setZero(modeInputDim);

  std::vector<double> statePositionBase;
  std::vector<double> statePositionArm;
  std::vector<double> stateVelocityBase;

  updateFullModelState(statePositionBase, statePositionArm, stateVelocityBase);

  switch (robotModelInfo.robotModelType)
  {
    case RobotModelType::MobileBase:
    {
      // Set state
      currentObservation.state[0] = statePositionBase[0];
      currentObservation.state[1] = statePositionBase[1];
      currentObservation.state[2] = statePositionBase[2];

      // Set input
      if (time > 0.0)
      {
        currentObservation.input[0] = currentInput[0];
        currentObservation.input[1] = currentInput[1];
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
        currentObservation.input[i] = statePositionArm[i];
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
          currentObservation.state[0] = statePositionBase[0];
          currentObservation.state[1] = statePositionBase[1];
          currentObservation.state[2] = statePositionBase[2];

          // Set input
          if (time > 0.0)
          {
            currentObservation.input[0] = currentInput[0];
            currentObservation.input[1] = currentInput[1];
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

            // Set input
            currentObservation.input[i] = statePositionArm[i];

            // Set full state arm
            currentObservation.full_state[stateDimBase + i] = statePositionArm[i];
          }
          break;
        }

        case ModelMode::WholeBodyMotion:
        {
          // Set state and input
          currentObservation.state[0] = statePositionBase[0];
          currentObservation.state[1] = statePositionBase[1];
          currentObservation.state[2] = statePositionBase[2];

          if (time > 0.0)
          {
            currentObservation.input[0] = currentInput[0];
            currentObservation.input[1] = currentInput[1];
          }

          for (size_t i = 0; i < statePositionArm.size(); i++)
          {
            currentObservation.state[stateDimBase + i] = statePositionArm[i];
            currentObservation.input[inputDimBase + i] = statePositionArm[i];
          }

          // Set full state
          currentObservation.full_state = currentObservation.state;
          break;
        }

        default:
          std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] ERROR: Invalid model mode!";
          while(1);
          break;
      }
      break;
    }

    default:
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] ERROR: Invalid robot model type!";
      while(1);
      break;
    }
  }

  /*
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.state size: " << currentObservation.state.size() << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.input size: " << currentObservation.input.size() << std::endl;
  std::cout << currentObservation.input << std::endl << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;
  std::cout << currentObservation.full_state << std::endl << std::endl;
  */

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] END" << std::endl;

  return currentObservation;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPCProblem(size_t modelModeInt, 
                                               bool activateSelfCollision,
                                               bool activateExtCollision,
                                               bool updateMPCFlag,
                                               bool updateMRTFlag)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] START" << std::endl;

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
  modelModeInt_ = modelModeInt;
  activateSelfCollision_ = activateSelfCollision;
  activateExtCollision_ = activateExtCollision;

  mpcTimer2_.startTimer();
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] modelModeInt: " << modelModeInt << std::endl;
  bool isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] isModeUpdated: " << isModeUpdated << std::endl;
  //printRobotModelInfo(robotModelInfo_);
  mpcTimer2_.endTimer();

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
    if (activateSelfCollision) 
    {
      //ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode0_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] EXT-COLLISION CONSTRAINT IS ACTIVE!" << std::endl;
      ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode0_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileBaseDynamics" << std::endl;
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
    if (activateSelfCollision) 
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] SELF-COLLISION CONSTRAINT IS ACTIVE!" << std::endl;
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode1_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] EXT-COLLISION CONSTRAINT IS ACTIVE!" << std::endl;
      ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode1_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: RobotArmDynamics" << std::endl;
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
    if (activateSelfCollision) 
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] SELF-COLLISION CONSTRAINT IS ACTIVE!" << std::endl;
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode2_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] EXT-COLLISION CONSTRAINT IS ACTIVE!" << std::endl;
      ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileManipulatorDynamics" << std::endl;
    ocp_.dynamicsPtr = dynamicsPtr_mode2_;
    mpcTimer9_.endTimer();
  }

  /*
   * Pre-computation
   */
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Pre-computation" << std::endl;
  if (usePreComputation_) 
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
    while(1);
    //ocp_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }

  // Rollout
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Rollout" << std::endl;
  mpcTimer10_.startTimer();
  rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
  mpcTimer10_.endTimer();

  // Initialization
  mpcTimer11_.startTimer();
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Initialization" << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] modeInputDim: " << modeInputDim << std::endl;
  initializerPtr_.reset(new DefaultInitializer(modeInputDim));
  mpcTimer11_.endTimer();

  // Update MPC
  if (updateMPCFlag)
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] UPDATE MPC" << std::endl;
    // Update MPC
    mpc_->initializeMPC(mpcSettings_, 
                        ddpSettings_, 
                        *rolloutPtr_, 
                        ocp_, 
                        *initializerPtr_);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);
    mpcNode_->setMPC(mpc_);
  }

  // Update MRT
  if (updateMRTFlag)
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] UPDATE MRT" << std::endl;

    mrt_loop_->setRobotModelInfo(robotModelInfo_);
    mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo_));
  } 

  mpcProblemReadyFlag_ = true;

  mpcTimer0_.endTimer();

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchNodes()
{
  if (!drlFlag_)
  {
    // Subscribe Model Mode
    //modelModeSubscriber_ = nodeHandle_.subscribe(model_mode_msg_name, 5, &MobileManipulatorInterface::modelModeCallback, this);
    auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] START" << std::endl;

      modelModeInt_ = msg->data;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] modelModeInt_: " << modelModeInt_ << std::endl;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] initActivateSelfCollision_: " << initActivateSelfCollision_ << std::endl;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] initActivateExtCollision_: " << initActivateExtCollision_ << std::endl;
      activateSelfCollision_ = initActivateSelfCollision_;
      activateExtCollision_ = initActivateExtCollision_;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] activateSelfCollision_: " << activateSelfCollision_ << std::endl;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] activateExtCollision_: " << activateExtCollision_ << std::endl;

      mpcProblemReadyFlag_ = false;
      newMPCProblemFlag_ = true;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] DEBUG_INF" << std::endl;
      //while(1);

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] END" << std::endl << std::endl;
    };
    modelModeSubscriber_ = nodeHandle_.subscribe<std_msgs::UInt8>(modelModeMsgName_, 1, modelModeCallback);
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes] DEBUG_INF" << std::endl;
  //while(1);

  // Clients
  //computeCommandClient_ = nodeHandle_.serviceClient<ocs2_msgs::calculateMPCTrajectory>(computeCommandServiceName_);

  // Services
  //calculateMPCTrajectoryService_ = nodeHandle_.advertiseService(calculateMPCTrajectoryServiceName_, &MobileManipulatorInterface::calculateMPCTrajectorySrv, this);

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] mpcTargetMsgName_: " << mpcTargetMsgName_ << std::endl;

  odomSubscriber_ = nodeHandle_.subscribe(odomMsgName_, 5, &MobileManipulatorInterface::odomCallback, this);
  jointStateSub_ = nodeHandle_.subscribe(armStateMsg_, 5, &MobileManipulatorInterface::jointStateCallback, this);

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes] Waiting " << odomMsgName_ << " and " << armStateMsg_ << "..." << std::endl;
  while(!initFlagBaseState_ || !initFlagArmState_){ros::spinOnce();}
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchNodes] Received " << odomMsgName_ << " and " << armStateMsg_ << "!" << std::endl;

  updateStateIndexMap();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::getEEPose(vector_t& eePose)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEEPose] START" << std::endl;

  std::string eeFrame = eeFrame_withNS_;
  if (robotModelInfo_.robotModelType == RobotModelType::MobileBase)
  {
    eeFrame = baseFrame_withNS_;
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

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEEPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setDiscreteActionDRLMPC(int drlActionDiscrete, double drlActionTimeHorizon)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMPC] START" << std::endl;
  
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMPC] DEBUG_INF" << std::endl;
  while(1);

  bool success;

  ocs2_msgs::setDiscreteActionDRL srv;
  srv.request.action = drlActionDiscrete;
  srv.request.time_horizon = drlActionTimeHorizon;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRL] Waiting for the service..." << std::endl;
  //ros::service::waitForService(setDiscreteActionDRLMPCServiceName_);
  if (setDiscreteActionDRLMPCClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setDiscreteActionDRLMPC] ERROR: Failed to call service!");
    success = false;
  }

  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setDiscreteActionDRLMPCSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                                            ocs2_msgs::setDiscreteActionDRL::Response &res)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMPCSrv] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMPCSrv] DEBUG_INF" << std::endl;
  while(1);

  drlActionDiscrete_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  res.success = true;

  //mapDiscreteActionDRL(drlActionDiscrete_);

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMPCSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setDiscreteActionDRLMRTSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                                            ocs2_msgs::setDiscreteActionDRL::Response &res)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMRTSrv] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMRTSrv] DEBUG_INF" << std::endl;
  while(1);

  drlActionDiscrete_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  res.success = true;

  mapDiscreteActionDRL(drlActionDiscrete_);

  setDiscreteActionDRLMPC(drlActionDiscrete_, drlActionTimeHorizon_);

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLMRTSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setContinuousActionDRLMPC(int drlActionId,
                                                           std::vector<double> drlActionContinuous, 
                                                           double drlActionTimeHorizon, 
                                                           bool drlActionLastStepFlag,
                                                           double drlActionLastStepDistanceThreshold)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPC] START" << std::endl;

  bool success;

  ocs2_msgs::setContinuousActionDRL srv;
  srv.request.id = drlActionId;
  srv.request.action = drlActionContinuous;
  srv.request.time_horizon = drlActionTimeHorizon;
  srv.request.last_step_flag = drlActionLastStepFlag;
  srv.request.last_step_distance_threshold = drlActionLastStepDistanceThreshold;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPC] Waiting for the service " << setContinuousActionDRLMPCServiceName_ << "..." << std::endl;
  ros::service::waitForService(setContinuousActionDRLMPCServiceName_);
  if (setContinuousActionDRLMPCClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setContinuousActionDRLMPC] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPC] END" << std::endl;
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setContinuousActionDRLMPCSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                                              ocs2_msgs::setContinuousActionDRL::Response &res)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] START" << std::endl;

  drlActionId_ = req.id;
  drlActionContinuous_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  drlActionLastStepFlag_ = req.last_step_flag;
  drlActionLastStepDistanceThreshold_ = req.last_step_distance_threshold;
  res.success = true;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] drlActionId_: " << drlActionId_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] drlActionContinuous_:" << std::endl;
  for (size_t i = 0; i < drlActionContinuous_.size(); i++)
  {
    std::cout << drlActionContinuous_[i] << std::endl;
  }
  
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] drlActionTimeHorizon_: " << drlActionTimeHorizon_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] drlActionLastStepFlag_: " << drlActionLastStepFlag_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] drlActionLastStepDistanceThreshold_: " << drlActionLastStepDistanceThreshold_ << std::endl;

  mapContinuousActionDRL(false);

  mpcProblemReadyFlag_ = false;
  newMPCProblemFlag_ = true;
  stopMPCFlag_ = true;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMPCSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setContinuousActionDRLMRTSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                                              ocs2_msgs::setContinuousActionDRL::Response &res)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] START" << std::endl;
  drlActionId_ = req.id;
  drlActionContinuous_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  drlActionLastStepFlag_ = req.last_step_flag;
  drlActionLastStepDistanceThreshold_ = req.last_step_distance_threshold;
  res.success = true;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] drlActionId_: " << drlActionId_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] drlActionContinuous_:" << std::endl;
  for (size_t i = 0; i < drlActionContinuous_.size(); i++)
  {
    std::cout << drlActionContinuous_[i] << std::endl;
  }
  
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] drlActionTimeHorizon_: " << drlActionTimeHorizon_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] drlActionLastStepFlag_: " << drlActionLastStepFlag_ << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] drlActionLastStepDistanceThreshold_: " << drlActionLastStepDistanceThreshold_ << std::endl;

  mapContinuousActionDRL(true);

  mpcProblemReadyFlag_ = false;
  newMPCProblemFlag_ = true;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLMRTSrv] END" << std::endl;
  return res.success;
}

/*
bool MobileManipulatorInterface::calculateMPCTrajectorySrv(ocs2_msgs::calculateMPCTrajectory::Request &req, 
                                                           ocs2_msgs::calculateMPCTrajectory::Response &res)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] DEBUG_INF" << std::endl;
  while(1);

  bool useCurrentPolicyFlag = req.use_current_policy_flag;

  double dt = req.dt;
  double time = req.time;
  
  std::vector<double> state = req.state;
  std::vector<double> full_state = req.full_state;
  std::vector<double> input = req.input;
  
  vector_t obs_input;
  SystemObservation currentObservation;
  currentObservation.time = time;

  obs_input.resize(input.size());
  for (size_t i = 0; i < input.size(); i++)
  {
    obs_input[i] = input[i];
  }

  currentObservation.state.resize(state.size());
  for (size_t i = 0; i < state.size(); i++)
  {
    currentObservation.state[i] = state[i];
  }

  currentObservation.full_state.resize(full_state.size());
  for (size_t i = 0; i < full_state.size(); i++)
  {
    currentObservation.full_state[i] = full_state[i];
  }
  
  currentObservation.input.resize(input.size());
  for (size_t i = 0; i < input.size(); i++)
  {
    currentObservation.input[i] = input[i];
  }

  tf2::Quaternion quat;
  quat.setRPY(req.target[3], req.target[4], req.target[5]);

  vector_t currentTarget(7);
  //currentTarget << 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  currentTarget << req.target[0], req.target[1], req.target[2], quat.x(), quat.y(), quat.z(), quat.w();

  const TargetTrajectories targetTrajectories({0}, {currentTarget}, {obs_input});
  referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));

  /// NUA TODO: SET THE PROBLEM IF NECESSARY!
  bool setMPCProblemFlag = req.mpc_problem_flag;
  
  std::vector<double> cmd;
  bool success = false;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] useCurrentPolicyFlag: " << useCurrentPolicyFlag << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] time: " << currentObservation.time << std::endl;

  // CALL THE MRT SERVICE
  success = computeCommandClient(useCurrentPolicyFlag, 
                                 dt,
                                 time,
                                 state,
                                 full_state,
                                 input,
                                 currentTarget,
                                 setMPCProblemFlag,
                                 cmd);

  res.cmd = cmd;
  res.success = success;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] END" << std::endl;
  return res.success;
}
*/

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

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modelMode: " << modelMode << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modeInputDim: " << modeInputDim << std::endl;

  matrix_t R = matrix_t::Zero(modeInputDim, modeInputDim);

  // Input cost of mobile base
  if (modelMode == 0 || modelMode == 2) 
  {
    const size_t inputDimBase = getInputDimBase(robotModelInfo_);

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] inputDimBase: " << inputDimBase << std::endl;

    matrix_t R_base = matrix_t::Zero(inputDimBase, inputDimBase);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.base", R_base, printOutFlag_);
    R.topLeftCorner(inputDimBase, inputDimBase) = R_base;
  }

  // Input cost of arm
  if (modelMode == 1 || modelMode == 2) 
  {
    const size_t inputDimArm = getInputDimArm(robotModelInfo_);

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] inputDimArm: " << inputDimArm << std::endl;

    matrix_t R_arm = matrix_t::Zero(inputDimArm, inputDimArm);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.arm", R_arm, printOutFlag_);
    R.bottomRightCorner(inputDimArm, inputDimArm) = R_arm;
  }

  if (printOutFlag_)
  {
    std::cout << "\n #### Input Cost Settings: ";
    std::cout << "\n #### =============================================================================\n";
    std::cout << "inputCost.R:  \n" << R << '\n';
    std::cout << " #### =============================================================================\n";
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

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] modeInputDim: " << modeInputDim << std::endl;

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit & modelMode != 0) 
  {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const size_t stateDimArm = getStateDimArm(robotModelInfo_);

    // Arm joint DOF limits from the parsed URDF      
    const vector_t lowerBound = model.lowerPositionLimit.tail(stateDimArm);
    const vector_t upperBound = model.upperPositionLimit.tail(stateDimArm);

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;

    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    //std::cout << "\n #### ArmJointPositionLimits Settings: ";
    //std::cout << "\n #### =============================================================================\n";
    //std::cout << " #### lowerBound: " << lowerBound.transpose() << '\n';
    //std::cout << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", printOutFlag_);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", printOutFlag_);
    //std::cout << " #### =============================================================================\n";
    
    stateLimits.reserve(modeStateDim);
    const size_t stateOffset = modeStateDim - stateDimArm;

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateOffset: " << stateOffset << std::endl;

    for (int i = 0; i < stateDimArm; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = stateOffset + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits.size(): " << stateLimits.size() << std::endl;
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
    //std::cout << "\n #### JointVelocityLimits Settings: ";
    //std::cout << "\n #### =============================================================================\n";
    //std::cout << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    //std::cout << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", printOutFlag_);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", printOutFlag_);
    //std::cout << " #### =============================================================================\n";

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

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits size: " << stateLimits.size() << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] inputLimits size: " << inputLimits.size() << std::endl;

  auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));

  boxConstraints->initializeOffset(0.0, vector_t::Zero(modeStateDim), vector_t::Zero(modeInputDim));

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] DEBUG INF" << std::endl;
  //while(1);

  return boxConstraints;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const std::string& prefix) 
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] START prefix: " << prefix << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  //const int stateDim = getStateDim(robotModelInfo_);
  //const int modeStateDim = getModeStateDim(robotModelInfo_);
  //const int modeInputDim = getModeInputDim(robotModelInfo_);

  std::string modelName = getModelModeString(robotModelInfo_) + "_end_effector_kinematics";
  scalar_t muPosition;
  scalar_t muOrientation;
  //std::cout << "\n #### " << prefix << " Settings: ";
  //std::cout << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", printOutFlag_);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", printOutFlag_);
  //std::cout << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) 
  {
    throw std::runtime_error("[MobileManipulatorInterface::getEndEffectorConstraint] ERROR: referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation_) 
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
    while(1);

    MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(*pinocchioInterfacePtr_, pinocchioMapping, {robotModelInfo_.robotArm.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } 
  else 
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] BEFORE pinocchioMappingCppAd" << std::endl;
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(robotModelInfo_);

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] BEFORE eeKinematics" << std::endl;
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_,
                                                     pinocchioMappingCppAd, 
                                                     robotModelInfo_,
                                                     modelName, 
                                                     libraryFolder_, 
                                                     recompileLibraries_, 
                                                     false);
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] AFTER eeKinematics" << std::endl;
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, robotModelInfo_));
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
  //while(1);

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const std::string& prefix) 
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.2;
  //std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  //std::vector<std::pair<std::string, std::string>> collisionLinkPairs;

  //std::cout << "\n #### SelfCollision Settings: ";
  //std::cout << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, "selfCollision.mu", printOutFlag_);
  loadData::loadPtreeValue(pt, delta, "selfCollision.delta", printOutFlag_);
  loadData::loadPtreeValue(pt, minimumDistance, "selfCollision.minimumDistance", printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, "selfCollision.collisionObjectPairs", collisionObjectPairs_, printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile_, "selfCollision.collisionLinkPairs", collisionLinkPairs_, printOutFlag_);
  //std::cout << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr_, collisionLinkPairs_, collisionObjectPairs_);
  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] DEBUG INF" << std::endl;
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

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getExtCollisionConstraint(const std::string& prefix) 
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  const int modelStateDim = getStateDim(robotModelInfo_);
  auto modelModeInt = getModelModeInt(robotModelInfo_);

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  //scalar_t maxDistance = 10;
  //std::cout << "\n #### ExtCollision Settings: ";
  //std::cout << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", printOutFlag_);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", printOutFlag_);
  loadData::loadPtreeValue(pt, maxDistance_, prefix + ".maxDistance", printOutFlag_);
  //std::cout << " #### =============================================================================\n";


  ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface(*pinocchioInterfacePtr_);
  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ true" << std::endl;

    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] DEBUG INF" << std::endl;
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
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ false" << std::endl;

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

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::updateStateIndexMap()
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] START" << std::endl;
  
  auto jointNames = robotModelInfo_.robotArm.jointNames;
  int n_joints = jointNames.size();
  stateIndexMap_.clear();
  //int c;

  sensor_msgs::JointState jointStateMsg = jointStateMsg_;
  for (size_t i = 0; i < n_joints; i++)
  {
    auto it = find(jointStateMsg.name.begin(), jointStateMsg.name.end(), jointNames[i]);

    // If element was found
    if (it != jointStateMsg.name.end()) 
    {
        // calculating the index
        int index = it - jointStateMsg.name.begin();
        std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] joint name: " << *it << " -> index: " << index << std::endl;
        stateIndexMap_.push_back(index);
    }
    else
    {
      throw std::runtime_error("[" + ns_ +  "][MRT_ROS_Gazebo_Loop::updateStateIndexMap] Error: Joint " + jointNames[i] + " not found!");
    }
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //std::cout << "[MobileManipulatorInterface::odomCallback] START " << std::endl;
  odomMsg_ = *msg;
  initFlagBaseState_ = true;
  //std::cout << "[MobileManipulatorInterface::odomCallback] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //std::cout << "[MobileManipulatorInterface::jointStateCallback] START" << std::endl;
  jointStateMsg_ = *msg;
  initFlagArmState_ = true;
  //std::cout << "[MobileManipulatorInterface::jointStateCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setTargetDRL(double x, double y, double z, double roll, double pitch, double yaw, double time_horizon)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] START" << std::endl;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] DEBUG_INF" << std::endl;
  //while(1);

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

  srv.request.time_horizon = time_horizon;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] x: " << x << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] y: " << y << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] z: " << z << std::endl;

  /*
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] x: " << x << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] y: " << y << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] z: " << z << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] roll: " << roll << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] pitch: " << pitch << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] yaw: " << yaw << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] qx: " << quat.x() << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] qy: " << quat.y() << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] qz: " << quat.z() << std::endl;
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] qw: " << quat.w() << std::endl;
  */

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] Waiting for the service " << setTargetDRLServiceName_ << "..." << std::endl;
  //ros::service::waitForService(setTargetDRLServiceName_);
  if (setTargetDRLClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setTargetDRL] ERROR: Failed to call service!");
    success = false;
  }

  //targetReceivedFlag_ = true;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapDiscreteActionDRL(int action)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] START" << std::endl;

  /// NUA TODO: NEEDS DEBUGGING!
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] DEBUG_INF" << std::endl;
  while(1);

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
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] " << tmp_name << ": " << tmp_val << std::endl;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapContinuousActionDRL(bool setTargetDRLFlag)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] START" << std::endl;

  double modelModeProb = drlActionContinuous_[0];
  double constraintProb = drlActionContinuous_[1];
  double target_x = drlActionContinuous_[2];
  double target_y = drlActionContinuous_[3];
  double target_z = drlActionContinuous_[4];
  double target_roll = drlActionContinuous_[5];
  double target_pitch = drlActionContinuous_[6];
  double target_yaw = drlActionContinuous_[7];
  
  // Set Model Mode
  if (modelModeProb <= 0.3)
  {
    //if (setTargetDRLFlag)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] BASE MOTION" << std::endl;
    }
    
    target_roll = 0.0;
    target_pitch = 0.0;
    target_z = 0.12;
    modelModeInt_ = 0;
  }
  else if (modelModeProb > 0.6)
  {
    //if (setTargetDRLFlag)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] WHOLE-BODY MOTION" << std::endl;
    }
    
    modelModeInt_ = 2;
  }
  else
  {
    //if (setTargetDRLFlag)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] ARM MOTION" << std::endl;
    }
    
    modelModeInt_ = 1;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] modelModeInt_: " << modelModeInt_ << std::endl;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] DEBUG_INF" << std::endl;
  //while(1);

  // Set Constraint Flags
  if (constraintProb <= 0.5)
  {
    if (setTargetDRLFlag)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] SELF-COLLISION DEACTIVATED!" << std::endl;
    }
    activateSelfCollision_ = false;
  }
  else
  {
    if (setTargetDRLFlag)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] SELF-COLLISION ACTIVATED!" << std::endl;
    }
    activateSelfCollision_ = true;
  }

  // Set Target
  if (setTargetDRLFlag)
  {
    setTargetDRL(target_x, target_y, target_z, target_roll, target_pitch, target_yaw, drlActionTimeHorizon_);
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCActionResult(int drlActionResult)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] START" << std::endl;
  bool success = false;
  ocs2_msgs::setMPCActionResult srv;
  srv.request.action_result = drlActionResult;
  srv.request.model_mode = modelModeInt_;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] drlActionResult: " << drlActionResult << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] modelModeInt_: " << modelModeInt_ << std::endl;

  if (setMPCActionResultClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMPCActionResult] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] END" << std::endl;
  
  return success;
}

/*
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::computeCommandClient(bool& use_current_policy_flag, 
                                                      double& dt,
                                                      double& time,
                                                      std::vector<double>& state,
                                                      std::vector<double>& full_state,
                                                      std::vector<double>& input,
                                                      vector_t& currentTarget,
                                                      bool& setMPCProblemFlag,
                                                      std::vector<double>& cmd)
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] START" << std::endl;
  
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] DEBUG_INF" << std::endl;
  while(1);

  bool success = false;

  ocs2_msgs::calculateMPCTrajectory srv;
  srv.request.use_current_policy_flag = use_current_policy_flag;
  
  srv.request.dt = dt;
  srv.request.time = time;

  srv.request.state = state;
  srv.request.full_state = full_state;
  srv.request.input = input;
  
  srv.request.target.resize(currentTarget.size());
  for (size_t i = 0; i < currentTarget.size(); i++)
  {
    srv.request.target[i] = currentTarget[i];
  }

  srv.request.mpc_problem_flag = setMPCProblemFlag;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] Waiting for the service..." << std::endl;
  //ros::service::waitForService(computeCommandServiceName_);
  if (computeCommandClient_.call(srv))
  {
    success = srv.response.success;
    cmd = srv.response.cmd;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::computeCommandClient] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] END" << std::endl;
  
  return success;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setStopMPCFlag(bool val)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlag] DEBUG_INF" << std::endl;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlag] DEBUG_INF" << std::endl;
  //while(1);

  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = val;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlag] Waiting for the service..." << std::endl;
  //ros::service::waitForService(setStopMPCFlagSrvName_);
  if (setStopMPCFlagClient_.call(srv))
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlag] DONE!" << std::endl;
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setStopMPCFlag] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlag] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setStopMPCFlagSrv(ocs2_msgs::setBool::Request &req, 
                                                   ocs2_msgs::setBool::Response &res)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] START" << std::endl;

  // std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] DEBUG_INF" << std::endl;
  // while(1);
  
  if (!req.val)
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] SELF-COLLISION DEACTIVATED!" << stopMPCFlag_ << std::endl;
    activateSelfCollision_ = false;
  }
  stopMPCFlag_ = true;

  res.success = true;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setStopMPCFlagSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCWaitingFlag(bool val)
{
  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlag] START" << std::endl;

  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = val;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCWaitingFlag] Waiting for the service..." << std::endl;
  ros::service::waitForService(setMPCWaitingFlagSrvName_);
  if (setMPCWaitingFlagClient_.call(srv))
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCWaitingFlag] DONE!" << std::endl;
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMPCWaitingFlag] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlag] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCWaitingFlagSrv(ocs2_msgs::setBool::Request &req, 
                                                      ocs2_msgs::setBool::Response &res)
{
  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlagSrv] START" << std::endl;
  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlagSrv] BEFORE mpcWaitingFlag_: " << mpcWaitingFlag_ << std::endl;
  mpcWaitingFlag_ = req.val;
  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlagSrv] AFTER mpcWaitingFlag_: " << mpcWaitingFlag_ << std::endl;

  res.success = true;

  //std::cout << "[MobileManipulatorInterface::setMPCWaitingFlagSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCReadyFlag(bool val)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCReadyFlag] START" << std::endl;
  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = val;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCReadyFlag] Waiting for the service..." << std::endl;
  ros::service::waitForService(setMPCReadyFlagSrvName_);
  if (setMPCReadyFlagClient_.call(srv))
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCReadyFlag] DONE!" << std::endl;
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMPCReadyFlag] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMPCReadyFlag] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCReadyFlagSrv(ocs2_msgs::setBool::Request &req, 
                                                    ocs2_msgs::setBool::Response &res)
{
  //std::cout << "[MobileManipulatorInterface::setMPCReadyFlagSrv] START" << std::endl;
  //std::cout << "[MobileManipulatorInterface::setMPCReadyFlagSrv] BEFORE mpcReadyFlag_: " << mpcReadyFlag_ << std::endl;
  mpcReadyFlag_ = req.val;
  //std::cout << "[MobileManipulatorInterface::setMPCReadyFlagSrv] AFTER mpcReadyFlag_: " << mpcReadyFlag_ << std::endl;

  res.success = true;

  //std::cout << "[MobileManipulatorInterface::setMPCReadyFlagSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMRTReadyFlag(bool val)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMRTReadyFlag] START" << std::endl;
  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = val;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMRTReadyFlag] Waiting for the service " << setMRTReadyFlagSrvName_ << "..." << std::endl;
  ros::service::waitForService(setMRTReadyFlagSrvName_);
  if (setMRTReadyFlagClient_.call(srv))
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMRTReadyFlag] DONE!" << std::endl;
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMRTReadyFlag] ERROR: Failed to call service!");
    success = false;
  }

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::setMRTReadyFlag] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMRTReadyFlagSrv(ocs2_msgs::setBool::Request &req, 
                                                    ocs2_msgs::setBool::Response &res)
{
  //std::cout << "[MobileManipulatorInterface::setMRTReadyFlagSrv] START" << std::endl;
  //std::cout << "[MobileManipulatorInterface::setMRTReadyFlagSrv] BEFORE mrtReadyFlag_: " << mrtReadyFlag_ << std::endl;
  mrtReadyFlag_ = req.val;
  //std::cout << "[MobileManipulatorInterface::setMRTReadyFlagSrv] AFTER mrtReadyFlag_: " << mrtReadyFlag_ << std::endl;

  res.success = true;

  //std::cout << "[MobileManipulatorInterface::setMRTReadyFlagSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchMPC()
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] START" << std::endl;
  //mpcNode_->spin();
  
  mpcNode_->updateStatusModelModeMPC(true);

  while (ros::ok() && ros::master::check()) 
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] IF stopMPCFlag_" << std::endl;
    if (stopMPCFlag_)
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] stopMPCFlag_ true" << std::endl;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] modelModeInt_: " << modelModeInt_ << std::endl;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] BEFORE updateStatusModelModeMPC" << std::endl;
      mpcNode_->updateStatusModelModeMPC(false);

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] DEBUG_INF" << std::endl;
      //while(1);
      
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] Waiting mpcReadyFlag..." << std::endl;
      while(!mpcNode_->getMPCReadyFlag()){spinOnce();}

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] BEFORE setMPCWaitingFlag" << std::endl;
      setMPCWaitingFlag(true);

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] BEFORE setMPCProblem" << std::endl;
      setMPCProblem(modelModeInt_, activateSelfCollision_, activateExtCollision_, true, false);
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] AFTER setMPCProblem" << std::endl;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] BEFORE setMPCReadyFlag" << std::endl;
      setMPCReadyFlag(true);

      // Reset flags
      stopMPCFlag_ = false;
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] BEFORE updateStatusModelModeMPC" << std::endl;
      mpcNode_->updateStatusModelModeMPC(true);
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] AFTER updateStatusModelModeMPC" << std::endl;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] DEBUG_INF" << std::endl;
      //while(1);

      mpcModeChangeCtr_++;
    }

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
  }

  mpcNode_->updateStatusModelModeMPC(false);
  
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchMRT()
{
  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] START" << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] DEBUG_INF" << std::endl;
  while(1);

  ros::Rate simRate(mpcSettings_.mrtDesiredFrequency_);

  while (ros::ok() && ros::master::check()) 
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] mrtIter_: " << mrtIter_ << std::endl;

    if (mrtIter_ == 0)
    {
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] FIRST TIMER mrt_loop_->run2..." << std::endl;
      resetFlag_ = mrt_loop_->run2(currentTarget_);
    }

    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] BEFORE mrtLoop2" << std::endl;
    resetFlag_ = mrt_loop_->mrtLoop2();

    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] resetFlag_: " << resetFlag_ << std::endl;

    mrtIter_++;

    spinOnce();
    //simRate.sleep();
  }

  mrt_->updateStatusModelModeMRT(true);

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::launchMRT] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mpcCallback(const ros::TimerEvent& event)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] START" << std::endl;
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] mpcIter_: " << mpcIter_ << std::endl;

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] DEBUG_INF" << std::endl;
  while(1);

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] IF stopMPCFlag_" << std::endl;
  if (stopMPCFlag_)
  {
    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE setMPCWaitingFlag" << std::endl;
    setMPCWaitingFlag(true);

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE setMPCProblem" << std::endl;
    setMPCProblem(modelModeInt_, activateSelfCollision_, activateExtCollision_, true, false);
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] AFTER setMPCProblem" << std::endl;

    std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE setMPCReadyFlag" << std::endl;
    setMPCReadyFlag(true);

    mpcModeChangeCtr_++;
  }

  std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE computeTrajectory" << std::endl;
  mpcNode_->computeTrajectory();
  mpcNode_->singleSpin();

  //spinOnce();

  mpcIter_++;

  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mrtCallback(const ros::TimerEvent& event)
{
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] START" << std::endl;
  
  if (targetReceivedFlag_)
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] mrtIter_: " << mrtIter_ << std::endl;

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE getCurrentTarget" << std::endl;
    vector_t currentTarget = mrt_loop_->getCurrentTarget();

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] IF newMPCProblemFlag_: " << newMPCProblemFlag_ << std::endl;
    if (newMPCProblemFlag_)
    {
      mrt_->updateStatusModelModeMRT(false);
      
      if (drlFlag_)
      {
        //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE setContinuousActionDRLMPC" << std::endl;
        setContinuousActionDRLMPC(drlActionId_,
                                  drlActionContinuous_, 
                                  drlActionTimeHorizon_, 
                                  drlActionLastStepFlag_,
                                  drlActionLastStepDistanceThreshold_);
        //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] AFTER setContinuousActionDRLMPC" << std::endl;
      }
      else
      {
        //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] mrtStuckCtr_: " << mrtStuckCtr_ << std::endl;
        //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE setStopMPCFlag" << std::endl;
        
        if (mrtStuckCtr_ > 5)
        {
          //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] SELF-COLLISION DEACTIVATED!" << std::endl;
          activateSelfCollision_ = false;
          setStopMPCFlag(false);
        }
        else
        {
          setStopMPCFlag(true);
        }
        
        //while (!setStopMPCFlag(true)){spinOnce();}
      }

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] Waiting mpcWaitingFlag_" << std::endl;
      while (!mpcWaitingFlag_){spinOnce();}
      
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] modelModeInt_: " << modelModeInt_ << std::endl;
      std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE setMPCProblem" << std::endl;
      setMPCProblem(modelModeInt_, activateSelfCollision_, activateExtCollision_, false, true);
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] AFTER setMPCProblem" << std::endl;

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] Waiting mpcReadyFlag_" << std::endl;
      while (!mpcReadyFlag_){spinOnce();}

      mrt_->updateStatusModelModeMRT(true);

      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE run2" << std::endl;
      resetFlag_ = mrt_loop_->run2(currentTarget);

      /*
      if (!resetFlag_ && drlFlag_)
      {
        setMRTReadyFlag(true);
      }
      */

      // Reset flags
      mpcWaitingFlag_ = false;
      mpcReadyFlag_ = false;
      newMPCProblemFlag_ = false;

      mrtModeChangeCtr_++;
    }

    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] IF mrtLoop2" << std::endl;
    if (!resetFlag_)
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE mrtLoop2" << std::endl;
      resetFlag_ = mrt_loop_->mrtLoop2();

      if (drlFlag_)
      {
        int drlActionResult = mrt_loop_->getDRLActionResult();

        if (drlActionResult != -1)
        {
          //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] DONE drlActionResult: " << drlActionResult << std::endl;
          setMPCActionResult(drlActionResult);
          targetReceivedFlag_ = false;
          //setMRTReadyFlag(true);
          mrt_loop_->setTimerStartedFlag(false);

          std::cout << "[" << interfaceName_ << "][" << ns_ <<  "]----------------------------END OF MPC ACTION SEQUENCE----------------------------" << std::endl;
          std::cout << "" << std::endl;
          //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] DEBUG_INF" << std::endl;
          //while(1);
        }
      }
      else
      {
        if (!resetFlag_)
        {
          mrtStuckCtr_ = 0;
        }
      }
    }
    else
    {
      //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] MANUAL newMPCProblemFlag_" << std::endl;
      newMPCProblemFlag_ = true;
      
      if (!drlFlag_)
      {
        mrtStuckCtr_++;
      }
    }

    currentTarget_ = mrt_loop_->getCurrentTarget();

    if (drlFlag_)
    {
      bool isTimeHorizonEnd = mrt_loop_->checkTimer(drlActionTimeHorizon_);
      if (isTimeHorizonEnd)
      {
        std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] END OF ACTION HORIZON!" << std::endl;
        int drlActionResult = 5;
        setMPCActionResult(drlActionResult);
        targetReceivedFlag_ = false;
        mrt_loop_->setTargetReceivedFlag(false);
        //setMRTReadyFlag(true);
        mrt_loop_->setTimerStartedFlag(false);

        std::cout << "[" << interfaceName_ << "][" << ns_ <<  "]----------------------------END OF MPC ACTION SEQUENCE----------------------------" << std::endl;
        std::cout << "" << std::endl;
      }
    }

    //spinOnce();

    mrtIter_++;
  }
  else
  {
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE targetReceivedFlag_: " << targetReceivedFlag_ << std::endl;
    targetReceivedFlag_ = mrt_loop_->getTargetReceivedFlag();
    //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] AFTER targetReceivedFlag_: " << targetReceivedFlag_ << std::endl;
  }
  
  //std::cout << "[" << interfaceName_ << "][" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] END" << std::endl;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
