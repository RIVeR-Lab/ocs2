// LAST UPDATE: 2023.12.01
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
    modelModeInt_(initModelModeInt)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

  /// NUA NOTE: DEPRICATED! NEED REVIEW AND UPDATE!
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;
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
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
  } 
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
  }

  /// Check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile_);
  if (boost::filesystem::exists(urdfFilePath)) 
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  }
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }

  /// Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder_);
  boost::filesystem::create_directories(libraryFolderPath);
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

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
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] BEFORE armJointFrameNames" << std::endl;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointFrameNames", armJointFrameNames, printOutFlag_);
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] AFTER armJointFrameNames" << std::endl;

  // Read the names of joints
  std::vector<std::string> armJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointNames", armJointNames, printOutFlag_);

  // Read the frame names
  std::string robotName, baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, sim_, "model_information.sim", printOutFlag_);
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
    std::cout << "#### model_information.sim: " << sim_ << std::endl;
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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createPinocchioInterface" << std::endl;
  // Create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames, baseFrame, worldFrameName_)));
  //std::cout << *pinocchioInterfacePtr_;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createRobotModelInfo" << std::endl;
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
    std::cout << "\n #### Model Settings:";
    std::cout << "\n #### =============================================================================\n";
  }
    loadData::loadPtreeValue(pt, drlFlag_, "model_settings.drlFlag", printOutFlag_);
    loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
    loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
    loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
    loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  if (printOutFlag_)
  {
    std::cout << " #### =============================================================================\n";
  }

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  //// NUA NOTE: Depricated...some initialization that requires node handle is omitted!
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  while(1);

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
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
    modelModeInt_(initModelModeInt)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] START" << std::endl;

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
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] Loading task file: " << taskFilePath << std::endl;
    }
  } 
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface(6)] Task file not found: " + taskFilePath.string());
  }

  /// Check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile_);
  if (boost::filesystem::exists(urdfFilePath)) 
  {
    if (printOutFlag_)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] Loading Pinocchio model from: " << urdfFilePath << std::endl;
    }
  }
  else 
  {
    throw std::invalid_argument("[MobileManipulatorInterface::MobileManipulatorInterface(6)] URDF file not found: " + urdfFilePath.string());
  }

  /// Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder_);
  boost::filesystem::create_directories(libraryFolderPath);
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

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
  loadData::loadPtreeValue<std::string>(pt, sim_, "model_information.sim", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, worldFrameName_, "model_information.worldFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", printOutFlag_);
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
  loadData::loadPtreeValue<std::string>(pt, collisionConstraintPoints, "model_information.collisionConstraintPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionCheckPoints, "model_information.collisionCheckPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, logSavePathRel_, "model_information.logSavePathRel", printOutFlag_);

  /// NUA TODO: SET THEM IN THE CONFIG!
  setActionDRLServiceName_ = "/set_action_drl";
  setTargetDRLServiceName_ = "/set_target_drl";
  calculateMPCTrajectoryServiceName_ = "/calculate_mpc_trajectory";
  computeCommandServiceName_ = "/compute_command";
  setMRTReadyServiceName_ = "/set_mrt_ready";
  setMPCActionResultServiceName_ = "/set_mpc_action_result";

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
    std::cout << "#### model_information.sim: " << sim_ << std::endl;
    std::cout << "#### model_information.baseFrame: " << baseFrame << std::endl;
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

  // Adding namespace
  ns_ = nodeHandle.getNamespace();
  armJointFrameNames_withNS_ = armJointFrameNames;
  baseFrame_withNS_ = baseFrame;
  armBaseFrame_withNS_ = armBaseFrame;
  eeFrame_withNS_ = eeFrame;
  if (ns_ != "/")
  {
    baseFrame_withNS_ = ns_ + "/" + baseFrame;
    armBaseFrame_withNS_ = ns_ + "/" + armBaseFrame;
    eeFrame_withNS_ = ns_ + "/" + eeFrame;

    if (printOutFlag_)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] ns: " << ns_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armJointFrameNames_withNS_: " << std::endl;
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
    setActionDRLServiceName_ = ns_ + setActionDRLServiceName_;
    setTargetDRLServiceName_ = ns_ + setTargetDRLServiceName_;
    calculateMPCTrajectoryServiceName_ = ns_ + calculateMPCTrajectoryServiceName_;
    computeCommandServiceName_ = ns_ + computeCommandServiceName_;
    setMRTReadyServiceName_ = ns_ + setMRTReadyServiceName_;
    setMPCActionResultServiceName_ = ns_ + setMPCActionResultServiceName_;

    if (printOutFlag_)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] baseFrame_withNS_: " << baseFrame_withNS_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armBaseFrame_withNS_: " << armBaseFrame_withNS_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] eeFrame_withNS_: " << eeFrame_withNS_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] odomMsgName_: " << odomMsgName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armStateMsg_: " << armStateMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] baseControlMsg_: " << baseControlMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] armControlMsg_: " << armControlMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] selfCollisionMsg_: " << selfCollisionMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] occupancyDistanceBaseMsg_: " << occupancyDistanceBaseMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] occupancyDistanceArmMsg_: " << occupancyDistanceArmMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] pointsOnRobotMsgName_: " << pointsOnRobotMsgName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] octomapMsg_: " << octomapMsg_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] modelModeMsgName_: " << modelModeMsgName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] mpcTargetMsgName_: " << mpcTargetMsgName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] targetMsgName_: " << targetMsgName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] goalFrameName_: " << goalFrameName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setActionDRLServiceName_: " << setActionDRLServiceName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setTargetDRLServiceName_: " << setTargetDRLServiceName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] calculateMPCTrajectoryServiceName_: " << calculateMPCTrajectoryServiceName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] computeCommandServiceName_: " << computeCommandServiceName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMRTReadyServiceName_: " << setMRTReadyServiceName_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] setMPCActionResultServiceName_: " << setMPCActionResultServiceName_ << std::endl;
    }
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] DEBUG INF" << std::endl;
  //while(1);

  // Create pinocchio interface
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createPinocchioInterface" << std::endl;
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames, worldFrameName_, baseFrame_withNS_)));
  //std::cout << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START createRobotModelInfo" << std::endl;
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
    std::cout << "\n #### Model Settings:";
    std::cout << "\n #### =============================================================================\n";
  }
    loadData::loadPtreeValue(pt, drlFlag_, "model_settings.drlFlag", printOutFlag_);
    loadData::loadPtreeValue(pt, drlActionType_, "model_settings.drlActionType", printOutFlag_);
    loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", printOutFlag_);
    loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", printOutFlag_);
    loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", printOutFlag_);
    loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", printOutFlag_);
  if (printOutFlag_)
  {
    std::cout << " #### =============================================================================\n";
  }

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp", printOutFlag_);
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", printOutFlag_);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START referenceManagerPtr_" << std::endl;
  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START rosReferenceManagerPtr_" << std::endl;
  // Set ROS Reference Manager
  std::string topicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    topicPrefix = ns_ + "/";
  }
  rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(topicPrefix, referenceManagerPtr_));
  rosReferenceManagerPtr_->subscribe(nodeHandle_);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START rolloutSettings_" << std::endl;
  // Set Rollout Settings
  rolloutSettings_ = rollout::loadSettings(taskFile_, "rollout", printOutFlag_);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START initializePointsOnRobotPtr" << std::endl;
  // Set PointsOnRobot
  initializePointsOnRobotPtr(collisionConstraintPoints);

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] DEBUG INF" << std::endl;
  //while(1);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START emuPtr_" << std::endl;
  // Set ExtMapUtility
  emuPtr_.reset(new ExtMapUtility());
  emuPtr_->setWorldFrameName(worldFrameName_);
  //emuPtr_->setPubOccDistArrayVisu(occupancyDistanceArmMsg_);
  emuPtr_->setNodeHandle(nodeHandle_);
  emuPtr_->subscribeOctMsg(octomapMsg_);

  // Create costs/constraints
  size_t modelModeInt;
  bool isModeUpdated;
  std::string modelName;

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 0" << std::endl;
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

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 1" << std::endl;
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
  
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] START Mode 2" << std::endl;
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
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] BEFORE mobileManipulatorVisu_" << std::endl;
  mobileManipulatorVisu_.reset(new ocs2::mobile_manipulator::MobileManipulatorVisualization(nodeHandle_, 
                                                                                            *pinocchioInterfacePtr_,
                                                                                            worldFrameName_,
                                                                                            ns_,
                                                                                            baseFrame,
                                                                                            urdfFile,
                                                                                            armStateMsg_,
                                                                                            robotModelInfo_,
                                                                                            activateSelfCollision_,
                                                                                            activateExtCollision_,
                                                                                            removeJointNames,
                                                                                            collisionObjectPairs_,
                                                                                            collisionLinkPairs_,
                                                                                            selfCollisionMsg_,
                                                                                            pointsOnRobotPtr_,
                                                                                            emuPtr_,
                                                                                            maxDistance_));
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] AFTER mobileManipulatorVisu_" << std::endl;

  launchNodes();

  
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] BEFORE MPC INITIALIZATION" << std::endl;
  setMPCProblem(modelModeInt_, true, false, false);

  /// NUA TODO: LEFT HERE: INITALIZE ONCE AND USE IT LATER!
  mpc_.reset(new ocs2::GaussNewtonDDP_MPC(mpcSettings_, 
                                          ddpSettings_, 
                                          *rolloutPtr_, 
                                          ocp_, 
                                          *initializerPtr_));

  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

  //std::string topicPrefix = "mobile_manipulator_";
  mpcNode_.reset(new ocs2::MPC_ROS_Interface(mpc_, topicPrefix));
  
  //mpcNode_->setModelModeInt(getModelModeInt(robotModelInfo_));

  mpcNode_->launchNodes(nodeHandle_);
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] AFTER MPC INITIALIZATION" << std::endl;

  //int mm = getModelModeInt(robotModelInfo_);
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] BEFORE MRT INITIALIZATION" << std::endl;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] BEFORE MRT INITIALIZATION" << std::endl;
  //mrt_.reset(new MRT_ROS_Interface(robotModelInfo_, topicPrefix));
  mrt_.reset(new MRT_ROS_Interface(topicPrefix));

  mrt_->initRollout(&*rolloutPtr_);

  mrt_->launchNodes(nodeHandle_);

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

  mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo_));
  
  mrt_loop_->subscribeObservers({mobileManipulatorVisu_});

  currentTarget_.resize(7);
  currentTarget_ << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  
  mrt_loop_->setTargetReceivedFlag(true);

  //mrt_loop_->run(currentTarget_);

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] DEBUG INF" << std::endl;
  //while(1);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface(6)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::initializePointsOnRobotPtr(std::string& collisionPointsName) 
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] START" << std::endl;
  //PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>();

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] collisionPointsName: " << collisionPointsName << std::endl;

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
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] pointsAndRadii:" << std::endl;
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
        //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius << std::endl;
      }
    }
  }
  else
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] ERROR: collision_points is not defined!" << std::endl;
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



  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::initializePointsOnRobotPtr] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::updateFullModelState(std::vector<double>& statePositionBase, 
                                                      std::vector<double>& statePositionArm,
                                                      std::vector<double>& stateVelocityBase)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] START " << std::endl;

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

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] statePositionBase:" << std::endl;
  for (size_t i = 0; i < statePositionBase.size(); i++)
  {
    std::cout << i << ": " << statePositionBase[i] << std::endl;
  }

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] statePositionArm:" << std::endl;
  for (size_t i = 0; i < statePositionArm.size(); i++)
  {
    std::cout << i << ": " << statePositionArm[i] << std::endl;
  }

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] stateVelocityBase:" << std::endl;
  for (size_t i = 0; i < stateVelocityBase.size(); i++)
  {
    std::cout << i << ": " << stateVelocityBase[i] << std::endl;
  }
  
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateFullModelState] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
SystemObservation MobileManipulatorInterface::getCurrentObservation(vector_t& currentInput, scalar_t time)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] START" << std::endl;
  
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
          std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] ERROR: Invalid model mode!";
          while(1);
          break;
      }
      break;
    }

    default:
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] ERROR: Invalid robot model type!";
      while(1);
      break;
    }
  }

  /*
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.state size: " << currentObservation.state.size() << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.input size: " << currentObservation.input.size() << std::endl;
  std::cout << currentObservation.input << std::endl << std::endl;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;
  std::cout << currentObservation.full_state << std::endl << std::endl;
  */

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getCurrentObservation] END" << std::endl;

  return currentObservation;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPCProblem(size_t modelModeInt, 
                                               bool activateSelfCollision,
                                               bool activateExtCollision,
                                               bool updateMPCBaseFlag)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  /*
  while(!mpcExitFlag_)
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] WAITING..." << std::endl;
    spinOnce();
  }
  */

  mpcProblemReadyFlag_ = false;

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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE modelModeInt: " << modelModeInt << std::endl;

  // Set MPC Problem Settings
  modelModeInt_ = modelModeInt;
  activateSelfCollision_ = activateSelfCollision;
  activateExtCollision_ = activateExtCollision;

  mpcTimer2_.startTimer();
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] modelModeInt: " << modelModeInt << std::endl;
  bool isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] isModeUpdated: " << isModeUpdated << std::endl;
  //printRobotModelInfo(robotModelInfo_);
  mpcTimer2_.endTimer();

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
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
    if (activateSelfCollision) 
    {
      //ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode0_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      //ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode0_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileBaseDynamics" << std::endl;
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
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode1_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      //ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode1_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: RobotArmDynamics" << std::endl;
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
      ocp_.stateSoftConstraintPtr->add("selfCollision", selfCollisionConstraintPtr_mode2_);
    }
    mpcTimer7_.endTimer();

    mpcTimer8_.startTimer();
    if (activateExtCollision) 
    {
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE extCollisionConstraintPtr_mode2_" << std::endl;
      ocp_.stateSoftConstraintPtr->add("extCollision", extCollisionConstraintPtr_mode2_);
    }
    mpcTimer8_.endTimer();

    mpcTimer9_.startTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] dynamicsPtr: MobileManipulatorDynamics" << std::endl;
    ocp_.dynamicsPtr = dynamicsPtr_mode2_;
    mpcTimer9_.endTimer();
  }

  /*
   * Pre-computation
   */
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Pre-computation" << std::endl;
  if (usePreComputation_) 
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
    while(1);
    //ocp_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }
  //std::cout << "" << std::endl;

  // Rollout
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Rollout" << std::endl;
  mpcTimer10_.startTimer();
  rolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
  mpcTimer10_.endTimer();

  // Initialization
  mpcTimer11_.startTimer();
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE Initialization" << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] modeInputDim: " << modeInputDim << std::endl;
  initializerPtr_.reset(new DefaultInitializer(modeInputDim));
  mpcTimer11_.endTimer();

  // Update MPC Base
  if (updateMPCBaseFlag)
  {
    //mpc_->setOCPTemp(ocp_);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] MODE UPDATEEEEEEEEEEEEEEEEE: " << modeInputDim << std::endl;

    /*
    mpc_->initializeMPC(mpcSettings_, 
                        ddpSettings_, 
                        *rolloutPtr_, 
                        ocp_, 
                        *initializerPtr_);
    */
    //mpc->setInitializerTemp(initializerPtr_);
    //mpc->setRolloutTemp(rolloutPtr_);

    //mrt_->initRollout(&*rolloutPtr_);
  

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////// REVIEW

    /*
    mpc_.reset(new ocs2::GaussNewtonDDP_MPC(mpcSettings_, 
                                            ddpSettings_, 
                                            *rolloutPtr_, 
                                            ocp_, 
                                            *initializerPtr_));
    */

    mpc_->initializeMPC(mpcSettings_, 
                        ddpSettings_, 
                        *rolloutPtr_, 
                        ocp_, 
                        *initializerPtr_);

    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

    std::string topicPrefix = "mobile_manipulator_";
    //mpcNode_.reset(new ocs2::MPC_ROS_Interface(mpc_, topicPrefix));

    mpcNode_->setMPC(mpc_);
    

    //mpcNode_->setModelModeInt(getModelModeInt(robotModelInfo_));

    //mpcNode_->launchNodes(nodeHandle_);
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] AFTER MPC INITIALIZATION" << std::endl;

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] BEFORE MRT INITIALIZATION" << std::endl;
    //mrt_.reset(new MRT_ROS_Interface(robotModelInfo_, topicPrefix));
    //mrt_.reset(new MRT_ROS_Interface(topicPrefix));

    //mrt_->initRollout(&*rolloutPtr_);

    //mrt_->launchNodes(nodeHandle_);

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] model mode: " << getModelModeString(robotModelInfo_) << std::endl;
    mrt_loop_->setRobotModelInfo(robotModelInfo_);

    /*
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
    */

    mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo_));
    
    //mrt_loop_->subscribeObservers({mobileManipulatorVisu_});

    //currentTarget_.resize(7);
    //currentTarget_ << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    
    //mrt_loop_->setTargetReceivedFlag(true);

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] AFTER MRT INITIALIZATION" << std::endl;

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] DEBUG_INF" << std::endl;
    //while(1);

    ////////////////////////////////////////////////////////////////////  REVIEW
    ////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////// 

  }








  mpcProblemReadyFlag_ = true;

  mpcTimer0_.endTimer();

  /*
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
  */

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCProblem] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::launchNodes()
{
  if (drlFlag_)
  {
    /*
    if (drlActionType_ == 0)
    {
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes] DISCRETE ACTION" << std::endl;
      setActionDRLService_ = nodeHandle_.advertiseService(setActionDRLServiceName_, &MobileManipulatorInterface::setDiscreteActionDRLSrv, this);
    }
    else
    {
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes] CONTINUOUS ACTION" << std::endl;
      setActionDRLService_ = nodeHandle_.advertiseService(setActionDRLServiceName_, &MobileManipulatorInterface::setContinuousActionDRLSrv, this);
    }
    */

    //setTargetDRLClient_ = nodeHandle_.serviceClient<ocs2_msgs::setTask>(setTargetDRLServiceName_);
    //setMRTReadyClient_ = nodeHandle_.serviceClient<ocs2_msgs::setBool>(setMRTReadyServiceName_);
    //setMPCActionResultClient_ = nodeHandle_.serviceClient<ocs2_msgs::setMPCActionResult>(setMPCActionResultServiceName_);
  }
  else
  {
    // Subscribe Model Mode
    auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] START" << std::endl;

      // Shutdown MRT
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] mrtShutDownFlag true"  << std::endl;
      //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

      modelModeInt_ = msg->data;
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] modelModeInt_: " << modelModeInt_ << std::endl;

      mpcProblemReadyFlag_ = false;
      newMPCFlag_ = true;

      /*
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] WAITING mpcWaitingFlag and mrtWaitingFlag_..." << std::endl;
      while (!mpcWaitingFlag_ || !mrtWaitingFlag_){
        //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] mpcWaitingFlag_: " << mpcWaitingFlag_ << std::endl;
        //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] mrtWaitingFlag_: " << mrtWaitingFlag_ << std::endl;
        spinOnce();}

      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] BEFORE setMPCProblem" << std::endl;
      setMPCProblem(modelModeInt_, true, false, true);
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] AFTER setMPCProblem" << std::endl;
      
      newMPCFlag_ = false;
      */
      //setMPC();

      //mpcTimer3_.startTimer();
      //setMPCProblem();
      //mpcTimer3_.endTimer();

      //mpcProblemReadyFlag_ = true;
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] mrtShutDownFlag true"  << std::endl;
      //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

      //std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
      //std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
      //std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
      //std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::modelModeCallback] END" << std::endl;
      std::cout << "" << std::endl;
    };
    modelModeSubscriber_ = nodeHandle_.subscribe<std_msgs::UInt8>(modelModeMsgName_, 1, modelModeCallback);
  }

  baseTwistPub_ = nodeHandle_.advertise<geometry_msgs::Twist>(baseControlMsg_, 1);
  armJointTrajectoryPub_ = nodeHandle_.advertise<trajectory_msgs::JointTrajectory>(armControlMsg_, 1);
  armJointVelocityPub_ = nodeHandle_.advertise<kinova_msgs::JointVelocity>(ns_ + "/arm_controller/velocity", 1);

  // Clients
  computeCommandClient_ = nodeHandle_.serviceClient<ocs2_msgs::calculateMPCTrajectory>(computeCommandServiceName_);

  // Services
  calculateMPCTrajectoryService_ = nodeHandle_.advertiseService(calculateMPCTrajectoryServiceName_, &MobileManipulatorInterface::calculateMPCTrajectorySrv, this);

  //modelModeSubscriber_ = nodeHandle_.subscribe(model_mode_msg_name, 5, &MobileManipulatorInterface::modelModeCallback, this);

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] mpcTargetMsgName_: " << mpcTargetMsgName_ << std::endl;
  // TargetTrajectories

  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) 
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] START" << std::endl;
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

    if (currentTarget_.size() == targetTrajectories.stateTrajectory[0].size())
    {
      mpcNode_->setTargetTrajectories(targetTrajectories);

      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] currentTarget_.size(): " << currentTarget_.size() << std::endl;
      for (size_t i = 0; i < currentTarget_.size(); i++)
      {
        //std::cout << i << " -> " << currentTarget_[i] << std::endl;
        currentTarget_[i] = targetTrajectories.stateTrajectory[0][i];
      }

      //getEEPose(currentTarget_);
      //currentTarget_.resize(7);
      //currentTarget_ << -1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0;
    }
    else
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] ERROR: Size mismatch!" << std::endl;
      getEEPose(currentTarget_);
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] currentTarget_ IS EE !!!" << std::endl;
    }

    /*
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] currentTarget_.size(): " << currentTarget_.size() << std::endl;
    for (size_t i = 0; i < currentTarget_.size(); i++)
    {
      std::cout << i << " -> " << currentTarget_[i] << std::endl;
    }
    */
    
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] TARGET RECEIVED!" << std::endl;
    //targetReceivedFlag_ = true;

    //targetReadyFlag_ = true;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes::targetTrajectoriesCallback] END" << std::endl;
  };
  targetTrajectoriesSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(mpcTargetMsgName_, 5, targetTrajectoriesCallback);

  odomSubscriber_ = nodeHandle_.subscribe(odomMsgName_, 5, &MobileManipulatorInterface::odomCallback, this);
  jointStateSub_ = nodeHandle_.subscribe(armStateMsg_, 5, &MobileManipulatorInterface::jointStateCallback, this);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes] Waiting " << odomMsgName_ << " and " << armStateMsg_ << "..." << std::endl;
  while(!initFlagBaseState_ || !initFlagArmState_){ros::spinOnce();}
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::launchNodes] Received " << odomMsgName_ << " and " << armStateMsg_ << "!" << std::endl;

  updateStateIndexMap();
 
  //spin();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::getEEPose(vector_t& eePose)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEEPose] START" << std::endl;

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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEEPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void MobileManipulatorInterface::getGraspPose(vector_t& targetPose)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getGraspPose] START" << std::endl;

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
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLSrv] START" << std::endl;

  drlActionDiscrete_ = req.action;
  drlActionTimeHorizon_ = req.time_horizon;
  res.success = true;

  mapDiscreteActionDRL(drlActionDiscrete_);

  //mpcTimer3_.startTimer();
  //setMPCProblem();
  //mpcTimer3_.endTimer();

  //mpcProblemReadyFlag_ = true;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLSrv] mrtShutDownFlag true"  << std::endl;
  //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

  //std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  //std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  //std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  //std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLSrv] DEBUG INF" << std::endl;
  while(1);

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setDiscreteActionDRLSrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setContinuousActionDRLSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                                           ocs2_msgs::setContinuousActionDRL::Response &res)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLSrv] START" << std::endl;
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
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLSrv] mrtShutDownFlag true"  << std::endl;
  //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "true", 1);

  /*
  std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
  std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
  std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  */

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLSrv] END" << std::endl;
  //std::cout << "" << std::endl;
  
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLSrv] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setContinuousActionDRLSrv] END" << std::endl;
  return res.success;
}

bool MobileManipulatorInterface::calculateMPCTrajectorySrv(ocs2_msgs::calculateMPCTrajectory::Request &req, 
                                                           ocs2_msgs::calculateMPCTrajectory::Response &res)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] START" << std::endl;

  bool useCurrentPolicyFlag = req.use_current_policy_flag;

  double dt = req.dt;
  double time = req.time;
  //std::vector<double> state;
  //std::vector<double> full_state;
  //std::vector<double> input;
  
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
  //size_t modelModeInt = req.model_mode;
  //bool activateSelfCollision = req.self_collision_flag;
  //bool activateExtCollision = req.ext_collision_flag;
  
  std::vector<double> cmd;
  bool success = false;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] useCurrentPolicyFlag: " << useCurrentPolicyFlag << std::endl;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] time: " << currentObservation.time << std::endl;

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

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateMPCTrajectorySrv] END" << std::endl;
  return res.success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMPC()
{
  bool mpcPrintOutFlag = false;

  if (mpcPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] START" << std::endl;

  RobotModelInfo robotModelInfo;
  std::string mpcTopicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    mpcTopicPrefix = ns_ + "/";
  }

  mpcIter_ = 0;
  while (ros::ok() && ros::master::check())
  {
    if (mpcPrintOutFlag)
    {
      std::cout << "===================== MPC START =====================" << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] START ITERATION: " << mpcIter_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mpcIter_: " << mpcIter_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mrtIter_: " << mrtIter_ << std::endl;
    }

    /*
    if (mpcIter_ > 0)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] DEBUG INF " << std::endl;
      while(1);
    }
    */

    //calculateMPCTrajectory();

    // Wait to sync mpc and mrt
    if (mpcPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] Waiting for syncing mpcIter_: " << mpcIter_ << " and mrtIter_: " << mrtIter_ << "..." << std::endl;
    while(mpcIter_ != mrtIter_){
      spinOnce();
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mpcIter_: " << mpcIter_ << std::endl;
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mrtIter_: " << mrtIter_ << std::endl;
    }

    /// Make sure mpc exit! NUA NOTE: CHECK IF IT IS STILL NECESSARY? 
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] Waiting for mrtExitFlag_: " << mrtExitFlag_ << std::endl;
    //while(!mrtExitFlag_){spinOnce();}
    

    /// Make sure mrt exit! NUA NOTE: CHECK IF IT IS STILL NECESSARY? 
    //std::string mrtExitEnvStatus = getenv("mrtExitFlag");
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mrtExitEnvStatus: " << mrtExitEnvStatus << std::endl;
    //while(mrtExitEnvStatus == "false")
    //{
    //  mrtExitEnvStatus = getenv("mrtExitFlag");
    //}
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] MRT EXIT HAPPENED" << std::endl;

    if (drlFlag_)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] START DRL TRAINING!!!" << std::endl;
      while(!targetReceivedFlag_)
      {
        setMRTReady();
        spinOnce();
      }
    }

    //mpcTimer3_.startTimer();
    if (mpcPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE setMPCProblem" << std::endl;
    setMPCProblem(modelModeInt_, true, false);
    //mpcTimer3_.endTimer();
    //printRobotModelInfo(robotModelInfo_);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE Setting MPC Parameters " << std::endl;
    //mpcTimer4_.startTimer();
    robotModelInfo = robotModelInfo_;
    //OptimalControlProblem ocp;
    //ocp.swap(ocp_);
    /*
    if (mpcIter_ % 2 == 0)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] ocp1_" << std::endl;
      ocp.swap(ocp1_);
    }
    else
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] ocp2_" << std::endl;
      ocp.swap(ocp2_);
    }
    */
    //rolloutPtr_.reset(new TimeTriggeredRollout(*ocp.dynamicsPtr, rolloutSettings_));
    //initializerPtr_.reset(new DefaultInitializer(robotModelInfo.modeInputDim));
    mpcProblemReadyFlag_ = true;
    //mpcTimer4_.endTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] AFTER Setting MPC Parameters " << std::endl;

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr" << std::endl;
    // ROS ReferenceManager
    //mpcTimer5_.startTimer();
    //rosReferenceManagerPtr_ = std::shared_ptr<ocs2::RosReferenceManager>(new ocs2::RosReferenceManager(robotModelName_, referenceManagerPtr_));
    //mpcTimer5_.endTimer();

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE rosReferenceManagerPtr subscribe" << std::endl;
    //mpcTimer6_.startTimer();
    //rosReferenceManagerPtr_->subscribe(nodeHandle_);
    //mpcTimer6_.endTimer();

    // MPC
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE mpc" << std::endl;
    //mpcTimer7_.startTimer();
    setenv("mpcShutDownFlag", "false", 1);
    /*
    ocs2::GaussNewtonDDP_MPC mpc(mpcSettings_, 
                                 ddpSettings_, 
                                 *rolloutPtr_, 
                                 ocp_, 
                                 *initializerPtr_);
    */
    //mpcTimer7_.endTimer();

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE mpc setReferenceManager" << std::endl;
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

    // Launch MPC ROS node
    if (mpcPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode" << std::endl;
    //mpcTimer8_.startTimer();
    //MPC_ROS_Interface mpcNode(mpc, mpcTopicPrefix);
    //mpcNode_->setModelModeInt(getModelModeInt(robotModelInfo));
    //mpcTimer8_.endTimer();

    /*
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE currentTarget" << std::endl;
    vector_t currentTarget(7);
    currentTarget << -1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0;

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE getCurrentObservation" << std::endl;
    vector_t currentInput(8);
    currentInput.setZero();
    SystemObservation currentObservation = getCurrentObservation(currentInput);

    
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE TargetTrajectories" << std::endl;
    TargetTrajectories targetTrajectories({currentObservation.time}, {currentTarget}, {currentObservation.input});

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] currentObservation.mode: " << currentObservation.mode << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] currentObservation.time: " << currentObservation.time << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] currentObservation.state size: " << currentObservation.state.size() << std::endl << currentObservation.state << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl << currentObservation.full_state << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] currentObservation.input size: " << currentObservation.input.size() << std::endl << currentObservation.input << std::endl;

    mpcNode.setTargetTrajectories(targetTrajectories);
    mpcNode.setSystemObservation(currentObservation);
    */

    //mpcTimer1_.endTimer();

    if (printOutFlag_)
    {
      /*
      std::cout << '\n';
      std::cout << "\n### MPC_ROS Benchmarking mpcTimer1_";
      std::cout << "\n###   Maximum : " << mpcTimer1_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cout << "\n###   Average : " << mpcTimer1_.getAverageInMilliseconds() << "[ms].";
      std::cout << "\n###   Latest  : " << mpcTimer1_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;

      
      std::cout << "\n### MPC_ROS Benchmarking mpcTimer2_";
      std::cout << "\n###   Maximum : " << mpcTimer2_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cout << "\n###   Average : " << mpcTimer2_.getAverageInMilliseconds() << "[ms].";
      std::cout << "\n###   Latest  : " << mpcTimer2_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
      std::cout << "\n### MPC_ROS Benchmarking mpcTimer3_";
      std::cout << "\n###   Maximum : " << mpcTimer3_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cout << "\n###   Average : " << mpcTimer3_.getAverageInMilliseconds() << "[ms].";
      std::cout << "\n###   Latest  : " << mpcTimer3_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
      
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
      */
    }

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    mpcLaunchReadyFlag_ = true;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] AFTER mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;

    if (mpcPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] BEFORE mpc mpcNode launchNodes" << std::endl;
    //mpcExitFlag_ = false;
    //printRobotModelInfo(robotModelInfo);
    //mpcNode.launchNodes(nodeHandle_);
    //mpcExitFlag_ = true;
    mpcLaunchReadyFlag_ = false;
    if (mpcPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] AFTER mpc mpcNode launchNodes" << std::endl;


    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] DEBUG INF" << std::endl;
    //while(1);

    mpcIter_++;

    if (mpcPrintOutFlag)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] END ITERATION: " << mpcIter_ << std::endl;
      std::cout << "=====================================================" << std::endl;
      std::cout << "====================== MPC END ======================" << std::endl;
    }
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::setMPC()
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPC] START" << std::endl;
  setMPCProblem(modelModeInt_, true, false);

  //mpc_->setOCPTemp(ocp_);

  //mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo_));

  //mrt_loop_->setRobotModelInfo(robotModelInfo_);
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPC] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::runMRT()
{
  bool mrtPrintOutFlag = false;

  if (mrtPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] START" << std::endl;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] DEBUG INF" << std::endl;
  //while(1);

  RobotModelInfo robotModelInfo;
  std::string mrtTopicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    mrtTopicPrefix = ns_ + "/";
  }

  getEEPose(currentTarget_);

  vector_t currentInput(8);
  currentInput.setZero();

  mrtIter_ = 0;
  while (ros::ok() && ros::master::check())
  {
    if (mrtPrintOutFlag)
    {
      std::cout << "*********************** MRT START *******************" << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] mpcIter_: " << mpcIter_ << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] mrtIter_: " << mrtIter_ << std::endl;
    }

    // Wait for sync mpc and mrt
    while(mpcIter_ != mrtIter_){spinOnce();}
    
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    //while(!mpcLaunchReadyFlag_){spinOnce();}
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER mpcLaunchReadyFlag_: " << mpcLaunchReadyFlag_ << std::endl;
    //mpcLaunchReadyFlag_ = false;

    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] WAITING for mpcProblemReadyFlag_" << std::endl;
    while(!mpcProblemReadyFlag_){spinOnce();}
    mpcProblemReadyFlag_ = false;

    if (mrtPrintOutFlag)
    {
      std::cout << "*********************** MRT START *******************" << std::endl;
      std::cout << "*****************************************************" << std::endl;
      std::cout << "*****************************************************" << std::endl;
      std::cout << "*****************************************************" << std::endl;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] START ITERATION: " << mrtIter_ << std::endl;
    }

    //mrtTimer1_.startTimer();

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE setMPCProblem" << std::endl;
    //mrtTimer2_.startTimer();
    //setMPCProblem();
    //OptimalControlProblem ocp;
    //ocp.swap(ocp_);
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE TimeTriggeredRollout" << std::endl;
    //mrtRolloutPtr_.reset(new TimeTriggeredRollout(*ocp_.dynamicsPtr, rolloutSettings_));
    robotModelInfo = robotModelInfo_;
    //printRobotModelInfo(robotModelInfo); 
    //mrtTimer2_.endTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER setMPCProblem" << std::endl;

    // MRT
    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE mrt" << std::endl;
    //mrtTimer3_.startTimer();
    //MRT_ROS_Interface mrt(robotModelInfo, mrtTopicPrefix);
    MRT_ROS_Interface mrt(mrtTopicPrefix);
    //mrtTimer3_.endTimer();

    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE initRollout" << std::endl;
    //mrtTimer4_.startTimer();
    mrt.initRollout(&*rolloutPtr_);
    //mrtTimer4_.endTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER initRollout" << std::endl;

    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE launchNodes" << std::endl;
    //mrtTimer5_.startTimer();
    mrt.launchNodes(nodeHandle_);
    //mrtTimer5_.endTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER launchNodes" << std::endl;

    // MRT loop
    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE mrt_loop" << std::endl;
    //mrtTimer6_.startTimer();
    MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle_, 
                                 mrt, 
                                 robotModelInfo,
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
                                 logSavePathRel_);

    //mrtTimer6_.endTimer();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER mrt_loop" << std::endl;

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE subscribeObservers" << std::endl;
    mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo));
    mrt_loop.subscribeObservers({mobileManipulatorVisu_});
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER subscribeObservers" << std::endl;

    if (drlFlag_)
    {
      //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] Waiting for targetReceivedFlag_..." << std::endl;
      //setMRTReady();
      //while(!targetReceivedFlag_);
      //{
        //setMRTReady();
        //spinOnce();
      //}
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
    //currentTarget_.resize(7);
    //currentTarget_ << -1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0;

    vector_t currentTarget;
    currentTarget = currentTarget_;
    mrt_loop.setTargetReceivedFlag(true);
    //mrtTimer7_.endTimer();
    
    //mrtTimer1_.endTimer();
    if (printOutFlag_)
    {
      /*
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
      */
    }
    
    setenv("mrtExitFlag", "false", 1);
    //mrtExitFlag_ = false;
    mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);

    /*
    std::cout << "[MobileManipulatorInterface::runMRT] currentTarget size: " << currentTarget.size() << std::endl;
    for (size_t i = 0; i < currentTarget.size(); i++)
    {
      std::cout << i << " -> " << currentTarget[i] << std::endl;
    }
    */

    // Run mrt_loop
    if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] BEFORE run" << std::endl;
    mrt_loop.run(currentTarget);

    /*
    //vector_t currentInput(8);
    //currentInput.setZero();
    SystemObservation initObservation = getCurrentObservation(currentInput);

    const TargetTrajectories initTargetTrajectories({0}, {currentTarget}, {initObservation.input});

    referenceManagerPtr_->setTargetTrajectories(std::move(initTargetTrajectories));
    mrt_loop.computeCommand(currentTarget, initObservation);
    */

    //currentInput = mrt_loop.getCurrentInput(); 

    //mrtExitFlag_ = true;
    setenv("mpcShutDownFlag", "true", 1);

    if (drlFlag_)
    {
      setMPCActionResult(mrt_loop.getDRLActionResult());
    }

    /*
    if (false)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] AFTER currentTarget size: " << currentTarget.size() << std::endl;
      for (size_t i = 0; i < currentTarget.size(); i++)
      {
        std::cout << i << " -> " << currentTarget[i] << std::endl;
      }
      std::cout << "------------" << std::endl;
    }
    */

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] DEBUG INF" << std::endl;
    //while(1);

    mrtIter_++;

    if (mrtPrintOutFlag)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] END ITERATION: " << mrtIter_ << std::endl;
      std::cout << "*****************************************************" << std::endl;
      std::cout << "********************* MRT END ***********************" << std::endl;
    }

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] DEBUG INF" << std::endl;
    //while(1);
  }

  if (mrtPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMRT] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mpcCallback(const ros::TimerEvent& event)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] START" << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] mpcIter_: " << mpcIter_ << std::endl;

  /*
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] Waiting for syncing mpcIter_: " << mpcIter_ << " and mrtIter_: " << mrtIter_ << "..." << std::endl;
  while(mpcIter_ != mrtIter_){
    spinOnce();
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mpcIter_: " << mpcIter_ << std::endl;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::runMPC] mrtIter_: " << mrtIter_ << std::endl;
  }
  */
  //mpcNode_->launchNodes(nodeHandle_);
  //mpcNode_->spin();

  //mpcNode_->computeTrajectory();

  if (newMPCFlag_)
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] WAITING mrtWaitingFlag_..." << std::endl;
    while (!mrtWaitingFlag_){
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] mrtWaitingFlag_: " << mrtWaitingFlag_ << std::endl;
      //spinOnce();
      }

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE setMPCProblem" << std::endl;
    setMPCProblem(modelModeInt_, true, false, true);
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] AFTER setMPCProblem" << std::endl;
    
    newMPCFlag_ = false;

    mpcModeChangeCtr_++;
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] BEFORE computeTrajectory" << std::endl;
  mpcNode_->computeTrajectory();
  mpcNode_->singleSpin();

  spinOnce();

  if (mpcModeChangeCtr_ > 0)
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] DEBUG_INF" << std::endl;
    //while(1);
  }

  mpcIter_++;

  //runMPC();

  /*
  bool useCurrentPolicyFlag = false;

  vector_t currentInput;

  SystemObservation currentObservation = getCurrentObservation(currentInput);

  vector_t currentTarget(7);
  currentTarget << 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  bool setMPCProblemFlag = true;
  size_t inputModelModeInt = 2;
  bool activateSelfCollision = true;
  bool activateExtCollision = false;

  std::vector<double> cmd;
  
  bool success = calculateMPCTrajectory(useCurrentPolicyFlag,
                                        currentTarget,
                                        currentObservation,
                                        cmd,
                                        setMPCProblemFlag,
                                        inputModelModeInt,
                                        activateSelfCollision,
                                        activateExtCollision);
  */

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mrtCallback(const ros::TimerEvent& event)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] START" << std::endl;

  /*
  if (mrtPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] WAITING for mpcProblemReadyFlag_" << std::endl;
    while(!mpcProblemReadyFlag_){spinOnce();}
    mpcProblemReadyFlag_ = false;
  */

  if (newMPCFlag_)
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] Waiting mpcProblemReadyFlag_" << std::endl;
    mrtWaitingFlag_ = true;
    while (!mpcProblemReadyFlag_){
      mrtWaitingFlag_ = true;
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] mrtWaitingFlag_: " << mrtWaitingFlag_ << std::endl;
      //spinOnce();
    }
    mrtWaitingFlag_ = false;
    resetFlag_ = true;

    mrtModeChangeCtr_++;
  }

  if (resetFlag_)
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] mrtIter_: " << mrtIter_ << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] resetFlag: " << resetFlag_ << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] currentTarget_: " << std::endl;
    std::cout << currentTarget_ << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE run" << std::endl;
    resetFlag_ = mrt_loop_->run2(currentTarget_);
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] DEBUG_INF" << std::endl;
  //while(1);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] BEFORE mrtLoop2" << std::endl;
  if (!resetFlag_)
  {
    resetFlag_ = mrt_loop_->mrtLoop2();

    if (resetFlag_)
    {
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] mrtLoop2 STUCK!!!" << std::endl;
    }
  }
  else
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] run2 STUCK!!!" << std::endl;
  }

  spinOnce();

  //mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);

  if (mrtModeChangeCtr_ < 0)
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] DEBUG_INF" << std::endl;
    while(1);
  }

  mrtIter_++;

  //runMRT();
  
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mrtCallback] END" << std::endl;
}

/*
void MobileManipulatorInterface::mpcMRTCallback(const ros::TimerEvent& event)
{
  bool mpcMRTPrintOutFlag = true;

  /// SET MPC
  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] START MPC" << std::endl;

  std::string topicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    topicPrefix = ns_ + "/";
  }

  if (mpcMRTPrintOutFlag)
      std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE setMPCProblem" << std::endl;
  setMPCProblem(modelModeInt_, true, false);
  RobotModelInfo robotModelInfo = robotModelInfo_;

  ocs2::GaussNewtonDDP_MPC mpc(mpcSettings_, 
                                 ddpSettings_, 
                                 *rolloutPtr_, 
                                 ocp_, 
                                 *initializerPtr_);
  
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE mpc mpcNode" << std::endl;
  MPC_ROS_Interface mpcNode(mpc, topicPrefix);
  //mpcNode.setModelModeInt(getModelModeInt(robotModelInfo));

  /// SET MRT
  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] START MRT" << std::endl;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] DEBUG INF" << std::endl;
  //while(1);

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE MRT_ROS_Interface" << std::endl;
  MRT_ROS_Interface mrt(topicPrefix);
  //MRT_ROS_Interface mrt(robotModelInfo, topicPrefix);

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE initRollout" << std::endl;
  mrt.initRollout(&*rolloutPtr_);

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE mrt.launchNodes" << std::endl;
  mrt.launchNodes(nodeHandle_);

  // MRT loop
  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE mrt_loop" << std::endl;
  MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle_, 
                                mrt, 
                                robotModelInfo,
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
                                logSavePathRel_);

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE subscribeObservers" << std::endl;
  mobileManipulatorVisu_->updateModelMode(getModelModeInt(robotModelInfo));
  mrt_loop.subscribeObservers({mobileManipulatorVisu_});

  if (mpcMRTPrintOutFlag)
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] BEFORE mpcNode.launchNodes" << std::endl;
  mpcNode.launchNodes(nodeHandle_);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mpcMRTCallback] END" << std::endl;
}
*/

/*
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::calculateTrajectory(bool useCurrentPolicyFlag,
                                                        vector_t& currentTarget,
                                                        SystemObservation& currentObservation,
                                                        std::vector<double>& cmd,
                                                        bool setMPCProblemFlag,
                                                        size_t modelModeInt,
                                                        bool activateSelfCollision,
                                                        bool activateExtCollision)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] START" << std::endl;

  /// NUA NOTE: UPDATE IF NECESSARY!
  //RobotModelInfo robotModelInfo = robotModelInfo_;
  std::string topicPrefix = "mobile_manipulator_";
  if (ns_ != "/")
  {
    topicPrefix = ns_ + "/";
  }

  // NECESSARY INPUTS:
  // modelModeInt_
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE setMPCProblem" << std::endl;
  //if (setMPCProblemFlag)
  //{
  //  setMPCProblem(modelModeInt, activateSelfCollision, activateExtCollision);
  //}

  if (debugCtr_ == 0)
  {
    setMPCProblem(modelModeInt, activateSelfCollision, activateExtCollision);
  }

  /////////////////// SETTING MPC //////////// START
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE GaussNewtonDDP_MPC" << std::endl;
  ocs2::GaussNewtonDDP_MPC mpc(mpcSettings_,
                               ddpSettings_,
                               *rolloutPtr_,
                               ocp_,
                               *initializerPtr_);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE setReferenceManager" << std::endl;
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE MPC_ROS_Interface" << std::endl;
  MPC_ROS_Interface mpcNode(mpc, topicPrefix);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE setModelModeInt" << std::endl;
  //mpcNode.setModelModeInt(modelModeInt_);
  
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE spin" << std::endl;
  //spinOnce();

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE currentTarget" << std::endl;
  //vector_t currentTarget(7);
  //currentTarget << 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE getCurrentObservation" << std::endl;
  //SystemObservation currentObservation = getCurrentObservation();

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE TargetTrajectories" << std::endl;
  TargetTrajectories targetTrajectories({currentObservation.time}, {currentTarget}, {currentObservation.input});

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] currentObservation.mode: " << currentObservation.mode << std::endl;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] currentObservation.time: " << currentObservation.time << std::endl;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] currentObservation.state size: " << currentObservation.state.size() << std::endl << currentObservation.state << std::endl;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl << currentObservation.full_state << std::endl;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] currentObservation.input size: " << currentObservation.input.size() << std::endl << currentObservation.input << std::endl;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] DEBUF_INF" << std::endl;
  //while(1);
  //spinOnce();
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] debugCtr_: " << debugCtr_ << std::endl;
  bool flag_reset = false;
  double time = 0;
  double dt = 1.0 / mpcSettings_.mrtDesiredFrequency_;
  //double dt = 0.05;
  ros::Rate simRate(mpcSettings_.mrtDesiredFrequency_);
  vector_t currentInput(8);
  currentInput.setZero();
  while (ros::ok)
  {
    SystemObservation currentObservation = getCurrentObservation(currentInput ,time);
    TargetTrajectories targetTrajectories({currentObservation.time}, {currentTarget}, {currentObservation.input});

    if (debugCtr_ == 0)
    {
      mpcNode.computeTraj2(targetTrajectories, currentObservation, true);
    }
    else
    {
      mpcNode.computeTraj2(targetTrajectories, currentObservation, flag_reset);
    }

    PrimalSolution currentPolicy = mpcNode.getPolicy();
    
    currentInput = currentPolicy.getDesiredInput(time);
    vector_t nextState = currentPolicy.getDesiredState(time+dt);
    mobileManipulatorVisu_->update(currentObservation, currentPolicy);

    std::vector<double> cmd;
    computeCommand(currentInput, nextState, cmd);

    time += dt;
    debugCtr_++;
    
    ros::spinOnce();
    simRate.sleep();
  }

  /// NUA TODO: Set it false, if it exceeds the allocated time limit!
  bool success = true;

  /*
  for (size_t i = 0; i < 10; i++)
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE getTargetTrajectories: " << referenceManagerPtr_->getTargetTrajectories().stateTrajectory.size() << std::endl;
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] BEFORE computeTraj2" << std::endl << std::endl;

    mpcNode.computeTraj2(targetTrajectories, currentObservation, flag_reset);
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] AFTER getTargetTrajectories: " << referenceManagerPtr_->getTargetTrajectories().stateTrajectory.size() << std::endl;

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] DEBUF_INF" << std::endl;
    //while(1);
  }
  * /
  
  //spinOnce();
  /////////////////// SETTING MPC //////////// END

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] DEBUF_INF" << std::endl;
  //while(1);

  // ---------------------------------------------------

  //double dt = 1.0 / mpcSettings_.mrtDesiredFrequency_;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] dt: " << dt << std::endl;
  
  //PrimalSolution currentPolicy = mpcNode.getPolicy();
  //vector_t currentInput_ = currentPolicy.getDesiredInput(dt);

  //std::vector<double> cmd;

  //mobileManipulatorVisu_->update(currentObservation, currentPolicy);

  //computeCommand(currentPolicy, currentObservation, cmd);

  /*
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] cmd:" << std::endl;
  for (size_t i = 0; i < cmd.size(); i++)
  {
    std::cout << i << " -> " << cmd[i] << std::endl;
  }
  * /
  

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] DEBUG_INF" << std::endl;
  //while(1);

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::calculateTrajectory] END" << std::endl;

  debugCtr_++;

  return success;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::computeCommand(vector_t& currentInput,
                                                vector_t& nextState,
                                                std::vector<double>& cmd)
{
  //std::cout << "[MobileManipulatorInterface::computeCommand] START" << std::endl;

  //std::vector<double> cmd;
  cmd.clear();
  geometry_msgs::Twist baseTwistMsg;
  trajectory_msgs::JointTrajectory armJointTrajectoryMsg;
  kinova_msgs::JointVelocity armJointVelocityMsg;

  //vector_t currentInput = currentPolicy.getDesiredInput(0);
  double dt = 1.0 / mpcSettings_.mrtDesiredFrequency_;
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::computeCommand] dt: " << dt << std::endl;

  // Set mobile base command
  if (robotModelInfo_.modelMode == ModelMode::BaseMotion || 
      robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
  {
    baseTwistMsg.linear.x = currentInput[0];
    baseTwistMsg.angular.z = currentInput[1];
    
    // Cut-off velocity
    if(abs(abs(baseTwistMsg.linear.x) - abs(prev_lin_x_)) > lin_x_cutoff_) 
    {
      baseTwistMsg.linear.x = prev_lin_x_ + copysign(1, baseTwistMsg.linear.x - prev_lin_x_) * lin_x_cutoff_;
    }
    
    if(abs(abs(baseTwistMsg.angular.z) - abs(prev_ang_z_)) > ang_z_cutoff_) 
    {
      baseTwistMsg.angular.z = prev_ang_z_ + copysign(1, baseTwistMsg.angular.z - prev_ang_z_) * ang_z_cutoff_;
    }
    prev_lin_x_ = baseTwistMsg.linear.x;
    prev_ang_z_ = baseTwistMsg.angular.z;
    
    cmd.push_back(baseTwistMsg.linear.x);
    cmd.push_back(baseTwistMsg.angular.z);
  }

  // Set arm command
  if (robotModelInfo_.modelMode == ModelMode::ArmMotion || 
      robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
  {
    int baseOffset = 0;
    if (robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
    {
      baseOffset = robotModelInfo_.mobileBase.stateDim;
    }
    int n_joints = robotModelInfo_.robotArm.jointNames.size();
    armJointTrajectoryMsg.joint_names.resize(n_joints);

    //PrimalSolution primalSolution = mrt_.getPolicy();
    //PrimalSolution primalSolution = currentPolicy;
    //auto nextState = currentPolicy.getDesiredState(dt);

    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.resize(n_joints);
    jtp.time_from_start = ros::Duration(dt);

    for (int i = 0; i < n_joints; ++i)
    {
      armJointTrajectoryMsg.joint_names[i] = robotModelInfo_.robotArm.jointNames[i];
      jtp.positions[i] = nextState[baseOffset + i];

      cmd.push_back(jtp.positions[i]);
    }
    armJointTrajectoryMsg.points.push_back(jtp);
  }

  // Publish command
  if (robotModelInfo_.modelMode == ModelMode::BaseMotion || 
      robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
  {
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] BASE PUB" << std::endl;
    baseTwistPub_.publish(baseTwistMsg);
  }

  if (robotModelInfo_.modelMode == ModelMode::ArmMotion || 
      robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
  {
    armJointVelocityMsg.joint1 = currentInput[2] * (180.0/M_PIf32);
    armJointVelocityMsg.joint2 = currentInput[3] * (180.0/M_PIf32);
    armJointVelocityMsg.joint3 = currentInput[4] * (180.0/M_PIf32);
    armJointVelocityMsg.joint4 = currentInput[5] * (180.0/M_PIf32);
    armJointVelocityMsg.joint5 = currentInput[6] * (180.0/M_PIf32);
    armJointVelocityMsg.joint6 = currentInput[7] * (180.0/M_PIf32);
    //std::cout << "[MRT_ROS_Gazebo_Loop::publishCommand] ARM PUB" << std::endl;
    
    armJointTrajectoryPub_.publish(armJointTrajectoryMsg);
    armJointVelocityPub_.publish(armJointVelocityMsg);
  }
  
  //std::cout << "[MobileManipulatorInterface::computeCommand] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorInterface::computeCommand] END" << std::endl << std::endl;
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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modelMode: " << modelMode << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] modeInputDim: " << modeInputDim << std::endl;

  matrix_t R = matrix_t::Zero(modeInputDim, modeInputDim);

  // Input cost of mobile base
  if (modelMode == 0 || modelMode == 2) 
  {
    const size_t inputDimBase = getInputDimBase(robotModelInfo_);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] inputDimBase: " << inputDimBase << std::endl;

    matrix_t R_base = matrix_t::Zero(inputDimBase, inputDimBase);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.base", R_base, printOutFlag_);
    R.topLeftCorner(inputDimBase, inputDimBase) = R_base;
  }

  // Input cost of arm
  if (modelMode == 1 || modelMode == 2) 
  {
    const size_t inputDimArm = getInputDimArm(robotModelInfo_);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getQuadraticInputCost] inputDimArm: " << inputDimArm << std::endl;

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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] modeStateDim: " << modeStateDim << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] modeInputDim: " << modeInputDim << std::endl;

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit & modelMode != 0) 
  {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const size_t stateDimArm = getStateDimArm(robotModelInfo_);

    // Arm joint DOF limits from the parsed URDF      
    const vector_t lowerBound = model.lowerPositionLimit.tail(stateDimArm);
    const vector_t upperBound = model.upperPositionLimit.tail(stateDimArm);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;

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

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateOffset: " << stateOffset << std::endl;

    for (int i = 0; i < stateDimArm; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = stateOffset + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits.size(): " << stateLimits.size() << std::endl;
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

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    //std::cout << lowerBound << std::endl << std::endl;
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    //std::cout << upperBound << std::endl << std::endl;
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits size: " << stateLimits.size() << std::endl;
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] inputLimits size: " << inputLimits.size() << std::endl;

  auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));

  boxConstraints->initializeOffset(0.0, vector_t::Zero(modeStateDim), vector_t::Zero(modeInputDim));

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getJointLimitSoftConstraint] DEBUG INF" << std::endl;
  //while(1);

  return boxConstraints;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const std::string& prefix) 
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] START prefix: " << prefix << std::endl;

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
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
    while(1);

    MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(*pinocchioInterfacePtr_, pinocchioMapping, {robotModelInfo_.robotArm.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } 
  else 
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] BEFORE pinocchioMappingCppAd" << std::endl;
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(robotModelInfo_);

    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] BEFORE eeKinematics" << std::endl;
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_,
                                                     pinocchioMappingCppAd, 
                                                     robotModelInfo_,
                                                     modelName, 
                                                     libraryFolder_, 
                                                     recompileLibraries_, 
                                                     false);
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] AFTER eeKinematics" << std::endl;
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, robotModelInfo_));
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
  //while(1);

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getEndEffectorConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const std::string& prefix) 
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] START" << std::endl;

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
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] DEBUG INF" << std::endl;
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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getSelfCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::unique_ptr<StateCost> MobileManipulatorInterface::getExtCollisionConstraint(const std::string& prefix) 
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] START" << std::endl;

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
    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ true" << std::endl;

    std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] DEBUG INF" << std::endl;
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
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] usePreComputation_ false" << std::endl;

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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::getExtCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::updateStateIndexMap()
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] START" << std::endl;
  
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
        std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] joint name: " << *it << " -> index: " << index << std::endl;
        stateIndexMap_.push_back(index);
    }
    else
    {
      throw std::runtime_error("[" + ns_ +  "][MRT_ROS_Gazebo_Loop::updateStateIndexMap] Error: Joint " + jointNames[i] + " not found!");
    }
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::updateStateIndexMap] END" << std::endl;
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
bool MobileManipulatorInterface::setTargetDRL(double x, double y, double z, double roll, double pitch, double yaw)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] START" << std::endl;

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

  targetReceivedFlag_ = true;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setTargetDRL] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapDiscreteActionDRL(int action)
{
  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] START" << std::endl;

  /// NUA TODO: NEEDS DEBUGGING!
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] DEBUG_INF" << std::endl;
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
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] " << tmp_name << ": " << tmp_val << std::endl;
  }

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapDiscreteActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorInterface::mapContinuousActionDRL(std::vector<double>& action)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] START" << std::endl;

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
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] BASE MOTION" << std::endl;
    target_roll = 0.0;
    target_pitch = 0.0;
    target_z = 0.12;
    modelModeInt_ = 0;
  }
  else if (modelModeProb > 0.6)
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] WHOLE-BODY MOTION" << std::endl;
    modelModeInt_ = 2;
  }
  else
  {
    //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] ARM MOTION" << std::endl;
    modelModeInt_ = 1;
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

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::mapContinuousActionDRL] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMRTReady()
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMRTReady] START" << std::endl;
  bool success = false;
  ocs2_msgs::setBool srv;
  srv.request.val = true;

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMRTReady] Waiting for the service..." << std::endl;
  ros::service::waitForService(setMRTReadyServiceName_);
  if (setMRTReadyClient_.call(srv))
  {
    success = srv.response.success;
  }
  else
  {
    ROS_ERROR("[MobileManipulatorInterface::setMRTReady] ERROR: Failed to call service!");
    success = false;
  }

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMRTReady] END" << std::endl;
  
  return success;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MobileManipulatorInterface::setMPCActionResult(int drlActionResult)
{
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] START" << std::endl;

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] drlActionResult: " << drlActionResult << std::endl;

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

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::setMPCActionResult] END" << std::endl;
  
  return success;
}

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
  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] START" << std::endl;
  
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

  //std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] Waiting for the service..." << std::endl;
  ros::service::waitForService(computeCommandServiceName_);
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

  std::cout << "[" << ns_ <<  "][MobileManipulatorInterface::computeCommandClient] END" << std::endl;
  
  return success;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
