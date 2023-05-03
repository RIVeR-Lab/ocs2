// LAST UPDATE: 2022.04.27
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile, 
                                                       const std::string& libraryFolder,
                                                       const std::string& urdfFile,
                                                       PointsOnRobot::points_radii_t pointsAndRadii)
  : taskFile_(taskFile), libraryFolder_(libraryFolder), urdfFile_(urdfFile)
{
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START" << std::endl;

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
  std::cerr << "[MobileManipulatorInterface::MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  /// Read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  
  /// Resolve meta-information about the model
  // Read robot type
  RobotModelType robotModelType = loadRobotType(taskFile_, "model_information.robotModelType");

  // Read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.removeJoints", removeJointNames, false);
  
  // Read the link names of joints
  std::vector<std::string> armJointFrameNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointFrameNames", armJointFrameNames, false);

  // Read the names of joints
  std::vector<std::string> armJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.armJointNames", armJointNames, false);

  // Read the frame names
  std::string baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

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
  std::cerr << " #### =============================================================================" << std::endl;

  // Create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames)));

  std::cerr << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  robotModelInfo_ = createRobotModelInfo(robotModelType,
                                         baseFrame, 
                                         armBaseFrame, 
                                         eeFrame,
                                         armJointFrameNames,
                                         armJointNames);

  // Set Model Settings
  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", true);
  loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", true);
  loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", true);
  loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", false);
  std::cerr << " #### =============================================================================\n";

  // Set DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc");

  // Set Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  // NUA TODO: We don't need to set it here!
  //int modelMode = getModelModeInt(robotModelInfo_);
  auto modelModeInt = 1;
  setMPCProblem(modelModeInt, pointsAndRadii);
  
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::setMPCProblem(size_t modelModeInt, PointsOnRobot::points_radii_t& pointsAndRadii)
{
  std::cout << "[MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  std::cout << "[MobileManipulatorInterface::setMPCProblem] modelMode: " << modelModeInt << std::endl;

  bool isModeUpdated = updateModelMode(robotModelInfo_, modelModeInt);

  printRobotModelInfo(robotModelInfo_);

  //std::cout << "[MobileManipulatorInterface::setMPCProblem] DEBUG INF" << std::endl;
  //while(1);

  //// Optimal control problem
  /// Cost
  problem_.costPtr->add("inputCost", getQuadraticInputCost());
  std::cout << "" << std::endl;

  /// Constraints
  // Joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint());
  std::cout << "" << std::endl;

  // Mobile base or End-effector state constraint
  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getEndEffectorConstraint" << std::endl;
  problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint("endEffector"));
  problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint("finalEndEffector"));
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getSelfCollisionConstraint" << std::endl;
  // Self-collision avoidance constraint
  if (activateSelfCollision_) 
  {
    if (robotModelInfo_.modelMode == ModelMode::ArmMotion || robotModelInfo_.modelMode == ModelMode::WholeBodyMotion)
    {
      problem_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint("selfCollision"));
    }
  }
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE getExtCollisionConstraint" << std::endl;
  // External-collision avoidance constraint
  //activateExtCollision_ = false;
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
    string world_frame_name = "world";
    string pub_name_oct_dist_visu = "occ_dist";
    string pub_name_oct_dist_array_visu = "occ_dist_array";

    emuPtr_->setWorldFrameName(world_frame_name);
    emuPtr_->setPubOccDistVisu(pub_name_oct_dist_visu);
    emuPtr_->setPubOccDistArrayVisu(pub_name_oct_dist_array_visu);
    
    problem_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint("extCollision"));
  }
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE Dynamics" << std::endl;
  // Dynamics
  switch (robotModelInfo_.modelMode) 
  {
    case ModelMode::BaseMotion:
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Base" << std::endl;
      problem_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        true));
      break;
    }
    
    case ModelMode::ArmMotion:
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Robotics Arm" << std::endl;
      problem_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      true));
      break;
    }
    
    case ModelMode::WholeBodyMotion: 
    {
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Manipulator" << std::endl;
      problem_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               true));
      break;
    }

    default:
      throw std::invalid_argument("[MobileManipulatorInterface::setMPCProblem] ERROR: Invalid model mode!");
  }
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Pre-computation" << std::endl;
  /*
   * Pre-computation
   */
  if (usePreComputation_) 
  {
    std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START preComputationPtr" << std::endl;
    while(1);
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }
  std::cout << "" << std::endl;

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Rollout" << std::endl;

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  auto modeInputDim = getModeInputDim(robotModelInfo_);
  initializerPtr_.reset(new DefaultInitializer(modeInputDim));

  //std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorInterface::setMPCProblem] END" << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  std::cout << "[MobileManipulatorInterface::tfCallback] START" << std::endl;

  tf2_msgs::TFMessage tf_msg = *msg;

  std::cout << "[MobileManipulatorInterface::tfCallback] END" << std::endl << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::launchNodes(ros::NodeHandle& nodeHandle)
{
  string oct_msg_name = "octomap_scan";
  string tf_msg_name = "tf";
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

  //sub_tf_msg_ = nodeHandle.subscribe(tf_msg_name, 10, &MobileManipulatorInterface::tfCallback, this);

  //spin();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost() 
{
  //const size_t modeStateDim = getStateDimTmp(robotModelInfo_);
  const size_t modeStateDim = getModeStateDim(robotModelInfo_);
  
  //const size_t modeInputDim = getInputDim(robotModelInfo_);
  const size_t modeInputDim = getModeInputDim(robotModelInfo_);
  
  auto modelMode = getModelModeInt(robotModelInfo_);

  std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modelMode: " << modelMode << std::endl;
  std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modeStateDim: " << modeStateDim << std::endl;
  std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] modeInputDim: " << modeInputDim << std::endl;

  matrix_t R = matrix_t::Zero(modeInputDim, modeInputDim);

  // Input cost of mobile base
  if (modelMode == 0 || modelMode == 2) 
  {
    const size_t inputDimBase = getInputDimBase(robotModelInfo_);

    std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] inputDimBase: " << inputDimBase << std::endl;

    matrix_t R_base = matrix_t::Zero(inputDimBase, inputDimBase);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.base", R_base);
    R.topLeftCorner(inputDimBase, inputDimBase) = R_base;
  }

  // Input cost of arm
  if (modelMode == 1 || modelMode == 2) 
  {
    const size_t inputDimArm = getInputDimArm(robotModelInfo_);

    std::cout << "[MobileManipulatorInterface::getQuadraticInputCost] inputDimArm: " << inputDimArm << std::endl;

    matrix_t R_arm = matrix_t::Zero(inputDimArm, inputDimArm);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.arm", R_arm);
    R.bottomRightCorner(inputDimArm, inputDimArm) = R_arm;
  }

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), modeStateDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint() 
{
  auto modelMode = getModelModeInt(robotModelInfo_);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  //const size_t modeStateDim = getStateDimTmp(robotModelInfo_);
  const size_t modeStateDim = getModeStateDim(robotModelInfo_);
  
  //const size_t modeInputDim = getInputDim(robotModelInfo_);
  const size_t modeInputDim = getModeInputDim(robotModelInfo_);

  std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] modeStateDim: " << modeStateDim << std::endl;
  std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] modeInputDim: " << modeInputDim << std::endl;

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit & modelMode != 0) 
  {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const size_t stateDimArm = getStateDimArm(robotModelInfo_);

    // Arm joint DOF limits from the parsed URDF      
    const vector_t lowerBound = model.lowerPositionLimit.tail(stateDimArm);
    const vector_t upperBound = model.upperPositionLimit.tail(stateDimArm);

    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;

    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    std::cerr << "\n #### ArmJointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";
    
    stateLimits.reserve(modeStateDim);
    const size_t stateOffset = modeStateDim - stateDimArm;

    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateOffset: " << stateOffset << std::endl;

    for (int i = 0; i < stateDimArm; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = stateOffset + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }

    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    std::cout << lowerBound << std::endl << std::endl;
    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    std::cout << upperBound << std::endl << std::endl;

    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits.size(): " << stateLimits.size() << std::endl;
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
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.base", lowerBoundBase);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.base", upperBoundBase);
      
      lowerBound.head(inputDimBase) = lowerBoundBase;
      upperBound.head(inputDimBase) = upperBoundBase;
    }

    if (modelMode == 1 || modelMode == 2)
    {
      const size_t inputDimArm = getInputDimArm(robotModelInfo_);

      // Arm joint DOFs velocity limits
      vector_t lowerBoundArm = vector_t::Zero(inputDimArm);
      vector_t upperBoundArm = vector_t::Zero(inputDimArm);
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.arm", upperBoundArm);
      
      lowerBound.tail(inputDimArm) = lowerBoundArm;
      upperBound.tail(inputDimArm) = upperBoundArm;
    }

    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;
    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

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

    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] lowerBound size: " << lowerBound.size() << std::endl;
    std::cout << lowerBound << std::endl << std::endl;
    std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] upperBound size: " << upperBound.size() << std::endl;
    std::cout << upperBound << std::endl << std::endl;
  }

  std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] stateLimits size: " << stateLimits.size() << std::endl;
  std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] inputLimits size: " << inputLimits.size() << std::endl;

  auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));

  std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] HELLO" << std::endl;

  boxConstraints -> initializeOffset(0.0, vector_t::Zero(modeStateDim), vector_t::Zero(modeInputDim));

  //std::cout << "[MobileManipulatorInterface::getJointLimitSoftConstraint] DEBUG INF" << std::endl;
  //while(1);

  return boxConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const std::string& prefix) 
{
  std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] START prefix: " << prefix << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  //const int stateDim = getStateDim(robotModelInfo_);
  //const int modeStateDim = getModeStateDim(robotModelInfo_);
  //const int modeInputDim = getModeInputDim(robotModelInfo_);

  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

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
    std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] BEFORE pinocchioMappingCppAd" << std::endl;
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(robotModelInfo_);

    std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] BEFORE eeKinematics" << std::endl;
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_,
                                                     pinocchioMappingCppAd, 
                                                     robotModelInfo_,
                                                     "end_effector_kinematics", 
                                                     libraryFolder_, 
                                                     recompileLibraries_, 
                                                     false);
    std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] AFTER eeKinematics" << std::endl;
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_, robotModelInfo_));
  }

  //std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] DEBUG INF" << std::endl;
  //while(1);

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const std::string& prefix) 
{
  std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.2;
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;

  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr_, collisionLinkPairs, collisionObjectPairs);
  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "[MobileManipulatorInterface::getSelfCollisionConstraint] Testing for " << numCollisionPairs << " collision pairs\n";

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

  std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
