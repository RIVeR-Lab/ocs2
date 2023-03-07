// LAST UPDATE: 2022.03.04
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
  RobotModelType robotModelType = mobile_manipulator::loadRobotType(taskFile_, "model_information.robotModelType");

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
  std::cout << "" << std::endl;
  std::cout << "*********************************" << std::endl;
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] START PinocchioInterface" << std::endl;
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, robotModelType, removeJointNames)));
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END PinocchioInterface" << std::endl;
  std::cout << "*********************************" << std::endl;
  std::cout << "" << std::endl;
  std::cerr << *pinocchioInterfacePtr_;

  // Set Robot Model Info
  robotModelInfo_ = mobile_manipulator::createRobotModelInfo(*pinocchioInterfacePtr_, 
                                                             robotModelType,
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

  // NUA TODO: We don't need to do it here!
  setMPCProblem(2, pointsAndRadii);
  
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::setMPCProblem(size_t modelMode, PointsOnRobot::points_radii_t& pointsAndRadii)
{
  std::cout << "[MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  bool isModeUpdated = updateModelMode(robotModelInfo_, modelMode);

  //// Optimal control problem
  /// Cost
  problem_.costPtr->add("inputCost", getQuadraticInputCost(modelMode));

  /// Constraints
  // Joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(modelMode));

  // Mobile base or End-effector state constraint
  if (modelMode == 0)
  {
    ///////// NUA TODO: COMPLETE!
  }
  else
  {
    problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint("endEffector"));
    problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint("finalEndEffector"));
  }

  // Self-collision avoidance constraint
  if (activateSelfCollision_) 
  {
    if (modelMode == 1 || modelMode == 2)
    {
      problem_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint("selfCollision"));
    }
  }

  // External-collision avoidance constraint
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
                                    "points_on_robot",
                                    libraryFolder_,
                                    recompileLibraries_,
                                    false,
                                    robotModelInfo_.mobileBase.baseFrame,
                                    robotModelInfo_.robotArm.baseFrame,
                                    robotModelInfo_.robotArm.eeFrame,
                                    robotModelInfo_.robotArm.jointFrameNames);
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
    
    problem_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint("extCollision", modelMode));
  }

  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE Dynamics" << std::endl;
  
  // Dynamics
  switch (modelMode) 
  {
    case 0:
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Base" << std::endl;
      problem_.dynamicsPtr.reset(new MobileBaseDynamics(robotModelInfo_, 
                                                        "MobileBaseDynamics", 
                                                        libraryFolder_, 
                                                        recompileLibraries_, 
                                                        true));
      break;

    case 1:
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Robotics Arm" << std::endl;
      problem_.dynamicsPtr.reset(new RobotArmDynamics(robotModelInfo_, 
                                                      "RobotArmDynamics", 
                                                      libraryFolder_, 
                                                      recompileLibraries_, 
                                                      true));
      break;

    case 2: 
      std::cout << "[MobileManipulatorInterface::setMPCProblem] Mobile Manipulator" << std::endl;
      problem_.dynamicsPtr.reset(new MobileManipulatorDynamics(robotModelInfo_, 
                                                               "MobileManipulatorDynamics", 
                                                               libraryFolder_, 
                                                               recompileLibraries_, 
                                                               true));
      break;

    default:
      throw std::invalid_argument("[MobileManipulatorInterface::setMPCProblem] ERROR: Invalid model mode!");
  }

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Pre-computation" << std::endl;

  /*
   * Pre-computation
   */
  if (usePreComputation_) 
  {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, robotModelInfo_));
  }

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Rollout" << std::endl;

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  auto modelInputDim = getInputDim(robotModelInfo_);
  initializerPtr_.reset(new DefaultInitializer(modelInputDim));

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
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost(size_t modelMode) 
{
  const size_t modelStateDim = getStateDim(robotModelInfo_);
  const size_t modelInputDim = getInputDim(robotModelInfo_);
  const size_t modelBaseInputDim = getInputDimBase(robotModelInfo_);
  const size_t modelArmInputDim = getInputDimArm(robotModelInfo_);

  matrix_t R = matrix_t::Zero(modelInputDim, modelInputDim);

  // Input cost of mobile base
  if (modelMode == 0 || modelMode == 2) 
  {
    matrix_t R_base = matrix_t::Zero(modelBaseInputDim, modelBaseInputDim);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.base", R_base);
    R.topLeftCorner(modelBaseInputDim, modelBaseInputDim) = R_base;
  }

  // Input cost of arm
  if (modelMode == 1 || modelMode == 2) 
  {
    // arm joints DOFs input costs
    matrix_t R_arm = matrix_t::Zero(modelArmInputDim, modelArmInputDim);
    loadData::loadEigenMatrix(taskFile_, "inputCost.R.arm", R_arm);
    R.bottomRightCorner(modelArmInputDim, modelArmInputDim) = R_arm;
  }

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), modelStateDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint(size_t modelMode) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int modelBaseStateDim = getStateDimBase(robotModelInfo_);
  const int modelArmStateDim = getStateDimArm(robotModelInfo_);
  const int modelStateDim = getStateDim(robotModelInfo_);

  const int modelBaseInputDim = getInputDimBase(robotModelInfo_);
  const int modelArmInputDim = getInputDimArm(robotModelInfo_);
  const int modelInputDim = getInputDim(robotModelInfo_);

  const auto& model = pinocchioInterfacePtr_->getModel();

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) 
  {
    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    stateLimits.reserve(modelStateDim);

    if (modelMode == 1 || modelMode == 2)
    {
      // Arm joint DOF limits from the parsed URDF      
      const vector_t lowerBound = model.lowerPositionLimit.tail(modelArmStateDim);
      const vector_t upperBound = model.upperPositionLimit.tail(modelArmStateDim);

      std::cerr << "\n #### ArmJointPositionLimits Settings: ";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
      std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
      loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
      loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
      std::cerr << " #### =============================================================================\n";
      
      for (int i = 0; i < modelArmStateDim; ++i) 
      {
        StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
        boxConstraint.index = modelBaseStateDim + i;
        boxConstraint.lowerBound = lowerBound(i);
        boxConstraint.upperBound = upperBound(i);
        boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
        stateLimits.push_back(std::move(boxConstraint));
      }
    }
    else
    {
      // No joint position limit for the mobile base
      stateLimits.clear();
    }
  }

  // Load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(modelInputDim);
    vector_t upperBound = vector_t::Zero(modelInputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    if (modelMode == 0 || modelMode == 2)
    {
      // Mobile base joint DOFs velocity limits
      vector_t lowerBoundBase = vector_t::Zero(modelBaseInputDim);
      vector_t upperBoundBase = vector_t::Zero(modelBaseInputDim);
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.base", lowerBoundBase);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.base", upperBoundBase);
      
      lowerBound.head(modelBaseInputDim) = lowerBoundBase;
      upperBound.head(modelBaseInputDim) = upperBoundBase;
    }

    if (modelMode == 1 || modelMode == 2)
    {
      // Arm joint DOFs velocity limits
      vector_t lowerBoundArm = vector_t::Zero(modelArmInputDim);
      vector_t upperBoundArm = vector_t::Zero(modelArmInputDim);
      
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
      loadData::loadEigenMatrix(taskFile_, "jointVelocityLimits.upperBound.arm", upperBoundArm);
      
      lowerBound.tail(modelArmInputDim) = lowerBoundArm;
      upperBound.tail(modelArmInputDim) = upperBoundArm;
    }

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(modelInputDim);
    for (int i = 0; i < modelInputDim; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::unique_ptr<StateInputSoftBoxConstraint>(new StateInputSoftBoxConstraint(stateLimits, inputLimits));
  boxConstraints -> initializeOffset(0.0, vector_t::Zero(modelStateDim), vector_t::Zero(modelInputDim));

  return boxConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const std::string& prefix) 
{
  std::cout << "[MobileManipulatorInterface::getEndEffectorConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  const int modelBaseStateDim = getStateDimBase(robotModelInfo_);
  const int modelArmStateDim = getStateDimArm(robotModelInfo_);
  const int modelStateDim = getStateDim(robotModelInfo_);

  const int modelBaseInputDim = getInputDimBase(robotModelInfo_);
  const int modelArmInputDim = getInputDimArm(robotModelInfo_);
  const int modelInputDim = getInputDim(robotModelInfo_);

  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) 
  {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation_) 
  {
    MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(*pinocchioInterfacePtr_, pinocchioMapping, {robotModelInfo_.robotArm.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } 
  else 
  {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(robotModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(*pinocchioInterfacePtr_,
                                                     pinocchioMappingCppAd, 
                                                     {robotModelInfo_.robotArm.eeFrame},
                                                     modelStateDim, 
                                                     modelInputDim,
                                                     "end_effector_kinematics", 
                                                     libraryFolder_, 
                                                     recompileLibraries_, 
                                                     false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

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

  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.2;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
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

  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation_) 
  {
    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorSelfCollisionConstraint(MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                                std::move(geometryInterface), 
                                                                                                minimumDistance));
  } 
  else 
  {
    constraint = std::unique_ptr<StateConstraint>(new SelfCollisionConstraintCppAd(*pinocchioInterfacePtr_, 
                                                                                    MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                    std::move(geometryInterface), 
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
std::unique_ptr<StateCost> MobileManipulatorInterface::getExtCollisionConstraint(const std::string& prefix, 
                                                                                 size_t modelMode) 
{
  std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] START" << std::endl;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);

  const int modelStateDim = getStateDim(robotModelInfo_);

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
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] PRECOMPUTED!" << std::endl;

    ///////// NUA TODO: NOT FUNCTIONAL AND COMPLETED YET!
    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorExtCollisionConstraint(MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                              std::move(extCollisionPinocchioGeometryInterface), 
                                                                                              pointsOnRobotPtr_,
                                                                                              maxDistance,
                                                                                              emuPtr_,
                                                                                              modelMode,
                                                                                              modelStateDim));
  }
  else
  {
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] CPPAD!" << std::endl;

    constraint = std::unique_ptr<StateConstraint>(new ExtCollisionConstraintCppAd(*pinocchioInterfacePtr_, 
                                                                                  MobileManipulatorPinocchioMapping(robotModelInfo_), 
                                                                                  std::move(extCollisionPinocchioGeometryInterface), 
                                                                                  modelMode,
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
