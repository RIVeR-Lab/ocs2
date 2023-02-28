/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

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
  // Read multi modal flag
  bool useMultiModel;
  loadData::loadPtreeValue(pt, useMultiModel, "model_information.useMultiModel", false);

  // Read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile_, "model_information.manipulatorModelType");
  
  // Read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.removeJoints", removeJointNames, false);
  
  // Read the link names of joints
  std::vector<std::string> jointParentFrameNames;
  loadData::loadStdVector<std::string>(taskFile_, "model_information.jointParentFrameNames", jointParentFrameNames, false);

  // Read the frame names
  std::string baseFrame, armBaseFrame, eeFrame;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", false);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### model_information.useMultiModel: " << useMultiModel;
  std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) 
  {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.jointParentFrameNames: ";
  for (const auto& name : jointParentFrameNames) 
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
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile_, modelType, removeJointNames)));
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END PinocchioInterface" << std::endl;
  std::cout << "*********************************" << std::endl;
  std::cout << "" << std::endl;
  std::cerr << *pinocchioInterfacePtr_;

  // ManipulatorModelInfo
  manipulatorModelInfo_ = mobile_manipulator::createManipulatorModelInfo(*pinocchioInterfacePtr_, 
                                                                         modelType,
                                                                         baseFrame, 
                                                                         armBaseFrame, 
                                                                         eeFrame,
                                                                         jointParentFrameNames);

  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, usePreComputation_, "model_settings.usePreComputation", true);
  loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", true);
  loadData::loadPtreeValue(pt, activateSelfCollision_, "selfCollision.activate", true);
  loadData::loadPtreeValue(pt, activateExtCollision_, "extCollision.activate", false);
  std::cerr << " #### =============================================================================\n";

  // DDP-MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc");

  // Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  // NUA TODO: We don't need to do it here!
  setMPCProblem(2, pointsAndRadii);
  
  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorInterface::setMPCProblem(size_t modalMode, PointsOnRobot::points_radii_t& pointsAndRadii)
{
  std::cout << "[MobileManipulatorInterface::setMPCProblem] START" << std::endl;

  std::cout << "[MobileManipulatorInterface::setMPCProblem] modalMode: " << modalMode << std::endl;

  /*
   * Optimal control problem
   */
  /// Cost
  ///////// NUA: ADAPT TO MULTI MODAL!
  problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile_));

  /// Constraints
  // Joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile_));

  // End-effector state constraint
  problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, 
                                                                               taskFile_, 
                                                                               "endEffector",
                                                                               usePreComputation_, 
                                                                               libraryFolder_, 
                                                                               recompileLibraries_));
  
  problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, 
                                                                                    taskFile_, 
                                                                                    "finalEndEffector",
                                                                                    usePreComputation_, 
                                                                                    libraryFolder_, 
                                                                                    recompileLibraries_));

  // Self-collision avoidance constraint
  if (activateSelfCollision_) 
  {
    problem_.stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, 
                                                                                     taskFile_, 
                                                                                     urdfFile_, 
                                                                                     "selfCollision", 
                                                                                     usePreComputation_, 
                                                                                     libraryFolder_, 
                                                                                     recompileLibraries_));
  }

  // External-collision avoidance constraint
  if (activateExtCollision_) 
  {
    std::cout << "[MobileManipulatorInterface::setMPCProblem] activateExtCollision: " << activateExtCollision_ << std::endl;

    //pointsOnRobotPtr_.reset(new PointsOnRobot(pointsAndRadii));
    createPointsOnRobotPtr(pointsAndRadii);
    
    if (pointsOnRobotPtr_->getNumOfPoints() > 0) 
    {
      //esdfCachingServerPtr_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
      //voxbloxInterpolatorPtr_ = esdfCachingServerPtr_->getInterpolator();

      pointsOnRobotPtr_->initialize(*pinocchioInterfacePtr_,
                                    MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                                    MobileManipulatorPinocchioMappingCppAd(manipulatorModelInfo_),
                                    "points_on_robot",
                                    libraryFolder_,
                                    recompileLibraries_,
                                    false,
                                    manipulatorModelInfo_.baseFrame,
                                    manipulatorModelInfo_.armBaseFrame,
                                    manipulatorModelInfo_.eeFrame,
                                    manipulatorModelInfo_.jointParentFrameNames);
    } 
    else 
    {
      pointsOnRobotPtr_ = nullptr;
    }

    emuPtr_.reset(new ExtMapUtility());
    string world_frame_name = "world";
    string pub_name_oct_dist_visu = "occ_dist";
    string pub_name_oct_dist_array_visu = "occ_dist_array";

    emuPtr_->setWorldFrameName(world_frame_name);
    emuPtr_->setPubOccDistVisu(pub_name_oct_dist_visu);
    emuPtr_->setPubOccDistArrayVisu(pub_name_oct_dist_array_visu);
    
    problem_.stateSoftConstraintPtr->add("extCollision", getExtCollisionConstraint(*pinocchioInterfacePtr_,
                                                                                   taskFile_, 
                                                                                   urdfFile_, 
                                                                                   "extCollision", 
                                                                                   usePreComputation_,
                                                                                   libraryFolder_, 
                                                                                   recompileLibraries_));
  }

  std::cout << "[MobileManipulatorInterface::setMPCProblem] BEFORE Dynamics" << std::endl;
  
  // Dynamics
  switch (manipulatorModelInfo_.manipulatorModelType) 
  {
    case ManipulatorModelType::DefaultManipulator: 
    {
      problem_.dynamicsPtr.reset(new DefaultManipulatorDynamics(manipulatorModelInfo_, 
                                                                "dynamics", 
                                                                libraryFolder_, 
                                                                recompileLibraries_, 
                                                                true));
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: 
    {
      problem_.dynamicsPtr.reset(new FloatingArmManipulatorDynamics(manipulatorModelInfo_, 
                                                                    "dynamics", 
                                                                    libraryFolder_, 
                                                                    recompileLibraries_, 
                                                                    true));
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: 
    {
      problem_.dynamicsPtr.reset(new FullyActuatedFloatingArmManipulatorDynamics(manipulatorModelInfo_, 
                                                                                 "dynamics", 
                                                                                 libraryFolder_, 
                                                                                 recompileLibraries_, 
                                                                                 true));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: 
    {
      problem_.dynamicsPtr.reset(new WheelBasedMobileManipulatorDynamics(manipulatorModelInfo_, 
                                                                         "dynamics", 
                                                                         libraryFolder_, 
                                                                         recompileLibraries_, 
                                                                         true));
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Pre-computation" << std::endl;

  /*
   * Pre-computation
   */
  if (usePreComputation_) 
  {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
  }

  std::cout << "[MobileManipulatorInterface::MobileManipulatorInterface] BEFORE Rollout" << std::endl;

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));

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
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getQuadraticInputCost(const std::string& taskFile) 
{
  matrix_t R = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;

  // arm base DOFs input costs
  if (baseInputDim > 0) 
  {
    matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
    loadData::loadEigenMatrix(taskFile, "inputCost.R.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  // arm joints DOFs input costs
  matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
  loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
  R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), manipulatorModelInfo_.stateDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, 
                                                                                const std::string& prefix,
                                                                                bool usePreComputation, 
                                                                                const std::string& libraryFolder,
                                                                                bool recompileLibraries) 
{
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
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
  if (usePreComputation) 
  {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } 
  else 
  {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, 
                                                     pinocchioMappingCppAd, 
                                                     {manipulatorModelInfo_.eeFrame},
                                                     manipulatorModelInfo_.stateDim, 
                                                     manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", 
                                                     libraryFolder, 
                                                     recompileLibraries, 
                                                     false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, 
                                                                                  const std::string& urdfFile,
                                                                                  const std::string& prefix, 
                                                                                  bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) 
{
  std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] START" << std::endl;

  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] numCollisionPairs: " << numCollisionPairs << std::endl;

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) 
  {
    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorSelfCollisionConstraint(MobileManipulatorPinocchioMapping(manipulatorModelInfo_), 
                                                                                               std::move(geometryInterface), 
                                                                                               minimumDistance));
  } 
  else 
  {
    constraint = std::unique_ptr<StateConstraint>(new SelfCollisionConstraintCppAd(pinocchioInterface, 
                                                                                   MobileManipulatorPinocchioMapping(manipulatorModelInfo_), 
                                                                                   std::move(geometryInterface), 
                                                                                   minimumDistance,
                                                                                   "self_collision", 
                                                                                   libraryFolder, 
                                                                                   recompileLibraries, 
                                                                                   false));
  }

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({mu, delta}));

  std::cout << "[MobileManipulatorInterface::getSelfCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::getExtCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                 const std::string& taskFile, 
                                                                                 const std::string& urdfFile,
                                                                                 const std::string& prefix, 
                                                                                 bool usePreComputation,
                                                                                 const std::string& libraryFolder,
                                                                                 bool recompileLibraries) 
{
  std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] START" << std::endl;

  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t maxDistance = 10;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### ExtCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, maxDistance, prefix + ".maxDistance", true);
  std::cerr << " #### =============================================================================\n";

  ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface(pinocchioInterface);
  std::unique_ptr<StateConstraint> constraint;
  
  if (usePreComputation) 
  {
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] PRECOMPUTED!" << std::endl;

    constraint = std::unique_ptr<StateConstraint>(new MobileManipulatorExtCollisionConstraint(MobileManipulatorPinocchioMapping(manipulatorModelInfo_), 
                                                                                            std::move(extCollisionPinocchioGeometryInterface), 
                                                                                            pointsOnRobotPtr_,
                                                                                            maxDistance,
                                                                                            emuPtr_));
  }
  else
  {
    std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] CPPAD!" << std::endl;

    constraint = std::unique_ptr<StateConstraint>(new ExtCollisionConstraintCppAd(pinocchioInterface, 
                                                                                  MobileManipulatorPinocchioMapping(manipulatorModelInfo_), 
                                                                                  std::move(extCollisionPinocchioGeometryInterface), 
                                                                                  pointsOnRobotPtr_,
                                                                                  maxDistance,
                                                                                  emuPtr_,
                                                                                  "ext_collision", 
                                                                                  libraryFolder, 
                                                                                  recompileLibraries, 
                                                                                  false));
  }

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({mu, delta}));

  std::cout << "[MobileManipulatorInterface::getExtCollisionConstraint] END" << std::endl;

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) 
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armInputDim = manipulatorModelInfo_.armDim;
  const auto& model = pinocchioInterface.getModel();

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) 
  {
    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
    const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

    std::cerr << "\n #### JointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i) 
    {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = baseStateDim + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }

  // Load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    vector_t upperBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    // Base DOFs velocity limits
    if (baseInputDim > 0) 
    {
      vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
      vector_t upperBoundBase = vector_t::Zero(baseInputDim);
      
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.lowerBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                lowerBoundBase);
      
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.upperBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                upperBoundBase);
      
      lowerBound.head(baseInputDim) = lowerBoundBase;
      upperBound.head(baseInputDim) = upperBoundBase;
    }

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
    
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(manipulatorModelInfo_.inputDim);
    for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) 
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
  boxConstraints -> initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
  
  return boxConstraints;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
