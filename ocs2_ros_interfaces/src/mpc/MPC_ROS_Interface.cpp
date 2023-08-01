// LAST UPDATE: 2023.07.20
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h"
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix)
  : mpc_(mpc),
    topicPrefix_(std::move(topicPrefix)),
    bufferPrimalSolutionPtr_(new PrimalSolution()),
    publisherPrimalSolutionPtr_(new PrimalSolution()),
    bufferCommandPtr_(new CommandData()),
    publisherCommandPtr_(new CommandData()),
    bufferPerformanceIndicesPtr_(new PerformanceIndex),
    publisherPerformanceIndicesPtr_(new PerformanceIndex) 
{
  //esdfCachingServerPtr_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));

  // start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
}

/*
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix)
  : mpc_baseMotion_(mpc),
    mpc_armMotion_(mpc),
    mpc_wholeBodyMotion_(mpc),
    topicPrefix_(std::move(topicPrefix)),
    bufferPrimalSolutionPtr_(new PrimalSolution()),
    publisherPrimalSolutionPtr_(new PrimalSolution()),
    bufferCommandPtr_(new CommandData()),
    publisherCommandPtr_(new CommandData()),
    bufferPerformanceIndicesPtr_(new PerformanceIndex),
    publisherPerformanceIndicesPtr_(new PerformanceIndex) 
{
  //esdfCachingServerPtr_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));

  // start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc_baseMotion, 
                                     MPC_BASE& mpc_armMotion, 
                                     MPC_BASE& mpc_wholeBodyMotion, 
                                     std::string topicPrefix)
  : mpc_baseMotion_(mpc_baseMotion),
    mpc_armMotion_(mpc_armMotion),
    mpc_wholeBodyMotion_(mpc_wholeBodyMotion),
    topicPrefix_(std::move(topicPrefix)),
    bufferPrimalSolutionPtr_(new PrimalSolution()),
    publisherPrimalSolutionPtr_(new PrimalSolution()),
    bufferCommandPtr_(new CommandData()),
    publisherCommandPtr_(new CommandData()),
    bufferPerformanceIndicesPtr_(new PerformanceIndex),
    publisherPerformanceIndicesPtr_(new PerformanceIndex) 
{
  //esdfCachingServerPtr_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));

  // start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MPC_ROS_Interface::~MPC_ROS_Interface() 
{
  std::cout << "[MPC_ROS_Interface::~MPC_ROS_Interface] SHUTTING DOWN..." << std::endl;
  shutdownNode();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int MPC_ROS_Interface::getModelModeInt()
{
  return modelModeInt_;
}

void MPC_ROS_Interface::setModelModeInt(int modelModeInt)
{
  modelModeInt_ = modelModeInt;
}

/*
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MPC_ROS_Interface::getMPCLaunchReadyFlag()
{
  return mpcLaunchReadyFlag_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::setMPCLaunchReadyFlag(bool mpcLaunchReadyFlag)
{
  mpcLaunchReadyFlag_ = mpcLaunchReadyFlag;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void MPC_ROS_Interface::setEsdfCachingServer(std::shared_ptr<voxblox::EsdfCachingServer> new_esdfCachingServerPtr)
{
  esdfCachingServerPtr_ = new_esdfCachingServerPtr;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::resetMpcNode(TargetTrajectories&& initTargetTrajectories) 
{
  std::lock_guard<std::mutex> resetLock(resetMutex_);
  
  mpc_.reset();

  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(initTargetTrajectories));
  
  mpcTimer_.reset();
  resetRequestedEver_ = true;
  terminateThread_ = false;
  readyToPublish_ = false;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) 
{
  if (static_cast<bool>(req.reset)) 
  {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories);
    resetMpcNode(std::move(targetTrajectories));
    res.done = static_cast<uint8_t>(true);

    std::cerr << "\n#####################################################"
              << "\n#####################################################"
              << "\n#################  MPC is reset.  ###################"
              << "\n#####################################################"
              << "\n#####################################################\n";
    return true;
  } 
  else 
  {
    ROS_WARN_STREAM("[MPC_ROS_Interface::resetMpcCallback] Reset request failed!");
    return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
ocs2_msgs::mpc_flattened_controller MPC_ROS_Interface::createMpcPolicyMsg(const PrimalSolution& primalSolution,
                                                                          const CommandData& commandData,
                                                                          const PerformanceIndex& performanceIndices) 
{
  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] START" << std::endl;

  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;
  mpcPolicyMsg.initObservation = ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);
  mpcPolicyMsg.planTargetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
  mpcPolicyMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
  mpcPolicyMsg.performanceIndices = ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

  switch (primalSolution.controllerPtr_->getType()) 
  {
    case ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    
    case ControllerType::LINEAR:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
  }

  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();

  mpcPolicyMsg.timeTrajectory.clear();
  mpcPolicyMsg.timeTrajectory.reserve(N);
  mpcPolicyMsg.stateTrajectory.clear();
  mpcPolicyMsg.stateTrajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);
  mpcPolicyMsg.postEventIndices.clear();
  mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

  // time
  for (auto t : primalSolution.timeTrajectory_) 
  {
    mpcPolicyMsg.timeTrajectory.emplace_back(t);
  }

  // post-event indices
  for (auto ind : primalSolution.postEventIndices_) 
  {
    mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
  }

  // state
  for (size_t k = 0; k < N; k++) 
  {
    ocs2_msgs::mpc_state mpcState;
    mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
    
    for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) 
    {
      mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) 
  {
    ocs2_msgs::mpc_input mpcInput;
    mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
    
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) 
    {
      mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
  }  // end of k loop

  // controller
  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  
  for (auto t : primalSolution.timeTrajectory_) 
  {
    mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

    policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
    timeTrajectoryTruncated.push_back(t);
  }  // end of k loop

  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] END" << std::endl;

  return mpcPolicyMsg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::publisherWorker() 
{
  //std::cout << "[MPC_ROS_Interface::publisherWorker] START" << std::endl;

  while (!terminateThread_) 
  {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_) 
    {
      break;
    }

    {
      std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
      publisherCommandPtr_.swap(bufferCommandPtr_);
      publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
    }

    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_, *publisherPerformanceIndicesPtr_);

    // publish the message
    mpcPolicyPublisher_.publish(mpcPolicyMsg);

    readyToPublish_ = false;
    lk.unlock();
    msgReady_.notify_one();
  }

  //std::cout << "[MPC_ROS_Interface::publisherWorker] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::copyToBuffer(const SystemObservation& mpcInitObservation) 
{
  //std::cout << "[MPC_ROS_Interface::copyToBuffer] START" << std::endl;

  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);

  scalar_t finalTime;
  // Get solution
  finalTime = mpcInitObservation.time + mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) 
  {
    finalTime = mpc_.getSolverPtr()->getFinalTime();
  }
  mpc_.getSolverPtr()->getPrimalSolution(finalTime, bufferPrimalSolutionPtr_.get());

  // Command
  bufferCommandPtr_->mpcInitObservation_ = mpcInitObservation;
  bufferCommandPtr_->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();

  // Performance indices
  *bufferPerformanceIndicesPtr_ = mpc_.getSolverPtr()->getPerformanceIndeces();

  //std::cout << "[MPC_ROS_Interface::copyToBuffer] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) 
{
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START" << std::endl;

  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (!resetRequestedEver_.load()) 
  {
    ROS_WARN_STREAM("[MPC_ROS_Interface::mpcObservationCallback] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
    return;
  }

  //if (shutDownFlag_)
  //{
  //  mpcObservationSubscriber_.shutdown();
  //  shutdownNode();
  //  std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] AFTER shutdownNode: " << std::endl;
  //}
  /*
  else
  {
    std::cout << "[MobileManipulatorInterface::launchNodes::modelModeCallback] NO shutdownNode: " << std::endl;
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] mpcProblemReadyFlag: " << getenv("mpcProblemReadyFlag") << std::endl;
  }
  */

  // current time, state, input, and subsystem
  const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  // measure the delay in running MPC
  mpcTimer_.startTimer();

  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] modelModeInt_: " << modelModeInt_ << std::endl;
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state size: " << currentObservation.state.size() << std::endl;
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;

  /*
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state:" << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;
  
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.input:" << std::endl;
  std::cout << currentObservation.input << std::endl;

  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state:" << std::endl;
  std::cout << currentObservation.full_state << std::endl;
  */

  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START mpc_.run" << std::endl;
  // run MPC
  //bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
  bool controllerIsUpdated;
  controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state, currentObservation.full_state);
  
  if (!controllerIsUpdated) 
  {
    return;
  }
  copyToBuffer(currentObservation);
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END mpc_.run" << std::endl;

  // Measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // Check MPC delay and solution window compatibility
  scalar_t timeWindow;
  timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) 
  {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }

  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) 
  {
    std::cerr << "[MPC_ROS_Interface::mpcObservationCallback] WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // Display time benchmarks
  if (mpc_.settings().debugPrint_) 
  {
    std::cerr << '\n';
    std::cerr << "\n### MPC_ROS Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif

  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::shutdownNode() 
{
  std::cout << "[MPC_ROS_Interface::shutdownNode] START" << std::endl;

#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("[MPC_ROS_Interface::shutdownNode] Shutting down workers ...");

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }

  ROS_INFO_STREAM("[MPC_ROS_Interface::shutdownNode] All workers are shut down.");
#endif

  // shutdown publishers
  mpcPolicyPublisher_.shutdown();
  mpcObservationSubscriber_.shutdown();

  //std::cout << "[MPC_ROS_Interface::shutdownNode] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::spin() 
{
  ROS_INFO_STREAM("[MPC_ROS_Interface::spin] Start spinning now...");
  // Equivalent to ros::spin() + check if master is alive
  while (ros::ok() && ros::master::check() && !shutDownFlag_) 
  {
    mpcShutDownFlag_ = getenv("mpcShutDownFlag");
    //std::cout << "[MPC_ROS_Interface::spin] mpcShutDownFlag_: " << mpcShutDownFlag_ << std::endl;

    if (mpcShutDownFlag_ == "true")
    {
      std::cout << "[MPC_ROS_Interface::spin] SHUTTING DOWN GUYS!!!" << std::endl;
      shutDownFlag_ = true;
    }
    /*
    else
    {
      std::cout << "[MPC_ROS_Interface::spin] OMG shutDownFlag_: false" << std::endl;
    }
    */

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  //shutdownNode();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) 
{
  ROS_INFO_STREAM("[MPC_ROS_Interface::launchNodes] MPC node is setting up...");

  // Subscribe Observation 
  mpcObservationSubscriber_ = nodeHandle.subscribe(topicPrefix_ + "_mpc_observation", 
                                                   1, 
                                                   &MPC_ROS_Interface::mpcObservationCallback, 
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());

  // Subscribe Model Mode
  /*
  auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
  {
    std::cout << "[MPC_ROS_Interface::launchNodes] START" << std::endl;

    modelModeInt_ = msg->data;
    std::cout << "[MPC_ROS_Interface::launchNodes] modelModeInt: " << modelModeInt_ << std::endl;
    std::cout << "[MPC_ROS_Interface::launchNodes] END" << std::endl;
    std::cout << "" << std::endl;

    shutDownFlag_ = true;

    statusModelModeMPCMsg_.data = false;
    statusModelModeMPCPublisher_.publish(statusModelModeMPCMsg_);
  };
  modelModeSubscriber_ = nodeHandle.subscribe<std_msgs::UInt8>(topicPrefix_ + "_model_mode", 1, modelModeCallback);
  */

  // Publish MPC Policy
  mpcPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_flattened_controller>(topicPrefix_ + "_mpc_policy", 1, true);

  // Publish Model Mode MPC Status
  statusModelModeMPCPublisher_ = nodeHandle.advertise<std_msgs::Bool>(topicPrefix_ + "_model_mode_mpc_status", 1, true);

  // Service Server to reset MPC
  mpcResetServiceServer_ = nodeHandle.advertiseService(topicPrefix_ + "_mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);

#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("[MPC_ROS_Interface::launchNodes] Publishing SLQ-MPC messages on a separate thread.");
#endif

  ROS_INFO_STREAM("[MPC_ROS_Interface::launchNodes] MPC node is ready.");

  statusModelModeMPCMsg_.data = true;
  statusModelModeMPCPublisher_.publish(statusModelModeMPCMsg_);

  spin();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::run() 
{
  std::cout << "[MPC_ROS_Interface::run] START" << std::endl;

  std::cout << "[MPC_ROS_Interface::run] DEBUG INF LOOP!" << std::endl;
  while(1);
  
  while (ros::ok() && ros::master::check() && !shutDownFlag_)
  {
    std::lock_guard<std::mutex> resetLock(resetMutex_);

    if (!resetRequestedEver_.load()) 
    {
      ROS_WARN_STREAM("[MPC_ROS_Interface::run] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
      return;
    }

    // current time, state, input, and subsystem
    const auto currentObservation = currentObservation_;

    /*
    std::cout << "[MPC_ROS_Interface::run] currentObservation.state:" << std::endl;
    std::cout << currentObservation.state << std::endl << std::endl;
    
    std::cout << "[MPC_ROS_Interface::run] currentObservation.input:" << std::endl;
    std::cout << currentObservation.input << std::endl;

    std::cout << "[MPC_ROS_Interface::run] currentObservation.full_state:" << std::endl;
    std::cout << currentObservation.full_state << std::endl;
    */

    // measure the delay in running MPC
    mpcTimer_.startTimer();

    // run MPC
    std::cout << "[MPC_ROS_Interface::run] START mpc_.run" << std::endl;

    //bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
    bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state, currentObservation.full_state);
    
    if (!controllerIsUpdated) 
    {
      return;
    }
    copyToBuffer(currentObservation);
    //std::cout << "[MPC_ROS_Interface::run] END mpc_.run" << std::endl;

    // Measure the delay for sending ROS messages
    mpcTimer_.endTimer();

    // Check MPC delay and solution window compatibility
    scalar_t timeWindow;
    timeWindow = mpc_.settings().solutionTimeWindow_;
    if (mpc_.settings().solutionTimeWindow_ < 0) 
    {
      timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
    }

    if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) 
    {
      std::cerr << "[MPC_ROS_Interface::run] WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    // Display time benchmarks
    if (mpc_.settings().debugPrint_) 
    {
      std::cerr << '\n';
      std::cerr << "\n### MPC_ROS Benchmarking";
      std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
      std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    }
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif
  }

  std::cout << "[MPC_ROS_Interface::run] END" << std::endl;
}

}  // namespace ocs2
