// LAST UPDATE: 2023.11.08
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
  //std::cout << "[MPC_ROS_Interface::~MPC_ROS_Interface] SHUTTING DOWN..." << std::endl;
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
  std::cout << "[MPC_ROS_Interface::resetMpcNode] START" << std::endl;

  std::lock_guard<std::mutex> resetLock(resetMutex_);

  std::cout << "[MPC_ROS_Interface::resetMpcNode] initTargetTrajectories size: " << initTargetTrajectories.size() << std::endl;
  std::cout << initTargetTrajectories << std::endl;

  mpc_.reset();

  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(initTargetTrajectories));
  
  mpcTimer_.reset();
  resetRequestedEver_ = true;
  terminateThread_ = false;
  readyToPublish_ = false;

  std::cout << "[MPC_ROS_Interface::resetMpcNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) 
{
  if (static_cast<bool>(req.reset)) 
  {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories);

    std::cout << "[MPC_ROS_Interface::resetMpcNode] RECEIVED targetTrajectories size: " << targetTrajectories.size() << std::endl;
    std::cout << targetTrajectories << std::endl;

    std::cout << "[MPC_ROS_Interface::resetMpcNode] WE USE THIS ONE INSTEAD testTargetTrajectories_ size: " << testTargetTrajectories_.size() << std::endl;
    std::cout << testTargetTrajectories_ << std::endl;

    std::cout << "[MPC_ROS_Interface::resetMpcNode] DONT FORGET TO CHANGE THISSSSSSSSSSSSSSSSSSSSSSSSSSSS BAAAAAAAAAAAACCCCCCCCCCCCCK" << std::endl;
    resetMpcNode(std::move(testTargetTrajectories_));
    res.done = static_cast<uint8_t>(true);

    std::cout << "\n#####################################################"
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
  std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] START" << std::endl;

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] primalSolution size: " << primalSolution.stateTrajectory_.size() << std::endl;
  //for (size_t i = 0; i < primalSolution.stateTrajectory_.size(); i++)
  //{
  // std::cout << i << ": " << primalSolution.stateTrajectory_[i] << std::endl;
  //}

  //ros::spinOnce();
  
  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] commandData size: " << commandData.mpcTargetTrajectories_.size() << std::endl;
  //std::cout << commandData.mpcTargetTrajectories_ << std::endl;

  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE initObservation" << std::endl;
  mpcPolicyMsg.initObservation = ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE planTargetTrajectories" << std::endl;
  mpcPolicyMsg.planTargetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
  
  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE modeSchedule" << std::endl;
  mpcPolicyMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE performanceIndices" << std::endl;
  mpcPolicyMsg.performanceIndices = ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE getType: " << std::endl;
  switch (primalSolution.controllerPtr_->getType()) 
  {
    case ControllerType::FEEDFORWARD:
      //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE FEEDFORWARD: " << std::endl;
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    
    case ControllerType::LINEAR:
      //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE LINEAR: " << std::endl;
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
  }

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE timeTrajectory_" << std::endl;
  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();
  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] N: " << N << std::endl;

  //ros::spinOnce();

  mpcPolicyMsg.timeTrajectory.clear();
  mpcPolicyMsg.timeTrajectory.reserve(N);
  mpcPolicyMsg.stateTrajectory.clear();
  mpcPolicyMsg.stateTrajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);
  mpcPolicyMsg.postEventIndices.clear();
  mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE time" << std::endl;
  // time
  for (auto t : primalSolution.timeTrajectory_) 
  {
    mpcPolicyMsg.timeTrajectory.emplace_back(t);
  }

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE indices" << std::endl;
  // post-event indices
  for (auto ind : primalSolution.postEventIndices_) 
  {
    mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
  }

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE state" << std::endl;
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

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE input" << std::endl;
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

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE controller" << std::endl;
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

  //ros::spinOnce();

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE flatten" << std::endl;
  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  //ros::spinOnce();

  std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] END" << std::endl;

  return mpcPolicyMsg;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::publisherWorker() 
{
  std::cout << "[MPC_ROS_Interface::publisherWorker] START" << std::endl;

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

    std::cout << "[MPC_ROS_Interface::publisherWorker] BEFORE createMpcPolicyMsg" << std::endl;
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_, *publisherPerformanceIndicesPtr_);
    std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER createMpcPolicyMsg" << std::endl;

    // publish the message
    mpcPolicyPublisher_.publish(mpcPolicyMsg);

    readyToPublish_ = false;
    lk.unlock();
    msgReady_.notify_one();
  }

  std::cout << "[MPC_ROS_Interface::publisherWorker] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::copyToBuffer(const SystemObservation& mpcInitObservation) 
{
  std::cout << "[MPC_ROS_Interface::copyToBuffer] START" << std::endl;

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

  std::cout << "[MPC_ROS_Interface::copyToBuffer] BEFORE getTargetTrajectories" << std::endl;
  bufferCommandPtr_->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();
  std::cout << "[MPC_ROS_Interface::copyToBuffer] AFTER getTargetTrajectories" << std::endl;

  // Performance indices
  *bufferPerformanceIndicesPtr_ = mpc_.getSolverPtr()->getPerformanceIndeces();

  /*
  std::cout << "[MPC_ROS_Interface::copyToBuffer] finalTime: " << finalTime << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] inputTrajectory_ size: " << bufferPrimalSolutionPtr_->inputTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->inputTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->inputTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::copyToBuffer] stateTrajectory_ size: " << bufferPrimalSolutionPtr_->stateTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->stateTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->stateTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::copyToBuffer] mpcInitObservation_.full_state size: " << bufferCommandPtr_->mpcInitObservation_.full_state.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcInitObservation_.full_state << std::endl;

  std::cout << "[MPC_ROS_Interface::copyToBuffer] mpcTargetTrajectories_ size: " << bufferCommandPtr_->mpcTargetTrajectories_.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcTargetTrajectories_ << std::endl;

  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ cost: " << bufferPerformanceIndicesPtr_->cost << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ dynamicsViolationSSE: " << bufferPerformanceIndicesPtr_->dynamicsViolationSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ equalityConstraintsSSE: " << bufferPerformanceIndicesPtr_->equalityConstraintsSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ equalityLagrangian: " << bufferPerformanceIndicesPtr_->equalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ inequalityLagrangian: " << bufferPerformanceIndicesPtr_->inequalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPerformanceIndicesPtr_ merit: " << bufferPerformanceIndicesPtr_->merit << std::endl;
  */

  std::cout << "[MPC_ROS_Interface::copyToBuffer] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) 
{
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START" << std::endl;

  // current time, state, input, and subsystem
  //const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  /*
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] modelModeInt_: " << modelModeInt_ << std::endl;
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state size: " << currentObservation.state.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;

  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state:" << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;
  
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.input:" << std::endl;
  std::cout << currentObservation.input << std::endl;

  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state:" << std::endl;
  std::cout << currentObservation.full_state << std::endl;

  std::cout << "[MobileManipulatorInterface::mpcObservationCallback] testCurrentObservation_.mode: " << testCurrentObservation_.mode << std::endl;
  std::cout << "[MobileManipulatorInterface::mpcObservationCallback] testCurrentObservation_.time: " << testCurrentObservation_.time << std::endl;
  std::cout << "[MobileManipulatorInterface::mpcObservationCallback] testCurrentObservation_.state size: " << testCurrentObservation_.state.size() << std::endl << testCurrentObservation_.state << std::endl;
  std::cout << "[MobileManipulatorInterface::mpcObservationCallback] testCurrentObservation_.full_state size: " << testCurrentObservation_.full_state.size() << std::endl << testCurrentObservation_.full_state << std::endl;
  std::cout << "[MobileManipulatorInterface::mpcObservationCallback] testCurrentObservation_.input size: " << testCurrentObservation_.input.size() << std::endl << testCurrentObservation_.input << std::endl;
  */

  //computeTraj(testTargetTrajectories_, testCurrentObservation_);
  computeTraj2(testTargetTrajectories_, testCurrentObservation_, false);

  /*
  if (!shutDownFlag_ && !internalShutDownFlag_)
  {
    std::lock_guard<std::mutex> resetLock(resetMutex_);

    if (!resetRequestedEver_.load()) 
    {
      //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service." << std::endl;
      return;
    }

    // current time, state, input, and subsystem
    //const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

    // measure the delay in running MPC
    mpcTimer_.startTimer();

    /*
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] modelModeInt_: " << modelModeInt_ << std::endl;
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state size: " << currentObservation.state.size() << std::endl;
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state size: " << currentObservation.full_state.size() << std::endl;

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state:" << std::endl;
    std::cout << currentObservation.state << std::endl << std::endl;
    
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.input:" << std::endl;
    std::cout << currentObservation.input << std::endl;

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state:" << std::endl;
    std::cout << currentObservation.full_state << std::endl;
    * /

    
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START mpc_.run" << std::endl;
    /*
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state:" << std::endl;
    std::cout << currentObservation.state << std::endl << std::endl;

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state:" << std::endl;
    std::cout << currentObservation.full_state << std::endl;
    * /

    // run MPC
    //bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
    bool controllerIsUpdated;
    internalShutDownFlag_ = false;
    controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state, currentObservation.full_state);

    internalShutDownFlag_ = mpc_.getInternalShutDownFlag();
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
    
    if (!controllerIsUpdated) 
    {
      return;
    }
    copyToBuffer(currentObservation);
    
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END mpc_.run" << std::endl << std::endl;

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
      std::cout << "[MPC_ROS_Interface::mpcObservationCallback] WARNING: The solution time window might be shorter than the MPC delay!\n";
    }

    // Display time benchmarks
    if (mpc_.settings().debugPrint_) 
    {
      std::cout << '\n';
      std::cout << "\n### MPC_ROS Benchmarking";
      std::cout << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cout << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
      std::cout << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
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
  else
  {
    if (!callbackEndFlag_)
    {
      /*
      if (internalShutDownFlag_)
      {
        std::cout << "[MPC_ROS_Interface::mpcObservationCallback] INTERNALLY SHUT DOWN!!!: " << internalShutDownFlag_ << std::endl;
      }
      std::cout << "[MPC_ROS_Interface::mpcObservationCallback] shutDownFlag_: " << shutDownFlag_ << std::endl;
      std::cout << "[MPC_ROS_Interface::mpcObservationCallback] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
      std::cout << "[MPC_ROS_Interface::mpcObservationCallback] Shutting down..." << std::endl;
      * /
    }
    callbackEndFlag_ = true;
  }
  */

  /*
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END ctr_: " << ctr_ << std::endl;

  if (ctr_ > 0)
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] DEBUG_INF" << std::endl;
    while(1);
  }
  
  ctr_++;
  */
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::shutdownNode() 
{
  //std::cout << "[MPC_ROS_Interface::shutdownNode] START" << std::endl;

#ifdef PUBLISH_THREAD
  //std::cout << "[MPC_ROS_Interface::shutdownNode] Shutting down workers ..." << std::endl;

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }

  //std::cout << "[MPC_ROS_Interface::shutdownNode] All workers are shut down." << std::endl;
#endif

  // shutdown publishers
  mpcPolicyPublisher_.shutdown();

   //std::cout << "[MPC_ROS_Interface::shutdownNode] BEFORE mpcObservationSubscriber_" << std::endl;
  mpcObservationSubscriber_.shutdown();

  //std::cout << "[MPC_ROS_Interface::shutdownNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::spin() 
{
  std::cout << "[MPC_ROS_Interface::spin] Start spinning now..." << std::endl;
  // Equivalent to ros::spin() + check if master is alive
  while (ros::ok() && ros::master::check() && !shutDownFlag_) 
  {
    mpcShutDownFlag_ = getenv("mpcShutDownFlag");
    //std::cout << "[MPC_ROS_Interface::spin] mpcShutDownFlag_: " << mpcShutDownFlag_ << std::endl;

    if (mpcShutDownFlag_ == "true")
    {
      //std::cout << "[MPC_ROS_Interface::spin] Shutting down..." << std::endl;
      shutDownFlag_ = true;
    }

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  //shutdownNode();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) 
{
  //std::cout << "[MPC_ROS_Interface::launchNodes] MPC node is setting up..." << std::endl;

  // Subscribe Observation 
  mpcObservationSubscriber_ = nodeHandle.subscribe(topicPrefix_ + "mpc_observation", 
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
  mpcPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_flattened_controller>(topicPrefix_ + "mpc_policy", 1, true);

  // Publish Model Mode MPC Status
  statusModelModeMPCPublisher_ = nodeHandle.advertise<std_msgs::Bool>(topicPrefix_ + "model_mode_mpc_status", 1, true);

  // Service Server to reset MPC
  mpcResetServiceServer_ = nodeHandle.advertiseService(topicPrefix_ + "mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);

  /*
#ifdef PUBLISH_THREAD
  std::cout << "[MPC_ROS_Interface::launchNodes] Publishing SLQ-MPC messages on a separate thread." << std::endl;
#endif
  std::cout << "[MPC_ROS_Interface::launchNodes] MPC node is ready." << std::endl;
  */
 
  statusModelModeMPCMsg_.data = true;
  statusModelModeMPCPublisher_.publish(statusModelModeMPCMsg_);

  spin();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::computeTraj(TargetTrajectories& targetTrajectories, SystemObservation& currentObservation) 
{
  std::cout << "[MPC_ROS_Interface::computeTraj] START" << std::endl;

  resetMpcNode(std::move(targetTrajectories));

  //ros::spinOnce();

  // current time, state, input, and subsystem
  //const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  bool controllerIsUpdated;
  internalShutDownFlag_ = false;

  std::cout << "[MPC_ROS_Interface::computeTraj] BEFORE run" << std::endl;
  mpcTimer_.startTimer();
  controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state, currentObservation.full_state);
  mpcTimer_.endTimer();
  std::cout << "[MPC_ROS_Interface::computeTraj] AFTER run" << std::endl;
  
  //ros::spinOnce();

  std::cout << "[MPC_ROS_Interface::computeTraj] controllerIsUpdated: " << controllerIsUpdated << std::endl;

  internalShutDownFlag_ = mpc_.getInternalShutDownFlag();
  std::cout << "[MPC_ROS_Interface::computeTraj] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;

  //ros::spinOnce();

  // Check MPC delay and solution window compatibility
  scalar_t timeWindow;
  timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) 
  {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }

  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) 
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] WARNING: The solution time window might be shorter than the MPC delay!" << std::endl;
  }

  //ros::spinOnce();

  // Display time benchmarks
  //if (mpc_.settings().debugPrint_) 
  if (true) 
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] ### MPC_ROS Benchmarking" << std::endl;
    std::cout << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cout << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl << std::endl;
  }

  //ros::spinOnce();

  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE policyBufferLock" << std::endl;
    std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE publisherCommandPtr_" << std::endl;
    publisherCommandPtr_.swap(bufferCommandPtr_);

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE publisherPrimalSolutionPtr_" << std::endl;
    publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);

    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE publisherPerformanceIndicesPtr_" << std::endl;
    publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
  }

  //ros::spinOnce();

  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE createMpcPolicyMsg" << std::endl;
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_, *publisherPerformanceIndicesPtr_);

  //ros::spinOnce();

  std::cout << "[MPC_ROS_Interface::computeTraj] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::computeTraj2(TargetTrajectories tt, SystemObservation co, bool flag_reset) 
{
  std::cout << "[MPC_ROS_Interface::computeTraj2] START ctr_: " << ctr_ << std::endl;

  if (flag_reset)
  {
    //resetMpcNode(std::move(tt));

    std::lock_guard<std::mutex> resetLock(resetMutex_);
    std::cout << "[MPC_ROS_Interface::computeTraj2] initTargetTrajectories size: " << tt.size() << std::endl;
    std::cout << tt << std::endl;
    mpc_.reset();
    mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(tt));
    loadData();
  }
  else
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] NO RESET!!!" << std::endl;
  }


  // very important, these are variables that are carried in between iterations
  mpc_.getSolverPtr()->loadInitialData();


  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(tt));

  if (ctr_ > 1)
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] DEBUG_INF" << std::endl;
    while(1);
  }

  mpcTimer_.reset();
  mpcTimer_.startTimer();

  bool controllerIsUpdated;
  controllerIsUpdated = mpc_.run(co.time, co.state, co.full_state);
  
  if (!controllerIsUpdated)
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] CONTROLLER IS NOT UPDATED BOOOOOOOOOOOOO" << std::endl;
    std::cout << "[MPC_ROS_Interface::computeTraj2] DEBUG_INF" << std::endl;
    while(1);
  }
  
  copyToBuffer(co);
  mpcTimer_.endTimer();


  /*
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] inputTrajectory_ size: " << bufferPrimalSolutionPtr_->inputTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->inputTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->inputTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::computeTraj2] stateTrajectory_ size: " << bufferPrimalSolutionPtr_->stateTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->stateTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->stateTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::computeTraj2] mpcInitObservation_.full_state size: " << bufferCommandPtr_->mpcInitObservation_.full_state.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcInitObservation_.full_state << std::endl;

  std::cout << "[MPC_ROS_Interface::computeTraj2] mpcTargetTrajectories_ size: " << bufferCommandPtr_->mpcTargetTrajectories_.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcTargetTrajectories_ << std::endl;

  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ cost: " << bufferPerformanceIndicesPtr_->cost << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ dynamicsViolationSSE: " << bufferPerformanceIndicesPtr_->dynamicsViolationSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ equalityConstraintsSSE: " << bufferPerformanceIndicesPtr_->equalityConstraintsSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ equalityLagrangian: " << bufferPerformanceIndicesPtr_->equalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ inequalityLagrangian: " << bufferPerformanceIndicesPtr_->inequalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::computeTraj2] bufferPerformanceIndicesPtr_ merit: " << bufferPerformanceIndicesPtr_->merit << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  std::cout << "***************************************************" << std::endl;
  */

  writeData();

  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);

  // Check MPC delay and solution window compatibility
  scalar_t timeWindow;
  timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) 
  {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - co.time;
  }

  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) 
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // Display time benchmarks
  if (true) 
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] ### MPC_ROS Benchmarking" << std::endl;
    std::cout << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cout << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cout << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl << std::endl;
  }

  //std::unique_lock<std::mutex> lk(publisherMutex_);
  //readyToPublish_ = true;
  //lk.unlock();
  //msgReady_.notify_one();
  //std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
  //publisherCommandPtr_.swap(bufferCommandPtr_);
  //publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
  //publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
  //ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
  //mpcPolicyPublisher_.publish(mpcPolicyMsg);
  
  std::cout << "[MPC_ROS_Interface::computeTraj2] END ctr_: " << ctr_ << std::endl;
  
  ctr_++;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::writeData()
{
  std::vector<double> primalSolution_time;
  std::vector<std::vector<double>> primalSolution_state;
  std::vector<std::vector<double>> primalSolution_input;

  std::vector<double> mpcTargetTrajectories_time;
  std::vector<std::vector<double>> mpcTargetTrajectories_state;
  std::vector<std::vector<double>> mpcTargetTrajectories_input;

  std::vector<double> initObservation_state;
  std::vector<double> initObservation_fullState;
  std::vector<double> initObservation_input;

  // Primal Solution
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->timeTrajectory_.size(); i++)
  {
    primalSolution_time.push_back(bufferPrimalSolutionPtr_->timeTrajectory_[i]);
  }

  for (size_t i = 0; i < bufferPrimalSolutionPtr_->stateTrajectory_.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferPrimalSolutionPtr_->stateTrajectory_[i].size(); j++)
    {
      tmp.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i][j]);
    }
    primalSolution_state.push_back(tmp);
  }

  for (size_t i = 0; i < bufferPrimalSolutionPtr_->inputTrajectory_.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferPrimalSolutionPtr_->inputTrajectory_[i].size(); j++)
    {
      tmp.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i][j]);
    }
    primalSolution_input.push_back(tmp);
  }

  // Target Trajectories
  for (size_t i = 0; i < bufferCommandPtr_->mpcTargetTrajectories_.timeTrajectory.size(); i++)
  {
    mpcTargetTrajectories_time.push_back(bufferCommandPtr_->mpcTargetTrajectories_.timeTrajectory[i]);
  }
  
  for (size_t i = 0; i < bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory[i].size(); j++)
    {
      tmp.push_back(bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory[i][j]);
    }
    mpcTargetTrajectories_state.push_back(tmp);
  }

  for (size_t i = 0; i < bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory[i].size(); j++)
    {
      tmp.push_back(bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory[i][j]);
    }
    mpcTargetTrajectories_input.push_back(tmp);
  }

  // Init Observation
  for (size_t i = 0; i < bufferCommandPtr_->mpcInitObservation_.state.size(); i++)
  {
    initObservation_state.push_back(bufferCommandPtr_->mpcInitObservation_.state[i]);
  }

  for (size_t i = 0; i < bufferCommandPtr_->mpcInitObservation_.full_state.size(); i++)
  {
    initObservation_fullState.push_back(bufferCommandPtr_->mpcInitObservation_.full_state[i]);
  }

  for (size_t i = 0; i < bufferCommandPtr_->mpcInitObservation_.input.size(); i++)
  {
    initObservation_input.push_back(bufferCommandPtr_->mpcInitObservation_.input[i]);
  }

  nlohmann::json j;
  j["primalSolution_time"] = primalSolution_time;
  j["primalSolution_state"] = primalSolution_state;
  j["primalSolution_input"] = primalSolution_input;

  j["mpcTargetTrajectories_time"] = mpcTargetTrajectories_time;
  j["mpcTargetTrajectories_state"] = mpcTargetTrajectories_state;
  j["mpcTargetTrajectories_input"] = mpcTargetTrajectories_input;

  j["initObservation_mode"] = bufferCommandPtr_->mpcInitObservation_.mode;
  j["initObservation_time"] = bufferCommandPtr_->mpcInitObservation_.time;
  j["initObservation_state"] = initObservation_state;
  j["initObservation_fullState"] = initObservation_fullState;
  j["initObservation_input"] = initObservation_input;

  std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
  std::string dataPath = pkg_dir + "dataset/ocs2/tmp/";

  boost::filesystem::create_directories(dataPath);
  std::string filename = "tmp.json";
  std::ofstream o(dataPath + filename);
  o << std::setw(4) << j << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::loadData()
{
  std::cout << "[MPC_ROS_Interface::loadData] START" << std::endl;

  std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
  std::string dataPath = pkg_dir + "dataset/ocs2/tmp/";

  std::ifstream f(dataPath + "tmp.json");
  nlohmann::json data = nlohmann::json::parse(f);

  std::vector<double> primalSolution_time = data["primalSolution_time"];
  std::vector<std::vector<double>> primalSolution_state = data["primalSolution_state"];
  std::vector<std::vector<double>> primalSolution_input = data["primalSolution_input"];

  std::vector<double> mpcTargetTrajectories_time;
  std::vector<std::vector<double>> mpcTargetTrajectories_state = data["mpcTargetTrajectories_state"];
  std::vector<std::vector<double>> mpcTargetTrajectories_input = data["mpcTargetTrajectories_input"];

  double initObservation_mode = data["initObservation_mode"];
  double initObservation_time = data["initObservation_time"];
  std::vector<double> initObservation_state = data["initObservation_state"];
  std::vector<double> initObservation_fullState =  data["initObservation_fullState"];
  std::vector<double> initObservation_input =  data["initObservation_input"];

  // Primal Solution
  bufferPrimalSolutionPtr_->timeTrajectory_.clear();
  for (size_t i = 0; i < primalSolution_time.size(); i++)
  {
    bufferPrimalSolutionPtr_->timeTrajectory_.push_back(primalSolution_time[i]);
  }

  bufferPrimalSolutionPtr_->stateTrajectory_.clear();
  bufferPrimalSolutionPtr_->stateTrajectory_.resize(primalSolution_state.size());
  for (size_t i = 0; i < primalSolution_state.size(); i++)
  {
    bufferPrimalSolutionPtr_->stateTrajectory_[i].resize(primalSolution_state[i].size());
    for (size_t j = 0; j < primalSolution_state[i].size(); j++)
    {
      bufferPrimalSolutionPtr_->stateTrajectory_[i][j] = primalSolution_state[i][j];
    }
  }

  bufferPrimalSolutionPtr_->inputTrajectory_.clear();
  bufferPrimalSolutionPtr_->inputTrajectory_.resize(primalSolution_input.size());
  for (size_t i = 0; i < primalSolution_input.size(); i++)
  {
    bufferPrimalSolutionPtr_->inputTrajectory_[i].resize(primalSolution_input[i].size());
    for (size_t j = 0; j < primalSolution_input[i].size(); j++)
    {
      bufferPrimalSolutionPtr_->inputTrajectory_[i][j] = primalSolution_input[i][j];
    }
  }

  // Target Trajectories
  bufferCommandPtr_->mpcTargetTrajectories_.timeTrajectory.clear();
  for (size_t i = 0; i < mpcTargetTrajectories_time.size(); i++)
  {
    bufferCommandPtr_->mpcTargetTrajectories_.timeTrajectory.push_back(mpcTargetTrajectories_time[i]);
  }
  
  bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory.clear();
  bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory.resize(mpcTargetTrajectories_state.size());
  for (size_t i = 0; i < mpcTargetTrajectories_state.size(); i++)
  {
    bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory[i].resize(mpcTargetTrajectories_state[i].size());
    for (size_t j = 0; j < mpcTargetTrajectories_state[i].size(); j++)
    {
      bufferCommandPtr_->mpcTargetTrajectories_.stateTrajectory[i][j] = mpcTargetTrajectories_state[i][j];
    }
  }

  bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory.clear();
  bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory.resize(mpcTargetTrajectories_input.size());
  for (size_t i = 0; i < mpcTargetTrajectories_input.size(); i++)
  {
    bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory[i].resize(mpcTargetTrajectories_input[i].size());
    for (size_t j = 0; j < mpcTargetTrajectories_input[i].size(); j++)
    {
      bufferCommandPtr_->mpcTargetTrajectories_.inputTrajectory[i] << mpcTargetTrajectories_input[i][j];
    }
  }

  // Init Observation
  bufferCommandPtr_->mpcInitObservation_.state.resize(initObservation_state.size());
  for (size_t i = 0; i < initObservation_state.size(); i++)
  {
    bufferCommandPtr_->mpcInitObservation_.state[i] = initObservation_state[i];
  }

  bufferCommandPtr_->mpcInitObservation_.full_state.resize(initObservation_fullState.size());
  for (size_t i = 0; i < initObservation_fullState.size(); i++)
  {
    bufferCommandPtr_->mpcInitObservation_.full_state[i] = initObservation_fullState[i];
  }

  bufferCommandPtr_->mpcInitObservation_.input.resize(initObservation_input.size());
  for (size_t i = 0; i < initObservation_input.size(); i++)
  {
    bufferCommandPtr_->mpcInitObservation_.input[i] = initObservation_input[i];
  }

  /*
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] inputTrajectory_ size: " << bufferPrimalSolutionPtr_->inputTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->inputTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->inputTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::loadData] stateTrajectory_ size: " << bufferPrimalSolutionPtr_->stateTrajectory_.size() << std::endl;
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->stateTrajectory_.size(); i++)
  {
    std::cout << i << ": " << bufferPrimalSolutionPtr_->stateTrajectory_[i] << std::endl;
  }
  
  std::cout << "[MPC_ROS_Interface::loadData] mpcInitObservation_.full_state size: " << bufferCommandPtr_->mpcInitObservation_.full_state.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcInitObservation_.full_state << std::endl;

  std::cout << "[MPC_ROS_Interface::loadData] mpcTargetTrajectories_ size: " << bufferCommandPtr_->mpcTargetTrajectories_.size() << std::endl;
  std::cout << bufferCommandPtr_->mpcTargetTrajectories_ << std::endl;

  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ cost: " << bufferPerformanceIndicesPtr_->cost << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ dynamicsViolationSSE: " << bufferPerformanceIndicesPtr_->dynamicsViolationSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ equalityConstraintsSSE: " << bufferPerformanceIndicesPtr_->equalityConstraintsSSE << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ equalityLagrangian: " << bufferPerformanceIndicesPtr_->equalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ inequalityLagrangian: " << bufferPerformanceIndicesPtr_->inequalityLagrangian << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPerformanceIndicesPtr_ merit: " << bufferPerformanceIndicesPtr_->merit << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  */
  
  std::cout << "[MPC_ROS_Interface::loadData] END" << std::endl;
}

}  // namespace ocs2
