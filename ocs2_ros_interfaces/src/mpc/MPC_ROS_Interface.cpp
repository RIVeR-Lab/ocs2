// LAST UPDATE: 2023.11.28
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
void MPC_ROS_Interface::setTargetTrajectories(TargetTrajectories& tt)
{
  //std::cout << "[MPC_ROS_Interface::setTargetTrajectories] START" << std::endl;
  currentTargetTrajectories_ = tt;
  //std::cout << "[MPC_ROS_Interface::setTargetTrajectories] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::setSystemObservation(SystemObservation& so)
{
  //std::cout << "[MPC_ROS_Interface::setSystemObservation] START" << std::endl;
  testCurrentObservation_ = so;
  //std::cout << "[MPC_ROS_Interface::setSystemObservation] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
int MPC_ROS_Interface::getModelModeInt()
{
  return modelModeInt_;
}

void MPC_ROS_Interface::setModelModeInt(int modelModeInt)
{
  modelModeInt_ = modelModeInt;
}
*/

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

    //std::cout << "[MPC_ROS_Interface::resetMpcNode] WE USE THIS ONE INSTEAD testTargetTrajectories_ size: " << testTargetTrajectories_.size() << std::endl;
    //std::cout << testTargetTrajectories_ << std::endl;
    //std::cout << "[MPC_ROS_Interface::resetMpcNode] DONT FORGET TO CHANGE THISSSSSSSSSSSSSSSSSSSSSSSSSSSS BAAAAAAAAAAAACCCCCCCCCCCCCK" << std::endl;
    
    resetMpcNode(std::move(targetTrajectories));
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
  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] START" << std::endl;

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
  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE flatten 1" << std::endl;
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  /*
  std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] policyMsgDataPointers size: " << policyMsgDataPointers.size() << std::endl;
  bool sameCheck = true;
  for (size_t i = 0; i < policyMsgDataPointers.size(); i++)
  {
    std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] policyMsgDataPointers " << i << " size: " << policyMsgDataPointers[i]->size() << std::endl;
    for (size_t j = 0; j < policyMsgDataPointers[i]->size(); j++)
    {
      //std::cout << j << " -> " << (*policyMsgDataPointers[i])[j] << std::endl;
      if ((*policyMsgDataPointers[i])[j] != mpcPolicyMsg.data[i].data[j])
      {
        sameCheck = false;
      }
    }
  }
  */

  //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] DEBUG_INF" << std::endl;
  //while(1);

  //ros::spinOnce();

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

    //std::cout << "[MPC_ROS_Interface::publisherWorker] BEFORE createMpcPolicyMsg" << std::endl;
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_, *publisherPerformanceIndicesPtr_);
    //std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER createMpcPolicyMsg" << std::endl;

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

  //std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPrimalSolutionPtr_ controllerPtr_ size: " << bufferPrimalSolutionPtr_->controllerPtr_->size() << std::endl;

  // Command
  bufferCommandPtr_->mpcInitObservation_ = mpcInitObservation;

  //std::cout << "[MPC_ROS_Interface::copyToBuffer] BEFORE getTargetTrajectories" << std::endl;
  bufferCommandPtr_->mpcTargetTrajectories_ = mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();
  //std::cout << "[MPC_ROS_Interface::copyToBuffer] AFTER getTargetTrajectories" << std::endl;

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
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service." << std::endl;
    return;
  }

  // current time, state, input, and subsystem
  const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

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
  */

  
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START mpc_.run" << std::endl;
  /*
  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.state:" << std::endl;
  std::cout << currentObservation.state << std::endl << std::endl;

  std::cout << "[MPC_ROS_Interface::mpcObservationCallback] currentObservation.full_state:" << std::endl;
  std::cout << currentObservation.full_state << std::endl;
  */

  // measure the delay in running MPC
  mpcTimer_.startTimer();

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
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] WARNING: The solution time window might be shorter than the MPC delay!" << std::endl;
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

  bool isUpdated = mpc_.updateOCP();
  /*
  if (isUpdated)
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] RESETTING AFTER OCP UPDATE!" << std::endl;
    resetMpcNode(std::move(currentTargetTrajectories_));
  }
  */




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
  //computeTraj2(testTargetTrajectories_, testCurrentObservation_, false);

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
    const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

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
    */

    
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

  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END ctr_: " << ctr_ << std::endl;
  /*
  if (ctr_ > 0)
  {
    std::cout << "[MPC_ROS_Interface::mpcObservationCallback] DEBUG_INF" << std::endl;
    while(1);
  }
  */
  ctr_++;
}

void MPC_ROS_Interface::spinTimerCallback(const ros::TimerEvent& event) 
{
  //std::cout << "[MPC_ROS_Interface::spinTimerCallback] START" << std::endl;

  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  //std::cout << "[MPC_ROS_Interface::spinTimerCallback] END" << std::endl;
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
void MPC_ROS_Interface::singleSpin() 
{
  //std::cout << "[MPC_ROS_Interface::singleSpin] START" << std::endl;

  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  //std::cout << "[MPC_ROS_Interface::singleSpin] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::spin() 
{
  std::cout << "[MPC_ROS_Interface::spin] Start spinning now..." << std::endl;
  // Equivalent to ros::spin() + check if master is alive
  //while (ros::ok() && ros::master::check() && !shutDownFlag_) 
  while (ros::ok() && ros::master::check()) 
  {
    /*
    mpcShutDownFlag_ = getenv("mpcShutDownFlag");
    //std::cout << "[MPC_ROS_Interface::spin] mpcShutDownFlag_: " << mpcShutDownFlag_ << std::endl;

    if (mpcShutDownFlag_ == "true")
    {
      //std::cout << "[MPC_ROS_Interface::spin] Shutting down..." << std::endl;
      shutDownFlag_ = true;
    }
    */

    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  std::cout << "[MPC_ROS_Interface::spin] MPC END" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  

  //shutdownNode();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) 
{
  std::cout << "[MPC_ROS_Interface::launchNodes] START" << std::endl;

  std::cout << "[MPC_ROS_Interface::launchNodes] MPC node is setting up..." << std::endl;

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

  //spinTimer_ = nodeHandle.createTimer(ros::Duration(0.01), &MPC_ROS_Interface::spinTimerCallback, this);

  //spin();

  std::cout << "[MPC_ROS_Interface::launchNodes] END" << std::endl;
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
    //loadData();
  }
  else
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] NO RESET !!!" << std::endl;
    mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(tt));
  }

  /*
  if (ctr_ >= 0)
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] DATA LOADED !!!" << std::endl;
    mpc_.getSolverPtr()->loadInitialData();
    loadData();
  }
  */
  //copyToBuffer(co);

  // very important, these are variables that are carried in between iterations
  //mpc_.getSolverPtr()->loadInitialData();

  //mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(tt));

  /*
  if (ctr_ > 5)
  {
    std::cout << "[MPC_ROS_Interface::computeTraj2] DEBUG_INF" << std::endl;
    while(1);
  }
  */

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

  //writeData();
  //loadData();

  //std::cout << "[MPC_ROS_Interface::computeTraj2] DEBUG_INF" << std::endl;
  //while(1);

  //ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
  //mpcPolicyPublisher_.publish(mpcPolicyMsg);

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
PrimalSolution MPC_ROS_Interface::getPolicy()
{
  //PrimalSolution ops = mpc_.getSolverPtr()->getPrimalSolution();
  return mpc_.getSolverPtr()->getPrimalSolution();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::writeData()
{
  //std::cout << "[MPC_ROS_Interface::writeData] START" << std::endl;

  std::vector<double> bufferPrimalSolution_time;
  std::vector<std::vector<double>> bufferPrimalSolution_state;
  std::vector<std::vector<double>> bufferPrimalSolution_input;
  std::vector<double> bufferPrimalSolution_controller_time_stamp;
  std::vector<std::vector<double>> bufferPrimalSolution_controller_bias_array;
  std::vector<std::vector<double>> bufferPrimalSolution_controller_delta_bias_array;
  std::vector<std::vector<std::vector<double>>> bufferPrimalSolution_controller_gain_array;

  std::vector<double> mpcTargetTrajectories_time;
  std::vector<std::vector<double>> mpcTargetTrajectories_state;
  std::vector<std::vector<double>> mpcTargetTrajectories_input;

  std::vector<double> initObservation_state;
  std::vector<double> initObservation_fullState;
  std::vector<double> initObservation_input;

  std::vector<double> optimalDualSolution_time;
  std::vector<size_t> optimalDualSolution_postEventIndices;

  std::vector<double> optimalDualSolution_final_stateEq_penalty;
  std::vector<std::vector<double>> optimalDualSolution_final_stateEq_lagrangian;
  std::vector<double> optimalDualSolution_final_stateIneq_penalty;
  std::vector<std::vector<double>> optimalDualSolution_final_stateIneq_lagrangian;
  std::vector<double> optimalDualSolution_final_stateInputEq_penalty;
  std::vector<std::vector<double>> optimalDualSolution_final_stateInputEq_lagrangian;
  std::vector<double> optimalDualSolution_final_stateInputIneq_penalty;
  std::vector<std::vector<double>> optimalDualSolution_final_stateInputIneq_lagrangian;

  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateEq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateEq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateIneq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateIneq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateInputEq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateInputEq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateInputIneq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateInputIneq_lagrangian;

  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateEq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateEq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateIneq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateIneq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateInputEq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateInputEq_lagrangian;
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateInputIneq_penalty;
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateInputIneq_lagrangian;

  std::vector<double> optimalPrimalSolution_time;
  std::vector<std::vector<double>> optimalPrimalSolution_state;
  std::vector<std::vector<double>> optimalPrimalSolution_input;
  std::vector<size_t> optimalPrimalSolution_postEventIndices;
  std::vector<double> optimalPrimalSolution_modeSchedule_eventTimes;
  std::vector<size_t> optimalPrimalSolution_modeSchedule_modeSequence;
  std::vector<double> optimalPrimalSolution_controller_time_stamp;
  std::vector<std::vector<double>> optimalPrimalSolution_controller_bias_array;
  std::vector<std::vector<double>> optimalPrimalSolution_controller_delta_bias_array;
  std::vector<std::vector<std::vector<double>>> optimalPrimalSolution_controller_gain_array;

  // Primal Solution
  for (size_t i = 0; i < bufferPrimalSolutionPtr_->timeTrajectory_.size(); i++)
  {
    bufferPrimalSolution_time.push_back(bufferPrimalSolutionPtr_->timeTrajectory_[i]);
  }

  for (size_t i = 0; i < bufferPrimalSolutionPtr_->stateTrajectory_.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferPrimalSolutionPtr_->stateTrajectory_[i].size(); j++)
    {
      tmp.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i][j]);
    }
    bufferPrimalSolution_state.push_back(tmp);
  }

  for (size_t i = 0; i < bufferPrimalSolutionPtr_->inputTrajectory_.size(); i++)
  {
    std::vector<double> tmp;
    for (size_t j = 0; j < bufferPrimalSolutionPtr_->inputTrajectory_[i].size(); j++)
    {
      tmp.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i][j]);
    }
    bufferPrimalSolution_input.push_back(tmp);
  }

  scalar_array_t bufferPrimalSolution_controllerTimeStamp = bufferPrimalSolutionPtr_->controllerPtr_->getTimeStamp();
  vector_array_t bufferPrimalSolution_controllerBiasArray = bufferPrimalSolutionPtr_->controllerPtr_->getBiasArray();
  vector_array_t bufferPrimalSolution_controllerDeltaBiasArray = bufferPrimalSolutionPtr_->controllerPtr_->getDeltaBiasArray();
  matrix_array_t bufferPrimalSolution_controllerGainArray = bufferPrimalSolutionPtr_->controllerPtr_->getGainArray();

  bufferPrimalSolution_controller_time_stamp.resize(bufferPrimalSolution_controllerTimeStamp.size());
  for (size_t i = 0; i < bufferPrimalSolution_controllerTimeStamp.size(); i++)
  {
    bufferPrimalSolution_controller_time_stamp[i] = bufferPrimalSolution_controllerTimeStamp[i];
  }
  
  bufferPrimalSolution_controller_bias_array.resize(bufferPrimalSolution_controllerBiasArray.size());
  for (size_t i = 0; i < bufferPrimalSolution_controllerBiasArray.size(); i++)
  {
    bufferPrimalSolution_controller_bias_array[i].resize(bufferPrimalSolution_controllerBiasArray[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_controllerBiasArray[i].size(); j++)
    {
      bufferPrimalSolution_controller_bias_array[i][j] = bufferPrimalSolution_controllerBiasArray[i][j];
    }
  }

  bufferPrimalSolution_controller_delta_bias_array.resize(bufferPrimalSolution_controllerDeltaBiasArray.size());
  for (size_t i = 0; i < bufferPrimalSolution_controllerDeltaBiasArray.size(); i++)
  {
    bufferPrimalSolution_controller_delta_bias_array[i].resize(bufferPrimalSolution_controllerDeltaBiasArray[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_controllerDeltaBiasArray[i].size(); j++)
    {
      bufferPrimalSolution_controller_delta_bias_array[i][j] = bufferPrimalSolution_controllerDeltaBiasArray[i][j];
    }
  }

  bufferPrimalSolution_controller_gain_array.resize(bufferPrimalSolution_controllerGainArray.size());
  for (size_t i = 0; i < bufferPrimalSolution_controllerGainArray.size(); i++)
  {
    bufferPrimalSolution_controller_gain_array[i].resize(bufferPrimalSolution_controllerGainArray[i].rows());
    for (size_t j = 0; j < bufferPrimalSolution_controllerGainArray[i].rows(); j++)
    {
      bufferPrimalSolution_controller_gain_array[i][j].resize(bufferPrimalSolution_controllerGainArray[i].cols());
      for (size_t k = 0; k < bufferPrimalSolution_controllerGainArray[i].cols(); k++)
      {
        bufferPrimalSolution_controller_gain_array[i][j][k] = bufferPrimalSolution_controllerGainArray[i](j,k);
      }
    }
  }

  /*
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_time_stamp size: " << bufferPrimalSolution_controller_time_stamp.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_bias_array size: " << bufferPrimalSolution_controller_bias_array.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_bias_array[0] size: " << bufferPrimalSolution_controller_bias_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_delta_bias_array size: " << bufferPrimalSolution_controller_delta_bias_array.size() << std::endl;
  //std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_delta_bias_array[0] size: " << bufferPrimalSolution_controller_delta_bias_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_gain_array size: " << bufferPrimalSolution_controller_gain_array.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_gain_array[0] size: " << bufferPrimalSolution_controller_gain_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] bufferPrimalSolution_controller_gain_array[0][0] size: " << bufferPrimalSolution_controller_gain_array[0][0].size() << std::endl;
  */

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

  // Dual Solution
  DualSolution ods = mpc_.getSolverPtr()->getDualSolution();

  for (size_t i = 0; i < ods.timeTrajectory.size(); i++)
  {
    optimalDualSolution_time.push_back(ods.timeTrajectory[i]);
  }

  for (size_t i = 0; i < ods.postEventIndices.size(); i++)
  {
    optimalDualSolution_postEventIndices.push_back(ods.postEventIndices[i]);
  }

  optimalDualSolution_final_stateEq_penalty.resize(ods.final.stateEq.size());
  optimalDualSolution_final_stateEq_lagrangian.resize(ods.final.stateEq.size());
  for (size_t i = 0; i < ods.final.stateEq.size(); i++)
  {
    optimalDualSolution_final_stateEq_penalty[i] = ods.final.stateEq[i].penalty;

    optimalDualSolution_final_stateEq_lagrangian[i].resize(ods.final.stateEq[i].lagrangian.size());
    for (size_t j = 0; j < ods.final.stateEq[i].lagrangian.size(); j++)
    {
      optimalDualSolution_final_stateEq_lagrangian[i][j] = ods.final.stateEq[i].lagrangian[j];
    }
  }

  optimalDualSolution_final_stateIneq_penalty.resize(ods.final.stateIneq.size());
  optimalDualSolution_final_stateIneq_lagrangian.resize(ods.final.stateIneq.size());
  for (size_t i = 0; i < ods.final.stateIneq.size(); i++)
  {
    optimalDualSolution_final_stateIneq_penalty[i] = ods.final.stateIneq[i].penalty;

    optimalDualSolution_final_stateIneq_lagrangian[i].resize(ods.final.stateIneq[i].lagrangian.size());
    for (size_t j = 0; j < ods.final.stateIneq[i].lagrangian.size(); j++)
    {
      optimalDualSolution_final_stateIneq_lagrangian[i][j] = ods.final.stateIneq[i].lagrangian[j];
    }
  }

  optimalDualSolution_final_stateInputEq_penalty.resize(ods.final.stateInputEq.size());
  optimalDualSolution_final_stateInputEq_lagrangian.resize(ods.final.stateInputEq.size());
  for (size_t i = 0; i < ods.final.stateInputEq.size(); i++)
  {
    optimalDualSolution_final_stateInputEq_penalty[i] = ods.final.stateInputEq[i].penalty;

    optimalDualSolution_final_stateInputEq_lagrangian[i].resize(ods.final.stateInputEq[i].lagrangian.size());
    for (size_t j = 0; j < ods.final.stateInputEq[i].lagrangian.size(); j++)
    {
      optimalDualSolution_final_stateInputEq_lagrangian[i][j] = ods.final.stateInputEq[i].lagrangian[j];
    }
  }

  optimalDualSolution_final_stateInputIneq_penalty.resize(ods.final.stateInputIneq.size());
  optimalDualSolution_final_stateInputIneq_lagrangian.resize(ods.final.stateInputIneq.size());
  for (size_t i = 0; i < ods.final.stateInputIneq.size(); i++)
  {
    optimalDualSolution_final_stateInputIneq_penalty[i] = ods.final.stateInputIneq[i].penalty;

    optimalDualSolution_final_stateInputIneq_lagrangian[i].resize(ods.final.stateInputIneq[i].lagrangian.size());
    for (size_t j = 0; j < ods.final.stateInputIneq[i].lagrangian.size(); j++)
    {
      optimalDualSolution_final_stateInputIneq_lagrangian[i][j] = ods.final.stateInputIneq[i].lagrangian[j];
    }
  }

  optimalDualSolution_preJumps_stateEq_penalty.resize(ods.preJumps.size());
  optimalDualSolution_preJumps_stateEq_lagrangian.resize(ods.preJumps.size());
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    optimalDualSolution_preJumps_stateEq_penalty[k].resize(ods.preJumps[k].stateEq.size());
    optimalDualSolution_preJumps_stateEq_lagrangian[k].resize(ods.preJumps[k].stateEq.size());
    for (size_t i = 0; i < ods.preJumps[k].stateEq.size(); i++)
    {
      optimalDualSolution_preJumps_stateEq_penalty[k][i] = ods.preJumps[k].stateEq[i].penalty;

      optimalDualSolution_preJumps_stateEq_lagrangian[k][i].resize(ods.preJumps[k].stateEq[i].lagrangian.size());
      for (size_t j = 0; j < ods.preJumps[k].stateEq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_preJumps_stateEq_lagrangian[k][i][j] = ods.preJumps[k].stateEq[i].lagrangian[j];
      }
    }
  }

  optimalDualSolution_preJumps_stateIneq_penalty.resize(ods.preJumps.size());
  optimalDualSolution_preJumps_stateIneq_lagrangian.resize(ods.preJumps.size());
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    optimalDualSolution_preJumps_stateIneq_penalty[k].resize(ods.preJumps[k].stateIneq.size());
    optimalDualSolution_preJumps_stateIneq_lagrangian[k].resize(ods.preJumps[k].stateIneq.size());
    for (size_t i = 0; i < ods.preJumps[k].stateIneq.size(); i++)
    {
      optimalDualSolution_preJumps_stateIneq_penalty[k][i] = ods.preJumps[k].stateIneq[i].penalty;

      optimalDualSolution_preJumps_stateIneq_lagrangian[k][i].resize(ods.preJumps[k].stateIneq[i].lagrangian.size());
      for (size_t j = 0; j < ods.preJumps[k].stateIneq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_preJumps_stateIneq_lagrangian[k][i][j] = ods.preJumps[k].stateIneq[i].lagrangian[j];
      }
    }
  }

  optimalDualSolution_preJumps_stateInputEq_penalty.resize(ods.preJumps.size());
  optimalDualSolution_preJumps_stateInputEq_lagrangian.resize(ods.preJumps.size());
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    optimalDualSolution_preJumps_stateInputEq_penalty[k].resize(ods.preJumps[k].stateInputEq.size());
    optimalDualSolution_preJumps_stateInputEq_lagrangian[k].resize(ods.preJumps[k].stateInputEq.size());
    for (size_t i = 0; i < ods.preJumps[k].stateInputEq.size(); i++)
    {
      optimalDualSolution_preJumps_stateInputEq_penalty[k][i] = ods.preJumps[k].stateInputEq[i].penalty;

      optimalDualSolution_preJumps_stateInputEq_lagrangian[k][i].resize(ods.preJumps[k].stateInputEq[i].lagrangian.size());
      for (size_t j = 0; j < ods.preJumps[k].stateInputEq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_preJumps_stateInputEq_lagrangian[k][i][j] = ods.preJumps[k].stateInputEq[i].lagrangian[j];
      }
    }
  }
  
  optimalDualSolution_preJumps_stateInputIneq_penalty.resize(ods.preJumps.size());
  optimalDualSolution_preJumps_stateInputIneq_lagrangian.resize(ods.preJumps.size());
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    optimalDualSolution_preJumps_stateInputIneq_penalty[k].resize(ods.preJumps[k].stateInputIneq.size());
    optimalDualSolution_preJumps_stateInputIneq_lagrangian[k].resize(ods.preJumps[k].stateInputIneq.size());
    for (size_t i = 0; i < ods.preJumps[k].stateInputIneq.size(); i++)
    {
      optimalDualSolution_preJumps_stateInputIneq_penalty[k][i] = ods.preJumps[k].stateInputIneq[i].penalty;

      optimalDualSolution_preJumps_stateInputIneq_lagrangian[k][i].resize(ods.preJumps[k].stateInputIneq[i].lagrangian.size());
      for (size_t j = 0; j < ods.preJumps[k].stateInputIneq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_preJumps_stateInputIneq_lagrangian[k][i][j] = ods.preJumps[k].stateInputIneq[i].lagrangian[j];
      }
    }
  }

  optimalDualSolution_intermediates_stateEq_penalty.resize(ods.intermediates.size());
  optimalDualSolution_intermediates_stateEq_lagrangian.resize(ods.intermediates.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    optimalDualSolution_intermediates_stateEq_penalty[k].resize(ods.intermediates[k].stateEq.size());
    optimalDualSolution_intermediates_stateEq_lagrangian[k].resize(ods.intermediates[k].stateEq.size());
    for (size_t i = 0; i < ods.intermediates[k].stateEq.size(); i++)
    {
      optimalDualSolution_intermediates_stateEq_penalty[k][i] = ods.intermediates[k].stateEq[i].penalty;

      optimalDualSolution_intermediates_stateEq_lagrangian[k][i].resize(ods.intermediates[k].stateEq[i].lagrangian.size());
      for (size_t j = 0; j < ods.intermediates[k].stateEq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_intermediates_stateEq_lagrangian[k][i][j] = ods.intermediates[k].stateEq[i].lagrangian[j];
      }
    }
  }

  optimalDualSolution_intermediates_stateIneq_penalty.resize(ods.intermediates.size());
  optimalDualSolution_intermediates_stateIneq_lagrangian.resize(ods.intermediates.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    optimalDualSolution_intermediates_stateIneq_penalty[k].resize(ods.intermediates[k].stateIneq.size());
    optimalDualSolution_intermediates_stateIneq_lagrangian[k].resize(ods.intermediates[k].stateIneq.size());
    for (size_t i = 0; i < ods.intermediates[k].stateIneq.size(); i++)
    {
      optimalDualSolution_intermediates_stateIneq_penalty[k][i] = ods.intermediates[k].stateIneq[i].penalty;

      optimalDualSolution_intermediates_stateIneq_lagrangian[k][i].resize(ods.intermediates[k].stateIneq[i].lagrangian.size());
      for (size_t j = 0; j < ods.intermediates[k].stateIneq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_intermediates_stateIneq_lagrangian[k][i][j] = ods.intermediates[k].stateIneq[i].lagrangian[j];
      }
    }
  }

  optimalDualSolution_intermediates_stateInputEq_penalty.resize(ods.intermediates.size());
  optimalDualSolution_intermediates_stateInputEq_lagrangian.resize(ods.intermediates.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    optimalDualSolution_intermediates_stateInputEq_penalty[k].resize(ods.intermediates[k].stateInputEq.size());
    optimalDualSolution_intermediates_stateInputEq_lagrangian[k].resize(ods.intermediates[k].stateInputEq.size());
    for (size_t i = 0; i < ods.intermediates[k].stateInputEq.size(); i++)
    {
      optimalDualSolution_intermediates_stateInputEq_penalty[k][i] = ods.intermediates[k].stateInputEq[i].penalty;

      optimalDualSolution_intermediates_stateInputEq_lagrangian[k][i].resize(ods.intermediates[k].stateInputEq[i].lagrangian.size());
      for (size_t j = 0; j < ods.intermediates[k].stateInputEq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_intermediates_stateInputEq_lagrangian[k][i][j] = ods.intermediates[k].stateInputEq[i].lagrangian[j];
      }
    }
  }
  
  optimalDualSolution_intermediates_stateInputIneq_penalty.resize(ods.intermediates.size());
  optimalDualSolution_intermediates_stateInputIneq_lagrangian.resize(ods.intermediates.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    optimalDualSolution_intermediates_stateInputIneq_penalty[k].resize(ods.intermediates[k].stateInputIneq.size());
    optimalDualSolution_intermediates_stateInputIneq_lagrangian[k].resize(ods.intermediates[k].stateInputIneq.size());
    for (size_t i = 0; i < ods.intermediates[k].stateInputIneq.size(); i++)
    {
      optimalDualSolution_intermediates_stateInputIneq_penalty[k][i] = ods.intermediates[k].stateInputIneq[i].penalty;

      optimalDualSolution_intermediates_stateInputIneq_lagrangian[k][i].resize(ods.intermediates[k].stateInputIneq[i].lagrangian.size());
      for (size_t j = 0; j < ods.intermediates[k].stateInputIneq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_intermediates_stateInputIneq_lagrangian[k][i][j] = ods.intermediates[k].stateInputIneq[i].lagrangian[j];
      }
    }
  }
  
  // Primal Solution
  PrimalSolution ops = mpc_.getSolverPtr()->getPrimalSolution();
  
  /*
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl; 
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl; 
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  //ops.controllerPtr_->display();
  std::cout << "[MPC_ROS_Interface::writeData] ops.timeTrajectory_ size: " << ops.timeTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] ops.stateTrajectory_ size: " << ops.stateTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] ops.inputTrajectory_ size: " << ops.inputTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] ops.controllerPtr_ size: " << ops.controllerPtr_->size() << std::endl; 
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl; 
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl; 
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl; 
  */

  optimalPrimalSolution_time.resize(ops.timeTrajectory_.size());
  for (size_t i = 0; i < ops.timeTrajectory_.size(); i++)
  {
    optimalPrimalSolution_time[i] = ops.timeTrajectory_[i];
  }

  optimalPrimalSolution_state.resize(ops.stateTrajectory_.size());
  for (size_t i = 0; i < ops.stateTrajectory_.size(); i++)
  {
    optimalPrimalSolution_state[i].resize(ops.stateTrajectory_[i].size());
    for (size_t j = 0; j < ops.stateTrajectory_[i].size(); j++)
    {
      optimalPrimalSolution_state[i][j] = ops.stateTrajectory_[i][j];
    }
  }

  optimalPrimalSolution_input.resize(ops.inputTrajectory_.size());
  for (size_t i = 0; i < ops.inputTrajectory_.size(); i++)
  {
    optimalPrimalSolution_input[i].resize(ops.inputTrajectory_[i].size());
    for (size_t j = 0; j < ops.inputTrajectory_[i].size(); j++)
    {
      optimalPrimalSolution_input[i][j] = ops.inputTrajectory_[i][j];
    }
  }

  optimalPrimalSolution_postEventIndices.resize(ops.postEventIndices_.size());
  for (size_t i = 0; i < ops.postEventIndices_.size(); i++)
  {
    optimalPrimalSolution_postEventIndices[i] = ops.postEventIndices_[i];
  }

  optimalPrimalSolution_modeSchedule_eventTimes.resize(ops.modeSchedule_.eventTimes.size());
  for (size_t i = 0; i < ops.modeSchedule_.eventTimes.size(); i++)
  {
    optimalPrimalSolution_modeSchedule_eventTimes[i] = ops.modeSchedule_.eventTimes[i];
  }

  optimalPrimalSolution_modeSchedule_modeSequence.resize(ops.modeSchedule_.modeSequence.size());
  for (size_t i = 0; i < ops.modeSchedule_.modeSequence.size(); i++)
  {
    optimalPrimalSolution_modeSchedule_modeSequence[i] = ops.modeSchedule_.modeSequence[i];
  }

  scalar_array_t controllerTimeStamp = ops.controllerPtr_->getTimeStamp();
  vector_array_t controllerBiasArray = ops.controllerPtr_->getBiasArray();
  vector_array_t controllerDeltaBiasArray = ops.controllerPtr_->getDeltaBiasArray();
  matrix_array_t controllerGainArray = ops.controllerPtr_->getGainArray();

  optimalPrimalSolution_controller_time_stamp.resize(controllerTimeStamp.size());
  for (size_t i = 0; i < controllerTimeStamp.size(); i++)
  {
    optimalPrimalSolution_controller_time_stamp[i] = controllerTimeStamp[i];
  }
  
  optimalPrimalSolution_controller_bias_array.resize(controllerBiasArray.size());
  for (size_t i = 0; i < controllerBiasArray.size(); i++)
  {
    optimalPrimalSolution_controller_bias_array[i].resize(controllerBiasArray[i].size());
    for (size_t j = 0; j < controllerBiasArray[i].size(); j++)
    {
      optimalPrimalSolution_controller_bias_array[i][j] = controllerBiasArray[i][j];
    }
  }

  optimalPrimalSolution_controller_delta_bias_array.resize(controllerDeltaBiasArray.size());
  for (size_t i = 0; i < controllerDeltaBiasArray.size(); i++)
  {
    optimalPrimalSolution_controller_delta_bias_array[i].resize(controllerDeltaBiasArray[i].size());
    for (size_t j = 0; j < controllerDeltaBiasArray[i].size(); j++)
    {
      optimalPrimalSolution_controller_delta_bias_array[i][j] = controllerDeltaBiasArray[i][j];
    }
  }

  optimalPrimalSolution_controller_gain_array.resize(controllerGainArray.size());
  for (size_t i = 0; i < controllerGainArray.size(); i++)
  {
    optimalPrimalSolution_controller_gain_array[i].resize(controllerGainArray[i].rows());
    for (size_t j = 0; j < controllerGainArray[i].rows(); j++)
    {
      optimalPrimalSolution_controller_gain_array[i][j].resize(controllerGainArray[i].cols());
      for (size_t k = 0; k < controllerGainArray[i].cols(); k++)
      {
        optimalPrimalSolution_controller_gain_array[i][j][k] = controllerGainArray[i](j,k);
      }
    }
  }
  
  /*
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_time_stamp size: " << optimalPrimalSolution_controller_time_stamp.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_bias_array size: " << optimalPrimalSolution_controller_bias_array.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_bias_array[0] size: " << optimalPrimalSolution_controller_bias_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_delta_bias_array size: " << optimalPrimalSolution_controller_delta_bias_array.size() << std::endl;
  //std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_delta_bias_array[0] size: " << optimalPrimalSolution_controller_delta_bias_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_gain_array size: " << optimalPrimalSolution_controller_gain_array.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_gain_array[0] size: " << optimalPrimalSolution_controller_gain_array[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::writeData] optimalPrimalSolution_controller_gain_array[0][0] size: " << optimalPrimalSolution_controller_gain_array[0][0].size() << std::endl;
  */

  //std::cout << "[MPC_ROS_Interface::writeData] DEBUG_INF" << std::endl;
  //while(1);

  //std::cout << "[MPC_ROS_Interface::writeData] controllerPtr_ size:" << ops.controllerPtr_->size() << std::endl;
  
  /*
  if (ops.controllerPtr_->getType() == ControllerType::LINEAR)
  {
    std::cout << "[MPC_ROS_Interface::writeData] ControllerType::LINEAR" << std::endl;
  }
  else if (ops.controllerPtr_->getType() == ControllerType::FEEDFORWARD)
  {
    std::cout << "[MPC_ROS_Interface::writeData] ControllerType::FEEDFORWARD" << std::endl;
  }
  else
  {
    std::cout << "[MPC_ROS_Interface::writeData] ControllerType::WTF?!" << std::endl;
  }
  //ops.controllerPtr_->display();
  */

  //std::cout << "[MPC_ROS_Interface::writeData] DEBUG_INF" << std::endl;
  //while(1);

  nlohmann::json j;
  j["bufferPrimalSolution_time"] = bufferPrimalSolution_time;
  j["bufferPrimalSolution_state"] = bufferPrimalSolution_state;
  j["bufferPrimalSolution_input"] = bufferPrimalSolution_input;
  j["bufferPrimalSolution_controller_time_stamp"] = bufferPrimalSolution_controller_time_stamp;
  j["bufferPrimalSolution_controller_bias_array"] = bufferPrimalSolution_controller_bias_array;
  j["bufferPrimalSolution_controller_delta_bias_array"] = bufferPrimalSolution_controller_delta_bias_array;
  j["bufferPrimalSolution_controller_gain_array"] = bufferPrimalSolution_controller_gain_array;

  j["mpcTargetTrajectories_time"] = mpcTargetTrajectories_time;
  j["mpcTargetTrajectories_state"] = mpcTargetTrajectories_state;
  j["mpcTargetTrajectories_input"] = mpcTargetTrajectories_input;

  j["initObservation_mode"] = bufferCommandPtr_->mpcInitObservation_.mode;
  j["initObservation_time"] = bufferCommandPtr_->mpcInitObservation_.time;
  j["initObservation_state"] = initObservation_state;
  j["initObservation_fullState"] = initObservation_fullState;
  j["initObservation_input"] = initObservation_input;

  j["optimalDualSolution_time"] = optimalDualSolution_time;
  j["optimalDualSolution_postEventIndices"] = optimalDualSolution_postEventIndices;
  
  j["optimalDualSolution_final_stateEq_penalty"] = optimalDualSolution_final_stateEq_penalty;
  j["optimalDualSolution_final_stateEq_lagrangian"] = optimalDualSolution_final_stateEq_lagrangian;
  j["optimalDualSolution_final_stateIneq_penalty"] = optimalDualSolution_final_stateIneq_penalty;
  j["optimalDualSolution_final_stateIneq_lagrangian"] = optimalDualSolution_final_stateIneq_lagrangian;
  j["optimalDualSolution_final_stateInputEq_penalty"] = optimalDualSolution_final_stateInputEq_penalty;
  j["optimalDualSolution_final_stateInputEq_lagrangian"] = optimalDualSolution_final_stateInputEq_lagrangian;
  j["optimalDualSolution_final_stateInputIneq_penalty"] = optimalDualSolution_final_stateInputIneq_penalty;
  j["optimalDualSolution_final_stateInputIneq_lagrangian"] = optimalDualSolution_final_stateInputIneq_lagrangian;
  
  j["optimalDualSolution_preJumps_stateEq_penalty"] = optimalDualSolution_preJumps_stateEq_penalty;
  j["optimalDualSolution_preJumps_stateEq_lagrangian"] = optimalDualSolution_preJumps_stateEq_lagrangian;
  j["optimalDualSolution_preJumps_stateIneq_penalty"] = optimalDualSolution_preJumps_stateIneq_penalty;
  j["optimalDualSolution_preJumps_stateIneq_lagrangian"] = optimalDualSolution_preJumps_stateIneq_lagrangian;
  j["optimalDualSolution_preJumps_stateInputEq_penalty"] = optimalDualSolution_preJumps_stateInputEq_penalty;
  j["optimalDualSolution_preJumps_stateInputEq_lagrangian"] = optimalDualSolution_preJumps_stateInputEq_lagrangian;
  j["optimalDualSolution_preJumps_stateInputIneq_penalty"] = optimalDualSolution_preJumps_stateInputIneq_penalty;
  j["optimalDualSolution_preJumps_stateInputIneq_lagrangian"] = optimalDualSolution_preJumps_stateInputIneq_lagrangian;

  j["optimalDualSolution_intermediates_stateEq_penalty"] = optimalDualSolution_intermediates_stateEq_penalty;
  j["optimalDualSolution_intermediates_stateEq_lagrangian"] = optimalDualSolution_intermediates_stateEq_lagrangian;
  j["optimalDualSolution_intermediates_stateIneq_penalty"] = optimalDualSolution_intermediates_stateIneq_penalty;
  j["optimalDualSolution_intermediates_stateIneq_lagrangian"] = optimalDualSolution_intermediates_stateIneq_lagrangian;
  j["optimalDualSolution_intermediates_stateInputEq_penalty"] = optimalDualSolution_intermediates_stateInputEq_penalty;
  j["optimalDualSolution_intermediates_stateInputEq_lagrangian"] = optimalDualSolution_intermediates_stateInputEq_lagrangian;
  j["optimalDualSolution_intermediates_stateInputIneq_penalty"] = optimalDualSolution_intermediates_stateInputIneq_penalty;
  j["optimalDualSolution_intermediates_stateInputIneq_lagrangian"] = optimalDualSolution_intermediates_stateInputIneq_lagrangian;

  j["optimalPrimalSolution_time"] = optimalPrimalSolution_time;
  j["optimalPrimalSolution_state"] = optimalPrimalSolution_state;
  j["optimalPrimalSolution_input"] = optimalPrimalSolution_input;
  j["optimalPrimalSolution_postEventIndices"] = optimalPrimalSolution_postEventIndices;
  j["optimalPrimalSolution_modeSchedule_eventTimes"] = optimalPrimalSolution_modeSchedule_eventTimes;
  j["optimalPrimalSolution_modeSchedule_modeSequence"] = optimalPrimalSolution_modeSchedule_modeSequence;
  j["optimalPrimalSolution_controller_time_stamp"] = optimalPrimalSolution_controller_time_stamp;
  j["optimalPrimalSolution_controller_bias_array"] = optimalPrimalSolution_controller_bias_array;
  j["optimalPrimalSolution_controller_delta_bias_array"] = optimalPrimalSolution_controller_delta_bias_array;
  j["optimalPrimalSolution_controller_gain_array"] = optimalPrimalSolution_controller_gain_array;

  std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
  std::string dataPath = pkg_dir + "dataset/ocs2/tmp/";

  boost::filesystem::create_directories(dataPath);
  std::string filename = "init_data.json";
  std::ofstream o(dataPath + filename);
  o << std::setw(4) << j << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::loadData()
{
  //std::cout << "[MPC_ROS_Interface::loadData] START" << std::endl;

  std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
  std::string dataPath = pkg_dir + "dataset/ocs2/tmp/";

  std::ifstream f(dataPath + "init_data.json");
  nlohmann::json data = nlohmann::json::parse(f);

  std::vector<double> bufferPrimalSolution_time = data["bufferPrimalSolution_time"];
  std::vector<std::vector<double>> bufferPrimalSolution_state = data["bufferPrimalSolution_state"];
  std::vector<std::vector<double>> bufferPrimalSolution_input = data["bufferPrimalSolution_input"];
  std::vector<double> bufferPrimalSolution_controller_time_stamp = data["bufferPrimalSolution_controller_time_stamp"];
  std::vector<std::vector<double>> bufferPrimalSolution_controller_bias_array = data["bufferPrimalSolution_controller_bias_array"];
  std::vector<std::vector<double>> bufferPrimalSolution_controller_delta_bias_array = data["bufferPrimalSolution_controller_delta_bias_array"];
  std::vector<std::vector<std::vector<double>>> bufferPrimalSolution_controller_gain_array = data["bufferPrimalSolution_controller_gain_array"];

  std::vector<double> mpcTargetTrajectories_time = data["mpcTargetTrajectories_time"];
  std::vector<std::vector<double>> mpcTargetTrajectories_state = data["mpcTargetTrajectories_state"];
  std::vector<std::vector<double>> mpcTargetTrajectories_input = data["mpcTargetTrajectories_input"];

  double initObservation_mode = data["initObservation_mode"];
  double initObservation_time = data["initObservation_time"];
  std::vector<double> initObservation_state = data["initObservation_state"];
  std::vector<double> initObservation_fullState =  data["initObservation_fullState"];
  std::vector<double> initObservation_input =  data["initObservation_input"];

  /////////////////////

  std::vector<double> optimalDualSolution_time = data["optimalDualSolution_time"];
  std::vector<size_t> optimalDualSolution_postEventIndices = data["optimalDualSolution_postEventIndices"];

  std::vector<double> optimalDualSolution_final_stateEq_penalty = data["optimalDualSolution_final_stateEq_penalty"];
  std::vector<std::vector<double>> optimalDualSolution_final_stateEq_lagrangian = data["optimalDualSolution_final_stateEq_lagrangian"];
  std::vector<double> optimalDualSolution_final_stateIneq_penalty = data["optimalDualSolution_final_stateIneq_penalty"];
  std::vector<std::vector<double>> optimalDualSolution_final_stateIneq_lagrangian = data["optimalDualSolution_final_stateIneq_lagrangian"];
  std::vector<double> optimalDualSolution_final_stateInputEq_penalty = data["optimalDualSolution_final_stateInputEq_penalty"];
  std::vector<std::vector<double>> optimalDualSolution_final_stateInputEq_lagrangian = data["optimalDualSolution_final_stateInputEq_lagrangian"];
  std::vector<double> optimalDualSolution_final_stateInputIneq_penalty = data["optimalDualSolution_final_stateInputIneq_penalty"];
  std::vector<std::vector<double>> optimalDualSolution_final_stateInputIneq_lagrangian = data["optimalDualSolution_final_stateInputIneq_lagrangian"];

  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateEq_penalty = data["optimalDualSolution_preJumps_stateEq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateEq_lagrangian = data["optimalDualSolution_preJumps_stateEq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateIneq_penalty = data["optimalDualSolution_preJumps_stateIneq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateIneq_lagrangian = data["optimalDualSolution_preJumps_stateIneq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateInputEq_penalty = data["optimalDualSolution_preJumps_stateInputEq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateInputEq_lagrangian = data["optimalDualSolution_preJumps_stateInputEq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_preJumps_stateInputIneq_penalty = data["optimalDualSolution_preJumps_stateInputIneq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_preJumps_stateInputIneq_lagrangian = data["optimalDualSolution_preJumps_stateInputIneq_lagrangian"];

  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateEq_penalty = data["optimalDualSolution_intermediates_stateEq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateEq_lagrangian = data["optimalDualSolution_intermediates_stateEq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateIneq_penalty = data["optimalDualSolution_intermediates_stateIneq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateIneq_lagrangian = data["optimalDualSolution_intermediates_stateIneq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateInputEq_penalty = data["optimalDualSolution_intermediates_stateInputEq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateInputEq_lagrangian = data["optimalDualSolution_intermediates_stateInputEq_lagrangian"];
  std::vector<std::vector<double>> optimalDualSolution_intermediates_stateInputIneq_penalty = data["optimalDualSolution_intermediates_stateInputIneq_penalty"];
  std::vector<std::vector<std::vector<double>>> optimalDualSolution_intermediates_stateInputIneq_lagrangian = data["optimalDualSolution_intermediates_stateInputIneq_lagrangian"];

  std::vector<double> optimalPrimalSolution_time = data["optimalPrimalSolution_time"];
  std::vector<std::vector<double>> optimalPrimalSolution_state = data["optimalPrimalSolution_state"];
  std::vector<std::vector<double>> optimalPrimalSolution_input = data["optimalPrimalSolution_input"];
  std::vector<size_t> optimalPrimalSolution_postEventIndices = data["optimalPrimalSolution_postEventIndices"];
  std::vector<double> optimalPrimalSolution_modeSchedule_eventTimes = data["optimalPrimalSolution_modeSchedule_eventTimes"];
  std::vector<size_t> optimalPrimalSolution_modeSchedule_modeSequence = data["optimalPrimalSolution_modeSchedule_modeSequence"];
  std::vector<double> optimalPrimalSolution_controller_time_stamp = data["optimalPrimalSolution_controller_time_stamp"];
  std::vector<std::vector<double>> optimalPrimalSolution_controller_bias_array = data["optimalPrimalSolution_controller_bias_array"];
  std::vector<std::vector<double>> optimalPrimalSolution_controller_delta_bias_array = data["optimalPrimalSolution_controller_delta_bias_array"];
  std::vector<std::vector<std::vector<double>>> optimalPrimalSolution_controller_gain_array = data["optimalPrimalSolution_controller_gain_array"];

  /////////////////////

  // Primal Solution
  bufferPrimalSolutionPtr_->timeTrajectory_.clear();
  for (size_t i = 0; i < bufferPrimalSolution_time.size(); i++)
  {
    bufferPrimalSolutionPtr_->timeTrajectory_.push_back(bufferPrimalSolution_time[i]);
  }

  //const size_t N_1 = bufferPrimalSolutionPtr_->timeTrajectory_.size();
  //size_array_t stateDim_1(N_1);
  //size_array_t inputDim_1(N_1);

  bufferPrimalSolutionPtr_->stateTrajectory_.clear();
  bufferPrimalSolutionPtr_->stateTrajectory_.resize(bufferPrimalSolution_state.size());
  for (size_t i = 0; i < bufferPrimalSolution_state.size(); i++)
  {
    //stateDim_1[i] = bufferPrimalSolution_state[i].size();
    bufferPrimalSolutionPtr_->stateTrajectory_[i].resize(bufferPrimalSolution_state[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_state[i].size(); j++)
    {
      bufferPrimalSolutionPtr_->stateTrajectory_[i][j] = bufferPrimalSolution_state[i][j];
    }
  }

  bufferPrimalSolutionPtr_->inputTrajectory_.clear();
  bufferPrimalSolutionPtr_->inputTrajectory_.resize(bufferPrimalSolution_input.size());
  for (size_t i = 0; i < bufferPrimalSolution_input.size(); i++)
  {
    //inputDim_1[i] = bufferPrimalSolution_input[i].size();
    bufferPrimalSolutionPtr_->inputTrajectory_[i].resize(bufferPrimalSolution_input[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_input[i].size(); j++)
    {
      bufferPrimalSolutionPtr_->inputTrajectory_[i][j] = bufferPrimalSolution_input[i][j];
    }
  }

  scalar_array_t bufferPrimalSolution_controllerTimeStamp;
  vector_array_t bufferPrimalSolution_controllerBiasArray;
  vector_array_t bufferPrimalSolution_controllerDeltaBiasArray;
  matrix_array_t bufferPrimalSolution_controllerGainArray;

  bufferPrimalSolution_controllerTimeStamp.resize(bufferPrimalSolution_controller_time_stamp.size());
  for (size_t i = 0; i < bufferPrimalSolution_controller_time_stamp.size(); i++)
  {
    bufferPrimalSolution_controllerTimeStamp[i] = bufferPrimalSolution_controller_time_stamp[i];
  }
  
  bufferPrimalSolution_controllerBiasArray.resize(bufferPrimalSolution_controller_bias_array.size());
  for (size_t i = 0; i < bufferPrimalSolution_controller_bias_array.size(); i++)
  {
    bufferPrimalSolution_controllerBiasArray[i].resize(bufferPrimalSolution_controller_bias_array[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_controller_bias_array[i].size(); j++)
    {
      bufferPrimalSolution_controllerBiasArray[i][j] = bufferPrimalSolution_controller_bias_array[i][j];
    }
  }

  bufferPrimalSolution_controllerDeltaBiasArray.resize(bufferPrimalSolution_controller_delta_bias_array.size());
  for (size_t i = 0; i < bufferPrimalSolution_controller_delta_bias_array.size(); i++)
  {
    bufferPrimalSolution_controllerDeltaBiasArray[i].resize(bufferPrimalSolution_controller_delta_bias_array[i].size());
    for (size_t j = 0; j < bufferPrimalSolution_controller_delta_bias_array[i].size(); j++)
    {
      bufferPrimalSolution_controllerDeltaBiasArray[i][j] = bufferPrimalSolution_controller_delta_bias_array[i][j];
    }
  }

  bufferPrimalSolution_controllerGainArray.resize(bufferPrimalSolution_controller_gain_array.size());
  for (size_t i = 0; i < bufferPrimalSolution_controller_gain_array.size(); i++)
  {
    int row_size = bufferPrimalSolution_controller_gain_array[i].size();
    int col_size = bufferPrimalSolution_controller_gain_array[i][0].size();
    bufferPrimalSolution_controllerGainArray[i].resize(row_size, col_size);
    for (size_t j = 0; j < row_size; j++)
    {
      for (size_t k = 0; k < col_size; k++)
      {
        bufferPrimalSolution_controllerGainArray[i](j,k) = bufferPrimalSolution_controller_gain_array[i][j][k];
      }
    }
  }

  bufferPrimalSolutionPtr_->controllerPtr_.reset(new LinearController);
  bufferPrimalSolutionPtr_->controllerPtr_->setTimeStamp(bufferPrimalSolution_controllerTimeStamp);
  bufferPrimalSolutionPtr_->controllerPtr_->setBiasArray(bufferPrimalSolution_controllerBiasArray);
  bufferPrimalSolutionPtr_->controllerPtr_->setDeltaBiasArray(bufferPrimalSolution_controllerDeltaBiasArray);
  bufferPrimalSolutionPtr_->controllerPtr_->setGainArray(bufferPrimalSolution_controllerGainArray);

  /*
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getTimeStamp size: " << bufferPrimalSolutionPtr_->controllerPtr_->getTimeStamp().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getBiasArray size: " << bufferPrimalSolutionPtr_->controllerPtr_->getBiasArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getBiasArray[0] size: " << bufferPrimalSolutionPtr_->controllerPtr_->getBiasArray()[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getDeltaBiasArray size: " << bufferPrimalSolutionPtr_->controllerPtr_->getDeltaBiasArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getGainArray size: " << bufferPrimalSolutionPtr_->controllerPtr_->getGainArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getGainArray rows: " << bufferPrimalSolutionPtr_->controllerPtr_->getGainArray()[0].rows() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] bufferPrimalSolutionPtr_ getGainArray cols: " << bufferPrimalSolutionPtr_->controllerPtr_->getGainArray()[0].cols() << std::endl;
  */

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

  // Dual Solution
  DualSolution ods;

  ods.timeTrajectory.resize(optimalDualSolution_time.size());
  for (size_t i = 0; i < ods.timeTrajectory.size(); i++)
  {
    ods.timeTrajectory[i] = optimalDualSolution_time[i];
  }

  ods.postEventIndices.resize(optimalDualSolution_postEventIndices.size());
  for (size_t i = 0; i < ods.postEventIndices.size(); i++)
  {
    ods.postEventIndices[i] = optimalDualSolution_postEventIndices[i];
  }

  ods.final.stateEq.resize(optimalDualSolution_final_stateEq_penalty.size());
  for (size_t i = 0; i < ods.final.stateEq.size(); i++)
  {
    ods.final.stateEq[i].penalty = optimalDualSolution_final_stateEq_penalty[i];

    ods.final.stateEq[i].lagrangian.resize(optimalDualSolution_final_stateEq_lagrangian[i].size());
    for (size_t j = 0; j < ods.final.stateEq[i].lagrangian.size(); j++)
    {
      ods.final.stateEq[i].lagrangian[j] = optimalDualSolution_final_stateEq_lagrangian[i][j];
    }
  }

  ods.final.stateIneq.resize(optimalDualSolution_final_stateIneq_penalty.size());
  for (size_t i = 0; i < ods.final.stateIneq.size(); i++)
  {
    ods.final.stateIneq[i].penalty = optimalDualSolution_final_stateIneq_penalty[i];

    ods.final.stateIneq[i].lagrangian.resize(optimalDualSolution_final_stateIneq_lagrangian[i].size());
    for (size_t j = 0; j < ods.final.stateIneq[i].lagrangian.size(); j++)
    {
      ods.final.stateIneq[i].lagrangian[j] = optimalDualSolution_final_stateIneq_lagrangian[i][j];
    }
  }

  ods.final.stateInputEq.resize(optimalDualSolution_final_stateInputEq_penalty.size());
  for (size_t i = 0; i < ods.final.stateInputEq.size(); i++)
  {
    ods.final.stateInputEq[i].penalty = optimalDualSolution_final_stateInputEq_penalty[i];
    
    ods.final.stateInputEq[i].lagrangian.resize(optimalDualSolution_final_stateInputEq_lagrangian[i].size());
    for (size_t j = 0; j < ods.final.stateInputEq[i].lagrangian.size(); j++)
    {
      ods.final.stateInputEq[i].lagrangian[j] = optimalDualSolution_final_stateInputEq_lagrangian[i][j];
    }
  }

  ods.final.stateInputIneq.resize(optimalDualSolution_final_stateInputIneq_penalty.size());
  for (size_t i = 0; i < ods.final.stateInputIneq.size(); i++)
  {
    ods.final.stateInputIneq[i].penalty = optimalDualSolution_final_stateInputIneq_penalty[i];

    ods.final.stateInputIneq[i].lagrangian.resize(optimalDualSolution_final_stateInputIneq_lagrangian.size());
    for (size_t j = 0; j < ods.final.stateInputIneq[i].lagrangian.size(); j++)
    {
      ods.final.stateInputIneq[i].lagrangian[j] = optimalDualSolution_final_stateInputIneq_lagrangian[i][j];
    }
  }

  ods.preJumps.resize(optimalDualSolution_preJumps_stateEq_penalty.size());
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    ods.preJumps[k].stateEq.resize(optimalDualSolution_preJumps_stateEq_penalty[k].size());
    for (size_t i = 0; i < ods.preJumps[k].stateEq.size(); i++)
    {
      ods.preJumps[k].stateEq[i].penalty = optimalDualSolution_preJumps_stateEq_penalty[k][i];

      ods.preJumps[k].stateEq[i].lagrangian.resize(optimalDualSolution_preJumps_stateEq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.preJumps[k].stateEq[i].lagrangian.size(); j++)
      {
        ods.preJumps[k].stateEq[i].lagrangian[j] = optimalDualSolution_preJumps_stateEq_lagrangian[k][i][j];
      }
    }
  }

  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    ods.preJumps[k].stateIneq.resize(optimalDualSolution_preJumps_stateIneq_penalty[k].size());
    for (size_t i = 0; i < ods.preJumps[k].stateIneq.size(); i++)
    {
      ods.preJumps[k].stateIneq[i].penalty = optimalDualSolution_preJumps_stateIneq_penalty[k][i];

      ods.preJumps[k].stateIneq[i].lagrangian.resize(optimalDualSolution_preJumps_stateIneq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.preJumps[k].stateIneq[i].lagrangian.size(); j++)
      {
        ods.preJumps[k].stateIneq[i].lagrangian[j] = optimalDualSolution_preJumps_stateIneq_lagrangian[k][i][j];
      }
    }
  }

  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    ods.preJumps[k].stateInputEq.resize(optimalDualSolution_preJumps_stateInputEq_penalty[k].size());
    for (size_t i = 0; i < ods.preJumps[k].stateInputEq.size(); i++)
    {
      ods.preJumps[k].stateInputEq[i].penalty = optimalDualSolution_preJumps_stateInputEq_penalty[k][i];

      ods.preJumps[k].stateInputEq[i].lagrangian.resize(optimalDualSolution_preJumps_stateInputEq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.preJumps[k].stateInputEq[i].lagrangian.size(); j++)
      {
        ods.preJumps[k].stateInputEq[i].lagrangian[j] = optimalDualSolution_preJumps_stateInputEq_lagrangian[k][i][j];
      }
    }
  }
  
  for (size_t k = 0; k < ods.preJumps.size(); k++)
  {
    ods.preJumps[k].stateInputIneq.resize(optimalDualSolution_preJumps_stateInputIneq_penalty[k].size());
    for (size_t i = 0; i < ods.preJumps[k].stateInputIneq.size(); i++)
    {
      ods.preJumps[k].stateInputIneq[i].penalty = optimalDualSolution_preJumps_stateInputIneq_penalty[k][i];

      ods.preJumps[k].stateInputIneq[i].lagrangian.resize(optimalDualSolution_preJumps_stateInputIneq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.preJumps[k].stateInputIneq[i].lagrangian.size(); j++)
      {
        ods.preJumps[k].stateInputIneq[i].lagrangian[j] = optimalDualSolution_preJumps_stateInputIneq_lagrangian[k][i][j];
      }
    }
  }

  optimalDualSolution_intermediates_stateEq_penalty.resize(ods.intermediates.size());
  optimalDualSolution_intermediates_stateEq_lagrangian.resize(ods.intermediates.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    optimalDualSolution_intermediates_stateEq_penalty[k].resize(ods.intermediates[k].stateEq.size());
    optimalDualSolution_intermediates_stateEq_lagrangian[k].resize(ods.intermediates[k].stateEq.size());
    for (size_t i = 0; i < ods.intermediates[k].stateEq.size(); i++)
    {
      optimalDualSolution_intermediates_stateEq_penalty[k][i] = ods.intermediates[k].stateEq[i].penalty;

      optimalDualSolution_intermediates_stateEq_lagrangian[k][i].resize(ods.intermediates[k].stateEq[i].lagrangian.size());
      for (size_t j = 0; j < ods.intermediates[k].stateEq[i].lagrangian.size(); j++)
      {
        optimalDualSolution_intermediates_stateEq_lagrangian[k][i][j] = ods.intermediates[k].stateEq[i].lagrangian[j];
      }
    }
  }

  ods.intermediates.resize(optimalDualSolution_intermediates_stateIneq_penalty.size());
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    ods.intermediates[k].stateIneq.resize(optimalDualSolution_intermediates_stateIneq_penalty[k].size());
    for (size_t i = 0; i < ods.intermediates[k].stateIneq.size(); i++)
    {
      ods.intermediates[k].stateIneq[i].penalty = optimalDualSolution_intermediates_stateIneq_penalty[k][i];

      ods.intermediates[k].stateIneq[i].lagrangian.resize(optimalDualSolution_intermediates_stateIneq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.intermediates[k].stateIneq[i].lagrangian.size(); j++)
      {
        ods.intermediates[k].stateIneq[i].lagrangian[j] = optimalDualSolution_intermediates_stateIneq_lagrangian[k][i][j];
      }
    }
  }

  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    ods.intermediates[k].stateInputEq.resize(optimalDualSolution_intermediates_stateInputEq_penalty[k].size());
    for (size_t i = 0; i < ods.intermediates[k].stateInputEq.size(); i++)
    {
      ods.intermediates[k].stateInputEq[i].penalty = optimalDualSolution_intermediates_stateInputEq_penalty[k][i];

      ods.intermediates[k].stateInputEq[i].lagrangian.resize(optimalDualSolution_intermediates_stateInputEq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.intermediates[k].stateInputEq[i].lagrangian.size(); j++)
      {
        ods.intermediates[k].stateInputEq[i].lagrangian[j] = optimalDualSolution_intermediates_stateInputEq_lagrangian[k][i][j];
      }
    }
  }
  
  for (size_t k = 0; k < ods.intermediates.size(); k++)
  {
    ods.intermediates[k].stateInputIneq.resize(optimalDualSolution_intermediates_stateInputIneq_penalty[k].size());
    for (size_t i = 0; i < ods.intermediates[k].stateInputIneq.size(); i++)
    {
      ods.intermediates[k].stateInputIneq[i].penalty = optimalDualSolution_intermediates_stateInputIneq_penalty[k][i];

      ods.intermediates[k].stateInputIneq[i].lagrangian.resize(optimalDualSolution_intermediates_stateInputIneq_lagrangian[k][i].size());
      for (size_t j = 0; j < ods.intermediates[k].stateInputIneq[i].lagrangian.size(); j++)
      {
        ods.intermediates[k].stateInputIneq[i].lagrangian[j] = optimalDualSolution_intermediates_stateInputIneq_lagrangian[k][i][j];
      }
    }
  }

  mpc_.getSolverPtr()->setOptimizedDualSolution(ods);

  ///////////////////

  PrimalSolution ops;
  
  ops.timeTrajectory_.resize(optimalPrimalSolution_time.size());
  for (size_t i = 0; i < ops.timeTrajectory_.size(); i++)
  {
    ops.timeTrajectory_[i] = optimalPrimalSolution_time[i];
  }

  const size_t N = ops.timeTrajectory_.size();
  size_array_t stateDim(N);
  size_array_t inputDim(N);

  ops.stateTrajectory_.resize(optimalPrimalSolution_state.size());
  for (size_t i = 0; i < ops.stateTrajectory_.size(); i++)
  {
    stateDim[i] = optimalPrimalSolution_state[i].size();
    ops.stateTrajectory_[i].resize(optimalPrimalSolution_state[i].size());
    for (size_t j = 0; j < ops.stateTrajectory_[i].size(); j++)
    {
      ops.stateTrajectory_[i][j] = optimalPrimalSolution_state[i][j];
    }
  }

  ops.inputTrajectory_.resize(optimalPrimalSolution_input.size());
  for (size_t i = 0; i < ops.inputTrajectory_.size(); i++)
  {
    inputDim[i] = optimalPrimalSolution_input[i].size();
    ops.inputTrajectory_[i].resize(optimalPrimalSolution_input[i].size());
    for (size_t j = 0; j < ops.inputTrajectory_[i].size(); j++)
    {
      ops.inputTrajectory_[i][j] = optimalPrimalSolution_input[i][j];
    }
  }

  ops.postEventIndices_.resize(optimalPrimalSolution_postEventIndices.size());
  for (size_t i = 0; i < ops.postEventIndices_.size(); i++)
  {
    ops.postEventIndices_[i] = optimalPrimalSolution_postEventIndices[i];
  }

  ops.modeSchedule_.eventTimes.resize(optimalPrimalSolution_modeSchedule_eventTimes.size());
  for (size_t i = 0; i < ops.modeSchedule_.eventTimes.size(); i++)
  {
    ops.modeSchedule_.eventTimes[i] = optimalPrimalSolution_modeSchedule_eventTimes[i];
  }

  ops.modeSchedule_.modeSequence.resize(optimalPrimalSolution_modeSchedule_modeSequence.size());
  for (size_t i = 0; i < ops.modeSchedule_.modeSequence.size(); i++)
  {
    ops.modeSchedule_.modeSequence[i] = optimalPrimalSolution_modeSchedule_modeSequence[i];
  }

  scalar_array_t optimalPrimalSolution_controllerTimeStamp;
  vector_array_t optimalPrimalSolution_controllerBiasArray;
  vector_array_t optimalPrimalSolution_controllerDeltaBiasArray;
  matrix_array_t optimalPrimalSolution_controllerGainArray;

  optimalPrimalSolution_controllerTimeStamp.resize(optimalPrimalSolution_controller_time_stamp.size());
  for (size_t i = 0; i < optimalPrimalSolution_controller_time_stamp.size(); i++)
  {
    optimalPrimalSolution_controllerTimeStamp[i] = optimalPrimalSolution_controller_time_stamp[i];
  }
  
  optimalPrimalSolution_controllerBiasArray.resize(optimalPrimalSolution_controller_bias_array.size());
  for (size_t i = 0; i < optimalPrimalSolution_controller_bias_array.size(); i++)
  {
    optimalPrimalSolution_controllerBiasArray[i].resize(optimalPrimalSolution_controller_bias_array[i].size());
    for (size_t j = 0; j < optimalPrimalSolution_controller_bias_array[i].size(); j++)
    {
      optimalPrimalSolution_controllerBiasArray[i][j] = optimalPrimalSolution_controller_bias_array[i][j];
    }
  }

  optimalPrimalSolution_controllerDeltaBiasArray.resize(optimalPrimalSolution_controller_delta_bias_array.size());
  for (size_t i = 0; i < optimalPrimalSolution_controller_delta_bias_array.size(); i++)
  {
    optimalPrimalSolution_controllerDeltaBiasArray[i].resize(optimalPrimalSolution_controller_delta_bias_array[i].size());
    for (size_t j = 0; j < optimalPrimalSolution_controller_delta_bias_array[i].size(); j++)
    {
      optimalPrimalSolution_controllerDeltaBiasArray[i][j] = optimalPrimalSolution_controller_delta_bias_array[i][j];
    }
  }

  optimalPrimalSolution_controllerGainArray.resize(optimalPrimalSolution_controller_gain_array.size());
  for (size_t i = 0; i < optimalPrimalSolution_controller_gain_array.size(); i++)
  {
    int row_size = optimalPrimalSolution_controller_gain_array[i].size();
    int col_size = optimalPrimalSolution_controller_gain_array[i][0].size();
    optimalPrimalSolution_controllerGainArray[i].resize(row_size, col_size);
    for (size_t j = 0; j < row_size; j++)
    {
      for (size_t k = 0; k < col_size; k++)
      {
        optimalPrimalSolution_controllerGainArray[i](j,k) = optimalPrimalSolution_controller_gain_array[i][j][k];
      }
    }
  }

  ops.controllerPtr_.reset(new LinearController);
  ops.controllerPtr_->setTimeStamp(optimalPrimalSolution_controllerTimeStamp);
  ops.controllerPtr_->setBiasArray(optimalPrimalSolution_controllerBiasArray);
  ops.controllerPtr_->setDeltaBiasArray(optimalPrimalSolution_controllerDeltaBiasArray);
  ops.controllerPtr_->setGainArray(optimalPrimalSolution_controllerGainArray);

  /*
  std::cout << "[MPC_ROS_Interface::loadData] ops getTimeStamp size: " << ops.controllerPtr_->getTimeStamp().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getBiasArray size: " << ops.controllerPtr_->getBiasArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getBiasArray[0] size: " << ops.controllerPtr_->getBiasArray()[0].size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getDeltaBiasArray size: " << ops.controllerPtr_->getDeltaBiasArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getGainArray size: " << ops.controllerPtr_->getGainArray().size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getGainArray rows: " << ops.controllerPtr_->getGainArray()[0].rows() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops getGainArray cols: " << ops.controllerPtr_->getGainArray()[0].cols() << std::endl;
  

  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  //ops.controllerPtr_->display();
  std::cout << "[MPC_ROS_Interface::loadData] ops.timeTrajectory_ size: " << ops.timeTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops.stateTrajectory_ size: " << ops.stateTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops.inputTrajectory_ size: " << ops.inputTrajectory_.size() << std::endl;
  std::cout << "[MPC_ROS_Interface::loadData] ops.controllerPtr_ size: " << ops.controllerPtr_->size() << std::endl;
  
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  std::cout << "/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/\\/" << std::endl;
  */

  mpc_.getSolverPtr()->setOptimizedPrimalSolution(ops);

  std::cout << "[MPC_ROS_Interface::loadData] END" << std::endl;
}

}  // namespace ocs2
