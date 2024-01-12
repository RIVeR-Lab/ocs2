// LAST UPDATE: 2024.01.12
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MRT_ROS_Interface::MRT_ROS_Interface(std::string topicPrefix, ros::TransportHints mrtTransportHints)
  : topicPrefix_(std::move(topicPrefix)), mrtTransportHints_(mrtTransportHints)
{
  // Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  terminateThread_ = false;
  readyToPublish_ = false;
  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
MRT_ROS_Interface::MRT_ROS_Interface(RobotModelInfo& robotModelInfo, std::string topicPrefix, ros::TransportHints mrtTransportHints)
  : robotModelInfo_(robotModelInfo), topicPrefix_(std::move(topicPrefix)), mrtTransportHints_(mrtTransportHints)
{
  // Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  terminateThread_ = false;
  readyToPublish_ = false;
  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MRT_ROS_Interface::~MRT_ROS_Interface() 
{
  //std::cout << "[MRT_ROS_Interface::~MRT_ROS_Interface] SHUTTING DOWN..." << std::endl;
  shutdownNodes();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
RobotModelInfo MRT_ROS_Interface::getRobotModelInfo()
{
  return robotModelInfo_;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::resetMpcNode(const TargetTrajectories& initTargetTrajectories) 
{
  //std::cout << "[MRT_ROS_Interface::resetMpcNode] START" << std::endl;

  this->reset();

  ocs2_msgs::reset resetSrv;
  resetSrv.request.reset = static_cast<uint8_t>(true);
  resetSrv.request.targetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

  while (!mpcResetServiceClient_.waitForExistence(ros::Duration(5.0)) && ::ros::ok() && ::ros::master::check()) 
  {
    ROS_ERROR_STREAM("[MRT_ROS_Interface::resetMpcNode] ERROR: Failed to call service to reset MPC, retrying...");
  }

  mpcResetServiceClient_.call(resetSrv);
  
  //std::cout << "[MRT_ROS_Interface::resetMpcNode] MPC node has been reset." << std::endl;
  //std::cout << "[MRT_ROS_Interface::resetMpcNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
int MRT_ROS_Interface::getModelModeInt()
{
  return modelModeInt_;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MRT_ROS_Interface::getShutDownFlag()
{
  return shutDownFlag_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::setShutDownFlag(bool shutDownFlag)
{
  shutDownFlag_ = shutDownFlag;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::setCurrentObservation(const SystemObservation& currentObservation) 
{
  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] START" << std::endl;
  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] terminateThread_: " << terminateThread_ << std::endl;
  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] BEFORE readyToPublish_: " << readyToPublish_ << std::endl;

#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] BEFORE createObservationMsg" << std::endl;
  // create the message
  mpcObservationMsg_ = ros_msg_conversions::createObservationMsg(currentObservation);
  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] AFTER createObservationMsg" << std::endl;

  // publish the current observation
#ifdef PUBLISH_THREAD
  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] READY TO PUBLISH!!!" << std::endl;
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  mpcObservationPublisher_.publish(mpcObservationMsg_);
#endif

  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] AFTER readyToPublish_: " << readyToPublish_ << std::endl;

  //std::cout << "[MRT_ROS_Interface::setCurrentObservation] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::publisherWorkerThread() 
{
  //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] START" << std::endl;
  //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] terminateThread_: " << terminateThread_ << std::endl;

  while (!terminateThread_) 
  {
    //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] try_lock: " << publisherMutex_.try_lock() << std::endl;
    std::unique_lock<std::mutex> lk(publisherMutex_);

    //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] readyToPublish_: " << readyToPublish_ << std::endl;
    //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] terminateThread_: " << terminateThread_ << std::endl;

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] terminateThread_: " << terminateThread_ << std::endl;
    if (terminateThread_) 
    {
      break;
    }

    mpcObservationMsgBuffer_ = std::move(mpcObservationMsg_);

    readyToPublish_ = false;

    lk.unlock();
    msgReady_.notify_one();

    //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] BEFORE publish" << std::endl << std::endl;
    mpcObservationPublisher_.publish(mpcObservationMsgBuffer_);
  }

  //std::cout << "[MRT_ROS_Interface::publisherWorkerThread] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, 
                                      CommandData& commandData,
                                      PrimalSolution& primalSolution, 
                                      PerformanceIndex& performanceIndices) 
{
  commandData.mpcInitObservation_ = ros_msg_conversions::readObservationMsg(msg.initObservation);
  commandData.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(msg.planTargetTrajectories);
  performanceIndices = ros_msg_conversions::readPerformanceIndicesMsg(msg.performanceIndices);

  const size_t N = msg.timeTrajectory.size();

  /*
  if (N == 0) 
  {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] ERROR: Controller message is empty!");
  }

  if (msg.stateTrajectory.size() != N && msg.inputTrajectory.size() != N) 
  {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] ERROR: State and input trajectories must have same length!");
  }

  if (msg.data.size() != N) 
  {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] ERROR: Data has the wrong length!");
  }
  */

  if (N > 0 && (msg.stateTrajectory.size() == N && msg.inputTrajectory.size() == N) && msg.data.size() == N)
  {
    primalSolution.clear();

    primalSolution.modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(msg.modeSchedule);

    size_array_t stateDim(N);
    size_array_t inputDim(N);
    primalSolution.timeTrajectory_.reserve(N);
    primalSolution.stateTrajectory_.reserve(N);
    primalSolution.inputTrajectory_.reserve(N);
    
    for (size_t i = 0; i < N; i++) 
    {
      stateDim[i] = msg.stateTrajectory[i].value.size();
      inputDim[i] = msg.inputTrajectory[i].value.size();
      primalSolution.timeTrajectory_.emplace_back(msg.timeTrajectory[i]);
      primalSolution.stateTrajectory_.emplace_back(Eigen::Map<const Eigen::VectorXf>(msg.stateTrajectory[i].value.data(), stateDim[i]).cast<scalar_t>());
      primalSolution.inputTrajectory_.emplace_back(Eigen::Map<const Eigen::VectorXf>(msg.inputTrajectory[i].value.data(), inputDim[i]).cast<scalar_t>());
    }

    primalSolution.postEventIndices_.reserve(msg.postEventIndices.size());
    for (auto ind : msg.postEventIndices) 
    {
      primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
    }

    std::vector<std::vector<float> const*> controllerDataPtrArray(N, nullptr);
    for (int i = 0; i < N; i++) 
    {
      controllerDataPtrArray[i] = &(msg.data[i].data);
    }

    // Instantiate the correct controller
    switch (msg.controllerType) 
    {
      case ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: 
      {
        auto controller = FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
        primalSolution.controllerPtr_.reset(new FeedforwardController(std::move(controller)));
        break;
      }
      
      case ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR: 
      {
        auto controller = LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_, controllerDataPtrArray);
        primalSolution.controllerPtr_.reset(new LinearController(std::move(controller)));
        break;
      }

      default:
        throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] ERROR: Unknown controllerType!");
    }
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg) 
{
  //std::cout << "[MRT_ROS_Interface::mpcPolicyCallback] START" << std::endl;

  // read new policy and command from msg
  std::unique_ptr<CommandData> commandPtr(new CommandData);
  std::unique_ptr<PrimalSolution> primalSolutionPtr(new PrimalSolution);
  std::unique_ptr<PerformanceIndex> performanceIndicesPtr(new PerformanceIndex);
  readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);

  this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));

  //std::cout << "[MRT_ROS_Interface::mpcPolicyCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::shutdownNodes() 
{
  //std::cout << "[MRT_ROS_Interface::shutdownNodes] START" << std::endl;
  
#ifdef PUBLISH_THREAD
  //std::cout << "[MRT_ROS_Interface::shutdownNodes] Shutting down workers..." << std::endl;
  shutdownPublisher();
  //std::cout << "[MRT_ROS_Interface::shutdownNodes] All workers are shut down." << std::endl;
#endif

  // clean up callback queue
  mrtCallbackQueue_.clear();
  mpcPolicySubscriber_.shutdown();

  // shutdown publishers
  mpcObservationPublisher_.shutdown();

  setenv("mrtExitFlag", "true", 1);

  //std::cout << "[MRT_ROS_Interface::shutdownNodes] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::shutdownPublisher() 
{
  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) 
  {
    publisherWorker_.join();
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::spinMRT() 
{
  //std::cout << "[MRT_ROS_Interface::spinMRT] START" << std::endl;
  mrtCallbackQueue_.callOne();
  //std::cout << "[MRT_ROS_Interface::spinMRT] END" << std::endl;
};

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::updateStatusModelModeMRT(bool statusModelModeMRT)
{
  statusModelModeMRTMsg_.data = statusModelModeMRT;
  statusModelModeMRTPublisher_.publish(statusModelModeMRTMsg_);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MRT_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) 
{
  this->reset();

  //std::cout << "[MRT_ROS_Interface::launchNodes] MRT node is setting up ..." << std::endl;

  // Publish Observation
  mpcObservationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(topicPrefix_ + "mpc_observation", 1);

  // Publish Model Mode MPC Status
  statusModelModeMRTPublisher_ = nodeHandle.advertise<std_msgs::Bool>("model_mode_mrt_status", 1, true);

  // Subscribe Policy
  auto ops = ros::SubscribeOptions::create<ocs2_msgs::mpc_flattened_controller>(
    topicPrefix_ + "mpc_policy",                                                       // topic name
    1,                                                                                  // queue length
    boost::bind(&MRT_ROS_Interface::mpcPolicyCallback, this, boost::placeholders::_1),  // callback
    ros::VoidConstPtr(),                                                                // tracked object
    &mrtCallbackQueue_                                                                  // pointer to callback queue object
  );
  ops.transport_hints = mrtTransportHints_;
  mpcPolicySubscriber_ = nodeHandle.subscribe(ops);

  //std::cout << "[MRT_ROS_Interface::launchNodes] modelModeCallback: " << topicPrefix_ + "_model_mode" << std::endl;
  // Subscribe Model Mode
  /*
  auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
  {
    std::cout << "[MRT_ROS_Interface::launchNodes] START" << std::endl;

    modelModeInt_ = msg->data;
    std::cout << "[MRT_ROS_Interface::launchNodes] modelModeInt: " << modelModeInt_ << std::endl;
    //updateModelMode(robotModelInfo_, modelModeInt);

    std::cout << "[MRT_ROS_Interface::launchNodes] END" << std::endl;
    std::cout << "" << std::endl;

    //shutdownNodes();
    shutDownFlag_ = true;

    statusModelModeMRTMsg_.data = false;
    statusModelModeMRTPublisher_.publish(statusModelModeMRTMsg_);
  };
  modelModeSubscriber_ = nodeHandle.subscribe<std_msgs::UInt8>(topicPrefix_ + "_model_mode", 1, modelModeCallback);
  */

  // Service client to reset MPC 
  mpcResetServiceClient_ = nodeHandle.serviceClient<ocs2_msgs::reset>(topicPrefix_ + "mpc_reset");

  /*
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("[MRT_ROS_Interface::launchNodes] Publishing MRT messages on a separate thread.");
#endif
  ROS_INFO_STREAM("[MRT_ROS_Interface::launchNodes] MRT node is ready.");
  */
 
  updateStatusModelModeMRT(true);
  /*
  statusModelModeMRTMsg_.data = true;
  statusModelModeMRTPublisher_.publish(statusModelModeMRTMsg_);
  */

  spinMRT();
}

}  // namespace ocs2
