// LAST UPDATE: 2024.03.21
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
MPC_ROS_Interface::MPC_ROS_Interface(std::shared_ptr<MPC_BASE> mpc, std::string topicPrefix)
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

  filename_ = getDateTime() + ".csv";

  // start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MPC_ROS_Interface::~MPC_ROS_Interface() 
{
  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::~MPC_ROS_Interface] SHUTTING DOWN..." << std::endl;
  shutdownNode();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MPC_ROS_Interface::getMPCReadyFlag()
{
  return mpcReadyFlag_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::setMPC(std::shared_ptr<MPC_BASE> mpc)
{
  mpc_ = mpc;
  resetRequestedEver_ = false;
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
  currentObservation_ = so;
  //std::cout << "[MPC_ROS_Interface::setSystemObservation] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::updateStatusModelModeMPC(bool statusModelModeMPC)
{
  //std::cout << "[MPC_ROS_Interface::updateStatusModelModeMPC] START" << std::endl;
  
  try
  {
    statusModelModeMPCMsg_.data = statusModelModeMPC;
    statusModelModeMPCPublisher_.publish(statusModelModeMPCMsg_);
  
  //std::cout << "[MPC_ROS_Interface::updateStatusModelModeMPC] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[MPC_ROS_Interface::updateStatusModelModeMPC] ERROR: CATCHUP! " << e.what() << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::resetMpcNode(TargetTrajectories&& initTargetTrajectories) 
{
  //std::cout << "[MPC_ROS_Interface::resetMpcNode] START" << std::endl;

  try
  {
    std::lock_guard<std::mutex> resetLock(resetMutex_);

    //std::cout << "[MPC_ROS_Interface::resetMpcNode] initTargetTrajectories size: " << initTargetTrajectories.size() << std::endl;
    //std::cout << initTargetTrajectories << std::endl;

    internalShutDownFlag_ = false;
    mpc_->setInternalShutDownFlag(false);

    mpc_->reset();

    mpc_->getSolverPtr()->getReferenceManager().setTargetTrajectories(std::move(initTargetTrajectories));
    
    mpcTimer_base_.reset();
    mpcTimer_arm_.reset();
    mpcTimer_wb_.reset();
    resetRequestedEver_ = true;
    terminateThread_ = false;
    readyToPublish_ = false;

    //std::cout << "[MPC_ROS_Interface::resetMpcNode] terminateThread_: " << terminateThread_ << std::endl;
    //std::cout << "[MPC_ROS_Interface::resetMpcNode] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[MPC_ROS_Interface::resetMpcNode] ERROR: CATCHUP! " << e.what() << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) 
{
  //std::cout << "[MPC_ROS_Interface::resetMpcNode] resetMpcCallback" << std::endl;

  if (static_cast<bool>(req.reset)) 
  {
    try
    {
      //std::cout << "[MPC_ROS_Interface::resetMpcNode] BEFORE readTargetTrajectoriesMsg" << std::endl;
      auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories);

      //std::cout << "[MPC_ROS_Interface::resetMpcCallback] RECEIVED targetTrajectories size: " << targetTrajectories.size() << std::endl;
      //std::cout << targetTrajectories << std::endl;

      resetMpcNode(std::move(targetTrajectories));
      res.done = static_cast<uint8_t>(true);

      /*
      std::cout << "\n#####################################################"
                << "\n#####################################################"
                << "\n#################  MPC is reset.  ###################"
                << "\n#####################################################"
                << "\n#####################################################\n";
      */
      return true;
    }
    catch (const std::exception& error)
    {
      const std::string msg = "[MPC_ROS_Interface::resetMpcCallback] ERROR: CATCHUP! \n";
      throw std::runtime_error(msg + error.what());
    }
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

  try
  {
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE initObservation" << std::endl;
    mpcPolicyMsg.initObservation = ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE planTargetTrajectories" << std::endl;
    mpcPolicyMsg.planTargetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
    
    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE modeSchedule" << std::endl;
    mpcPolicyMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE performanceIndices" << std::endl;
    mpcPolicyMsg.performanceIndices = ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

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

    mpcPolicyMsg.timeTrajectory.clear();
    mpcPolicyMsg.timeTrajectory.reserve(N);
    mpcPolicyMsg.stateTrajectory.clear();
    mpcPolicyMsg.stateTrajectory.reserve(N);
    mpcPolicyMsg.data.clear();
    mpcPolicyMsg.data.reserve(N);
    mpcPolicyMsg.postEventIndices.clear();
    mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE time" << std::endl;
    // time
    for (auto t : primalSolution.timeTrajectory_) 
    {
      mpcPolicyMsg.timeTrajectory.emplace_back(t);
    }

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE indices" << std::endl;
    // post-event indices
    for (auto ind : primalSolution.postEventIndices_) 
    {
      mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
    }

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

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] BEFORE flatten" << std::endl;
    // serialize controller into data buffer
    primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

    //std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] END" << std::endl;

    return mpcPolicyMsg;
  }
  catch(const std::exception& e)
  {
    std::cout << "[MPC_ROS_Interface::createMpcPolicyMsg] ERROR: CATCHUP! " << e.what() << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::publisherWorker() 
{
  //std::cout << "[MPC_ROS_Interface::publisherWorker] START" << std::endl;
  //std::cout << "[MPC_ROS_Interface::publisherWorker] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
  //std::cout << "[MPC_ROS_Interface::publisherWorker] terminateThread_: " << terminateThread_ << std::endl;

  while (ros::ok() && ros::master::check()) 
  {
    //std::cout << "[MPC_ROS_Interface::publisherWorker] START WHILE" << std::endl;

    try
    {
      std::unique_lock<std::mutex> lk(publisherMutex_);

      //std::cout << "[MPC_ROS_Interface::publisherWorker] BEFORE msgReady_" << std::endl;
      //std::cout << "[MPC_ROS_Interface::publisherWorker] terminateThread_: " << terminateThread_ << std::endl;
      //std::cout << "[MPC_ROS_Interface::publisherWorker] readyToPublish_: " << readyToPublish_ << std::endl;
      msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });
      //std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER msgReady_" << std::endl;

      if (!terminateThread_) 
      {
        //std::cout << "[MPC_ROS_Interface::publisherWorker] BEFORE lock_guard" << std::endl;
        {
          std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
          publisherCommandPtr_.swap(bufferCommandPtr_);
          publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
          publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
        }
        //std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER lock_guard" << std::endl;

        PrimalSolution currentPrimalSolution = *publisherPrimalSolutionPtr_;
        CommandData currentCommandData = *publisherCommandPtr_;
        PerformanceIndex currentPerformanceIndices = *publisherPerformanceIndicesPtr_;

        //std::cout << "[MPC_ROS_Interface::publisherWorker] BEFORE createMpcPolicyMsg" << std::endl;
        //std::cout << "[MPC_ROS_Interface::publisherWorker] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
        //std::cout << "[MPC_ROS_Interface::publisherWorker] terminateThread_: " << terminateThread_ << std::endl;
        ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(currentPrimalSolution, currentCommandData, currentPerformanceIndices);
        //std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER createMpcPolicyMsg" << std::endl;

        // publish the message
        mpcPolicyPublisher_.publish(mpcPolicyMsg);
        //std::cout << "[MPC_ROS_Interface::publisherWorker] AFTER mpcPolicyPublisher_" << std::endl;

        readyToPublish_ = false;
        lk.unlock();
        msgReady_.notify_one();

        //break;
      }

      //std::cout << "[MPC_ROS_Interface::publisherWorker] END WHILE" << std::endl << std::endl;
    }
    catch(const std::exception& e)
    {
      std::cout << "[MPC_ROS_Interface::publisherWorker] ERROR: CATCHUP! " << e.what() << std::endl;
    }
  }

  //std::cout << "[MPC_ROS_Interface::publisherWorker] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::copyToBuffer(const SystemObservation& mpcInitObservation) 
{
  try
  {
    //std::cout << "[MPC_ROS_Interface::copyToBuffer] START" << std::endl;

    // buffer policy mutex
    std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
    scalar_t finalTime;

    // Get solution
    finalTime = mpcInitObservation.time + mpc_->settings().solutionTimeWindow_;
    if (mpc_->settings().solutionTimeWindow_ < 0) 
    {
      finalTime = mpc_->getSolverPtr()->getFinalTime();
    }
    mpc_->getSolverPtr()->getPrimalSolution(finalTime, bufferPrimalSolutionPtr_.get());

    //std::cout << "[MPC_ROS_Interface::copyToBuffer] bufferPrimalSolutionPtr_ controllerPtr_ size: " << bufferPrimalSolutionPtr_->controllerPtr_->size() << std::endl;

    // Command
    bufferCommandPtr_->mpcInitObservation_ = mpcInitObservation;

    //std::cout << "[MPC_ROS_Interface::copyToBuffer] BEFORE getTargetTrajectories" << std::endl;
    bufferCommandPtr_->mpcTargetTrajectories_ = mpc_->getSolverPtr()->getReferenceManager().getTargetTrajectories();
    //std::cout << "[MPC_ROS_Interface::copyToBuffer] AFTER getTargetTrajectories" << std::endl;

    // Performance indices
    *bufferPerformanceIndicesPtr_ = mpc_->getSolverPtr()->getPerformanceIndeces();

    //std::cout << "[MPC_ROS_Interface::copyToBuffer] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[MPC_ROS_Interface::copyToBuffer] ERROR: CATCHUP! " << e.what() << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) 
{
  //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START" << std::endl;

  ocs2::SystemObservation currentObservation;
  try
  {
    // current time, state, input, and subsystem
    currentObservation = ros_msg_conversions::readObservationMsg(*msg);
    //setSystemObservation(currentObservation);
  }
  catch (const std::exception& error)
  {
    const std::string msg = "[MPC_ROS_Interface::mpcObservationCallback] ERROR: CATCHUP readObservationMsg \n";
    throw std::runtime_error(msg + error.what());
  }

  try
  {
    // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] state" << std::endl;
    // std::cout << currentObservation.state.size() << std::endl;
    state_size_ = currentObservation.state.size();

    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] input" << std::endl;
    //std::cout << currentObservation.input << std::endl;

    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE resetLock" << std::endl;
    std::lock_guard<std::mutex> resetLock(resetMutex_);
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] AFTER resetLock" << std::endl;

    if (!resetRequestedEver_.load()) 
    {
      //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service." << std::endl;
      return;
    }

    if (internalShutDownFlag_) 
    {
      //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] internalShutDownFlag_ is ON! MPC should be reset first!" << std::endl;
      return;
    }

    mpcReadyFlag_ = false;

    // run MPC
    bool controllerIsUpdated;
  
    // Start the timer before running MPC
    if (state_size_ == 3)
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START TIMER BASE" << std::endl;
      mpcTimer_base_.startTimer();
    }
    else if (state_size_ == 6)
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START TIMER ARM" << std::endl;
      mpcTimer_arm_.startTimer();
    }
    else
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] START TIMER WHOLE-BODY" << std::endl;
      mpcTimer_wb_.startTimer();
    }
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE mpc_->run" << std::endl;
    controllerIsUpdated = mpc_->run(currentObservation.time, currentObservation.state, currentObservation.full_state);
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] AFTER mpc_->run" << std::endl;
    // Stop the timer before running MPC
    if (state_size_ == 3)
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] STOP TIMER BASE" << std::endl;
      mpcTimer_base_.endTimer();
      ctime_max_base_ = mpcTimer_base_.getMaxIntervalInMilliseconds();
      ctime_avg_base_ = mpcTimer_base_.getAverageInMilliseconds();

      internalShutDownFlag_ = mpc_->getInternalShutDownFlag();
      if (internalShutDownFlag_)
      {
        ctr_err_base_++;
      }
    }
    else if (state_size_ == 6)
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] STOP TIMER ARM" << std::endl;
      mpcTimer_arm_.endTimer();
      ctime_max_arm_ = mpcTimer_arm_.getMaxIntervalInMilliseconds();
      ctime_avg_arm_ = mpcTimer_arm_.getAverageInMilliseconds();

      internalShutDownFlag_ = mpc_->getInternalShutDownFlag();
      if (internalShutDownFlag_)
      {
        ctr_err_arm_++;
      }
    }
    else
    {
      // std::cout << "[MPC_ROS_Interface::mpcObservationCallback] STOP TIMER WHOLE-BODY" << std::endl;
      mpcTimer_wb_.endTimer();
      ctime_max_wb_ = mpcTimer_wb_.getMaxIntervalInMilliseconds();
      ctime_avg_wb_ = mpcTimer_wb_.getAverageInMilliseconds();

      internalShutDownFlag_ = mpc_->getInternalShutDownFlag();
      if (internalShutDownFlag_)
      {
        ctr_err_wb_++;
      }
    }

    // internalShutDownFlag_ = mpc_->getInternalShutDownFlag();

    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] BEFORE terminateThread_: " << terminateThread_ << std::endl;
    if (internalShutDownFlag_)
    {
      //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] internalShutDownFlag_: " << internalShutDownFlag_ << std::endl;
      terminateThread_ = true;
    }
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] AFTER terminateThread_: " << terminateThread_ << std::endl;
    
    if (!controllerIsUpdated) 
    {
      return;
    }
    copyToBuffer(currentObservation);

    // Check MPC delay and solution window compatibility
    scalar_t timeWindow;
    timeWindow = mpc_->settings().solutionTimeWindow_;
    if (mpc_->settings().solutionTimeWindow_ < 0) 
    {
      timeWindow = mpc_->getSolverPtr()->getFinalTime() - currentObservation.time;
    }

    // if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) 
    // {
    //   //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] WARNING: The solution time window might be shorter than the MPC delay!" << std::endl;
    // }

    // Display time benchmarks
    if (mpc_->settings().debugPrint_) 
    {
      std::cout << '\n';
      std::cout << "\n### MPC_ROS Benchmarking";
      std::cout << "\n###   Maximum : " << mpcTimer_wb_.getMaxIntervalInMilliseconds() << "[ms].";
      std::cout << "\n###   Average : " << mpcTimer_wb_.getAverageInMilliseconds() << "[ms].";
      std::cout << "\n###   Latest  : " << mpcTimer_wb_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
    }

#ifdef PUBLISH_THREAD
    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] IN PUBLISH_THREAD" << std::endl;
    std::unique_lock<std::mutex> lk(publisherMutex_);
    readyToPublish_ = true;
    lk.unlock();
    msgReady_.notify_one();
#else
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_, *bufferPerformanceIndicesPtr_);
    mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif

    mpcReadyFlag_ = true;

    //std::cout << "[MPC_ROS_Interface::mpcObservationCallback] END" << std::endl << std::endl;

    if (state_size_ == 3)
    {
      ctr_base_++;
    }
    else if (state_size_ == 6)
    {
      ctr_arm_++;
    }
    else
    {
      ctr_wb_++;
    }
    writeData();
  }
  catch (const std::exception& error)
  {
    const std::string msg = "[MPC_ROS_Interface::mpcObservationCallback] ERROR: CATCHUP mpc_->run \n";
    throw std::runtime_error(msg + error.what());
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::writeData()
{
  if (writeDataFlag_)
  {
    // std::cout << "[MPC_ROS_Interface::writeData] START" << std::endl;

    std::string pkg_dir = ros::package::getPath("mobiman_simulation") + "/";
    std::string dataPath = pkg_dir + "dataset/ocs2/mpc_computation_time/";
    boost::filesystem::create_directories(dataPath);

    // std::cout << "[MPC_ROS_Interface::writeData] dataPath: " << dataPath << std::endl;

    std::ofstream myfile;
    myfile.open(dataPath + filename_);
    myfile << "ctr_base," << ctr_base_ << "\n";
    myfile << "ctr_err_base," << ctr_err_base_ << "\n";
    myfile << "ctime_max_base [ms]," << ctime_max_base_ << "\n";
    myfile << "ctime_avg_base [ms]," << ctime_avg_base_ << "\n";
    myfile << "ctr_arm," << ctr_arm_ << "\n";
    myfile << "ctr_err_arm," << ctr_err_arm_ << "\n";
    myfile << "ctime_max_arm [ms]," << ctime_max_arm_ << "\n";
    myfile << "ctime_avg_arm [ms]," << ctime_avg_arm_ << "\n";
    myfile << "ctr_wb," << ctr_wb_ << "\n";
    myfile << "ctr_err_wb," << ctr_err_wb_ << "\n";
    myfile << "ctime_max_wb [ms]," << ctime_max_wb_ << "\n";
    myfile << "ctime_avg_wb [ms]," << ctime_avg_wb_ << "\n";
    myfile.close();

    // if (ctr_base_ > 5)
    // {
    //   std::cout << "[MPC_ROS_Interface::writeData] DEBUG_INF" << std::endl;
    //   while(1);
    // }

    // std::cout << "[MPC_ROS_Interface::writeData] END" << std::endl;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
const std::string MPC_ROS_Interface::getDateTime() 
{
  time_t now = std::time(0);
  struct std::tm tstruct;
  char buf[80];
  tstruct = *std::localtime(&now);

  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tstruct);

  return buf;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::shutdownNode() 
{
  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::shutdownNode] START" << std::endl;

#ifdef PUBLISH_THREAD
  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::shutdownNode] Shutting down workers ..." << std::endl;

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }

  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::shutdownNode] All workers are shut down." << std::endl;
#endif

  // shutdown publishers
  mpcPolicyPublisher_.shutdown();

  //std::cout << "[MPC_ROS_Interface::shutdownNode] BEFORE mpcObservationSubscriber_" << std::endl;
  //mpcObservationSubscriber_.shutdown();

  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::shutdownNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) 
{
  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::launchNodes] START" << std::endl;

  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::launchNodes] MPC node is setting up..." << std::endl;

  // Subscribe Observation 
  mpcObservationSubscriber_ = nodeHandle.subscribe(topicPrefix_ + "mpc_observation", 
                                                   1, 
                                                   &MPC_ROS_Interface::mpcObservationCallback, 
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());

  // Publish MPC Policy
  mpcPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_flattened_controller>(topicPrefix_ + "mpc_policy", 1, true);

  // Publish Model Mode MPC Status
  statusModelModeMPCPublisher_ = nodeHandle.advertise<std_msgs::Bool>("model_mode_mpc_status", 5, true);

  // Service Server to reset MPC
  mpcResetServiceServer_ = nodeHandle.advertiseService(topicPrefix_ + "mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);
 
  //updateStatusModelModeMPC(true);
  /*
  statusModelModeMPCMsg_.data = true;
  statusModelModeMPCPublisher_.publish(statusModelModeMPCMsg_);
  */

  if (printOutFlag_)
    std::cout << "[MPC_ROS_Interface::launchNodes] END" << std::endl;
}

}  // namespace ocs2
