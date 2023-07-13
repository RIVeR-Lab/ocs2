// LAST UPDATE: 2023.07.13
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_msgs/reset.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

//#include <ocs2_ros_interfaces/EsdfCachingServer.hpp>
//#include <ocs2_ext_collision/ext_map_utility.h>

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MPC communication interface using ROS.
 */
class MPC_ROS_Interface 
{
  public:
    /**
     * Constructor.
     *
     * @param [in] mpc: The underlying MPC class to be used.
     * @param [in] topicPrefix: The robot's name.
     */
    explicit MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix = "anonymousRobot");

    /**
     * Constructor.
     *
     * NUA TODO: Update description.
     * @param [in] mpc: The underlying MPC class to be used.
     * @param [in] mpc: The underlying MPC class to be used.
     * @param [in] mpc: The underlying MPC class to be used.
     * @param [in] topicPrefix: The robot's name.
     */
    //explicit MPC_ROS_Interface(MPC_BASE& mpc_mobileBase, 
    //                           MPC_BASE& mpc_robotArm, 
    //                           MPC_BASE& mpc_mobileManipulator, 
    //                           std::string topicPrefix = "anonymousRobot");

    /**
     * Destructor.
     */
    virtual ~MPC_ROS_Interface();

    int getModelModeInt();

    /**
     * Set the Esdf Caching Server
     */
    //void setEsdfCachingServer(std::shared_ptr<voxblox::EsdfCachingServer> new_esdfCachingServer);

    /**
     * Set the External Map Utility
     */
    //void setExtMapUtility(std::shared_ptr<ExtMapUtility> emuPtr);

    /**
     * Resets the class to its instantiation state.
     *
     * @param [in] initTargetTrajectories: The initial desired cost trajectories.
     */
    void resetMpcNode(TargetTrajectories&& initTargetTrajectories);

    /**
     * Shutdowns the ROS node.
     */
    void shutdownNode();

    /**
     * Spins ROS.
     */
    void spin();

    /**
     * This is the main routine which launches all the nodes required for MPC to run which includes:
     * (1) The MPC policy publisher (either feedback or feedforward policy).
     * (2) The observation subscriber which gets the current measured state to invoke the MPC run routine.
     */
    void launchNodes(ros::NodeHandle& nodeHandle);

    void run();

  protected:
    /**
     * Callback to reset MPC.
     *
     * @param req: Service request.
     * @param res: Service response.
     */
    bool resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res);

    /**
     * NUA TODO: Add description.
     *
     * @param req: Service request.
     * @param res: Service response.
     */
    bool updateModelModeCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res);

    /**
     * Creates MPC Policy message.
     *
     * @param [in] primalSolution: The policy data of the MPC.
     * @param [in] commandData: The command data of the MPC.
     * @param [in] performanceIndices: The performance indices data of the solver.
     * @return MPC policy message.
     */
    static ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const PrimalSolution& primalSolution, const CommandData& commandData,
                                                                  const PerformanceIndex& performanceIndices);

    /**
     * Handles ROS publishing thread.
     */
    void publisherWorker();

    /**
     * Updates the buffer variables from the MPC object. This method is automatically called by advanceMpc()
     *
     * @param [in] mpcInitObservation: The observation used to run the MPC.
     */
    void copyToBuffer(const SystemObservation& mpcInitObservation);

    /**
     * The callback method which receives the current observation, invokes the MPC algorithm,
     * and finally publishes the optimized policy.
     *
     * @param [in] msg: The observation message.
     */
    void mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  protected:
    /*
    * Variables
    */
    MPC_BASE& mpc_;

    bool shutDownFlag_ = false;

    // 0: baseMotion
    // 0: armMotion
    // 0: wholeBodyMotion
    int modelModeInt_ = 2;

    // 0: Not Ready (In process of switching, old modelMode is in use)
    // 1: Ready
    std_msgs::Bool statusModelModeMPCMsg_;

    std::string topicPrefix_;

    ocs2::SystemObservation currentObservation_;

    std::shared_ptr<ros::NodeHandle> nodeHandlerPtr_;

    //std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServerPtr_;
    //std::shared_ptr<ExtMapUtility> emuPtr_;

    // Subscribers
    ros::Subscriber modelModeSubscriber_;
    ros::Subscriber mpcObservationSubscriber_;
    ros::Subscriber mpcTargetTrajectoriesSubscriber_;
    
    // Publishers
    ros::Publisher mpcPolicyPublisher_;
    ros::Publisher statusModelModeMPCPublisher_;
    
    // Services
    ros::ServiceServer mpcResetServiceServer_;

    std::unique_ptr<CommandData> bufferCommandPtr_;
    std::unique_ptr<CommandData> publisherCommandPtr_;
    std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
    std::unique_ptr<PrimalSolution> publisherPrimalSolutionPtr_;
    std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;
    std::unique_ptr<PerformanceIndex> publisherPerformanceIndicesPtr_;

    mutable std::mutex bufferMutex_;  // for policy variables with prefix (buffer*)

    // multi-threading for publishers
    std::atomic_bool terminateThread_{false};
    std::atomic_bool readyToPublish_{false};
    std::thread publisherWorker_;
    std::mutex publisherMutex_;
    std::condition_variable msgReady_;

    benchmark::RepeatedTimer mpcTimer_;

    // MPC reset
    std::mutex resetMutex_;
    std::atomic_bool resetRequestedEver_{false};
};

}  // namespace ocs2
