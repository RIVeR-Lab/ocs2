// LAST UPDATE: 2023.07.19
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

#include <string>
#include <XmlRpcValue.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <std_msgs/UInt8.h>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first. NUA NOTE: WHY?
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>
#include <cstdlib>
//#include <voxblox/interpolator/interpolator.h>

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/dynamics/MultiModelFunctions.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
//#include <ocs2_ros_interfaces/EsdfCachingServer.hpp>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h>

#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_ext_collision/ExtCollisionConstraint.h>
#include <ocs2_ext_collision/ExtCollisionConstraintCppAd.h>
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>

#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorExtCollisionConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/MobileBaseDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/RobotArmDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/MobileManipulatorDynamics.h"
#include <ocs2_mobile_manipulator/MobileManipulatorVisualization.h>

#include "ocs2_mobile_manipulator/setBool.h"

namespace ocs2 {
namespace mobile_manipulator {

//using typename voxblox::EsdfCachingVoxel;
//using voxblox::Interpolator;

/**
 * Mobile Manipulator Robot Interface class
 */
class MobileManipulatorInterface final : public RobotInterface 
{
  public:
    /**
     * Constructor
     *
     * @note Creates directory for generated library into if it does not exist.
     * @throw Invalid argument error if input task file or urdf file does not exist.
     *
     * @param [in] taskFile: The absolute path to the configuration file for the MPC.
     * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
     * @param [in] urdfFile: The absolute path to the URDF file for the robot.
     */
    MobileManipulatorInterface(const std::string& taskFile, 
                               const std::string& libraryFolder, 
                               const std::string& urdfFile,
                               PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>(),
                               int initModelModeInt=2);

    MobileManipulatorInterface(ros::NodeHandle& nodeHandle,
                               const std::string& taskFile, 
                               const std::string& libraryFolder, 
                               const std::string& urdfFile,
                               PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>(),
                               int initModelModeInt=2);

    /*
    const vector_t& getInitialState()
    { 
      return initialState_;
    }
    */

    const std::string& getTaskFile() const
    { 
      return taskFile_;
    }

    const std::string& getLibraryFolder() const 
    { 
      return libraryFolder_;
    }

    const std::string& getUrdfFile() const
    { 
      return urdfFile_;
    }

    ddp::Settings& ddpSettings() 
    { 
      return ddpSettings_; 
    }

    mpc::Settings& mpcSettings() 
    {
      return mpcSettings_; 
    }

    const OptimalControlProblem& getOptimalControlProblem() const override 
    { 
      return ocp_; 
    }

    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override 
    { 
      return referenceManagerPtr_; 
    }

    const Initializer& getInitializer() const override 
    { 
      return *initializerPtr_; 
    }

    const RolloutBase& getRollout() const 
    { 
      return *rolloutPtr_; 
    }

    const PinocchioInterface& getPinocchioInterface() const 
    { 
      return *pinocchioInterfacePtr_; 
    }

    const RobotModelInfo& getRobotModelInfo() const 
    { 
      return robotModelInfo_; 
    }

    std::shared_ptr<PointsOnRobot> getPointsOnRobotPtr() 
    { 
      return pointsOnRobotPtr_;
    }

    std::string getBaseStateMsg() 
    { 
      return baseStateMsg_;
    }

    std::string getArmStateMsg() 
    { 
      return armStateMsg_;
    }

    std::string getBaseControlMsg() 
    { 
      return baseControlMsg_;
    }

    std::string getArmControlMsg() 
    { 
      return armControlMsg_;
    }

    /*
    std::shared_ptr<voxblox::EsdfCachingServer> getEsdfCachingServerPtr() 
    { 
      return esdfCachingServerPtr_;
    }
    */

    /*
    std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> getVoxbloxInterpolatorPtr()
    {
      return voxbloxInterpolatorPtr_;
    }
    */

    void setNodeHandle(ros::NodeHandle& nodeHandle) 
    { 
      nodeHandle_ = nodeHandle;
    }

    void setPointsOnRobotPtr(std::shared_ptr<PointsOnRobot> newPointsOnRobotPtr) 
    { 
      pointsOnRobotPtr_ = newPointsOnRobotPtr;
    }

    void createPointsOnRobotPtr(PointsOnRobot::points_radii_t& pointsAndRadii) 
    { 
      pointsOnRobotPtr_.reset(new PointsOnRobot(pointsAndRadii));
    }

    void setMPCProblem(size_t modelMode, PointsOnRobot::points_radii_t& pointsAndRadii, bool nextFlag=false);

    /*
    void setEsdfCachingServerPtr(std::shared_ptr<voxblox::EsdfCachingServer> newEsdfCachingServerPtr) 
    { 
      esdfCachingServerPtr_ = newEsdfCachingServerPtr;
    }
    */

    /*
    void setVoxbloxInterpolatorPtr(std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> newVoxbloxInterpolatorPtr)
    {
      voxbloxInterpolatorPtr_ = newVoxbloxInterpolatorPtr;
    }
    */

    //void modelModeCallback(const std_msgs::UInt8::ConstPtr& msg);

    //void mpcModeSwitchCountCallback(const std_msgs::UInt8::ConstPtr& msg);

    void launchNodes(ros::NodeHandle& nodeHandle);

    void getEEPose(vector_t& eePose);

    void getTargetPose(vector_t& targetPose);

    //void setMPCModeSwitchCountPublisher();

    //void setMPCModeSwitchCountSubscriber();

    //void publishMPCModeSwitchCount();

    /*
    bool setMPCInitReadyFlag(bool val);

    bool setMRTInitReadyFlag(bool val);

    bool setMPCProblemReadyFlag(bool val);

    bool setMRTExitFlag(bool val);
    
    bool setMPCLaunchReadyFlag(bool val);

    bool setMPCInitReadyFlagSrv(ocs2_mobile_manipulator::setBool::Request &req, 
                                ocs2_mobile_manipulator::setBool::Response &res);

    bool setMRTInitReadyFlagSrv(ocs2_mobile_manipulator::setBool::Request &req, 
                                ocs2_mobile_manipulator::setBool::Response &res);

    bool setMPCProblemReadyFlagSrv(ocs2_mobile_manipulator::setBool::Request &req, 
                                   ocs2_mobile_manipulator::setBool::Response &res);

    bool setMRTExitFlagSrv(ocs2_mobile_manipulator::setBool::Request &req, 
                           ocs2_mobile_manipulator::setBool::Response &res);

    bool setMPCLaunchReadyFlagSrv(ocs2_mobile_manipulator::setBool::Request &req, 
                                  ocs2_mobile_manipulator::setBool::Response &res);
    */

    void runMPC();

    void runMRT();

    void mpcCallback(const ros::TimerEvent& event);

    void mrtCallback(const ros::TimerEvent& event);
  
  private:
    std::unique_ptr<StateInputCost> getQuadraticInputCost();

    std::unique_ptr<StateInputCost> getJointLimitSoftConstraint();
    
    std::unique_ptr<StateCost> getEndEffectorConstraint(const std::string& prefix);
    
    std::unique_ptr<StateCost> getSelfCollisionConstraint(const std::string& prefix);

    std::unique_ptr<StateCost> getExtCollisionConstraint(const std::string& prefix);

    ros::NodeHandle nodeHandle_;
    tf::TransformListener tfListener_;

    ros::Timer mpcTimer_;
    ros::Timer mrtTimer_;

    const std::string taskFile_;
    const std::string libraryFolder_;
    const std::string urdfFile_;
    std::string robotModelName_ = "mobile_manipulator";
    std::string worldFrameName_ = "world";
    std::string targetFrameName_ = "grasp";

    size_t initModelModeInt_ = 2;

    int mpcIter_ = 0; 
    int mrtIter_ = 0; 

    //int modeSwitchCount_ = 0;
    //int mpcModeSwitchCount_ = 0;
    //int mrtModeSwitchCount_ = 0;

    //bool mpcInitReadyFlag_ = false;
    //bool mrtInitReadyFlag_ = false;
    //bool mpcProblemReadyFlag_ = false;
    bool mrtExitFlag_ = true;
    bool mpcLaunchReadyFlag_ = false;

    int mpcShutDownEnvStatus_ = setenv("mpcShutDownFlag", "false", 1);
    int mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);
    //int mrtExitFlag_ = setenv("mrtExitFlag", "true", 1);
    //int mpcLaunchReadyFlag_ = setenv("mpcLaunchReadyFlag", "false", 1);

    //bool mpcReadyNextFlag_ = false;
    //bool mpcLaunchReadyFlag_ = false;
    //bool mrtReadyNextFlag_ = false;

    bool printOutFlag_ = false;
    bool usePreComputation_;
    bool recompileLibraries_;
    bool activateSelfCollision_;
    bool activateExtCollision_;

    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    OptimalControlProblem ocp_;
    std::shared_ptr<ReferenceManager> referenceManagerPtr_;
    
    ocs2::rollout::Settings rolloutSettings_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
    
    std::unique_ptr<Initializer> initializerPtr_;

    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    RobotModelInfo robotModelInfo_;

    std::string baseStateMsg_;
    std::string armStateMsg_;

    std::string baseControlMsg_;
    std::string armControlMsg_;

    std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_;
    std::shared_ptr<ExtMapUtility> emuPtr_;

    /// NUA NOTE: ADDING THE REQUIRED MPC & MRT PROCESSES
    PointsOnRobot::points_radii_t pointsAndRadii_;
    std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_;
    //ocs2::GaussNewtonDDP_MPC mpc_;
    //ocs2::GaussNewtonDDP_MPC mpcNext_;
    //MPC_ROS_Interface mpcNode_;
    //MPC_ROS_Interface mpcNodeNext_;

    vector_t currentTarget_;

    benchmark::RepeatedTimer mpcTimer1_;
    benchmark::RepeatedTimer mpcTimer2_;
    benchmark::RepeatedTimer mpcTimer3_;
    benchmark::RepeatedTimer mpcTimer4_;
    benchmark::RepeatedTimer mpcTimer5_;
    benchmark::RepeatedTimer mpcTimer6_;
    benchmark::RepeatedTimer mpcTimer7_;
    benchmark::RepeatedTimer mpcTimer8_;

    benchmark::RepeatedTimer mrtTimer1_;
    benchmark::RepeatedTimer mrtTimer2_;
    benchmark::RepeatedTimer mrtTimer3_;
    benchmark::RepeatedTimer mrtTimer4_;
    benchmark::RepeatedTimer mrtTimer5_;
    benchmark::RepeatedTimer mrtTimer6_;
    benchmark::RepeatedTimer mrtTimer7_;

    ros::Subscriber modelModeSubscriber_;
    ros::Subscriber targetTrajectoriesSubscriber_;
    //ros::Subscriber mpcModeSwitchCountSubscriber_;

    //ros::Publisher mpcModeSwitchCountPublisher_;

    //ros::ServiceServer setMPCInitReadyFlagService_;
    //ros::ServiceServer setMRTInitReadyFlagService_;
    //ros::ServiceServer setMPCProblemReadyFlagService_;
    //ros::ServiceServer setMRTExitFlagService_;
    //ros::ServiceServer setMPCLaunchReadyFlagService_;

    //ros::ServiceClient setMPCInitReadyFlagClient_;
    //ros::ServiceClient setMRTInitReadyFlagClient_;
    //ros::ServiceClient setMPCProblemReadyFlagClient_;
    //ros::ServiceClient setMRTExitFlagClient_;
    //ros::ServiceClient setMPCLaunchReadyFlagClient_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
