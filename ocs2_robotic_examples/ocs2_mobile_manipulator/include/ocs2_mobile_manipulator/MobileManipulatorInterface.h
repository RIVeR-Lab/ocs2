// LAST UPDATE: 2023.08.23
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
//#include <ocs2_core/misc/LoadData.h>
//#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include "ocs2_msgs/setDiscreteActionDRL.h"
#include "ocs2_msgs/setContinuousActionDRL.h"
#include "ocs2_msgs/setMPCActionResult.h"

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
                               int initModelModeInt=2);

    MobileManipulatorInterface(ros::NodeHandle& nodeHandle,
                               const std::string& taskFile, 
                               const std::string& libraryFolder, 
                               const std::string& urdfFile,
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
      /*
      if (mpcIter_ % 2 == 0)
      {
        return ocp1_;
      }
      else
      {
        return ocp2_; 
      }
      */
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

    /*
    std::shared_ptr<PointsOnRobot> getPointsOnRobotPtr() 
    { 
      return pointsOnRobotPtr_;
    }
    */

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

    /*
    void setPointsOnRobotPtr(std::shared_ptr<PointsOnRobot> newPointsOnRobotPtr) 
    { 
      pointsOnRobotPtr_ = newPointsOnRobotPtr;
    }
    */

    void initializePointsOnRobotPtr(std::string& collisionPointsName);

    void setMPCProblem();

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

    void launchNodes(ros::NodeHandle& nodeHandle);

    void getEEPose(vector_t& eePose);

    bool setDiscreteActionDRLSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                 ocs2_msgs::setDiscreteActionDRL::Response &res);

    bool setContinuousActionDRLSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                   ocs2_msgs::setContinuousActionDRL::Response &res);

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

    bool setTargetDRL(double x, double y, double z, double roll, double pitch, double yaw);

    void mapDiscreteActionDRL(int action);

    void mapContinuousActionDRL(std::vector<double>& action);

    bool setMRTReady();

    bool setMPCActionResult(int drlActionResult);

    struct mpcProblemSettings
    {
      int modelMode = 2;
      //std::vector<std::string> binarySettingNames = {"internalTargetCost",
      //                                               "selfCollisionConstraint",
      //                                               "externalCollisionConstraint"};
      std::vector<std::string> binarySettingNames = {"selfCollisionConstraint"};
      std::vector<bool> binarySettingValues;
    };

    ros::NodeHandle nodeHandle_;
    tf::TransformListener tfListener_;

    ros::Timer mpcTimer_;
    ros::Timer mrtTimer_;

    std::string sim_;
    std::string ns_;

    std::string baseFrame_withNS_;
    std::string armBaseFrame_withNS_;
    std::string eeFrame_withNS_;
    std::vector<std::string> armJointFrameNames_withNS_;

    const std::string taskFile_;
    const std::string libraryFolder_;
    const std::string urdfFile_;
    std::string robotModelName_ = "mobile_manipulator";
    std::string worldFrameName_ = "world";
    std::string goalFrameName_;

    std::string modelModeMsgName_;
    std::string mpcTargetMsgName_;
    std::string targetMsgName_;
    //std::string goalMsgName_;

    std::vector<std::pair<size_t, size_t>> collisionObjectPairs_;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs_;

    size_t initModelModeInt_ = 2;
    size_t modelModeIntQuery_ = 2;

    // 0: Go
    // 1: Go & Pick
    // 2: Go & Drop
    //// NUA NOTE: For now it is fixed to 1 for drl!
    int taskMode_ = 1;

    double err_threshold_pos_ = 0.1;
    double err_threshold_ori_yaw_ = 0.1;
    double err_threshold_ori_quat_ = 0.05;

    int mpcIter_ = 0; 
    int mrtIter_ = 0; 

    bool targetReceivedFlag_ = false;
    bool mpcProblemReadyFlag_ = false;
    bool mpcExitFlag_ = true;
    bool mrtExitFlag_ = true;
    bool mpcLaunchReadyFlag_ = false;

    int mpcShutDownEnvStatus_ = setenv("mpcShutDownFlag", "false", 1);
    int mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);
    int mrtExitEnvStatus_ = setenv("mrtExitFlag", "true", 1);

    bool printOutFlag_ = false;
    bool usePreComputation_;
    bool recompileLibraries_;
    bool activateSelfCollision_;
    bool activateExtCollision_;
    
    bool drlFlag_ = false;
    int drlActionType_ = 1;
    int drlActionDiscrete_;
    std::vector<double> drlActionContinuous_;
    double drlActionTimeHorizon_;
    bool drlActionLastStepFlag_;
    double drlActionLastStepDistanceThreshold_;
    mpcProblemSettings mpcProblemSettings_;

    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    
    OptimalControlProblem ocp_;
    //OptimalControlProblem ocp1_;
    //OptimalControlProblem ocp2_;

    std::shared_ptr<ReferenceManager> referenceManagerPtr_;
    std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_;
    
    ocs2::rollout::Settings rolloutSettings_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
    //std::unique_ptr<RolloutBase> mpcRolloutPtr_;
    //std::unique_ptr<RolloutBase> mrtRolloutPtr_;
    
    std::unique_ptr<Initializer> initializerPtr_;

    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;

    /// NUA NOTE: Should it depend on model mode?
    std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_;

    double maxDistance_ = 10;
    std::string selfCollisionMsg_;
    std::string occupancyDistanceBaseMsg_;
    std::string occupancyDistanceArmMsg_;
    std::string pointsOnRobotMsgName_;
    std::string octomapMsg_;
    std::shared_ptr<ExtMapUtility> emuPtr_;
    PointsOnRobot::points_radii_t pointsAndRadii_;

    std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorVisualization> mobileManipulatorVisu_;

    RobotModelInfo robotModelInfo_;

    // NUA TODO: Added to speed up the mpc problem initialization, but GET RID OF THESE ASAP!
    // -----------------
    std::shared_ptr<StateInputCost> quadraticInputCostPtr_mode0_;
    std::shared_ptr<StateInputCost> quadraticInputCostPtr_mode1_;
    std::shared_ptr<StateInputCost> quadraticInputCostPtr_mode2_;
    
    std::shared_ptr<StateInputCost> jointLimitSoftConstraintPtr_mode0_;
    std::shared_ptr<StateInputCost> jointLimitSoftConstraintPtr_mode1_;
    std::shared_ptr<StateInputCost> jointLimitSoftConstraintPtr_mode2_;

    std::shared_ptr<StateCost> endEffectorIntermediateConstraintPtr_mode0_;
    std::shared_ptr<StateCost> endEffectorIntermediateConstraintPtr_mode1_;
    std::shared_ptr<StateCost> endEffectorIntermediateConstraintPtr_mode2_;

    std::shared_ptr<StateCost> endEffectorFinalConstraintPtr_mode0_;
    std::shared_ptr<StateCost> endEffectorFinalConstraintPtr_mode1_;
    std::shared_ptr<StateCost> endEffectorFinalConstraintPtr_mode2_;

    std::shared_ptr<StateCost> selfCollisionConstraintPtr_mode0_;
    std::shared_ptr<StateCost> selfCollisionConstraintPtr_mode1_;
    std::shared_ptr<StateCost> selfCollisionConstraintPtr_mode2_;

    /// NUA NOTE: Should it depend on model mode?
    //std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_mode0_;
    //std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_mode1_;
    //std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_mode2_;

    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode0_;
    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode1_;
    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode2_;

    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode0_;
    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode1_;
    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode2_;
    // -----------------

    std::string baseStateMsg_;
    std::string armStateMsg_;

    std::string baseControlMsg_;
    std::string armControlMsg_;
    
    std::string logSavePathRel_;
    
    vector_t currentTarget_;

    benchmark::RepeatedTimer mpcTimer0_;
    benchmark::RepeatedTimer mpcTimer1_;
    benchmark::RepeatedTimer mpcTimer2_;
    benchmark::RepeatedTimer mpcTimer3_;
    benchmark::RepeatedTimer mpcTimer4_;
    benchmark::RepeatedTimer mpcTimer5_;
    benchmark::RepeatedTimer mpcTimer6_;
    benchmark::RepeatedTimer mpcTimer7_;
    benchmark::RepeatedTimer mpcTimer8_;
    benchmark::RepeatedTimer mpcTimer9_;
    benchmark::RepeatedTimer mpcTimer10_;
    benchmark::RepeatedTimer mpcTimer11_;

    benchmark::RepeatedTimer mrtTimer1_;
    benchmark::RepeatedTimer mrtTimer2_;
    benchmark::RepeatedTimer mrtTimer3_;
    benchmark::RepeatedTimer mrtTimer4_;
    benchmark::RepeatedTimer mrtTimer5_;
    benchmark::RepeatedTimer mrtTimer6_;
    benchmark::RepeatedTimer mrtTimer7_;

    ros::Subscriber modelModeSubscriber_;
    ros::Subscriber targetTrajectoriesSubscriber_;

    ros::ServiceClient setTargetDRLClient_;
    ros::ServiceClient setMPCActionResultClient_;
    ros::ServiceClient setMRTReadyClient_;

    ros::ServiceServer setActionDRLService_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
