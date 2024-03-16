// LAST UPDATE: 2024.03.15
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/dynamics/MultiModelFunctions.h>

#include "ocs2_msgs/setDiscreteActionDRL.h"
#include "ocs2_msgs/setContinuousActionDRL.h"
#include "ocs2_msgs/setMPCActionResult.h"
#include "ocs2_msgs/calculateMPCTrajectory.h"

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
     * /// NUA TODO: UPDATE DESCRIPTION!
     * Constructor
     *
     * @note Creates directory for generated library into if it does not exist.
     * @throw Invalid argument error if input task file or urdf file does not exist.
     *
     * @param [in] taskFile: The absolute path to the configuration file for the MPC.
     * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
     * @param [in] urdfFile: The absolute path to the URDF file for the robot.
     */

    MobileManipulatorInterface(ros::NodeHandle& nodeHandle,
                               const std::string& taskFile, 
                               const std::string& libraryFolder, 
                               const std::string& urdfFile,
                               int initModelModeInt=2,
                               std::string interfaceName="",
                               bool printOutFlag=false);

    // DESCRIPTION: TODO...
    const std::string& getTaskFile() const
    { 
      return taskFile_;
    }

    // DESCRIPTION: TODO...
    const std::string& getLibraryFolder() const 
    { 
      return libraryFolder_;
    }

    // DESCRIPTION: TODO...
    const std::string& getUrdfFile() const
    { 
      return urdfFile_;
    }

    // DESCRIPTION: TODO...
    ddp::Settings& ddpSettings() 
    { 
      return ddpSettings_; 
    }

    // DESCRIPTION: TODO...
    mpc::Settings& mpcSettings() 
    {
      return mpcSettings_; 
    }

    // DESCRIPTION: TODO...
    const OptimalControlProblem& getOptimalControlProblem() const override 
    { 
      return ocp_;
    }

    // DESCRIPTION: TODO...
    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override 
    {
      return referenceManagerPtr_; 
    }

    // DESCRIPTION: TODO...
    const Initializer& getInitializer() const override 
    { 
      return *initializerPtr_; 
    }

    // DESCRIPTION: TODO...
    const RolloutBase& getRollout() const 
    { 
      return *rolloutPtr_; 
    }

    // DESCRIPTION: TODO...
    const PinocchioInterface& getPinocchioInterface() const 
    { 
      return *pinocchioInterfacePtr_; 
    }

    // DESCRIPTION: TODO...
    const RobotModelInfo& getRobotModelInfo() const 
    { 
      return robotModelInfo_; 
    }

    // DESCRIPTION: TODO...
    std::string getOdomMsgName() 
    { 
      return odomMsgName_;
    }

    // DESCRIPTION: TODO...
    std::string getBaseStateMsg() 
    { 
      return baseStateMsg_;
    }

    // DESCRIPTION: TODO...
    std::string getArmStateMsg() 
    { 
      return armStateMsg_;
    }

    // DESCRIPTION: TODO...
    std::string getBaseControlMsg() 
    { 
      return baseControlMsg_;
    }

    // DESCRIPTION: TODO...
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

    // DESCRIPTION: TODO...
    void setNodeHandle(ros::NodeHandle& nodeHandle);

    // DESCRIPTION: TODO...
    void setPrintOutFlag(bool printOutFlag);

    // DESCRIPTION: TODO...
    tf2::Quaternion getQuaternionFromRPY(double roll, double pitch, double yaw);

    // DESCRIPTION: TODO...
    void getRPYFromQuaternion(tf2::Quaternion quat, double& roll, double& pitch, double& yaw);

    // DESCRIPTION: TODO...
    void getEEPose(vector_t& eePose);

    // DESCRIPTION: TODO...
    void initializeMPC();

    // DESCRIPTION: TODO...
    void initializeMRT();

    // DESCRIPTION: TODO...
    void initializePointsOnRobotPtr(std::string& collisionPointsName);

    // DESCRIPTION: TODO...
    void updateFullModelState(std::vector<double>& statePositionBase, 
                              std::vector<double>& statePositionArm,
                              std::vector<double>& stateVelocityBase);

    // DESCRIPTION: TODO...
    SystemObservation getCurrentObservation(vector_t& currentInput, scalar_t time=0.0);

    // DESCRIPTION: TODO...
    void setMPCProblem(size_t modelModeInt=2, 
                       bool activateSelfCollision=false, 
                       bool activateExtCollision=false,
                       bool updateMPCFlag=false,
                       bool updateMRTFlag=false);

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

    // DESCRIPTION: TODO...
    void launchNodes();

    // DESCRIPTION: TODO...
    bool setDiscreteActionDRLMPC(int drlActionId,
                                 int drlActionDiscrete,
                                 double drlActionTimeHorizon);

    bool setDiscreteActionDRLMPCSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                    ocs2_msgs::setDiscreteActionDRL::Response &res);

    bool setDiscreteActionDRLMRTSrv(ocs2_msgs::setDiscreteActionDRL::Request &req, 
                                    ocs2_msgs::setDiscreteActionDRL::Response &res);

    bool setContinuousActionDRLMPC(int drlActionId,
                                   std::vector<double> drlActionContinuous, 
                                   double drlActionTimeHorizon);

    bool setContinuousActionDRLMPCSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                      ocs2_msgs::setContinuousActionDRL::Response &res);

    bool setContinuousActionDRLMRTSrv(ocs2_msgs::setContinuousActionDRL::Request &req, 
                                      ocs2_msgs::setContinuousActionDRL::Response &res);

    //bool calculateMPCTrajectorySrv(ocs2_msgs::calculateMPCTrajectory::Request &req, 
    //                               ocs2_msgs::calculateMPCTrajectory::Response &res);

    // DESCRIPTION: TODO...
    bool setStopMPCFlag(bool val);

    // DESCRIPTION: TODO...
    bool setStopMPCFlagSrv(ocs2_msgs::setBool::Request &req, 
                           ocs2_msgs::setBool::Response &res);

    // DESCRIPTION: TODO...
    bool setMPCWaitingFlag(bool val);

    // DESCRIPTION: TODO...
    bool setMPCWaitingFlagSrv(ocs2_msgs::setBool::Request &req, 
                              ocs2_msgs::setBool::Response &res);

    // DESCRIPTION: TODO...
    bool setMPCReadyFlag(bool val);

    // DESCRIPTION: TODO...
    bool setMPCReadyFlagSrv(ocs2_msgs::setBool::Request &req, 
                            ocs2_msgs::setBool::Response &res);

    // DESCRIPTION: TODO...
    bool setMRTReadyFlag(bool val);

    // DESCRIPTION: TODO...
    bool setMRTReadyFlagSrv(ocs2_msgs::setBool::Request &req, 
                            ocs2_msgs::setBool::Response &res);

    // DESCRIPTION: TODO...
    void launchMPC();
    
    // DESCRIPTION: TODO...
    void launchMRT();

    // DESCRIPTION: TODO...
    void mpcCallback(const ros::TimerEvent& event);

    // DESCRIPTION: TODO...
    void mrtCallback(const ros::TimerEvent& event);
  
  private:
    // DESCRIPTION: TODO...
    std::unique_ptr<StateInputCost> getQuadraticInputCost();

    // DESCRIPTION: TODO...
    std::unique_ptr<StateInputCost> getJointLimitSoftConstraint();
    
    // DESCRIPTION: TODO...
    std::unique_ptr<StateCost> getEndEffectorConstraint(const std::string& prefix);
    
    // DESCRIPTION: TODO...
    std::unique_ptr<StateCost> getSelfCollisionConstraint(const std::string& prefix);

    // DESCRIPTION: TODO...
    std::unique_ptr<StateCost> getExtCollisionConstraint(const std::string& prefix);

    // DESCRIPTION: TODO...
    void updateStateIndexMap();

    // DESCRIPTION: TODO...
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // DESCRIPTION: TODO...
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /*
    DESCRIPTION: TODO...Maps a value from the range [-1, 1] to a new range specified by the user.
    
    Args:
        val (double): The value to be mapped (within the range [-1, 1]).
        minVal (double): The minimum value of the new range.
        maxVal (double): The maximum value of the new range.
    
    Returns:
        double: The mapped value.
    */
    double mapActionTarget(double val, double minVal, double maxVal);

    // DESCRIPTION: TODO...
    bool setTargetDRL(string targetName, double x, double y, double z, double roll, double pitch, double yaw, double time_horizon=0.0);

    // DESCRIPTION: TODO...
    void mapDiscreteActionDRL(bool setTargetDRLFlag=true);

    // DESCRIPTION: TODO...
    void mapContinuousActionDRL(bool setTargetDRLFlag=true);

    // DESCRIPTION: TODO...
    bool setMPCActionResult(int drlActionResult, int timestep, double comErrorNormTotal);

    // DESCRIPTION: TODO...
    void readDiscreteTrajectoryData(vector<std::string> dataPath, vector<geometry_msgs::Pose>& discreteTrajectoryData);

    /*
    // DESCRIPTION: TODO...
    struct mpcProblemSettings
    {
      int modelMode;
      //std::vector<std::string> binarySettingNames = {"internalTargetCost",
      //                                               "selfCollisionConstraint",
      //                                               "externalCollisionConstraint"};
      std::vector<std::string> binarySettingNames = {"selfCollisionConstraint"};
      std::vector<bool> binarySettingValues;
    };
    */

    bool resetFlag_ = true;

    ros::NodeHandle nodeHandle_;
    tf::TransformListener tfListener_;

    bool initFlagBaseState_ = false;
    bool initFlagArmState_ = false;
    nav_msgs::Odometry odomMsg_;
    sensor_msgs::JointState jointStateMsg_;

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

    std::string worldFrameName_;;
    std::string goalFrameName_;
    std::string baseFrameName_;

    std::string modelModeMsgName_;
    std::string mpcTargetMsgName_;
    std::string targetMsgName_;

    std::string collisionConstraintPoints_;
    std::string collisionCheckPoints_;

    std::vector<std::pair<size_t, size_t>> collisionObjectPairs_;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs_;

    std::vector<std::string> removeJointNames_;

    size_t initModelModeInt_;
    size_t modelModeInt_;

    /// NUA TODO: SET IN CONFIG!
    double errThresholdPos_;
    double errThresholdOriYaw_;
    double errThresholdOriQuat_;

    int mpcIter_ = 0; 
    int mrtIter_ = 0; 

    bool targetReceivedFlag_ = false;
    bool mpcProblemReadyFlag_ = false;

    //int mpcShutDownEnvStatus_ = setenv("mpcShutDownFlag", "false", 1);
    //int mrtShutDownEnvStatus_ = setenv("mrtShutDownFlag", "false", 1);
    //int mrtExitEnvStatus_ = setenv("mrtExitFlag", "true", 1);

    std::string interfaceName_;
    bool printOutFlag_ = false;
    bool usePreComputation_;
    bool recompileLibraries_;
    bool activateSelfCollision_;
    bool activateExtCollision_;
    bool initActivateSelfCollision_;
    bool initActivateExtCollision_;
    
    bool drlFlag_;
    int drlActionType_;
    int drlActionId_;

    //mpcProblemSettings mpcProblemSettings_;
    vector<int> drlActionDiscreteNum_;
    int drlActionDiscrete_;
    
    std::vector<std::string> discreteTrajectoryDataPath_;
    std::vector<geometry_msgs::Pose> discreteTrajectoryData_;

    std::vector<double> drlActionContinuous_;
    double drlActionTimeHorizon_;
    double drlTargetRangeMinX_;
    double drlTargetRangeMinY_;
    double drlTargetRangeMinZ_;
    double drlTargetRangeMaxX_;
    double drlTargetRangeMaxY_;
    double drlTargetRangeMaxZ_;
    double drlGoalRangeMinX_;
    double drlGoalRangeMinY_;
    double drlGoalRangeMinZ_;
    double drlGoalRangeMaxX_;
    double drlGoalRangeMaxY_;
    double drlGoalRangeMaxZ_;
    //bool drlActionLastStepFlag_;
    //double drlActionLastStepDistanceThreshold_;

    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    
    OptimalControlProblem ocp_;

    std::shared_ptr<ReferenceManager> referenceManagerPtr_;
    std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_;
    
    ocs2::rollout::Settings rolloutSettings_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
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

    std::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
    std::shared_ptr<MPC_ROS_Interface> mpcNode_;
    std::shared_ptr<MRT_ROS_Interface> mrt_;
    std::shared_ptr<MRT_ROS_Gazebo_Loop> mrt_loop_;

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

    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode0_;
    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode1_;
    std::shared_ptr<StateCost> extCollisionConstraintPtr_mode2_;

    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode0_;
    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode1_;
    std::shared_ptr<SystemDynamicsBase> dynamicsPtr_mode2_;
    // -----------------

    std::string odomMsgName_;
    std::string baseStateMsg_;
    std::string armStateMsg_;
    std::vector<int> stateIndexMap_;

    std::string baseControlMsg_;
    std::string armControlMsg_;
    
    std::string logSavePathRel_;
    
    vector_t currentTarget_;

    std::string setDiscreteActionDRLMPCServiceName_;
    std::string setDiscreteActionDRLMRTServiceName_;
    std::string setContinuousActionDRLMPCServiceName_;
    std::string setContinuousActionDRLMRTServiceName_;
    std::string setTargetDRLServiceName_;
    //std::string calculateMPCTrajectoryServiceName_;
    //std::string computeCommandServiceName_;
    std::string setMPCActionResultServiceName_;
    std::string setStopMPCFlagSrvName_;
    std::string setMPCWaitingFlagSrvName_;
    std::string setMPCReadyFlagSrvName_;
    std::string setMRTReadyFlagSrvName_;

    bool newMPCProblemFlag_ = false;
    bool stopMPCFlag_ = false;
    bool mpcWaitingFlag_ = false;
    bool mpcReadyFlag_ = false;
    bool mrtReadyFlag_ = false;
    bool setMPCResultFlag_ = false;
    
    int mpcModeChangeCtr_ = 0;
    int mrtModeChangeCtr_ = 0;
    int mrtStuckCtr_ = 0;

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
    ros::Subscriber odomSubscriber_;
    ros::Subscriber jointStateSub_;

    //ros::Publisher baseTwistPub_;
    //ros::Publisher armJointTrajectoryPub_;
    //ros::Publisher armJointVelocityPub_;

    ros::ServiceClient setDiscreteActionDRLMPCClient_;
    ros::ServiceClient setContinuousActionDRLMPCClient_;
    ros::ServiceClient setTargetDRLClient_;
    ros::ServiceClient setMPCActionResultClient_;
    //ros::ServiceClient computeCommandClient_;
    ros::ServiceClient setStopMPCFlagClient_;
    ros::ServiceClient setMPCWaitingFlagClient_;
    ros::ServiceClient setMPCReadyFlagClient_;
    ros::ServiceClient setMRTReadyFlagClient_;

    ros::ServiceServer setDiscreteActionDRLMPCService_;
    ros::ServiceServer setDiscreteActionDRLMRTService_;
    ros::ServiceServer setContinuousActionDRLMPCService_;
    ros::ServiceServer setContinuousActionDRLMRTService_;
    //ros::ServiceServer calculateMPCTrajectoryService_;
    ros::ServiceServer setStopMPCFlagService_;
    ros::ServiceServer setMPCWaitingFlagService_;
    ros::ServiceServer setMPCReadyFlagService_;
    ros::ServiceServer setMRTReadyFlagService_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
