// LAST UPDATE: 2024.02.23
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

#include "ocs2_ros_interfaces/command/TargetTrajectoriesGazebo.h"

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo::TargetTrajectoriesGazebo(const std::string& ns,
                                                   const std::string& topicPrefix,
                                                   const std::string& gazeboModelMsgName,
                                                   std::string robotName,
                                                   std::vector<std::string>& targetNames,
                                                   std::string dropTargetName,
                                                   GoalPoseToTargetTrajectories goalPoseToTargetTrajectories,
                                                   bool drlFlag)
  : targetServer_("target_marker"), 
    autoTargetServer_("auto_target_marker"), 
    dropTargetServer_("drop_target_marker"), 
    modelModeServer_("model_mode_marker", "", false), 
    ns_(ns),
    gazeboModelMsgName_(gazeboModelMsgName),
    robotName_(robotName), 
    targetNames_(targetNames), 
    dropTargetName_(dropTargetName),
    goalPoseToTargetTrajectories_(std::move(goalPoseToTargetTrajectories)),
    drlFlag_(drlFlag)
{
  tflistenerPtr_ = new tf::TransformListener;

  /// NUA TODO: SET BELOW IN CONFIG!
  graspPositionOffset_.x() = 0;
  graspPositionOffset_.y() = -0.2;
  graspPositionOffset_.z() = 0;
  graspOrientationOffsetMatrix_ <<  1.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0,
                                    0.0, -1.0, 0.0;
  //graspOrientationOffsetMatrix_ <<  1.0, 0.0, 0.0,
  //                                  0.0, 1.0, 0.0,
  //                                  0.0, 0.0, 1.0;

  dropPositionOffset_.x() = 0;
  dropPositionOffset_.y() = 0.2;
  dropPositionOffset_.z() = 1.0;
  dropOrientationOffsetMatrix_ << 1.0, 0.0, 0.0,
                                  0.0, 0.0, -1.0,
                                  0.0, 1.0, 0.0;
  //dropOrientationOffsetMatrix_ = graspOrientationOffsetMatrix_;

  //worldFrameName_ = "world";
  //robotFrameName_ = "base_link"; 
  //goalFrameName_ = "golazo"; 
  //graspFrameName_ = "grasp"; 
  //dropFrameName_ = "drop";
  //eeFrameName_ = "j2n6s300_end_effector";

  modelModeMsgName_ = "model_mode";
  goalVisuMsgName_ = "goal_visu";
  targetVisuMsgName_ = "target_visu";
  setTaskSrvName_ = "set_task";
  setPickedFlagSrvName_ = "set_picked_flag";
  setSystemObservationSrvName_ = "set_system_observation";
  setTargetDRLSrvName_ = "set_target_drl";

  mpcObservationMsgName_ = "mpc_observation";
  modelModeMPCStatusMsgName_ = "model_mode_mpc_status";
  modelModeMRTStatusMsgName_ = "model_mode_mrt_status";
  targetTrajectoriesMsgName_ = "mpc_target";
  //mobimanGoalObsMsgName_ = "mobiman_goal_obs";

  if (ns != "/")
  {
    //robotFrameName_ = ns + "/" + robotFrameName_;
    //goalFrameName_ = ns + "/" + goalFrameName_;
    //graspFrameName_ = ns + "/" + graspFrameName_;
    //dropFrameName_ = ns + "/" + dropFrameName_;
    //eeFrameName_ = ns + "/" + eeFrameName_;

    modelModeMsgName_ = ns + "/" + modelModeMsgName_;
    goalVisuMsgName_ = ns + "/" + goalVisuMsgName_;
    targetVisuMsgName_ = ns + "/" + targetVisuMsgName_;
    setTaskSrvName_ = ns + "/" + setTaskSrvName_;
    setPickedFlagSrvName_ = ns + "/" + setPickedFlagSrvName_;
    setSystemObservationSrvName_ = ns + "/" + setSystemObservationSrvName_;
    setTargetDRLSrvName_ = ns + "/" + setTargetDRLSrvName_;

    mpcObservationMsgName_ = ns + "/" + mpcObservationMsgName_;
    modelModeMPCStatusMsgName_ = ns + "/" + modelModeMPCStatusMsgName_;
    modelModeMRTStatusMsgName_ = ns + "/" + modelModeMRTStatusMsgName_;
    targetTrajectoriesMsgName_ = ns + "/" + targetTrajectoriesMsgName_;
    //mobimanGoalObsMsgName_ = ns + "/" + mobimanGoalObsMsgName_;
  }

  //setGoalTrajectoryFrameName(robotFrameName_);
  /// NUA TODO: SET ABOVE IN CONFIG!

  /*
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::TargetTrajectoriesGazebo] targetNames_: " << std::endl;
  for (size_t i = 0; i < targetNames_.size(); i++)
  {
    std::cout << i << " -> " << targetNames_[i] << std::endl;
  }
  */

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::TargetTrajectoriesGazebo] DEBUG_INF" << std::endl;
  //while(1);
}

TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo()
{
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo] Calling Destructor..." << std::endl;
  delete[] tflistenerPtr_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo::TargetTrajectoriesGazebo(const TargetTrajectoriesGazebo& ttg)
  : targetServer_("target_marker"), 
    autoTargetServer_("auto_target_marker"), 
    dropTargetServer_("drop_target_marker"), 
    modelModeServer_("model_mode_marker", "", false)
{
  initCallbackFlag_ = ttg.initCallbackFlag_;
  worldFrameName_ = ttg.worldFrameName_;
  robotFrameName_ = ttg.robotFrameName_;
  goalFrameName_ = ttg.goalFrameName_; 
  graspFrameName_ = ttg.graspFrameName_; 
  dropFrameName_ = ttg.dropFrameName_; 
  eeFrameName_ = ttg.eeFrameName_;

  targetNames_ = ttg.targetNames_;
  robotName_ = ttg.robotName_;
  robotPose_ = ttg.robotPose_;

  graspPositionOffset_ = ttg.graspPositionOffset_;
  graspOrientationOffsetMatrix_ = ttg.graspOrientationOffsetMatrix_;

  currentTargetName_ = ttg.currentTargetName_;
  currentTargetPosition_ = ttg.currentTargetPosition_;
  currentTargetOrientation_ = ttg.currentTargetOrientation_;
    
  currentTargetNames_ = ttg.currentTargetNames_;
  currentTargetPositions_ = ttg.currentTargetPositions_;
  currentTargetOrientations_ = ttg.currentTargetOrientations_;
  //currentTargetPositionsWrtRobot_ = ttg.currentTargetPositionsWrtRobot_;
  //currentTargetOrientationsWrtRobot_ = ttg.currentTargetOrientationsWrtRobot_;

  dummyGoalPosition_ = ttg.dummyGoalPosition_;
  dummyGoalOrientation_ = ttg.dummyGoalOrientation_;

  targetMarkerArray_ = ttg.targetMarkerArray_;
  targetMarkerArrayPublisher_ = ttg.targetMarkerArrayPublisher_;

  tflistenerPtr_ = ttg.tflistenerPtr_;

  h_mode_last_ = ttg.h_mode_last_;

  menuHandlerTarget_ = ttg.menuHandlerTarget_;
  menuHandlerAutoTarget_ = ttg.menuHandlerAutoTarget_;
  menuHandlerDropTarget_ = ttg.menuHandlerDropTarget_;
  menuHandlerModelMode_ = ttg.menuHandlerModelMode_;

  latestObservation_ = ttg.latestObservation_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo& TargetTrajectoriesGazebo::operator=(const TargetTrajectoriesGazebo& ttg)
{
  initCallbackFlag_ = ttg.initCallbackFlag_;
  
  worldFrameName_ = ttg.worldFrameName_;
  robotFrameName_ = ttg.robotFrameName_;
  goalFrameName_ = ttg.goalFrameName_; 
  graspFrameName_ = ttg.graspFrameName_; 
  dropFrameName_ = ttg.dropFrameName_; 
  eeFrameName_ = ttg.eeFrameName_;

  targetNames_ = ttg.targetNames_;
  robotName_ = ttg.robotName_;
  robotPose_ = ttg.robotPose_;

  graspPositionOffset_ = ttg.graspPositionOffset_;
  graspOrientationOffsetMatrix_ = ttg.graspOrientationOffsetMatrix_;

  currentTargetName_ = ttg.currentTargetName_;
  currentTargetPosition_ = ttg.currentTargetPosition_;
  currentTargetOrientation_ = ttg.currentTargetOrientation_;
    
  currentTargetNames_ = ttg.currentTargetNames_;
  currentTargetPositions_ = ttg.currentTargetPositions_;
  currentTargetOrientations_ = ttg.currentTargetOrientations_;
  //currentTargetPositionsWrtRobot_ = ttg.currentTargetPositionsWrtRobot_;
  //currentTargetOrientationsWrtRobot_ = ttg.currentTargetOrientationsWrtRobot_;

  dummyGoalPosition_ = ttg.dummyGoalPosition_;
  dummyGoalOrientation_ = ttg.dummyGoalOrientation_;

  targetMarkerArray_ = ttg.targetMarkerArray_;
  targetMarkerArrayPublisher_ = ttg.targetMarkerArrayPublisher_;

  tflistenerPtr_ = ttg.tflistenerPtr_;

  h_mode_last_ = ttg.h_mode_last_;

  menuHandlerTarget_ = ttg.menuHandlerTarget_;
  menuHandlerAutoTarget_ = ttg.menuHandlerAutoTarget_;
  menuHandlerDropTarget_ = ttg.menuHandlerDropTarget_;
  menuHandlerModelMode_ = ttg.menuHandlerModelMode_;

  latestObservation_ = ttg.latestObservation_;

  return *this;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void TargetTrajectoriesGazebo::setGoalTrajectoryFrameName(std::string goalTrajectoryFrameName)
{
  goalTrajectoryFrameName_ = goalTrajectoryFrameName;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setGoalTrajectoryQueueDt(double goalTrajectoryQueueDt)
{
  goalTrajectoryQueueDt_ = goalTrajectoryQueueDt;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setWorldFrameName(std::string worldFrameName)
{
  worldFrameName_ = worldFrameName;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setWorldFrameName] worldFrameName_: " << worldFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setRobotFrameName(std::string robotFrameName)
{
  robotFrameName_ = robotFrameName;
  if (ns_ != "/")
  {
    robotFrameName_ = ns_ + "/" + robotFrameName_;
  }
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setRobotFrameName] robotFrameName_: " << robotFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setGoalFrameName(std::string goalFrameName)
{
  goalFrameName_ = goalFrameName;
  if (ns_ != "/")
  {
    goalFrameName_ = ns_ + "/" + goalFrameName_;
  }
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::TargetTrajectoriesGazebo] goalFrameName_: " << goalFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setEEFrameName(std::string eeFrameName)
{
  eeFrameName_ = eeFrameName;
  if (ns_ != "/")
  {
    eeFrameName_ = ns_ + "/" + eeFrameName_;
  }
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::TargetTrajectoriesGazebo] eeFrameName_: " << eeFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setGraspFrameName(std::string graspFrameName)
{
  graspFrameName_ = graspFrameName;
  if (ns_ != "/")
  {
    graspFrameName_ = ns_ + "/" + graspFrameName_;
  }
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setGraspFrameName] graspFrameName_: " << graspFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setDropFrameName(std::string dropFrameName)
{
  dropFrameName_ = dropFrameName;
  if (ns_ != "/")
  {
    dropFrameName_ = ns_ + "/" + dropFrameName_;
  }
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setDropFrameName] dropFrameName_: " << dropFrameName_ << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setTaskMode(int taskMode)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTaskMode] START" << std::endl;
  taskMode_ = taskMode;

  if (taskMode == 1)
  {
    pickedFlag_ = false;
  }
  else if (taskMode == 2)
  {
    pickedFlag_ = true;
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTaskMode] taskMode: " << taskMode << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTaskMode] pickedFlag_: " << pickedFlag_ << std::endl;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTaskMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::launchNode(ros::NodeHandle& nodeHandle)
{
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] START" << std::endl;

  /// Interactive Marker
  // Create an interactive marker for user defined target
  menuHandlerTarget_.insert("Manual target", boost::bind(&TargetTrajectoriesGazebo::processFeedbackTarget, this, _1));

  // Create an interactive marker for auto target
  menuHandlerAutoTarget_.insert("Auto target", boost::bind(&TargetTrajectoriesGazebo::processFeedbackAutoTarget, this, _1));

  /// Subscribers
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) 
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    policyReceivedFlag_ = true;

    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode::observationCallback] mpcObservationMsgName_: " << mpcObservationMsgName_ << std::endl;
  };
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] mpcObservationMsgName_: " << mpcObservationMsgName_ << std::endl;
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(mpcObservationMsgName_, 5, observationCallback);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] modelModeMPCStatusMsgName_: " << modelModeMPCStatusMsgName_ << std::endl;
  statusModelModeMPCSubscriber_ = nodeHandle.subscribe<std_msgs::Bool>(modelModeMPCStatusMsgName_, 5, &TargetTrajectoriesGazebo::statusModelModeMPCCallback, this);
  
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] modelModeMRTStatusMsgName_: " << modelModeMRTStatusMsgName_ << std::endl;
  statusModelModeMRTSubscriber_ = nodeHandle.subscribe<std_msgs::Bool>(modelModeMRTStatusMsgName_, 5, &TargetTrajectoriesGazebo::statusModelModeMRTCallback, this);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] gazeboModelMsgName_: " << gazeboModelMsgName_ << std::endl;
  if (gazeboModelMsgName_ != "")
  {
    gazeboModelStatesSubscriber_ = nodeHandle.subscribe(gazeboModelMsgName_, 5, &TargetTrajectoriesGazebo::gazeboModelStatesCallback, this);
  }
  tfSubscriber_ = nodeHandle.subscribe("/tf", 5, &TargetTrajectoriesGazebo::tfCallback, this);

  /// Publishers
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] targetTrajectoriesMsgName_: " << targetTrajectoriesMsgName_ << std::endl;
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, targetTrajectoriesMsgName_));

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] modelModeMsgName_: " << modelModeMsgName_ << std::endl;
  modelModePublisher_ = nodeHandle.advertise<std_msgs::UInt8>(modelModeMsgName_, 1, false);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] goalVisuMsgName_: " << goalVisuMsgName_ << std::endl;
  goalMarkerArrayPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(goalVisuMsgName_, 10);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] targetVisuMsgName_: " << targetVisuMsgName_ << std::endl;
  targetMarkerArrayPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(targetVisuMsgName_, 20);
  //mobimanGoalObsPublisher_ = nodeHandle.advertise<ocs2_msgs::MobimanGoalObservation>(mobimanGoalObsMsgName_, 10);

  /// Clients
  //setTaskModeClient_ = nodeHandle.serviceClient<ocs2_msgs::setInt>("/set_task_mode");

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] setTaskSrvName_: " << setTaskSrvName_ << std::endl;
  setTaskClient_ = nodeHandle.serviceClient<ocs2_msgs::setTask>(setTaskSrvName_);

  /// Services
  //setTaskModeService_ = nodeHandle.advertiseService("set_task_mode", &TargetTrajectoriesGazebo::setTaskModeSrv, this);
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] setPickedFlagSrvName_: " << setPickedFlagSrvName_ << std::endl;
  setPickedFlagService_ = nodeHandle.advertiseService(setPickedFlagSrvName_, &TargetTrajectoriesGazebo::setPickedFlagSrv, this);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] setSystemObservationSrvName_: " << setSystemObservationSrvName_ << std::endl;
  setSystemObservationService_ = nodeHandle.advertiseService(setSystemObservationSrvName_, &TargetTrajectoriesGazebo::setSystemObservationSrv, this);

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] setTargetDRLSrvName_: " << setTargetDRLSrvName_ << std::endl;
  setTargetDRLService_ = nodeHandle.advertiseService(setTargetDRLSrvName_, &TargetTrajectoriesGazebo::setTargetDRLSrv, this);

  while(!initTFCallbackFlag_){ros::spinOnce();}

  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::launchNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateDummyGoal(double x, double y, double z, double roll, double pitch, double yaw)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] START" << std::endl;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(roll, pitch, yaw);

  dummyGoalPosition_.x() = x;
  dummyGoalPosition_.y() = y;
  dummyGoalPosition_.z() = z;

  dummyGoalOrientation_.x() = quat_tf.x();
  dummyGoalOrientation_.y() = quat_tf.y();
  dummyGoalOrientation_.z() = quat_tf.z();
  dummyGoalOrientation_.w() = quat_tf.w();

  goalPosition_ = dummyGoalPosition_;
  goalOrientation_ = dummyGoalOrientation_;

  if (printOutFlag_)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] x: " << x << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] y: " << y << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] z: " << z << std::endl;

    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] roll: " << roll << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] pitch: " << pitch << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] yaw: " << yaw << std::endl;
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] DEBUG_INF" << std::endl;
  //while(1);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDummyGoal] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateObservationAndTarget() 
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateObservationAndTarget] START" << std::endl;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateObservationAndTarget] DEBUG_INF" << std::endl;
  while(1);

  if (initCallbackFlag_)
  {
    // Get the latest observation
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // Set TargetTrajectories for ocs2 mpc
    const auto targetTrajectories = goalPoseToTargetTrajectories_(currentTargetPosition_, currentTargetOrientation_, observation);

    // Publish TargetTrajectories for ocs2 mpc
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

    // Update target visualization
    updateTarget();
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateObservationAndTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget] START" << std::endl;

  //while (!initCallbackFlag_);
  
  // Create an interactive markers
  auto targetInteractiveMarker = createInteractiveMarkerTarget();

  // Add the interactive marker to our collection &
  // Tell the server to call processFeedback() when feedback arrives for it
  targetServer_.clear();
  targetServer_.insert(targetInteractiveMarker);
  menuHandlerTarget_.apply(targetServer_, targetInteractiveMarker.name);

  // 'commit' changes and send to all clients
  targetServer_.applyChanges();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo:: initializeInteractiveMarkerAutoTarget()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerAutoTarget] START" << std::endl;

  //while (!initCallbackFlag_);
  
  // Create an interactive markers
  auto autoTargetInteractiveMarker = createInteractiveMarkerAutoTarget();

  // Add the interactive marker to our collection &
  // Tell the server to call processFeedback() when feedback arrives for it
  autoTargetServer_.clear();
  autoTargetServer_.insert(autoTargetInteractiveMarker);
  menuHandlerAutoTarget_.apply(autoTargetServer_, autoTargetInteractiveMarker.name);

  // 'commit' changes and send to all clients
  autoTargetServer_.applyChanges();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerAutoTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void TargetTrajectoriesGazebo::initializeInteractiveMarkerDropTarget()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerDropTarget] START" << std::endl;

  while (!initCallbackFlag_){ros::spinOnce();}
  updateDropTarget();

  // Create an interactive markers
  auto dropTargetInteractiveMarker = createInteractiveMarkerDropTarget();

  // Add the interactive marker to our collection &
  // Tell the server to call processFeedback() when feedback arrives for it
  dropTargetServer_.clear();
  dropTargetServer_.insert(dropTargetInteractiveMarker);
  menuHandlerDropTarget_.apply(dropTargetServer_, dropTargetInteractiveMarker.name);

  // 'commit' changes and send to all clients
  dropTargetServer_.applyChanges();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerDropTarget] END" << std::endl;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode] START" << std::endl;

  //while (!initCallbackFlag_);
  
  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarkerModelMode();

  createMenuModelMode();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  modelModeServer_.insert(interactiveMarker);
  menuHandlerModelMode_.apply(modelModeServer_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  modelModeServer_.applyChanges();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::getTransform(std::string frame_from, std::string frame_to, tf::StampedTransform& stf)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] START " << std::endl;

  bool success = false;
  try
  {
    tflistenerPtr_->waitForTransform(frame_from, frame_to, ros::Time::now(), ros::Duration(1.0));
    tflistenerPtr_->lookupTransform(frame_from, frame_to, ros::Time(0), stf);

    /*
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] worldFrameName_: " << worldFrameName_ << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] baseFrameName_: " << baseFrameName_ << std::endl;

    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] pos x: " << tf_robot_wrt_world_.getOrigin().x() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] pos y: " << tf_robot_wrt_world_.getOrigin().y() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] pos z: " << tf_robot_wrt_world_.getOrigin().z() << std::endl;
    */
   success = true;
  }
  catch (tf::TransformException ex)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] ERROR: Couldn't get transform!" << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] frame_from: " << frame_from << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] frame_to: " << frame_to << std::endl;
    //ROS_ERROR("%s", ex.what());
  }

  return success;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::getTransform] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::transformPose(std::string frame_from,
                                             std::string frame_to,
                                             geometry_msgs::Pose& p_from,
                                             geometry_msgs::Pose& p_to)
{
  tf::Pose p_from_tf;
  geometry_msgs::Pose p_from_msg = p_from;
  tf::poseMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Pose> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Pose> p_to_stamped_tf;
  geometry_msgs::PoseStamped p_to_stamped_msg;

  try
  {
    tflistenerPtr_->waitForTransform(frame_to, frame_from, ros::Time(0), ros::Duration(1.0));
    tflistenerPtr_->transformPose(frame_to, p_from_stamped_tf, p_to_stamped_tf);

    tf::poseStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
    p_to = p_to_stamped_msg.pose;

    return true;
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("[TargetTrajectoriesGazebo::transformPose] Couldn't get transform!");
    ROS_ERROR("%s",ex.what());

    return false;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::transformPose] DEBUG INF" << std::endl;
    //while(1);
    //ros::Duration(1.0).sleep();
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::rotateQuaternion(Eigen::Quaterniond& quat, 
                                                Eigen::Vector3d& rpy_rot, 
                                                Eigen::Quaterniond& quat_new)
{
  tf2::Quaternion q_orig, q_rot, q_new;

  q_orig.setX(quat.x());
  q_orig.setY(quat.y());
  q_orig.setZ(quat.z());
  q_orig.setW(quat.w());

  // Rotate the previous pose about X
  double roll = rpy_rot.x();

  // Rotate the previous pose about Y
  double pitch = rpy_rot.y();
  
  // Rotate the previous pose about Z
  double yaw = rpy_rot.z();  

  q_rot.setRPY(roll, pitch, yaw);

  // Calculate the new orientation
  q_new = q_rot * q_orig;

  q_new.normalize();

  quat_new.x() = q_new.x();
  quat_new.y() = q_new.y();
  quat_new.z() = q_new.z();
  quat_new.w() = q_new.w();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::print(geometry_msgs::Point po)
{
  std::cout << "(" << po.x << ", " << po.y << ", " << po.z << ")" << std::endl; 
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int TargetTrajectoriesGazebo::isIn(std::vector<std::string>& vec, std::string s)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::isIn] START" << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::isIn] vec.size(): " << vec.size() << std::endl;

  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i].compare(s) == 0)
    {
      return i;
    }
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::isIn] END" << std::endl;

  return -1;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
double TargetTrajectoriesGazebo::calculateEuclideanDistance(Eigen::Vector3d& p1, geometry_msgs::Pose& p2)
{
  return ( sqrt( pow(p1.x() - p2.position.x, 2) + pow(p1.y() - p2.position.y, 2) + pow(p1.z() - p2.position.z, 2) ) );
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::pair<double, int> TargetTrajectoriesGazebo::findClosestDistance(std::vector<Eigen::Vector3d>& pos_vec, geometry_msgs::Pose& query_p)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] START" << std::endl;

  std::pair<double, int> result;
  result.first = -1;
  result.second = -1;
  
  if (pos_vec.size() <= 0)
  {
    return result;
  }
  
  double temp_dist = calculateEuclideanDistance(pos_vec[0], query_p);
  result.first = temp_dist;
  result.second = 0;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] currentTargetName 0: " << currentTargetNames_[0] << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] temp_dist: " << temp_dist << std::endl;
  for(int i = 1; i < pos_vec.size(); i++)
  {
    temp_dist = calculateEuclideanDistance(pos_vec[i], query_p);

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] currentTargetName " << i << ": " << currentTargetNames_[i] << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] temp_dist: " << temp_dist << std::endl;

    if(result.first > temp_dist)
    {
      result.first = temp_dist;
      result.second = i;
    }
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] result.first: " << result.first << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] result.second: " << result.second << std::endl;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::findClosestDistance] END" << std::endl;

  return result;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::gazeboModelStatesCallback(const gazebo_msgs::ModelStatesPtr& msg) 
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::gazeboModelStatesCallback] Incoming..." << std::endl;  
  modelStatesMsg_ = *msg;
  initCallbackFlag_ = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::tfCallback] Incoming..." << std::endl;

  tf::StampedTransform tf_ee_wrt_world;
  tf::StampedTransform tf_robot_wrt_world;
  tf::StampedTransform tf_target_wrt_world;
  tf::StampedTransform tf_target_wrt_robot;

  geometry_msgs::Pose robotPose;
  std::vector<std::string> currentTargetNames;
  std::vector<Eigen::Vector3d> currentTargetPositions;
  std::vector<Eigen::Quaterniond> currentTargetOrientations;
  //std::vector<Eigen::Vector3d> currentTargetPositionsWrtRobot;
  //std::vector<Eigen::Quaterniond> currentTargetOrientationsWrtRobot;

  try
  {
    if (tflistenerPtr_->waitForTransform(worldFrameName_, eeFrameName_, ros::Time(0), ros::Duration(1.0)))
    {
      tflistenerPtr_->lookupTransform(worldFrameName_, eeFrameName_, ros::Time(0), tf_ee_wrt_world);
      tf_ee_wrt_world_ = tf_ee_wrt_world;
    }

    if (tflistenerPtr_->waitForTransform(worldFrameName_, robotFrameName_, ros::Time(0), ros::Duration(1.0)))
    {
      tflistenerPtr_->lookupTransform(worldFrameName_, robotFrameName_, ros::Time(0), tf_robot_wrt_world);
      //tf_robot_wrt_world_ = tf_robot_wrt_world;

      robotPose.position.x = tf_robot_wrt_world.getOrigin().x();
      robotPose.position.y = tf_robot_wrt_world.getOrigin().y();
      robotPose.position.z = tf_robot_wrt_world.getOrigin().z();
      robotPose.orientation.x = tf_robot_wrt_world.getRotation().x();
      robotPose.orientation.y = tf_robot_wrt_world.getRotation().y();
      robotPose.orientation.z = tf_robot_wrt_world.getRotation().z();
      robotPose.orientation.w = tf_robot_wrt_world.getRotation().w();

      robotPose_ = robotPose;
    }

    bool updateFlag = false;
    for (size_t i = 0; i < targetNames_.size(); i++)
    {
      if(tflistenerPtr_->frameExists(targetNames_[i]))
      {
        tflistenerPtr_->waitForTransform(worldFrameName_, targetNames_[i], ros::Time(0), ros::Duration(1.0));
        tflistenerPtr_->lookupTransform(worldFrameName_, targetNames_[i], ros::Time(0), tf_target_wrt_world);

        tflistenerPtr_->waitForTransform(robotFrameName_, targetNames_[i], ros::Time(0), ros::Duration(1.0));
        tflistenerPtr_->lookupTransform(robotFrameName_, targetNames_[i], ros::Time(0), tf_target_wrt_robot);

        currentTargetNames.push_back(targetNames_[i]);

        Eigen::Vector3d pos;
        pos << tf_target_wrt_world.getOrigin().x(), tf_target_wrt_world.getOrigin().y(), tf_target_wrt_world.getOrigin().z();
        currentTargetPositions.push_back(pos);

        Eigen::Quaterniond quat;
        quat.x() = tf_target_wrt_world.getRotation().x();
        quat.y() = tf_target_wrt_world.getRotation().y();
        quat.z() = tf_target_wrt_world.getRotation().z();
        quat.w() = tf_target_wrt_world.getRotation().w();
        currentTargetOrientations.push_back(quat);

        /*
        Eigen::Vector3d pos_wrt_robot;
        pos_wrt_robot << tf_target_wrt_robot.getOrigin().x(), tf_target_wrt_robot.getOrigin().y(), tf_target_wrt_robot.getOrigin().z();
        currentTargetPositionsWrtRobot.push_back(pos_wrt_robot);

        Eigen::Quaterniond quat_wrt_robot;
        quat_wrt_robot.x() = tf_target_wrt_robot.getRotation().x();
        quat_wrt_robot.y() = tf_target_wrt_robot.getRotation().y();
        quat_wrt_robot.z() = tf_target_wrt_robot.getRotation().z();
        quat_wrt_robot.w() = tf_target_wrt_robot.getRotation().w();
        currentTargetOrientationsWrtRobot.push_back(quat_wrt_robot);
        */

        updateFlag = true;
      }
    }
    
    if (updateFlag)
    {
      currentTargetNames_ = currentTargetNames;
      currentTargetPositions_ = currentTargetPositions;
      currentTargetOrientations_ = currentTargetOrientations;
      //currentTargetPositionsWrtRobot_ = currentTargetPositionsWrtRobot;
      //currentTargetOrientationsWrtRobot_ = currentTargetOrientationsWrtRobot;

      initTFCallbackFlag_ = true;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[TargetTrajectoriesGazebo::tfCallback] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::statusModelModeMPCCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMPCCallback] START" << std::endl;

  statusModelModeMPC_ = msg->data;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMPCCallback] modelModeInt: " << statusModelModeMPC_ << std::endl;

  if (!drlFlag_)
  {
    initializeInteractiveMarkerModelMode();
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMPCCallback] END" << std::endl;
  //std::cout << "" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::statusModelModeMRTCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] START" << std::endl;

  statusModelModeMRT_ = msg->data;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] modelModeInt: " << statusModelModeMRT_ << std::endl;
  
  if (!drlFlag_)
  {
    initializeInteractiveMarkerModelMode();
  }
  
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::statusModelModeMRTCallback] END" << std::endl;
  //std::cout << "" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateTargetInfo()
{
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTargetInfo] START" << std::endl;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTargetInfo] DEBUG_INF" << std::endl;
  while(1);

  gazebo_msgs::ModelStates ms = modelStatesMsg_;

  geometry_msgs::Pose robotPose;
  std::vector<std::string> currentTargetNames;
  std::vector<Eigen::Vector3d> currentTargetPositions;
  std::vector<Eigen::Quaterniond> currentTargetOrientations;

  for (size_t i = 0; i < ms.name.size(); i++)
  {
    if (ms.name[i] == robotName_)
    {
      robotPose = ms.pose[i];
    }

    if (isIn(targetNames_, ms.name[i]) >= 0)
    {
      currentTargetNames.push_back(ms.name[i]);

      Eigen::Vector3d pos;
      pos << ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z;
      currentTargetPositions.push_back(pos);

      Eigen::Quaterniond quat;
      quat.x() = ms.pose[i].orientation.x;
      quat.y() = ms.pose[i].orientation.y;
      quat.z() = ms.pose[i].orientation.z;
      quat.w() = ms.pose[i].orientation.w;
      currentTargetOrientations.push_back(quat);
    }
  }

  robotPose_ = robotPose;
  currentTargetNames_ = currentTargetNames;
  currentTargetPositions_ = currentTargetPositions;
  currentTargetOrientations_ = currentTargetOrientations;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTargetInfo] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateGoal(bool autoFlag)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] START" << std::endl;

  Eigen::Vector3d goalPos;
  Eigen::Quaterniond goalOri;

  //Eigen::Vector3d goalPosWrtRobot;
  //Eigen::Quaterniond goalOriWrtRobot;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] taskMode_: " << taskMode_ << std::endl;

  if (taskMode_ == 1)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] taskMode_: 1" << std::endl;
    //updateTargetInfo();

    geometry_msgs::Pose robotPose = robotPose_;
    std::vector<std::string> currentTargetNames = currentTargetNames_;
    std::vector<Eigen::Vector3d> targetPositions = currentTargetPositions_;
    std::vector<Eigen::Quaterniond> targetOrientations = currentTargetOrientations_;
    //std::vector<Eigen::Vector3d> targetPositionsWrtRobot = currentTargetPositionsWrtRobot_;
    //std::vector<Eigen::Quaterniond> targetOrientationsWrtRobot = currentTargetOrientationsWrtRobot_;

    if (currentTargetNames_.size() > 0)
    {
      int idx = isIn(currentTargetNames, currentTargetName_);

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] currentTargetName_: " << currentTargetName_ << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] idx: " << idx << std::endl;

      if (idx >= 0)
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] REGULAR" << std::endl;

        goalPos = targetPositions[idx];
        goalOri = targetOrientations[idx];

        //goalPosWrtRobot = targetPositionsWrtRobot[idx];
        //goalOriWrtRobot = targetOrientationsWrtRobot[idx];
      }
      else
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] SWITCH" << std::endl;

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] idx: " << idx << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] currentTargetName_: " << currentTargetName_ << std::endl;
        /*
        std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] currentTargetNames: " << std::endl;
        for (size_t i = 0; i < currentTargetNames.size(); i++)
        {
          std::cout << currentTargetNames[i] << std::endl;
        }
        */

        std::pair<double, int> closest_p;
        closest_p = findClosestDistance(targetPositions, robotPose);

        currentTargetName_ = currentTargetNames_[closest_p.second];
        goalPos = targetPositions[closest_p.second];
        goalOri = targetOrientations[closest_p.second];

        //goalPosWrtRobot = targetPositionsWrtRobot[closest_p.second];
        //goalOriWrtRobot = targetOrientationsWrtRobot[closest_p.second];

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] closest_p.second: " << closest_p.second << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] currentTargetName_: " << currentTargetName_ << std::endl;
      }

      goalPosition_ = goalPos;
      goalOrientation_ = goalOri;

      //goalPositionWrtRobot_ = goalPosWrtRobot;
      //goalOrientationWrtRobot_ = goalOriWrtRobot;

      if (autoFlag)
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] AUTO" << std::endl;
        updateGraspPose();
      }
      else
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] MANUAL" << std::endl;
        updateGraspPose(goalPosition_, goalOrientation_);
      }

      graspReadyFlag_ = true;

      //publishTargetVisu();
      //publishGraspFrame();
    }
    /*
    else
    {
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] DUMMY GOAL" << std::endl;

      goalPosition_ = dummyGoalPosition_;
      goalOrientation_ = dummyGoalOrientation_;
    }
    */
  }
  else if (taskMode_ == 2)
  {
    /// NUA NOTE: ADD WRT ROBOT POSE OF THE DROP TARGET!!!
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] DEBUG_INF" << std::endl;
    while(1);

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] taskMode_: 2" << std::endl;
    gazebo_msgs::ModelStates ms = modelStatesMsg_;

    for (size_t i = 0; i < ms.name.size(); i++)
    {
      if (ms.name[i] == dropTargetName_)
      {
        goalPos.x() = ms.pose[i].position.x;
        goalPos.y() = ms.pose[i].position.y;
        goalPos.z() = ms.pose[i].position.z;

        goalOri.x() = ms.pose[i].orientation.x;
        goalOri.y() = ms.pose[i].orientation.y;
        goalOri.z() = ms.pose[i].orientation.z;
        goalOri.w() = ms.pose[i].orientation.w;

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] targetPos.x(): " << targetPos.x() << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] targetPos.y(): " << targetPos.y() << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] targetPos.z(): " << targetPos.z() << std::endl;

        break;
      }
    }

    goalPosition_ = goalPos;
    goalOrientation_ = goalOri;

    if (autoFlag)
    {
      updateDropPose();
    }
    else
    {
      updateDropPose(goalPosition_, goalOrientation_);
    }

    dropReadyFlag_ = true;
    
    //publishTargetVisu();
    //publishDropFrame();
  }
  /*
  else
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] DUMMY GOAL DEFAULT" << std::endl;

    goalPosition_ = dummyGoalPosition_;
    goalOrientation_ = dummyGoalOrientation_;
  }
  */

  fillGoalVisu();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateGoal(const Eigen::Vector3d& goalPos, const Eigen::Quaterniond& goalOri)
{
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal(2)] START" << std::endl;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal(2)] DEBUG INF" << std::endl;
  //while(1);

  goalPosition_ = goalPos;
  goalOrientation_ = goalOri;

  fillGoalVisu();
  //publishTargetVisu();
  
  if (printOutFlag_)
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGoal(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateTarget(bool autoFlag)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] START" << std::endl;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] DEBUG_INF" << std::endl;
  while(1);

  Eigen::Vector3d targetPos;
  Eigen::Quaterniond targetOri;

  //Eigen::Vector3d targetPosWrtRobot;
  //Eigen::Quaterniond targetOriWrtRobot;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] taskMode_: " << taskMode_ << std::endl;

  if (taskMode_ == 1)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] taskMode_: 1" << std::endl;
    //updateTargetInfo();

    geometry_msgs::Pose robotPose = robotPose_;
    std::vector<std::string> currentTargetNames = currentTargetNames_;
    std::vector<Eigen::Vector3d> targetPositions = currentTargetPositions_;
    std::vector<Eigen::Quaterniond> targetOrientations = currentTargetOrientations_;
    //std::vector<Eigen::Vector3d> targetPositionsWrtRobot = currentTargetPositionsWrtRobot_;
    //std::vector<Eigen::Quaterniond> targetOrientationsWrtRobot = currentTargetOrientationsWrtRobot_;

    if (currentTargetNames_.size() > 0)
    {
      int idx = isIn(currentTargetNames, currentTargetName_);

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] idx: " << idx << std::endl;

      if (idx >= 0)
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] REGULAR" << std::endl;

        targetPos = targetPositions[idx];
        targetOri = targetOrientations[idx];

        //targetPosWrtRobot = targetPositionsWrtRobot[idx];
        //targetOriWrtRobot = targetOrientationsWrtRobot[idx];
      }
      else
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] SWITCH" << std::endl;

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] idx: " << idx << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] currentTargetNames: " << std::endl;
        for (size_t i = 0; i < currentTargetNames.size(); i++)
        {
          std::cout << currentTargetNames[i] << std::endl;
        }

        std::pair<double, int> closest_p;
        closest_p = findClosestDistance(targetPositions, robotPose);

        currentTargetName_ = currentTargetNames_[closest_p.second];
        targetPos = targetPositions[closest_p.second];
        targetOri = targetOrientations[closest_p.second];

        //targetPosWrtRobot = targetPositionsWrtRobot[closest_p.second];
        //targetOriWrtRobot = targetOrientationsWrtRobot[closest_p.second];

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] closest_p.second: " << closest_p.second << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;
      }

      currentTargetPosition_ = targetPos;
      currentTargetOrientation_ = targetOri;

      //currentTargetPositionWrtRobot_ = targetPosWrtRobot;
      //currentTargetOrientationWrtRobot_ = targetOriWrtRobot;

      if (autoFlag)
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] AUTO" << std::endl;
        updateGraspPose();
      }
      else
      {
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] MANUAL" << std::endl;
        updateGraspPose(currentTargetPosition_, currentTargetOrientation_);
      }

      graspReadyFlag_ = true;

      //publishTargetVisu();
      //publishGraspFrame();
    }
  }
  else if (taskMode_ == 2)
  {
    /// NUA NOTE: ADD WRT ROBOT POSE OF THE DROP TARGET!!!
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] DEBUG_INF" << std::endl;
    while(1);

    gazebo_msgs::ModelStates ms = modelStatesMsg_;

    for (size_t i = 0; i < ms.name.size(); i++)
    {
      if (ms.name[i] == dropTargetName_)
      {
        targetPos.x() = ms.pose[i].position.x;
        targetPos.y() = ms.pose[i].position.y;
        targetPos.z() = ms.pose[i].position.z;

        targetOri.x() = ms.pose[i].orientation.x;
        targetOri.y() = ms.pose[i].orientation.y;
        targetOri.z() = ms.pose[i].orientation.z;
        targetOri.w() = ms.pose[i].orientation.w;

        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] targetPos.x(): " << targetPos.x() << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] targetPos.y(): " << targetPos.y() << std::endl;
        //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] targetPos.z(): " << targetPos.z() << std::endl;

        break;
      }
    }

    currentTargetPosition_ = targetPos;
    currentTargetOrientation_ = targetOri;

    if (autoFlag)
    {
      updateDropPose();
    }
    else
    {
      updateDropPose(currentTargetPosition_, currentTargetOrientation_);
    }

    dropReadyFlag_ = true;

    
    //publishTargetVisu();
    //publishDropFrame();
  }

  fillTargetVisu();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] DEBUG INF" << std::endl;
  //while(1);

  targetReadyFlag_ = true;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateTarget(const Eigen::Vector3d& targetPos, const Eigen::Quaterniond& targetOri)
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] START" << std::endl;

    currentTargetPosition_ = targetPos;
    currentTargetOrientation_ = targetOri;

    /*
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] x: " << targetPos.x() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] y: " << targetPos.y() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] z: " << targetPos.z() << std::endl;

    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] qx: " << targetOri.x() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] qy: " << targetOri.y() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] qz: " << targetOri.z() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] qw: " << targetOri.w() << std::endl << std::endl;
    */

    fillTargetVisu();
    //publishTargetVisu();

    targetReadyFlag_ = true;
    
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget(2)] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateTarget] ERROR: CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateTarget] ERROR: CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
  }
}

void TargetTrajectoriesGazebo::updateGraspPose()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGraspPose] START" << std::endl;

  Eigen::Vector3d goalPosition = goalPosition_;
  Eigen::Quaterniond goalOrientation = goalOrientation_;

  tf::Transform transform_goal_wrt_world;
  tf::Transform transform_grasp_wrt_goal;
  tf::Transform transform_grasp_wrt_world;

  //transform_goal_wrt_world.setIdentity();
  transform_goal_wrt_world.setOrigin(tf::Vector3(goalPosition.x(), goalPosition.y(), goalPosition.z()));
  transform_goal_wrt_world.setRotation(tf::Quaternion(goalOrientation.x(), goalOrientation.y(), goalOrientation.z(), goalOrientation.w()));

  //transform_grasp_wrt_goal.setIdentity();
  transform_grasp_wrt_goal.setOrigin(tf::Vector3(graspPositionOffset_.x(), graspPositionOffset_.y(), graspPositionOffset_.z()));
  Eigen::Quaterniond graspOri_wrt_goal(graspOrientationOffsetMatrix_);
  transform_grasp_wrt_goal.setRotation(tf::Quaternion(graspOri_wrt_goal.x(), graspOri_wrt_goal.y(), graspOri_wrt_goal.z(), graspOri_wrt_goal.w()));
  
  transform_grasp_wrt_world = transform_goal_wrt_world * transform_grasp_wrt_goal;

  Eigen::Vector3d graspPos;
  graspPos.x() = transform_grasp_wrt_world.getOrigin().x();
  graspPos.y() = transform_grasp_wrt_world.getOrigin().y();
  graspPos.z() = transform_grasp_wrt_world.getOrigin().z();

  //Eigen::Matrix3d mat = goalOrientation.matrix() * dropOrientationOffsetMatrix_;
  Eigen::Quaterniond graspOri(transform_grasp_wrt_world.getRotation().w(), 
                              transform_grasp_wrt_world.getRotation().x(), 
                              transform_grasp_wrt_world.getRotation().y(), 
                              transform_grasp_wrt_world.getRotation().z());

  // Alternative: Rotating the target orientation by rpy
  //Eigen::Vector3d rpy_rot(M_PI, 0, 0);
  //rotateQuaternion(currentTargetOrientation, rpy_rot, graspOri);

  if (targetIsGoalFlag_)
  {
    //if (printOutFlag_)
    //  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGraspPose] TARGET IS GOAL!" << std::endl;
    Eigen::Vector3d graspPositionEstimated = graspPositionEstimated_; 
    updateTarget(graspPositionEstimated, graspOri);
  }

  currentGraspPosition_ = graspPos;
  currentGraspOrientation_ = graspOri;

  //graspFrameReadyFlag_ = true;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGraspPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateGraspPose(const Eigen::Vector3d& graspPos, const Eigen::Quaterniond& graspOri)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGraspPose(2)] START" << std::endl;
  currentGraspPosition_ = graspPos;
  currentGraspOrientation_ = graspOri;
  //graspFrameReadyFlag_ = true;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateGraspPose(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateDropPose()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDropPose] START" << std::endl;

  Eigen::Vector3d goalPosition = goalPosition_;
  Eigen::Quaterniond goalOrientation = goalOrientation_;

  tf::Transform transform_goal_wrt_world;
  tf::Transform transform_drop_wrt_goal;
  tf::Transform transform_drop_wrt_world;

  transform_goal_wrt_world.setOrigin(tf::Vector3(goalPosition.x(), goalPosition.y(), goalPosition.z()));
  transform_goal_wrt_world.setRotation(tf::Quaternion(goalOrientation.x(), goalOrientation.y(), goalOrientation.z(), goalOrientation.w()));

  //transform_drop_wrt_goal.setIdentity();
  transform_drop_wrt_goal.setOrigin(tf::Vector3(dropPositionOffset_.x(), dropPositionOffset_.y(), dropPositionOffset_.z()));
  Eigen::Quaterniond dropOri_wrt_goal(dropOrientationOffsetMatrix_);
  transform_drop_wrt_goal.setRotation(tf::Quaternion(dropOri_wrt_goal.x(), dropOri_wrt_goal.y(), dropOri_wrt_goal.z(), dropOri_wrt_goal.w()));

  transform_drop_wrt_world = transform_goal_wrt_world * transform_drop_wrt_goal;

  Eigen::Vector3d dropPos;
  dropPos.x() = transform_drop_wrt_world.getOrigin().x();
  dropPos.y() = transform_drop_wrt_world.getOrigin().y();
  dropPos.z() = transform_drop_wrt_world.getOrigin().z();

  //Eigen::Matrix3d mat = goalOrientation.matrix() * dropOrientationOffsetMatrix_;
  Eigen::Quaterniond dropOri(transform_drop_wrt_world.getRotation().w(), 
                             transform_drop_wrt_world.getRotation().x(), 
                             transform_drop_wrt_world.getRotation().y(), 
                             transform_drop_wrt_world.getRotation().z());

  currentDropPosition_ = dropPos;
  currentDropOrientation_ = dropOri;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDropPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateDropPose(const Eigen::Vector3d& targetPos, const Eigen::Quaterniond& targetOri)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDropPose(2)] START" << std::endl;

  currentDropPosition_ = targetPos;
  currentDropOrientation_ = targetOri;
  //dropFrameReadyFlag_ = true;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateDropPose(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::setTargetToEEPose()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] START" << std::endl;

  while(!initTFCallbackFlag_){ros::spinOnce();}

  tf::StampedTransform tf_ee_wrt_world = tf_ee_wrt_world_;

  currentTargetPosition_.x() = tf_ee_wrt_world.getOrigin().x();
  currentTargetPosition_.y() = tf_ee_wrt_world.getOrigin().y();
  currentTargetPosition_.z() = tf_ee_wrt_world.getOrigin().z();
  currentTargetOrientation_.x() = tf_ee_wrt_world.getRotation().x();
  currentTargetOrientation_.y() = tf_ee_wrt_world.getRotation().y();
  currentTargetOrientation_.z() = tf_ee_wrt_world.getRotation().z();
  currentTargetOrientation_.w() = tf_ee_wrt_world.getRotation().w();

  /*
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] pos x: " << currentTargetPosition_.x() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] pos y: " << currentTargetPosition_.y() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] pos z: " << currentTargetPosition_.z() << std::endl;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] ori x: " << currentTargetOrientation_.x() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] ori y: " << currentTargetOrientation_.y() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] ori z: " << currentTargetOrientation_.z() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] ori w: " << currentTargetOrientation_.w() << std::endl;
  */

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateEEPose(Eigen::Vector3d& eePos, Eigen::Quaterniond& eeOri)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] START" << std::endl;

  tf::StampedTransform tf_ee_wrt_world = tf_ee_wrt_world_;

  eePos.x() = tf_ee_wrt_world.getOrigin().x();
  eePos.y() = tf_ee_wrt_world.getOrigin().y();
  eePos.z() = tf_ee_wrt_world.getOrigin().z();
  eeOri.x() = tf_ee_wrt_world.getRotation().x();
  eeOri.y() = tf_ee_wrt_world.getRotation().y();
  eeOri.z() = tf_ee_wrt_world.getRotation().z();
  eeOri.w() = tf_ee_wrt_world.getRotation().w();

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateEEPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::fillGoalVisu()
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillGoalVisu] START" << std::endl;

    visualization_msgs::MarkerArray goalMarkerArray;
    //goalMarkerArray_.markers.clear();

    /// NUA TODO: User should able to add more goal!
    int n_goal = 1;

    for(int i = 0; i < n_goal; i++)
    {
      Eigen::Vector3d goalPosition = goalPosition_;
      Eigen::Quaterniond goalOrientation = goalOrientation_;
      geometry_msgs::Point goal_point;
      geometry_msgs::Point grasp_or_drop_point;
      goal_point.x = goalPosition.x();
      goal_point.y = goalPosition.y();
      goal_point.z = goalPosition.z();
      grasp_or_drop_point = goal_point;

      if (taskMode_ == 1)
      {
        Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
        grasp_or_drop_point.x = currentGraspPosition.x();
        grasp_or_drop_point.y = currentGraspPosition.y();
        grasp_or_drop_point.z = currentGraspPosition.z();
      }
      else if (taskMode_ == 2)
      {
        Eigen::Vector3d currentDropPosition = currentDropPosition_;
        grasp_or_drop_point.x = currentDropPosition.x();
        grasp_or_drop_point.y = currentDropPosition.y();
        grasp_or_drop_point.z = currentDropPosition.z();

        goal_point.z = grasp_or_drop_point.z;
      }

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillGoalVisu] grasp_or_drop_point: " << std::endl;
      //print(grasp_or_drop_point);

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillGoalVisu] goal_point: " << std::endl;
      //print(goal_point);

      visualization_msgs::Marker goal_visu;
      goal_visu.ns = "goal_" + std::to_string(i+1);
      goal_visu.id = i+1;
      goal_visu.action = visualization_msgs::Marker::ADD;
      goal_visu.type = visualization_msgs::Marker::ARROW;
      goal_visu.points.push_back(grasp_or_drop_point);
      goal_visu.points.push_back(goal_point);
      goal_visu.pose.orientation.x = 0;
      goal_visu.pose.orientation.y = 0;
      goal_visu.pose.orientation.z = 0;
      goal_visu.pose.orientation.w = 1;
      goal_visu.scale.x = 0.04;
      goal_visu.scale.y = 0.08;
      goal_visu.scale.z = 0.08;
      goal_visu.color.r = 0.5;
      goal_visu.color.g = 0.0;
      goal_visu.color.b = 0.5;
      goal_visu.color.a = 0.5;
      goal_visu.header.frame_id = worldFrameName_;

      goalMarkerArray.markers.push_back(goal_visu);
    }

    goalMarkerArray_ = goalMarkerArray;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillGoalVisu] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillGoalVisu] ERROR: CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::fillGoalVisu] ERROR: CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::fillTargetVisu()
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillTargetVisu] START" << std::endl;

    visualization_msgs::MarkerArray targetMarkerArray;
    //targetMarkerArray_.markers.clear();

    /// NUA TODO: User should able to add more target!
    int target_size = 1;

    for(int i = 0; i < 3; i++)
    {
      Eigen::Vector3d currentTargetPosition = currentTargetPosition_;
      Eigen::Quaterniond currentTargetOrientation = currentTargetOrientation_;
      //geometry_msgs::Point target_point;
      //geometry_msgs::Point grasp_or_drop_point;
      //target_point.x = currentTargetPosition.x();
      //target_point.y = currentTargetPosition.y();
      //target_point.z = currentTargetPosition.z();
      //grasp_or_drop_point = target_point;

      /*
      if (taskMode_ == 1)
      {
        Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
        grasp_or_drop_point.x = currentGraspPosition.x();
        grasp_or_drop_point.y = currentGraspPosition.y();
        grasp_or_drop_point.z = currentGraspPosition.z();
      }
      else if (taskMode_ == 2)
      {
        Eigen::Vector3d currentDropPosition = currentDropPosition_;
        grasp_or_drop_point.x = currentDropPosition.x();
        grasp_or_drop_point.y = currentDropPosition.y();
        grasp_or_drop_point.z = currentDropPosition.z();

        target_point.z = grasp_or_drop_point.z;
      }
      */

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillTargetVisu] grasp_or_drop_point: " << std::endl;
      //print(grasp_or_drop_point);

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillTargetVisu] target_point: " << std::endl;
      //print(target_point);

      visualization_msgs::Marker target_visu;
      target_visu.ns = "target_" + std::to_string(i+1);
      target_visu.id = i+1;
      target_visu.action = visualization_msgs::Marker::ADD;
      target_visu.type = visualization_msgs::Marker::CYLINDER;
      //target_visu.type = visualization_msgs::Marker::ARROW;
      //target_visu.points.push_back(grasp_or_drop_point);
      //target_visu.points.push_back(target_point);
      target_visu.pose.position.x = currentTargetPosition.x();
      target_visu.pose.position.y = currentTargetPosition.y();
      target_visu.pose.position.z = currentTargetPosition.z();
      target_visu.pose.orientation.x = currentTargetOrientation.x();
      target_visu.pose.orientation.y = currentTargetOrientation.y();
      target_visu.pose.orientation.z = currentTargetOrientation.z();
      target_visu.pose.orientation.w = currentTargetOrientation.w();
      //target_visu.scale.x = 0.05;
      //target_visu.scale.y = 0.05;
      //target_visu.scale.z = 0.1;
      
      if (i == 0)
      {
        target_visu.scale.x = 0.2;
        target_visu.scale.y = 0.02;
        target_visu.scale.z = 0.02;

        target_visu.color.r = 0.8;
        target_visu.color.g = 0.0;
        target_visu.color.b = 0.0;
      }
      if (i == 1)
      {
        target_visu.scale.x = 0.02;
        target_visu.scale.y = 0.2;
        target_visu.scale.z = 0.02;

        target_visu.color.r = 0.0;
        target_visu.color.g = 0.8;
        target_visu.color.b = 0.0;
      }
      if (i == 2)
      {
        target_visu.scale.x = 0.02;
        target_visu.scale.y = 0.02;
        target_visu.scale.z = 0.2;

        target_visu.color.r = 0.0;
        target_visu.color.g = 0.0;
        target_visu.color.b = 0.8;
      }
      
      target_visu.color.a = 0.5;
      target_visu.header.frame_id = worldFrameName_;

      targetMarkerArray.markers.push_back(target_visu);
    }

    targetMarkerArray_ = targetMarkerArray;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillTargetVisu] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::fillTargetVisu] ERROR: CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::fillTargetVisu] ERROR: CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishGoalVisu()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalVisu] START" << std::endl;

  /// NUA TODO: User should able to add more target!
  std::lock_guard<std::mutex> lock(goalMarkerArrayMutex_);
  visualization_msgs::MarkerArray goalMarkerArray = goalMarkerArray_;
  int target_size = goalMarkerArray.markers.size();

  if (target_size > 0)
  {
    for(int i = 0; i < target_size; i++)
    {
      //goalMarkerArray.markers[i].header.seq++;
      goalMarkerArray.markers[i].header.stamp = ros::Time::now();
    }
    goalMarkerArrayPublisher_.publish(goalMarkerArray);
  }
  //else
  //{
  //  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalVisu] NO GOAL!" << std::endl;
  //}

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalVisu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishTargetVisu()
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetVisu] START" << std::endl;

    /// NUA TODO: User should able to add more target!
    std::lock_guard<std::mutex> lock(targetMarkerArrayMutex_);
    visualization_msgs::MarkerArray targetMarkerArray = targetMarkerArray_;
    int target_size = targetMarkerArray.markers.size();

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetVisu] target_size: " << target_size << std::endl;
    if (target_size > 0)
    {
      for(int i = 0; i < target_size; i++)
      {
        //targetMarkerArray.markers[i].header.seq++;
        targetMarkerArray.markers[i].header.stamp = ros::Time::now();
      }
      targetMarkerArrayPublisher_.publish(targetMarkerArray);
    }
    //else
    //{
    //  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetVisu] NO TARGET!" << std::endl;
    //}

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetVisu] END" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetVisu] ERROR: CATCHUP! " << e.what() << std::endl;
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
  }
  
  
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishGoalFrame()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] START" << std::endl;

  Eigen::Vector3d goalPosition = goalPosition_;
  Eigen::Quaterniond goalOrientation = goalOrientation_;

  /*
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalPosition x: " << goalPosition.x() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalPosition y: " << goalPosition.y() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalPosition z: " << goalPosition.z() << std::endl;

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalOrientation x: " << goalOrientation.x() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalOrientation y: " << goalOrientation.y() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalOrientation z: " << goalOrientation.z() << std::endl;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] goalOrientation w: " << goalOrientation.w() << std::endl;
  */

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] DEBUG_INF" << std::endl;
  //while(1);

  static tf::TransformBroadcaster tf_br;
  tf::Transform tf_virtual;
  tf_virtual.setOrigin(tf::Vector3(goalPosition.x(), goalPosition.y(), goalPosition.z()));
  tf_virtual.setRotation(tf::Quaternion(goalOrientation.x(), goalOrientation.y(), goalOrientation.z(), goalOrientation.w()));

  tf_br.sendTransform(tf::StampedTransform(tf_virtual, ros::Time::now(), worldFrameName_, goalFrameName_));

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGoalFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishGraspFrame()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGraspFrame] START" << std::endl;

  if (graspReadyFlag_ && taskMode_ == 1 && !pickedFlag_)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGraspFrame] START" << std::endl;

    Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
    Eigen::Quaterniond currentGraspOrientation = currentGraspOrientation_;

    static tf::TransformBroadcaster tf_br;
    tf::Transform tf_virtual;
    tf_virtual.setOrigin(tf::Vector3(currentGraspPosition.x(), currentGraspPosition.y(), currentGraspPosition.z()));
    tf_virtual.setRotation(tf::Quaternion(currentGraspOrientation.x(), currentGraspOrientation.y(), currentGraspOrientation.z(), currentGraspOrientation.w()));

    tf_br.sendTransform(tf::StampedTransform(tf_virtual, ros::Time::now(), worldFrameName_, graspFrameName_));

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGraspFrame] END" << std::endl;
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishGraspFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishDropFrame()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishDropFrame] START" << std::endl;

  if (dropReadyFlag_ && taskMode_ == 2 && pickedFlag_)
  {
    Eigen::Vector3d currentDropPosition = currentDropPosition_;
    Eigen::Quaterniond currentDropOrientation = currentDropOrientation_;

    static tf::TransformBroadcaster tf_br;
    tf::Transform tf_virtual;
    tf_virtual.setOrigin(tf::Vector3(currentDropPosition.x(), currentDropPosition.y(), currentDropPosition.z()));
    tf_virtual.setRotation(tf::Quaternion(currentDropOrientation.x(), currentDropOrientation.y(), currentDropOrientation.z(), currentDropOrientation.w()));

    tf_br.sendTransform(tf::StampedTransform(tf_virtual, ros::Time::now(), worldFrameName_, dropFrameName_));
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishDropFrame] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishTargetTrajectories()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetTrajectories] START" << std::endl;

  //while(!policyReceivedFlag_){ros::spinOnce();}
  //policyReceivedFlag_ = false;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetTrajectories] targetReadyFlag_: " << targetReadyFlag_ << std::endl;
  if (targetReadyFlag_)
  {
    // Get the latest observation
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // Get target trajectories
    Eigen::Vector3d currentTargetPosition = currentTargetPosition_;
    Eigen::Quaterniond currentTargetOrientation = currentTargetOrientation_;
    auto targetTrajectories = goalPoseToTargetTrajectories_(currentTargetPosition, currentTargetOrientation, observation);
    targetTrajectories.taskMode = taskMode_;

    // Publish target trajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetTrajectories] PUBLISHED" << std::endl;
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetTrajectories] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void TargetTrajectoriesGazebo::publishMobimanGoalObs(bool onlyPosFlag)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] START" << std::endl;

  std::deque<std::vector<double>> goalTrajectoryQueue = goalTrajectoryQueue_;
  int qsize = goalTrajectoryQueue.size();
  int minSize = (mobimanGoalObsTrajSampleNum_-1) * mobimanGoalObsTrajSampleFreq_;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] qsize: " << qsize << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] minSize: " << minSize << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] goalTrajectoryQueueDt_: " << goalTrajectoryQueueDt_ << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] mobimanGoalObsTrajSampleNum_: " << mobimanGoalObsTrajSampleNum_ << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] mobimanGoalObsTrajSampleFreq_: " << mobimanGoalObsTrajSampleFreq_ << std::endl;

  if (qsize > minSize && goalTrajectoryQueueDt_ > 0)
  {
    std::vector<double> goalTrajectory;
    int idx;
    int dim_dt = 6;
    int posOffset = 0;
    if (onlyPosFlag)
    {
      posOffset = 3;
      dim_dt = 3;
    }

    for (size_t i = 0; i < mobimanGoalObsTrajSampleNum_; i++)
    {
      idx = (qsize - 1) - (i * mobimanGoalObsTrajSampleFreq_);
      goalTrajectory.insert(goalTrajectory.end(), goalTrajectoryQueue[idx].begin(), goalTrajectoryQueue[idx].end()-posOffset); 
    }

    /*
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishMobimanGoalObs] goalTrajectory size: " << goalTrajectory.size() << std::endl;
    for (size_t i = 0; i < goalTrajectory.size(); i++)
    {
      std::cout << i << " -> " << goalTrajectory[i] << std::endl;
    }
    * /

    ocs2_msgs::MobimanGoalObservation mgo;
    mgo.header.seq = mobimanGoalObsSeq_;
    mgo.header.frame_id = goalTrajectoryFrameName_;
    mgo.header.stamp = ros::Time::now();
    mgo.dim_dt = dim_dt;
    mgo.dt = goalTrajectoryQueueDt_ * mobimanGoalObsTrajSampleFreq_;
    mgo.obs = goalTrajectory;
    mobimanGoalObsPublisher_.publish(mgo);

    if (mobimanGoalObsSeq_ >= INT_MAX)
    {
      mobimanGoalObsSeq_ = 0;
    }
    else
    {
      mobimanGoalObsSeq_++;
    }
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::publishTargetTrajectories] END" << std::endl << std::endl;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishTargetTrajectories(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
  //while(!policyReceivedFlag_){ros::spinOnce();}
  //policyReceivedFlag_ = false;

  // Get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  // Get target trajectories
  auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);
  targetTrajectories.taskMode = taskMode_;

  // Publish target trajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateCallback(const ros::TimerEvent& event)
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] START" << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE updateGoal" << std::endl;
    updateGoal(true);
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] AFTER updateGoal" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: updateGoal CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: updateGoal CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishTargetTrajectories" << std::endl;
    publishTargetTrajectories();
    //publishMobimanGoalObs();
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishTargetTrajectories CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishTargetTrajectories CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishGoalVisu" << std::endl;
    publishGoalVisu();
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishGoalVisu CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishGoalVisu CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishTargetVisu" << std::endl;
    publishTargetVisu();
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishTargetVisu CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishTargetVisu CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishGoalFrame" << std::endl;
    publishGoalFrame();
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishGoalFrame CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishGoalFrame CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishGraspFrame" << std::endl;
    publishGraspFrame();
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] AFTER publishGraspFrame" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishGraspFrame CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishGraspFrame CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }

  /// NUA TODO: ADD THIS WHEN REQUIRED!
  /*
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] BEFORE publishDropFrame" << std::endl;
    publishDropFrame();

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] END" << std::endl << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::updateCallback] ERROR: publishDropFrame CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::updateCallback] ERROR: publishDropFrame CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
  }
  */
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void TargetTrajectoriesGazebo::goalTrajectoryTimerCallback(const ros::TimerEvent& event)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::goalTrajectoryTimerCallback] START" << std::endl;

  /*
  auto currentTime = std::chrono::steady_clock::now();
  double currentDuration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime_).count() * 0.001;
  startTime_ = currentTime;
  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::goalTrajectoryTimerCallback] currentDuration: " << currentDuration << std::endl;
  * /

  double roll, pitch, yaw;
  Eigen::Vector3d goalPosition;
  Eigen::Quaterniond goalOrientation;

  if (goalTrajectoryFrameName_ == robotFrameName_)
  {
    goalPosition = goalPositionWrtRobot_;
    goalOrientation = goalOrientationWrtRobot_;
  }
  else
  {
    goalPosition = goalPosition_;
    goalOrientation = goalOrientation_;
  }

  tf::Quaternion quatBase(goalOrientation.x(), goalOrientation.y(), goalOrientation.z(), goalOrientation.w());
  tf::Matrix3x3 matBase(quatBase);
  matBase.getRPY(roll, pitch, yaw);

  std::vector<double> goalTrajectory = {goalPosition.x(), goalPosition.y(), goalPosition.z(), roll, pitch, yaw};

  goalTrajectoryQueue_.push_back(goalTrajectory);
  if (goalTrajectoryQueue_.size() >= goalTrajectoryQueueSize_)
  {
    goalTrajectoryQueue_.pop_front();
  }

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::goalTrajectoryTimerCallback] DEBUG_INF" << std::endl;
  //while (1);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::goalTrajectoryTimerCallback] END" << std::endl << std::endl;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerTarget() const 
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = worldFrameName_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Target";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = 0.0;
  interactiveMarker.pose.position.y = 0.0;
  interactiveMarker.pose.position.z = 1.0;

  // create a grey box marker
  const auto boxMarker = []() 
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerAutoTarget() const 
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "AutoTarget";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = 0.0;
  interactiveMarker.pose.position.y = 1.0;
  interactiveMarker.pose.position.z = 1.0;

  visualization_msgs::Marker boxMarker;
  boxMarker.type = visualization_msgs::Marker::CUBE;
  boxMarker.scale.x = 0.1;
  boxMarker.scale.y = 0.1;
  boxMarker.scale.z = 0.1;
  boxMarker.color.a = 0.5;
  boxMarker.color.r = 0.5;
  boxMarker.color.g = 0.0;
  boxMarker.color.b = 0.5;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  //control.name = "rotate_x";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  //control.name = "rotate_z";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  //control.name = "rotate_y";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerDropTarget() const 
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "DropTarget";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = dropTargetPosition_.x();
  interactiveMarker.pose.position.y = dropTargetPosition_.y();
  interactiveMarker.pose.position.z = dropTargetPosition_.z() + 1.0;

  visualization_msgs::Marker boxMarker;
  boxMarker.type = visualization_msgs::Marker::CYLINDER;
  boxMarker.scale.x = 0.1;
  boxMarker.scale.y = 0.1;
  boxMarker.scale.z = 0.2;
  boxMarker.color.a = 0.5;
  boxMarker.color.r = 0.0;
  boxMarker.color.g = 1.0;
  boxMarker.color.b = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  return interactiveMarker;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerModelMode() const
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] START" << std::endl;

  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "ModelMode";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = 1.0;
  interactiveMarker.pose.position.y = 0.0;
  interactiveMarker.pose.position.z = 1.0;

  visualization_msgs::Marker boxMarker;
  boxMarker.type = visualization_msgs::Marker::SPHERE;
  boxMarker.scale.x = 0.1;
  boxMarker.scale.y = 0.1;
  boxMarker.scale.z = 0.1;
  boxMarker.color.a = 0.5;
  boxMarker.color.r = 1.0;
  boxMarker.color.g = 0.1;
  boxMarker.color.b = 0.1;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] RED" << std::endl;
  if (statusModelModeMPC_ && statusModelModeMRT_)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] GREEN" << std::endl;
    boxMarker.color.r = 0.1;
    boxMarker.color.g = 1.0;
    boxMarker.color.b = 0.1;
  }
  else if (statusModelModeMPC_)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] BLUE" << std::endl;
    boxMarker.color.r = 0.1;
    boxMarker.color.g = 0.1;
    boxMarker.color.b = 1.0;
  }
  else if (statusModelModeMRT_)
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] MAGENTA" << std::endl;
    boxMarker.color.r = 1.0;
    boxMarker.color.g = 0.1;
    boxMarker.color.b = 1.0;
  }

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  //control.name = "rotate_x";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  //control.name = "rotate_z";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  //control.name = "rotate_y";
  //control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  //interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  // create menu marker
  visualization_msgs::InteractiveMarkerControl control_button;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  //control.always_visible = true;
  interactiveMarker.controls.push_back(control_button);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] END" << std::endl;

  return interactiveMarker;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::createMenuModelMode()
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createMenuModelMode] START" << std::endl;

  //interactive_markers::MenuHandler menuHandlerModelMode;

  if (!initMenuModelModeFlag_)
  {
    h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::BaseMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
    menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

    h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::ArmMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
    menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

    h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::WholeBodyMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
    menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

    //check the very last entry
    menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::CHECKED );
  }

  initMenuModelModeFlag_ = true;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::createMenuModelMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::processFeedbackTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] START" << std::endl;

  // Set task mode
  taskMode_ = 0;

  // Set desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, 
                                 feedback->pose.position.y, 
                                 feedback->pose.position.z);
  const Eigen::Quaterniond orientation(feedback->pose.orientation.w, 
                                       feedback->pose.orientation.x, 
                                       feedback->pose.orientation.y,
                                       feedback->pose.orientation.z);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] Waiting a new policy..." << std::endl;
  //while(!policyReceivedFlag_){ros::spinOnce();}
  //policyReceivedFlag_ = false;

  // Get the latest observation
  //SystemObservation observation;
  //{
  //  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  //  observation = latestObservation_;
  //}

  // Update target
  updateGoal(position, orientation);
  updateTarget(position, orientation);

  //publishTargetTrajectories();

  // Get target trajectories
  //auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);
  //targetTrajectories.taskMode = taskMode_;

  // Publish target trajectories
  //targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

  // Run service client to set task
  geometry_msgs::Pose target;
  target.position.x = position.x();
  target.position.y = position.y();
  target.position.z = position.z();
  target.orientation.x = orientation.x();
  target.orientation.y = orientation.y();
  target.orientation.z = orientation.z();
  target.orientation.w = orientation.w();
  bool taskModeSuccess = setTask(taskMode_, target);
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] taskModeSuccess: " << taskModeSuccess << std::endl;

  //targetReadyFlag_ = true;

  //ros::Duration(5.0).sleep();

  // Get target trajectories
  //auto targetTrajectories2 = goalPoseToTargetTrajectories_(position, orientation, observation);
  //targetTrajectories2.taskMode = taskMode_;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] DUBLO" << std::endl;

  // Publish target trajectories
  //targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories2);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] END" << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::processFeedbackAutoTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackAutoTarget] START" << std::endl;

  // Set task mode
  if (!pickedFlag_)
  {
    taskMode_ = 1;
  }
  else
  {
    taskMode_ = 2;
  }

  // Update target
  updateGoal(true);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackAutoTarget] taskMode_: " << taskMode_ << std::endl;

  // Set desired state trajectory
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  if (taskMode_ == 1)
  {
    position = currentGraspPosition_;
    orientation = currentGraspOrientation_;
  }
  else if (taskMode_ == 2)
  {
    position = currentDropPosition_;
    orientation = currentDropOrientation_;
  }

  updateTarget(position, orientation);

  //publishTargetTrajectories(position, orientation);

  // Get the latest observation
  //SystemObservation observation;
  //{
  //  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  //  observation = latestObservation_;
  //}

  // Get target trajectories
  //auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);
  //targetTrajectories.taskMode = taskMode_;

  // Publish target trajectories
  //targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

  // Run service client to set task
  geometry_msgs::Pose target;
  target.position.x = position.x();
  target.position.y = position.y();
  target.position.z = position.z();
  target.orientation.x = orientation.x();
  target.orientation.y = orientation.y();
  target.orientation.z = orientation.z();
  target.orientation.w = orientation.w();
  bool taskModeSuccess = setTask(taskMode_, target);
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackTarget] taskModeSuccess: " << taskModeSuccess << std::endl;

  //targetReadyFlag_ = true;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackAutoTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void TargetTrajectoriesGazebo::processFeedbackDropTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackDropTarget] START" << std::endl;

  // Desired state trajectory
  const Eigen::Vector3d position= dropPosition_;
  const Eigen::Quaterniond orientation = dropOrientation_;

  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  updateTarget(position, orientation);

  // get TargetTrajectories
  auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);
  targetTrajectories.taskMode = 1;

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackDropTarget] END" << std::endl;
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::processFeedbackModelMode(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackModelMode] START" << std::endl;
  std_msgs::UInt8 model_mode_int;
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last_ = feedback->menu_entry_id;
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::CHECKED );

  switch (feedback->menu_entry_id) 
  {
    case 1:
    {
      if (printOutFlag_)
        std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::BaseMotion" << std::endl;
      model_mode_int.data = 0;
      break;
    }
    
    case 2:
    {
      if (printOutFlag_)
        std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::ArmMotion" << std::endl;
      model_mode_int.data = 1;
      break;
    }
    
    case 3: 
    {
      if (printOutFlag_)
        std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::WholeBodyMotion" << std::endl;
      model_mode_int.data = 2;
      break;
    }

    default:
      throw std::invalid_argument("[TargetTrajectoriesGazebo::processFeedbackModelMode] ERROR: Invalid menu entry id");
  }

  menuHandlerModelMode_.reApply(modelModeServer_);
  modelModeServer_.applyChanges();

  modelModePublisher_.publish(model_mode_int);

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::processFeedbackModelMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::setPickedFlagSrv(ocs2_msgs::setBool::Request &req, 
                                                ocs2_msgs::setBool::Response &res)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setPickedFlagSrv] START" << std::endl;
  pickedFlag_ = req.val;
  res.success = true;

  if (pickedFlag_)
  {
    taskMode_ = 2;
  }
  else
  {
    taskMode_ = 1;
  }

  updateGoal(true);

  std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setPickedFlagSrv] pickedFlag_: " << pickedFlag_ << std::endl;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setPickedFlagSrv] END" << std::endl;
  return true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::setSystemObservationSrv(ocs2_msgs::setSystemObservation::Request &req, 
                                                       ocs2_msgs::setSystemObservation::Response &res)
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setSystemObservationSrv] START" << std::endl;
    latestObservation_ = ros_msg_conversions::readObservationMsg(req.obs);
    res.success = true;

    policyReceivedFlag_ = true;
    
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setSystemObservationSrv] END" << std::endl;
    return true;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setSystemObservationSrv] ERROR: CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::setSystemObservationSrv] ERROR: CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //return false;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::setTargetDRLSrv(ocs2_msgs::setTask::Request &req, 
                                               ocs2_msgs::setTask::Response &res)
{
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] START" << std::endl;

  Eigen::Vector3d targetPos;
  Eigen::Quaterniond targetOri;
  Eigen::Vector3d graspPositionEstimated;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] taskMode_: " << taskMode_ << std::endl;
  int taskMode = taskMode_;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] targetName: " << req.targetName << std::endl;
  std::string targetName = req.targetName;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] timeHorizon: " << req.time_horizon << std::endl;
  double timeHorizon = req.time_horizon;

  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE res.success" << std::endl;
  res.success = true;
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER res.success" << std::endl;

  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] START try 1" << std::endl;

    if (targetName == "target")
    {
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] TARGET IS TARGET" << std::endl;

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE target_x: " << req.targetPose.position.x << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE target_y: " << req.targetPose.position.y << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE target_z: " << req.targetPose.position.z << std::endl;

      targetIsGoalFlag_ = false; 
      geometry_msgs::Pose targetPoseWrtRobot = req.targetPose;
      geometry_msgs::Pose targetPoseWrtWorld;
      transformPose(robotFrameName_, worldFrameName_, targetPoseWrtRobot, targetPoseWrtWorld);
      targetPos.x() = targetPoseWrtWorld.position.x;
      targetPos.y() = targetPoseWrtWorld.position.y;
      targetPos.z() = targetPoseWrtWorld.position.z;

      targetOri.x() = targetPoseWrtWorld.orientation.x;
      targetOri.y() = targetPoseWrtWorld.orientation.y;
      targetOri.z() = targetPoseWrtWorld.orientation.z;
      targetOri.w() = targetPoseWrtWorld.orientation.w;

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER target_x: " << targetPoseWrtWorld.position.x << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER target_y: " << targetPoseWrtWorld.position.y << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER target_z: " << targetPoseWrtWorld.position.z << std::endl;

      //updateTarget(targetPos, targetOri);

      graspPositionEstimated.x() = 0.0;
      graspPositionEstimated.y() = 0.0;
      graspPositionEstimated.z() = 0.0;
    }
    else if (targetName == "goal")
    {
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] TARGET IS GOAL" << std::endl;

      geometry_msgs::Pose graspPoseEstimatedWrtGoal = req.targetPose;
      geometry_msgs::Pose graspPoseEstimatedWrtWorld;
      bool isSuccess = transformPose(graspFrameName_, worldFrameName_, graspPoseEstimatedWrtGoal, graspPoseEstimatedWrtWorld);

      targetPos.x() = graspPoseEstimatedWrtWorld.position.x;
      targetPos.y() = graspPoseEstimatedWrtWorld.position.y;
      targetPos.z() = graspPoseEstimatedWrtWorld.position.z;

      targetOri.x() = graspPoseEstimatedWrtWorld.orientation.x;
      targetOri.y() = graspPoseEstimatedWrtWorld.orientation.y;
      targetOri.z() = graspPoseEstimatedWrtWorld.orientation.z;
      targetOri.w() = graspPoseEstimatedWrtWorld.orientation.w;

      //updateTarget(targetPos, targetOri);

      graspPositionEstimated.x() = graspPoseEstimatedWrtWorld.position.x;
      graspPositionEstimated.y() = graspPoseEstimatedWrtWorld.position.y;
      graspPositionEstimated.z() = graspPoseEstimatedWrtWorld.position.z;

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] isSuccess: " << isSuccess << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] graspPositionEstimated x: " << graspPositionEstimated.x() << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] graspPositionEstimated y: " << graspPositionEstimated.y() << std::endl;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] graspPositionEstimated z: " << graspPositionEstimated.z() << std::endl;

      targetIsGoalFlag_ = true;
    }
    else
    {
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: Invalid target name!" << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] DEBUG_INF" << std::endl;
      while(1);
    }

    graspPositionEstimated_ = graspPositionEstimated;

    //Eigen::Vector3d targetPos(targetPose.position.x, targetPose.position.y, targetPose.position.z);
    //Eigen::Quaterniond targetOri(targetPose.orientation.w, targetPose.orientation.x, targetPose.orientation.y, targetPose.orientation.z);

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] x: " << targetPos.x() << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] y: " << targetPos.y() << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] z: " << targetPos.z() << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] qx: " << targetPos[1] << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] qy: " << targetPos[2] << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] qz: " << targetPos[3] << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] qw: " << targetPos[0] << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] time_horizon: " << req.time_horizon << std::endl;

    if (targetPos.z() < -0.1)
    {
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] targetName: " << targetName << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] req.targetPose x: " << req.targetPose.position.x << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] req.targetPose y: " << req.targetPose.position.y << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] req.targetPose z: " << req.targetPose.position.z << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] WTF x wrt WORLD: " << targetPos.x() << std::endl << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] WTF y wrt WORLD: " << targetPos.y() << std::endl << std::endl;
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] WTF z wrt WORLD: " << targetPos.z() << std::endl << std::endl;
      targetPos.z() = 0.0;

      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] DEBUG_INF" << std::endl;
      //while(1);
    }

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] END try 1" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: PART 1 CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: PART 1 CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //return false;
  }
  
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] START try 2" << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE updateTarget" << std::endl;
    updateTarget(targetPos, targetOri);
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER updateTarget" << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] END try 2" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: updateTarget CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: updateTarget CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //return false;
  }
  
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] START try 3" << std::endl;

    // Run service client to set task
    geometry_msgs::Pose target;
    target.position.x = targetPos.x();
    target.position.y = targetPos.y();
    target.position.z = targetPos.z();
    target.orientation.x = targetOri.x();
    target.orientation.y = targetOri.y();
    target.orientation.z = targetOri.z();
    target.orientation.w = targetOri.w();

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] taskMode: " << taskMode << std::endl;
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] BEFORE setTask" << std::endl;
    bool taskModeSuccess = setTask(taskMode, target, timeHorizon);
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] AFTER setTask" << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] END try 3" << std::endl;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] taskMode: " << taskMode << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target pos x: " << targetPos.x() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target pos y: " << targetPos.y() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target pos z: " << targetPos.z() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target ori x: " << targetOri.x() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target ori y: " << targetOri.y() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target ori z: " << targetOri.z() << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] target ori w: " << targetOri.w() << std::endl;
    
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] timeHorizon" << timeHorizon << std::endl;

    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: setTask CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::setTargetDRLSrv] ERROR: setTask CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
    //return false;
  }
  
  //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTargetDRLSrv] END" << std::endl;
  return true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool TargetTrajectoriesGazebo::setTask(int taskMode, geometry_msgs::Pose targetPose, double time_horizon)
{
  try
  {
    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] START" << std::endl;

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] DEBUG_INF" << std::endl;
    //while(1);

    std::string currentTargetName = currentTargetName_;

    /*
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] taskMode: " << taskMode << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] currentTargetName: " << currentTargetName << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] time_horizon: " << time_horizon << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.position.x: " << targetPose.position.x << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.position.y: " << targetPose.position.y << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.position.z: " << targetPose.position.z << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.orientation.x: " << targetPose.orientation.x << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.orientation.y: " << targetPose.orientation.y << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.orientation.z: " << targetPose.orientation.z << std::endl;
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] targetPose.orientation.w: " << targetPose.orientation.w << std::endl;
    */

    bool success = false;
    ocs2_msgs::setTask srv;
    srv.request.taskMode = taskMode;
    srv.request.targetName = currentTargetName;
    srv.request.targetAttachLinkName = currentTargetName + "_base_link";
    srv.request.targetPose = targetPose;
    srv.request.time_horizon = time_horizon;
    if (setTaskClient_.call(srv))
    {
      success = srv.response.success;
      //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] success: " << success << std::endl;
    }
    else
    {
      std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] ERROR: Failed to call service!" << std::endl;
      ROS_ERROR("[TargetTrajectoriesGazebo::setTask] ERROR: Failed to call service!");
      success = false;
    }

    //std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] END" << std::endl;
    
    return success;
  }
  catch(const std::exception& e)
  {
    std::cout << "[" << ns_ <<  "][TargetTrajectoriesGazebo::setTask] ERROR: CATCHUP! " << e.what() << std::endl;
    //const std::string msg = "[TargetTrajectoriesGazebo::setTask] ERROR: CATCHUP! \n";
    //throw std::runtime_error(msg + e.what());
    //std::cerr << e.what() << '\n';
  }
}

}  // namespace ocs2
