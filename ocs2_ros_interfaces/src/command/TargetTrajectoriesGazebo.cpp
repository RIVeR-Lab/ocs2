// LAST UPDATE: 2022.06.17
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
TargetTrajectoriesGazebo::TargetTrajectoriesGazebo(ros::NodeHandle& nodeHandle,
                                                   const std::string& topicPrefix,
                                                   const std::string& gazeboModelMsgName,
                                                   std::string robotName,
                                                   std::vector<std::string>& targetNames,
                                                   GoalPoseToTargetTrajectories goalPoseToTargetTrajectories)
  : targetServer_("target_marker"), 
    modelModeServer_("model_mode_marker", "", false), 
    robotName_(robotName), 
    targetNames_(targetNames), 
    goalPoseToTargetTrajectories_(std::move(goalPoseToTargetTrajectories)) 
{
  tflistenerPtr_ = new tf::TransformListener;

  graspPosOffset_.x() = 0;
  graspPosOffset_.y() = 0;
  graspPosOffset_.z() = 0.3;

  /// Interactive Marker
  // create an interactive marker for our server
  menuHandlerTarget_.insert("Send target pose", boost::bind(&TargetTrajectoriesGazebo::processFeedbackTarget, this, _1));

  /// Subscribers
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) 
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  gazeboModelStatesSubscriber_ = nodeHandle.subscribe(gazeboModelMsgName, 10, &TargetTrajectoriesGazebo::gazeboModelStatesCallback, this);

  /// Publishers
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
  modelModePublisher_ = nodeHandle.advertise<std_msgs::UInt8>(topicPrefix + "_model_mode", 1, false);

  targetMarkerArrayPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topicPrefix + "_goal", 10);
}

TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo()
{
  std::cout << "[TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo] Calling Destructor..." << std::endl;
  delete[] tflistenerPtr_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo::TargetTrajectoriesGazebo(const TargetTrajectoriesGazebo& ttg)
  : targetServer_("target_marker"), 
    modelModeServer_("model_mode_marker", "", false)
{
  initFlag_ = ttg.initFlag_;
  worldFrameName_ = ttg.worldFrameName_;
  robotFrameName_ = ttg.robotFrameName_;

  targetNames_ = ttg.targetNames_;
  robotName_ = ttg.robotName_;
  robotPose_ = ttg.robotPose_;

  graspPosOffset_ = ttg.graspPosOffset_;

  currentTargetName_ = ttg.currentTargetName_;
  currentTargetPosition_ = ttg.currentTargetPosition_;
  currentTargetOrientation_ = ttg.currentTargetOrientation_;
    
  currentTargetNames_ = ttg.currentTargetNames_;
  currentTargetPositions_ = ttg.currentTargetPositions_;
  currentTargetOrientations_ = ttg.currentTargetOrientations_;

  targetMarkerArray_ = ttg.targetMarkerArray_;
  targetMarkerArrayPublisher_ = ttg.targetMarkerArrayPublisher_;

  tflistenerPtr_ = ttg.tflistenerPtr_;

  h_first_entry_ = ttg.h_first_entry_;
  h_mode_last_ = ttg.h_mode_last_;

  menuHandlerTarget_ = ttg.menuHandlerTarget_;
  menuHandlerModelMode_ = ttg.menuHandlerModelMode_;

  latestObservation_ = ttg.latestObservation_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo& TargetTrajectoriesGazebo::operator=(const TargetTrajectoriesGazebo& ttg)
{
  initFlag_ = ttg.initFlag_;
  worldFrameName_ = ttg.worldFrameName_;
  robotFrameName_ = ttg.robotFrameName_;

  targetNames_ = ttg.targetNames_;
  robotName_ = ttg.robotName_;
  robotPose_ = ttg.robotPose_;

  graspPosOffset_ = ttg.graspPosOffset_;

  currentTargetName_ = ttg.currentTargetName_;
  currentTargetPosition_ = ttg.currentTargetPosition_;
  currentTargetOrientation_ = ttg.currentTargetOrientation_;
    
  currentTargetNames_ = ttg.currentTargetNames_;
  currentTargetPositions_ = ttg.currentTargetPositions_;
  currentTargetOrientations_ = ttg.currentTargetOrientations_;

  targetMarkerArray_ = ttg.targetMarkerArray_;
  targetMarkerArrayPublisher_ = ttg.targetMarkerArrayPublisher_;

  tflistenerPtr_ = ttg.tflistenerPtr_;

  h_first_entry_ = ttg.h_first_entry_;
  h_mode_last_ = ttg.h_mode_last_;

  menuHandlerTarget_ = ttg.menuHandlerTarget_;
  menuHandlerModelMode_ = ttg.menuHandlerModelMode_;

  latestObservation_ = ttg.latestObservation_;

  return *this;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateObservationAndTarget() 
{
  //std::cout << "[TargetTrajectoriesGazebo::updateObservationAndTarget] START" << std::endl;

  if (initFlag_)
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

    // Update and publish target visualization
    updateTarget();
  }

  //std::cout << "[TargetTrajectoriesGazebo::updateObservationAndTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget()
{
  //std::cout << "[TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget] START" << std::endl;

  while (initFlag_);
  
  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarkerTarget();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  targetServer_.insert(interactiveMarker);
  menuHandlerTarget_.apply(targetServer_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  targetServer_.applyChanges();

  //std::cout << "[TargetTrajectoriesGazebo::initializeInteractiveMarkerTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode()
{
  //std::cout << "[TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode] START" << std::endl;

  while (initFlag_);
  
  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarkerModelMode();

  createMenuModelMode();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  modelModeServer_.insert(interactiveMarker);
  menuHandlerModelMode_.apply(modelModeServer_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  modelModeServer_.applyChanges();

  //std::cout << "[TargetTrajectoriesGazebo::initializeInteractiveMarkerModelMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::transformPose(std::string& frame_from,
                                             std::string& frame_to,
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
    tflistenerPtr_-> waitForTransform(frame_to, frame_from, ros::Time::now(), ros::Duration(1.0));
    tflistenerPtr_->transformPose(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("[TargetTrajectoriesGazebo::transformPose] Couldn't get transform!");
    ROS_ERROR("%s",ex.what());

    std::cout << "[TargetTrajectoriesGazebo::transformPose] DEBUG INF" << std::endl;
    while(1);
    //ros::Duration(1.0).sleep();
  }

  tf::poseStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_to = p_to_stamped_msg.pose;
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
  double roll = M_PI;

  // Rotate the previous pose about Y
  double pitch = 0;
  
  // Rotate the previous pose about Z
  double yaw = 0;  

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
int TargetTrajectoriesGazebo::isIn(std::vector<std::string>& vec, std::string& s)
{
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i] == s)
    {
      return i;
    }
  }
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
  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] START" << std::endl;

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

  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] currentTargetName 0: " << currentTargetNames_[0] << std::endl;
  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] temp_dist: " << temp_dist << std::endl;
  for(int i = 1; i < pos_vec.size(); i++)
  {
    temp_dist = calculateEuclideanDistance(pos_vec[i], query_p);

    //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] currentTargetName " << i << ": " << currentTargetNames_[i] << std::endl;
    //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] temp_dist: " << temp_dist << std::endl;

    if(result.first > temp_dist)
    {
      result.first = temp_dist;
      result.second = i;
    }
  }

  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] result.first: " << result.first << std::endl;
  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] result.second: " << result.second << std::endl;

  //std::cout << "[TargetTrajectoriesGazebo::findClosestDistance] END" << std::endl;

  return result;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::gazeboModelStatesCallback(const gazebo_msgs::ModelStatesPtr& msg) 
{
  //std::cout << "[TargetTrajectoriesGazebo::gazeboModelStatesCallback] Incoming..." << std::endl;
  
  gazebo_msgs::ModelStates ms = *msg;

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

    if (isIn(targetNames_, ms.name[i]) > 0)
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
  
  initFlag_ = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateTarget()
{
  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] START" << std::endl;

  Eigen::Vector3d targetPos;
  Eigen::Quaterniond targetOri;
  geometry_msgs::Pose robotPose = robotPose_;
  std::vector<std::string> currentTargetNames = currentTargetNames_;
  std::vector<Eigen::Vector3d> targetPositions = currentTargetPositions_;
  std::vector<Eigen::Quaterniond> targetOrientations = currentTargetOrientations_;

  int idx = isIn(currentTargetNames, currentTargetName_);

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;
  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] idx: " << idx << std::endl;

  if (idx >= 0)
  {
    //std::cout << "[TargetTrajectoriesGazebo::updateTarget] REGULAR" << std::endl;

    targetPos = targetPositions[idx];
    targetOri = targetOrientations[idx];

    currentTargetPosition_ = targetPos;
    currentTargetOrientation_ = targetOri;
  }
  else
  {
    //std::cout << "[TargetTrajectoriesGazebo::updateTarget] SWITCH" << std::endl;

    /*
    std::cout << "[TargetTrajectoriesGazebo::updateTarget] idx: " << idx << std::endl;
    std::cout << "[TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;
    std::cout << "[TargetTrajectoriesGazebo::updateTarget] currentTargetNames: " << std::endl;
    for (size_t i = 0; i < currentTargetNames.size(); i++)
    {
      std::cout << currentTargetNames[i] << std::endl;
    }
    */

    std::pair<double, int> closest_p;
    closest_p = findClosestDistance(targetPositions, robotPose);

    currentTargetName_ = currentTargetNames_[closest_p.second];
    targetPos = targetPositions[closest_p.second];
    targetOri = targetOrientations[closest_p.second];

    //std::cout << "[TargetTrajectoriesGazebo::updateTarget] closest_p.second: " << closest_p.second << std::endl;
    //std::cout << "[TargetTrajectoriesGazebo::updateTarget] currentTargetName_: " << currentTargetName_ << std::endl;

    currentTargetPosition_ = targetPos;
    currentTargetOrientation_ = targetOri;

    /*
    if (ctr_ > 0)
    {
      std::cout << "[TargetTrajectoriesGazebo::updateTarget] DEBUG INF ctr_: " << ctr_ << std::endl;
      while(1);
    }

    ctr_++;
    */
  }

  fillTargetVisu(true);
  publishTargetVisu();
  publishGraspFrame();

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateTarget(const Eigen::Vector3d& targetPos, const Eigen::Quaterniond& targetOri)
{
  //std::cout << "[TargetTrajectoriesGazebo::updateTarget(2)] START" << std::endl;

  currentTargetPosition_ = targetPos;
  currentTargetOrientation_ = targetOri;
  
  fillTargetVisu(false);
  publishTargetVisu();

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget(2)] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget(2)] END" << std::endl;
}

void TargetTrajectoriesGazebo::updateGraspPose()
{
  //std::cout << "[TargetTrajectoriesGazebo::getGraspPose] START" << std::endl;

  Eigen::Vector3d graspPos;
  Eigen::Quaterniond graspOri;

  geometry_msgs::Pose graspPos_wrt_world;
  geometry_msgs::Pose graspPos_wrt_target;
  graspPos_wrt_target.position.x = graspPosOffset_.x();
  graspPos_wrt_target.position.y = graspPosOffset_.y();
  graspPos_wrt_target.position.z = graspPosOffset_.z();
  graspPos_wrt_target.orientation.x = 0;
  graspPos_wrt_target.orientation.y = 0;
  graspPos_wrt_target.orientation.z = 0;
  graspPos_wrt_target.orientation.w = 1;

  //std::cout << "[TargetTrajectoriesGazebo::getGraspPose] currentTargetName_: " << currentTargetName_ << std::endl;

  transformPose(currentTargetName_, worldFrameName_, graspPos_wrt_target, graspPos_wrt_world);

  graspPos.x() = graspPos_wrt_world.position.x;
  graspPos.y() = graspPos_wrt_world.position.y;
  graspPos.z() = graspPos_wrt_world.position.z;

  Eigen::Quaterniond currentTargetOrientation = currentTargetOrientation_;
  Eigen::Vector3d rpy_rot(M_PI, 0, 0);

  rotateQuaternion(currentTargetOrientation, rpy_rot, graspOri);

  currentGraspPosition_ = graspPos;
  currentGraspOrientation_ = graspOri;

  //std::cout << "[TargetTrajectoriesGazebo::getGraspPose] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::updateGraspPose(Eigen::Vector3d& graspPos, Eigen::Quaterniond& graspOri)
{
  //std::cout << "[TargetTrajectoriesGazebo::getGraspPose(2)] START" << std::endl;
  currentGraspPosition_ = graspPos;
  currentGraspOrientation_ = graspOri;
  //std::cout << "[TargetTrajectoriesGazebo::getGraspPose(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::fillTargetVisu(bool graspFlag)
{
  //std::cout << "[TargetTrajectoriesGazebo::fillTargetVisu] START" << std::endl;

  targetMarkerArray_.markers.clear();

  /// NUA TODO: User should able to add more target!
  int target_size = 1;

  for(int i = 0; i < target_size; i++)
  {
    Eigen::Vector3d currentTargetPosition = currentTargetPosition_;
    Eigen::Quaterniond currentTargetOrientation = currentTargetOrientation_;
    geometry_msgs::Point target_point;
    geometry_msgs::Point grasp_point;
    target_point.x = currentTargetPosition.x();
    target_point.y = currentTargetPosition.y();
    target_point.z = currentTargetPosition.z();
    grasp_point = target_point;

    if (graspFlag)
    {
      updateGraspPose();
    }
    else
    {
      updateGraspPose(currentTargetPosition, currentTargetOrientation);
    }

    Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
    grasp_point.x = currentGraspPosition.x();
    grasp_point.y = currentGraspPosition.y();
    grasp_point.z = currentGraspPosition.z();

    //std::cout << "[TargetTrajectoriesGazebo::getGraspPose] grasp_point: " << std::endl;
    //print(grasp_point);

    //std::cout << "[TargetTrajectoriesGazebo::getGraspPose] target_point: " << std::endl;
    //print(target_point);

    visualization_msgs::Marker target_visu;
    target_visu.ns = "grasp_target_" + std::to_string(i+1);
    target_visu.id = i+1;
    target_visu.action = visualization_msgs::Marker::ADD;
    target_visu.type = visualization_msgs::Marker::ARROW;
    target_visu.points.push_back(grasp_point);
    target_visu.points.push_back(target_point);
    target_visu.pose.orientation.x = 0;
    target_visu.pose.orientation.y = 0;
    target_visu.pose.orientation.z = 0;
    target_visu.pose.orientation.w = 1;
    target_visu.scale.x = 0.04;
    target_visu.scale.y = 0.08;
    target_visu.scale.z = 0.08;
    target_visu.color.r = 0.0;
    target_visu.color.g = 0.0;
    target_visu.color.b = 0.0;
    target_visu.color.a = 0.5;
    target_visu.header.frame_id = worldFrameName_;

    targetMarkerArray_.markers.push_back(target_visu);
  }

  //std::cout << "[TargetTrajectoriesGazebo::fillTargetVisu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishTargetVisu()
{
  //std::cout << "[TargetTrajectoriesGazebo::publishTargetVisu] START" << std::endl;

  /// NUA TODO: User should able to add more target!
  visualization_msgs::MarkerArray targetMarkerArray = targetMarkerArray_;
  int target_size = targetMarkerArray.markers.size();

  if (target_size > 0)
  {
    for(int i = 0; i < target_size; i++)
    {
      targetMarkerArray.markers[i].header.seq++;
      targetMarkerArray.markers[i].header.stamp = ros::Time::now();
    }
    targetMarkerArrayPublisher_.publish(targetMarkerArray);

    publishGraspFrame();
  }

  //std::cout << "[TargetTrajectoriesGazebo::publishTargetVisu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishGraspFrame()
{
  //std::cout << "[TargetTrajectoriesGazebo::publishVirtualFrames] START" << std::endl;

  Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
  Eigen::Quaterniond currentGraspOrientation = currentGraspOrientation_;

  static tf::TransformBroadcaster tf_br;
  tf::Transform tf_virtual;
  tf_virtual.setOrigin(tf::Vector3(currentGraspPosition.x(), currentGraspPosition.y(), currentGraspPosition.z()));
  tf_virtual.setRotation(tf::Quaternion(currentGraspOrientation.x(), currentGraspOrientation.y(), currentGraspOrientation.z(), currentGraspOrientation.w()));

  tf_br.sendTransform(tf::StampedTransform(tf_virtual, ros::Time::now(), worldFrameName_, "grasp"));

  //std::cout << "[TargetTrajectoriesGazebo::publishVirtualFrames] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerTarget() const 
{
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
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
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarkerModelMode() const
{
  //std::cout << "[TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] START" << std::endl;

  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "ModelMode";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = 1.0;
  interactiveMarker.pose.position.y = 0.0;
  interactiveMarker.pose.position.z = 1.0;

  // create a grey box marker
  const auto boxMarker = []() 
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
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

  //std::cout << "[TargetTrajectoriesGazebo::createInteractiveMarkerModelMode] END" << std::endl;

  return interactiveMarker;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::createMenuModelMode()
{
  //std::cout << "[TargetTrajectoriesGazebo::createMenuModelMode] START" << std::endl;

  h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::BaseMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

  h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::ArmMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

  h_mode_last_ = menuHandlerModelMode_.insert("ModelMode::WholeBodyMotion", boost::bind(&TargetTrajectoriesGazebo::processFeedbackModelMode, this, _1));
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );

  //check the very last entry
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::CHECKED );

  //std::cout << "[TargetTrajectoriesGazebo::createMenuModelMode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::processFeedbackTarget(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  //std::cout << "[TargetTrajectoriesGazebo::processFeedbackTarget] START" << std::endl;

  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, 
                                 feedback->pose.position.y, 
                                 feedback->pose.position.z);
  const Eigen::Quaterniond orientation(feedback->pose.orientation.w, 
                                       feedback->pose.orientation.x, 
                                       feedback->pose.orientation.y,
                                       feedback->pose.orientation.z);

  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  updateTarget(position, orientation);

  // get TargetTrajectories
  const auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

  //std::cout << "[TargetTrajectoriesGazebo::processFeedbackTarget] END" << std::endl;
}

void TargetTrajectoriesGazebo::processFeedbackModelMode(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //std::cout << "[TargetTrajectoriesGazebo::processFeedbackModelMode] START" << std::endl;
  std_msgs::UInt8 model_mode_int;
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last_ = feedback->menu_entry_id;
  menuHandlerModelMode_.setCheckState( h_mode_last_, interactive_markers::MenuHandler::CHECKED );

  switch (feedback->menu_entry_id) 
  {
    case 1:
    {
      std::cout << "[TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::BaseMotion" << std::endl;
      model_mode_int.data = 0;
      break;
    }
    
    case 2:
    {
      std::cout << "[TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::ArmMotion" << std::endl;
      model_mode_int.data = 1;
      break;
    }
    
    case 3: 
    {
      std::cout << "[TargetTrajectoriesGazebo::processFeedbackModelMode] Switching to ModelMode::WholeBodyMotion" << std::endl;
      model_mode_int.data = 2;
      break;
    }

    default:
      throw std::invalid_argument("[TargetTrajectoriesGazebo::processFeedbackModelMode] ERROR: Invalid menu entry id");
  }

  menuHandlerModelMode_.reApply(modelModeServer_);
  modelModeServer_.applyChanges();

  modelModePublisher_.publish(model_mode_int);

  //std::cout << "[TargetTrajectoriesGazebo::processFeedbackModelMode] END" << std::endl;
}

}  // namespace ocs2
