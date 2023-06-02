// LAST UPDATE: 2022.05.30
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
  : server_("simple_marker"), robotName_(robotName), targetNames_(targetNames), goalPoseToTargetTrajectories_(std::move(goalPoseToTargetTrajectories)) 
{
  tflistenerPtr_ = new tf::TransformListener;

  graspPosOffset_.x() = 0;
  graspPosOffset_.y() = 0;
  graspPosOffset_.z() = 0.3;

  /// Interactive Marker
  // create an interactive marker for our server
  menuHandler_.insert("Send target pose", boost::bind(&TargetTrajectoriesGazebo::processFeedback, this, _1));

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

  targetMarkerArrayPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topicPrefix + "_target", 10);
}

TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo()
{
  std::cout << "[TargetTrajectoriesGazebo::~TargetTrajectoriesGazebo] Calling Destructor..." << std::endl;
  delete[] tflistenerPtr_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
TargetTrajectoriesGazebo::TargetTrajectoriesGazebo(const TargetTrajectoriesGazebo& ttg) : server_("simple_marker")
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

  menuHandler_ = ttg.menuHandler_;

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

  menuHandler_ = ttg.menuHandler_;

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

    // Update Target
    updateTarget();

    // Set TargetTrajectories
    const auto targetTrajectories = goalPoseToTargetTrajectories_(currentTargetPosition_, currentTargetOrientation_, observation);

    // Publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }

  //std::cout << "[TargetTrajectoriesGazebo::updateObservationAndTarget] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::initializeInteractiveMarker()
{
  while (initFlag_);
  
  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(interactiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  menuHandler_.apply(server_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();
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
    std::cout << "[TargetTrajectoriesGazebo::updateTarget] SWITCH" << std::endl;

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

  fillTargetVisu();
  publishTargetVisu();
  publishGraspFrame();

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[TargetTrajectoriesGazebo::updateTarget] END" << std::endl;
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
void TargetTrajectoriesGazebo::fillTargetVisu()
{
  targetMarkerArray_.markers.clear();

  /// NUA TODO: User should able to add more target!
  int target_size = 1;

  for(int i = 0; i < target_size; i++)
  {
    Eigen::Vector3d currentTargetPosition = currentTargetPosition_;
    geometry_msgs::Point target_point;
    target_point.x = currentTargetPosition.x();
    target_point.y = currentTargetPosition.y();
    target_point.z = currentTargetPosition.z();

    updateGraspPose();

    Eigen::Vector3d currentGraspPosition = currentGraspPosition_;
    geometry_msgs::Point grasp_point;
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
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void TargetTrajectoriesGazebo::publishTargetVisu()
{
  /// NUA TODO: User should able to add more target!
  int target_size = 1;

  if (target_size > 0)
  {
    for(int i = 0; i < target_size; i++)
    {
      targetMarkerArray_.markers[i].header.seq++;
      targetMarkerArray_.markers[i].header.stamp = ros::Time::now();
    }
        
    targetMarkerArrayPublisher_.publish(targetMarkerArray_);
  }
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
visualization_msgs::InteractiveMarker TargetTrajectoriesGazebo::createInteractiveMarker() const 
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
void TargetTrajectoriesGazebo::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
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

  // get TargetTrajectories
  const auto targetTrajectories = goalPoseToTargetTrajectories_(position, orientation, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

}  // namespace ocs2
