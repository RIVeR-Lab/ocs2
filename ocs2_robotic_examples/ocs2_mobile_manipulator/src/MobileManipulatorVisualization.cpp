// LAST UPDATE: 2023.08.18
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
//#include <urdf/model.h>
//#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

//#include <ocs2_core/misc/LoadData.h>
//#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/dynamics/MultiModelFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorVisualization.h>
//#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
//#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) 
{
  for (; firstIt != lastIt; ++firstIt) 
  {
    firstIt->header = header;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) 
{
  for (; firstIt != lastIt; ++firstIt) 
  {
    firstIt->id = startId++;
  }
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
MobileManipulatorVisualization::MobileManipulatorVisualization(ros::NodeHandle& nodeHandle, 
                                                               const PinocchioInterface& pinocchioInterface,
                                                               const std::string& worldFrameName,
                                                               const std::string& baseFrameName,
                                                               const std::string& urdfFile,
                                                               const std::string& jointStateMsgName,
                                                               const RobotModelInfo& robotModelInfo,
                                                               const bool& selfCollisionFlag,
                                                               const bool& extCollisionFlag,
                                                               const std::vector<std::string>& removeJointNames,
                                                               std::vector<std::pair<size_t, size_t>>& collisionObjectPairs,
                                                               std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                                               std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                                               std::shared_ptr<ExtMapUtility> emuPtr,
                                                               double maxDistance)
  : pinocchioInterface_(pinocchioInterface),
    worldFrameName_(worldFrameName),
    baseFrameName_(baseFrameName),
    urdfFile_(urdfFile),
    jointStateMsgName_(jointStateMsgName),
    robotModelInfo_(robotModelInfo),
    selfCollisionFlag_(selfCollisionFlag),
    extCollisionFlag_(extCollisionFlag),
    removeJointNames_(removeJointNames),
    collisionObjectPairs_(collisionObjectPairs),
    collisionLinkPairs_(collisionLinkPairs),
    pointsOnRobotPtr_(pointsOnRobotPtr),
    emuPtr_(emuPtr),
    maxDistance_(maxDistance),
    pinocchioInterfaceInternal_(pinocchioInterface),
    distances_(pointsOnRobotPtr->getNumOfPoints()),
    markerPublisher_(nodeHandle.advertise<visualization_msgs::MarkerArray>("distance_markers", 10, true))
{
  std::cout << "[MobileManipulatorVisualization::MobileManipulatorVisualization] START" << std::endl;
  launchVisualizerNode(nodeHandle);
  std::cout << "[MobileManipulatorVisualization::MobileManipulatorVisualization] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) 
{
  std::cout << "[MobileManipulatorVisualization::launchVisualizerNode] START" << std::endl;

  // Subscribers
  jointStatesSub_ = nodeHandle.subscribe(jointStateMsgName_, 5, &MobileManipulatorVisualization::jointStateCallback, this);
  tfSub_ = nodeHandle.subscribe("/tf", 5, &MobileManipulatorVisualization::tfCallback, this);

  // Publishers
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);

  // Create pinocchio interface
  pinocchioInterfaceInternal_ = mobile_manipulator::createPinocchioInterface(urdfFile_, robotModelInfo_.robotModelType, removeJointNames_);

  // activate markers for self-collision visualization
  if (selfCollisionFlag_) 
  {
    PinocchioGeometryInterface geometryInterface(pinocchioInterfaceInternal_, collisionLinkPairs_, collisionObjectPairs_);
    
    const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();

    // Set geometry visualization markers
    visualizationInterfacePtr_.reset(new GeometryInterfaceVisualization(std::move(pinocchioInterfaceInternal_), geometryInterface, nodeHandle));
  }

  std::cout << "[MobileManipulatorVisualization::launchVisualizerNode] Waiting for initTFTransformFlag_ and initJointStateFlag_..." << std::endl;
  while(!initTFTransformFlag_ || !initJointStateFlag_){ros::spinOnce();}

  std::cout << "[MobileManipulatorVisualization::launchVisualizerNode] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //std::cerr << "[MobileManipulatorVisualization::tfCallback] START " << std::endl;

  try
  {
    tfListener_.waitForTransform(worldFrameName_, baseFrameName_, ros::Time::now(), ros::Duration(1.0));
    tfListener_.lookupTransform(worldFrameName_, baseFrameName_, ros::Time(0), tf_robot_wrt_world_);

    /*
    std::cout << "[MobileManipulatorVisualization::tfCallback] worldFrameName_: " << worldFrameName_ << std::endl;
    std::cout << "[MobileManipulatorVisualization::tfCallback] baseFrameName_: " << baseFrameName_ << std::endl;

    std::cout << "[MobileManipulatorVisualization::tfCallback] pos x: " << tf_robot_wrt_world_.getOrigin().x() << std::endl;
    std::cout << "[MobileManipulatorVisualization::tfCallback] pos y: " << tf_robot_wrt_world_.getOrigin().y() << std::endl;
    std::cout << "[MobileManipulatorVisualization::tfCallback] pos z: " << tf_robot_wrt_world_.getOrigin().z() << std::endl;
    */
   
    initTFTransformFlag_ = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MobileManipulatorVisualization::tfCallback] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  //std::cerr << "[MobileManipulatorVisualization::tfCallback] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointStateMsg_ = *msg;
  initJointStateFlag_ = true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateModelMode(size_t modelModeInt)
{
  ocs2::updateModelMode(robotModelInfo_, modelModeInt);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::update(const SystemObservation& observation, 
                                            const PrimalSolution& policy,
                                            const CommandData& command) 
{
  //std::cout << "[MobileManipulatorVisualization::update] START" << std::endl;

  const ros::Time timeStamp = ros::Time::now();
  publishOptimizedTrajectory(timeStamp, policy, observation.full_state);
  
  /*
  if (visualizationInterfacePtr_ != nullptr) 
  {
    publishSelfCollisionDistances(observation.state, observation.full_state, robotModelInfo_);
  }
  */

  //std::cout << "[MobileManipulatorVisualization::update] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::publishSelfCollisionDistances() 
{
  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances] START" << std::endl;

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  RobotModelInfo robotModelInfo = robotModelInfo_;

  timer6_.startTimer();
  
  timer7_.startTimer();
  ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo);
  const vector_t q = pinocchioMapping.getPinocchioJointPosition(state_, fullState_);
  pinocchio::forwardKinematics(model, data, q);
  const auto results = visualizationInterfacePtr_->getGeometryInterface().computeDistances(pinocchioInterface_);
  timer7_.endTimer();
  
  timer8_.startTimer();
  visualization_msgs::MarkerArray markerArray;
  constexpr size_t numMarkersPerResult = 5;

  visualization_msgs::Marker markerTemplate;
  markerTemplate.color = ros_msg_helpers::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = worldFrameName_;
  markerTemplate.header.stamp = ros::Time::now();
  markerTemplate.pose.orientation = ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult, markerTemplate);

  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances] results.size(): " << results.size() << std::endl;

  for (size_t i = 0; i < results.size(); ++i) 
  {
    // I apologize for the magic numbers, it's mostly just visualization numbers(so 0.02 scale corresponds rougly to 0.02 cm)

    for (size_t j = 0; j < numMarkersPerResult; ++j) 
    {
      markerArray.markers[numMarkersPerResult * i + j].ns = std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first) +
                                                            " - " +
                                                            std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second);
    }

    // The actual distance line, also denoting direction of the distance
    markerArray.markers[numMarkersPerResult * i].type = visualization_msgs::Marker::ARROW;
    markerArray.markers[numMarkersPerResult * i].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i].id = numMarkersPerResult * i;
    markerArray.markers[numMarkersPerResult * i].scale.x = 0.01;
    markerArray.markers[numMarkersPerResult * i].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i].scale.z = 0.04;

    // Dots at the end of the arrow, denoting the close locations on the body
    markerArray.markers[numMarkersPerResult * i + 1].type = visualization_msgs::Marker::SPHERE_LIST;
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i + 1].scale.x = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].id = numMarkersPerResult * i + 1;

    // Text denoting the object number in the geometry model, raised above the spheres
    markerArray.markers[numMarkersPerResult * i + 2].id = numMarkersPerResult * i + 2;
    markerArray.markers[numMarkersPerResult * i + 2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 2].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 2].pose.position = ros_msg_helpers::getPointMsg(results[i].nearest_points[0]);
    markerArray.markers[numMarkersPerResult * i + 2].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 2].text = "obj " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first);
    markerArray.markers[numMarkersPerResult * i + 3].id = numMarkersPerResult * i + 3;
    markerArray.markers[numMarkersPerResult * i + 3].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 3].pose.position = ros_msg_helpers::getPointMsg(results[i].nearest_points[1]);
    markerArray.markers[numMarkersPerResult * i + 3].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 3].text = "obj " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second);
    markerArray.markers[numMarkersPerResult * i + 3].scale.z = 0.02;

    // Text above the arrow, denoting the distance
    markerArray.markers[numMarkersPerResult * i + 4].id = numMarkersPerResult * i + 4;
    markerArray.markers[numMarkersPerResult * i + 4].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 4].pose.position = ros_msg_helpers::getPointMsg((results[i].nearest_points[0] + results[i].nearest_points[1]) / 2.0);
    markerArray.markers[numMarkersPerResult * i + 4].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 4].text = "dist " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first) + " - " +
                                                                                     std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second) + 
                                                                                     ": " + std::to_string(results[i].min_distance);
    markerArray.markers[numMarkersPerResult * i + 4].scale.z = 0.02;
  }
  timer8_.endTimer();

  timer9_.startTimer();
  markerPublisher_.publish(markerArray);
  timer9_.endTimer();

  timer6_.endTimer();

  /*
  std::cout << "### MPC_ROS Benchmarking timer6_ TOTAL:" << std::endl;
  std::cout << "###   Maximum : " << timer6_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer6_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer6_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer7_ computeDistances:" << std::endl;
  std::cout << "###   Maximum : " << timer7_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer7_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer7_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer8_ fill marker:" << std::endl;
  std::cout << "###   Maximum : " << timer8_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer8_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer8_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer9_ markerPublisher_:" << std::endl;
  std::cout << "###   Maximum : " << timer9_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer9_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer9_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;
  */

  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::publishSelfCollisionDistances(const ocs2::vector_t& state, const ocs2::vector_t& fullState, const RobotModelInfo& modelInfo) 
{
  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances(3)] START" << std::endl;

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo);
  const vector_t q = pinocchioMapping.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
  const auto results = visualizationInterfacePtr_->getGeometryInterface().computeDistances(pinocchioInterface_);

  visualization_msgs::MarkerArray markerArray;

  constexpr size_t numMarkersPerResult = 5;

  visualization_msgs::Marker markerTemplate;
  markerTemplate.color = ros_msg_helpers::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = worldFrameName_;
  markerTemplate.header.stamp = ros::Time::now();
  markerTemplate.pose.orientation = ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult, markerTemplate);

  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances(3)] results.size(): " << results.size() << std::endl;

  for (size_t i = 0; i < results.size(); ++i) 
  {
    // I apologize for the magic numbers, it's mostly just visualization numbers(so 0.02 scale corresponds rougly to 0.02 cm)

    for (size_t j = 0; j < numMarkersPerResult; ++j) 
    {
      markerArray.markers[numMarkersPerResult * i + j].ns = std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first) +
                                                            " - " +
                                                            std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second);
    }

    // The actual distance line, also denoting direction of the distance
    markerArray.markers[numMarkersPerResult * i].type = visualization_msgs::Marker::ARROW;
    markerArray.markers[numMarkersPerResult * i].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i].id = numMarkersPerResult * i;
    markerArray.markers[numMarkersPerResult * i].scale.x = 0.01;
    markerArray.markers[numMarkersPerResult * i].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i].scale.z = 0.04;

    // Dots at the end of the arrow, denoting the close locations on the body
    markerArray.markers[numMarkersPerResult * i + 1].type = visualization_msgs::Marker::SPHERE_LIST;
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i + 1].scale.x = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].id = numMarkersPerResult * i + 1;

    // Text denoting the object number in the geometry model, raised above the spheres
    markerArray.markers[numMarkersPerResult * i + 2].id = numMarkersPerResult * i + 2;
    markerArray.markers[numMarkersPerResult * i + 2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 2].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 2].pose.position = ros_msg_helpers::getPointMsg(results[i].nearest_points[0]);
    markerArray.markers[numMarkersPerResult * i + 2].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 2].text = "obj " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first);
    markerArray.markers[numMarkersPerResult * i + 3].id = numMarkersPerResult * i + 3;
    markerArray.markers[numMarkersPerResult * i + 3].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 3].pose.position = ros_msg_helpers::getPointMsg(results[i].nearest_points[1]);
    markerArray.markers[numMarkersPerResult * i + 3].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 3].text = "obj " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second);
    markerArray.markers[numMarkersPerResult * i + 3].scale.z = 0.02;

    // Text above the arrow, denoting the distance
    markerArray.markers[numMarkersPerResult * i + 4].id = numMarkersPerResult * i + 4;
    markerArray.markers[numMarkersPerResult * i + 4].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 4].pose.position = ros_msg_helpers::getPointMsg((results[i].nearest_points[0] + results[i].nearest_points[1]) / 2.0);
    markerArray.markers[numMarkersPerResult * i + 4].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 4].text = "dist " + std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].first) + " - " +
                                                                                     std::to_string(visualizationInterfacePtr_->getGeometryInterface().getGeometryModel().collisionPairs[i].second) + 
                                                                                     ": " + std::to_string(results[i].min_distance);
    markerArray.markers[numMarkersPerResult * i + 4].scale.z = 0.02;
  }

  markerPublisher_.publish(markerArray);

  //std::cout << "[MobileManipulatorVisualization::publishSelfCollisionDistances(3)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::getPoint(std::string& baseFrameName, std::string& pointFrameName, geometry_msgs::Point p)
{
  //std::cerr << "[MobileManipulatorVisualization::getPoint] START " << std::endl;

  tf::StampedTransform tf_point_wrt_base;
  try
  {
    tfListener_.waitForTransform(baseFrameName, pointFrameName, ros::Time::now(), ros::Duration(1.0));
    tfListener_.lookupTransform(baseFrameName, pointFrameName, ros::Time(0), tf_point_wrt_base);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[MobileManipulatorVisualization::getPoint] ERROR: Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  //std::cerr << "[MobileManipulatorVisualization::getPoint] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateStateIndexMap()
{
  //std::cout << "[MobileManipulatorVisualization::updateStateIndexMap] START" << std::endl;
  
  auto jointNames = robotModelInfo_.robotArm.jointNames;
  int n_joints = jointNames.size();
  stateIndexMap_.clear();

  boost::shared_ptr<sensor_msgs::JointState const> jointStatePtrMsg = ros::topic::waitForMessage<sensor_msgs::JointState>(jointStateMsgName_);

  /*
  std::cout << "[MobileManipulatorVisualization::updateStateIndexMap] jointStatePtrMsg->name.size(): " << jointStatePtrMsg->name.size() << std::endl; 
  for (size_t i = 0; i < jointStatePtrMsg->name.size(); i++)
  {
    std::cout << jointStatePtrMsg->name[i] << std::endl;
  }
  */

  for (size_t i = 0; i < n_joints; i++)
  {
    auto it = find(jointStatePtrMsg->name.begin(), jointStatePtrMsg->name.end(), jointNames[i]);

    // If element was found
    if (it != jointStatePtrMsg->name.end()) 
    {
        // calculating the index
        int index = it - jointStatePtrMsg->name.begin();
        stateIndexMap_.push_back(index);
    }
    else
    {
      throw std::runtime_error("[MobileManipulatorVisualization::updateStateIndexMap] Error: Joint " + jointNames[i] + " not found!");
    }
  }

  std::cout << "[MobileManipulatorVisualization::updateStateIndexMap] stateIndexMap_ size: " << stateIndexMap_.size() << std::endl;
  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    std::cout << i << " -> " << stateIndexMap_[i] << std::endl;
  }

  //std::cout << "[MobileManipulatorVisualization::updateStateIndexMap] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorVisualization::updateStateIndexMap] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateModelState()
{
  //std::cout << "[MobileManipulatorVisualization::updateModelState] START " << std::endl;

  RobotModelInfo robotModelInfo = robotModelInfo_;

  auto stateDimBase = getStateDimBase(robotModelInfo);
  auto stateDim = getStateDim(robotModelInfo);
  auto modeStateDim = getModeStateDim(robotModelInfo);

  fullState_.resize(stateDim);
  state_.resize(modeStateDim);

  //std::cout << "[MobileManipulatorVisualization::updateModelState] stateDimBase: " << stateDimBase << std::endl;
  //std::cout << "[MobileManipulatorVisualization::updateModelState] stateDim: " << stateDim << std::endl;
  //std::cout << "[MobileManipulatorVisualization::updateModelState] modeStateDim: " << modeStateDim << std::endl;

  switch (robotModelInfo.robotModelType)
  {
    case RobotModelType::MobileBase:
    {
      //std::cout << "[MobileManipulatorVisualization::updateModelState] RobotModelType::MobileBase" << std::endl;
      // Set state
      state_[0] = statePositionBase_[0];
      state_[1] = statePositionBase_[1];
      state_[2] = statePositionBase_[2];

      // Set full state
      fullState_ = state_;
      break;
    }

    case RobotModelType::RobotArm:
    { 
      //std::cout << "[MobileManipulatorVisualization::updateModelState] RobotModelType::RobotArm" << std::endl;
      // Set state and input
      for (size_t i = 0; i < statePositionArm_.size(); i++)
      {
        state_[i] = statePositionArm_[i];
      }

      // Set full state
      fullState_ = state_;
      break;
    }

    case RobotModelType::MobileManipulator:
    {
      //std::cout << "[MobileManipulatorVisualization::updateModelState] RobotModelType::MobileManipulator" << std::endl;
      switch (robotModelInfo.modelMode)
      {
        case ModelMode::BaseMotion:
        {
          //std::cout << "[MobileManipulatorVisualization::updateModelState] ModelMode::BaseMotion" << std::endl;
          // Set state
          state_[0] = statePositionBase_[0];
          state_[1] = statePositionBase_[1];
          state_[2] = statePositionBase_[2];

          // Set full state
          fullState_[0] = statePositionBase_[0];
          fullState_[1] = statePositionBase_[1];
          fullState_[2] = statePositionBase_[2];

          for (size_t i = 0; i < statePositionArm_.size(); i++)
          {
            fullState_[stateDimBase + i] = statePositionArm_[i];
          }
          break;
        }

        case ModelMode::ArmMotion:
        {
          //std::cout << "[MobileManipulatorVisualization::updateModelState] ModelMode::ArmMotion" << std::endl;
          // Set full state base
          fullState_[0] = statePositionBase_[0];
          fullState_[1] = statePositionBase_[1];
          fullState_[2] = statePositionBase_[2];

          for (size_t i = 0; i < statePositionArm_.size(); i++)
          {
            // Set state
            state_[i] = statePositionArm_[i];

            // Set full state arm
            fullState_[stateDimBase + i] = statePositionArm_[i];
          }
          break;
        }

        case ModelMode::WholeBodyMotion:
        {
          //std::cout << "[MobileManipulatorVisualization::updateModelState] ModelMode::WholeBodyMotion" << std::endl;
          // Set state
          state_[0] = statePositionBase_[0];
          state_[1] = statePositionBase_[1];
          state_[2] = statePositionBase_[2];

          for (size_t i = 0; i < statePositionArm_.size(); i++)
          {
            state_[stateDimBase + i] = statePositionArm_[i];
          }

          // Set full state
          fullState_ = state_;
          break;
        }

        default:
          std::cout << "[MobileManipulatorVisualization::updateModelState] ERROR: Invalid model mode!";
          break;
      }
      break;
    }

    default:
    {
      std::cout << "[MobileManipulatorVisualization::updateModelState] ERROR: Invalid robot model type!";
      break;
    }
  }

  //std::cout << "[MobileManipulatorVisualization::updateModelState] END " << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateState()
{
  //std::cout << "[MobileManipulatorVisualization::updateState] START " << std::endl;

  statePositionBase_.clear();
  statePositionArm_.clear();

  tf::StampedTransform tf_robot_wrt_world = tf_robot_wrt_world_;
  sensor_msgs::JointState jointStateMsg = jointStateMsg_;

  // Set mobile base state
  tf::Matrix3x3 matrix_robot_wrt_world;
  statePositionBase_.push_back(tf_robot_wrt_world.getOrigin().x());
  statePositionBase_.push_back(tf_robot_wrt_world.getOrigin().y());
  matrix_robot_wrt_world = tf::Matrix3x3(tf_robot_wrt_world.getRotation());
  double roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world;
  matrix_robot_wrt_world.getRPY(roll_robot_wrt_world, pitch_robot_wrt_world, yaw_robot_wrt_world);
  statePositionBase_.push_back(yaw_robot_wrt_world);

  for (int i = 0; i < stateIndexMap_.size(); ++i)
  {
    statePositionArm_.push_back(jointStateMsg.position[stateIndexMap_[i]]);
  }

  /*
  std::cout << "[MobileManipulatorVisualization::updateState] statePositionBase_ size: " << statePositionBase_.size() << std::endl;
  for (size_t i = 0; i < statePositionBase_.size(); i++)
  {
    std::cout << i << " -> " << statePositionBase_[i] << std::endl;
  }
  
  std::cout << "[MobileManipulatorVisualization::updateState] statePositionArm_ size: " << statePositionArm_.size() << std::endl;
  for (size_t i = 0; i < statePositionArm_.size(); i++)
  {
    std::cout << i << " -> " << statePositionArm_[i] << std::endl;
  }
  */

  updateModelState();

  //std::cout << "[MobileManipulatorVisualization::updateState] END " << std::endl << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateExtCollisionDistances(bool normalize_flag)
{
  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] START" << std::endl;

  timer0_.startTimer();

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] START updateState" << std::endl;
  //timer1_.startTimer();
  //updateState();
  //timer1_.endTimer();

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] START getPointsPositionCppAd" << std::endl;
  int numPoints = pointsOnRobotPtr_->getNumOfPoints();
  timer2_.startTimer();
  Eigen::VectorXd positionPointsOnRobot = pointsOnRobotPtr_->getPointsPositionCppAd(state_, fullState_);
  timer2_.endTimer();
  Eigen::VectorXd radii = pointsOnRobotPtr_->getRadii();

  assert(positionPointsOnRobot.size() % 3 == 0);
  
  double distance;
  p0_vec_.clear();
  p1_vec_.clear();

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] START getNearestOccupancyDist" << std::endl;
  timer3_.startTimer();
  pointsOnRobotPtr_->getPointsEigenToGeometryMsgsVec(positionPointsOnRobot, p0_vec_);
  std::vector<double> min_distances;
  bool collision = emuPtr_->getNearestOccupancyDist(numPoints, positionPointsOnRobot, radii, maxDistance_, p1_vec_, min_distances, true);
  timer3_.endTimer();

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] collision: " << collision << std::endl;

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] START fillOccDistanceArrayVisu" << std::endl;
  timer4_.startTimer();
  emuPtr_->fillOccDistanceArrayVisu(p0_vec_, p1_vec_);
  timer4_.endTimer();

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] BEFORE publishOccDistanceArrayVisu" << std::endl;
  timer5_.startTimer();
  pointsOnRobotPtr_->publishPointsOnRobotVisu();
  emuPtr_->publishOccDistanceArrayVisu();
  timer5_.endTimer();
  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] AFTER publishOccDistanceArrayVisu" << std::endl;
  
  timer0_.endTimer();

  /*
  std::cout << "### MPC_ROS Benchmarking timer0_ TOTAL:" << std::endl;
  std::cout << "###   Maximum : " << timer0_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer0_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer0_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer2_ getPointsPositionCppAd:" << std::endl;
  std::cout << "###   Maximum : " << timer2_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer2_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer2_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer3_ getNearestOccupancyDist:" << std::endl;
  std::cout << "###   Maximum : " << timer3_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer3_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer3_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer4_ fillOccDistanceArrayVisu:" << std::endl;
  std::cout << "###   Maximum : " << timer4_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer4_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer4_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer5_ publishOccDistanceArrayVisu:" << std::endl;
  std::cout << "###   Maximum : " << timer5_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer5_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer5_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;
  */

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[MobileManipulatorVisualization::updateExtCollisionDistances] END" << std::endl << std::endl;;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::updateOctCallback(const ros::TimerEvent& event)
{
  //std::cout << "[MobileManipulatorVisualization::octUpdateCallback] START" << std::endl;

  emuPtr_->updateOct();

  //std::cout << "[MobileManipulatorVisualization::octUpdateCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::distanceVisualizationCallback(const ros::TimerEvent& event)
{
  //std::cout << "[MobileManipulatorVisualization::distanceVisualizationCallback] START" << std::endl;

  timer1_.startTimer();
  updateState();
  timer1_.endTimer();

  updateExtCollisionDistances(false);

  publishSelfCollisionDistances();

  /*
  std::cout << "### MPC_ROS Benchmarking timer1_ updateState:" << std::endl;
  std::cout << "###   Maximum : " << timer1_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer1_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer1_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;
  */

  //std::cout << "[MobileManipulatorVisualization::distanceVisualizationCallback] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/*
void MobileManipulatorVisualization::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation) 
{
  // publish world -> base transform
  const auto r_world_base = getBasePosition(observation.state, robotModelInfo_);
  const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(observation.state, robotModelInfo_);

  geometry_msgs::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "world";
  base_tf.child_frame_id = robotModelInfo_.mobileBase.baseFrame;
  base_tf.transform.translation = ros_msg_helpers::getVectorMsg(r_world_base);
  base_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(q_world_base);
  tfBroadcaster_.sendTransform(base_tf);      

  // publish joints transforms
  const auto j_arm = getArmJointAngles(observation.state, robotModelInfo_);
  std::map<std::string, scalar_t> jointPositions;

  for (size_t i = 0; i < robotModelInfo_.robotArm.jointFrameNames.size(); i++) 
  {
    jointPositions[robotModelInfo_.robotArm.jointFrameNames[i]] = j_arm(i);
  }

  for (const auto& name : removeJointNames_) 
  {
    jointPositions[name] = 0.0;
  }
  robotStatePublisherPtr_ -> publishTransforms(jointPositions, timeStamp);
}
*/

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::publishTargetTrajectories(const ros::Time& timeStamp,
                                                                      const TargetTrajectories& targetTrajectories) 
{
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition = targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = targetTrajectories.stateTrajectory.back().tail(4);

  geometry_msgs::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "world";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);

  tfBroadcaster_.sendTransform(command_tf);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy) 
{
  std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory(2)] START" << std::endl;

  std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory(2)] DEBUG INF" << std::endl;
  while(1);

  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto& mpcStateTrajectory = policy.stateTrajectory_;
  visualization_msgs::MarkerArray markerArray;

  //std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory] START Base" << std::endl;
  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  //std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory] START EE" << std::endl;
  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  std::vector<geometry_msgs::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) 
  {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    const auto eeIndex = model.getBodyId(robotModelInfo_.robotArm.eeFrame);
    const vector_t eePosition = data.oMf[eeIndex].translation();
    endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  //std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory] START Extract" << std::endl;
  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) 
  {
    // extract from observation
    const auto r_world_base = getBasePosition(state, robotModelInfo_);
    const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(state, robotModelInfo_);

    // convert to ros message
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(r_world_base);

    pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("world", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);

  std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory(2)] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void MobileManipulatorVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy, vector_t fullState) 
{
  //std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory(3)] START" << std::endl;

  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto& mpcStateTrajectory = policy.stateTrajectory_;
  visualization_msgs::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  std::vector<geometry_msgs::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) 
  {
    MobileManipulatorPinocchioMapping pinocchioMapping(robotModelInfo_);
    const vector_t q = pinocchioMapping.getPinocchioJointPosition(state, fullState);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const auto eeIndex = model.getBodyId(robotModelInfo_.robotArm.eeFrame);
    const vector_t eePosition = data.oMf[eeIndex].translation();
    endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
  });
  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) 
  {
    // extract from observation
    const auto r_world_base = getBasePosition(state, robotModelInfo_);
    const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(state, robotModelInfo_);

    // convert to ros message
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(r_world_base);

    pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("world", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);

  //std::cout << "[MobileManipulatorVisualization::publishOptimizedTrajectory(3)] END" << std::endl;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
