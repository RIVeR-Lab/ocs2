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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/package.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
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
void MobileManipulatorVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) 
{
  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  const std::string urdfName = "robot_description";
  urdf::Model model;
  if (!model.initParam(urdfName)) 
  {
    ROS_ERROR("[MobileManipulatorVisualization::launchVisualizerNode] URDF model load was NOT successful");
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) 
  {
    ROS_ERROR("[MobileManipulatorVisualization::launchVisualizerNode] Failed to extract kdl tree from xml robot description");
  }

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);

  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);
  
  // Get ROS parameter
  //std::string urdfFile, taskFile;
  //nodeHandle.getParam("/urdfFile", urdfFile);
  //nodeHandle.getParam("/taskFile", taskFile);
  
  // read manipulator type
  //RobotModelType modelType = loadRobotType(taskFile_, "model_information.robotModelType");
  
  // read the joints to make fixed
  loadData::loadStdVector<std::string>(taskFile_, "model_information.removeJoints", removeJointNames_, false);

  // create pinocchio interface
  PinocchioInterface pinocchioInterface(mobile_manipulator::createPinocchioInterface(urdfFile_, robotModelInfo_.robotModelType, removeJointNames_));

  // read if self-collision checking active
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);

  // activate markers for self-collision visualization
  if (activateSelfCollision) 
  {
    std::string prefix = "selfCollision";
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
    scalar_t mu = 1e-2;
    scalar_t delta = 1e-3;
    scalar_t minimumDistance = 0.0;

    //std::cerr << "\n #### SelfCollision Settings: ";
    //std::cerr << "\n #### =============================================================================\n";
    loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
    loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
    loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
    loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
    loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
    //std::cerr << " #### =============================================================================\n";

    PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
    
    const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();

    // set geometry visualization markers
    geometryVisualization_.reset(new GeometryInterfaceVisualization(std::move(pinocchioInterface), geometryInterface, nodeHandle));
  }
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
  
  if (geometryVisualization_ != nullptr) 
  {
    publishDistances(observation.state, observation.full_state, robotModelInfo_);
  }

  //std::cout << "[MobileManipulatorVisualization::update] END" << std::endl;
}

void MobileManipulatorVisualization::publishDistances(const ocs2::vector_t& state, const ocs2::vector_t& fullState, const RobotModelInfo& modelInfo) 
{
  //std::cout << "[GeometryInterfaceVisualization::publishDistances] START" << std::endl;

  std::string prefix = "selfCollision";
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;

  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile_, prefix + ".collisionLinkPairs", collisionLinkPairs, true);

  PinocchioInterface pinocchioInterface(mobile_manipulator::createPinocchioInterface(urdfFile_, robotModelInfo_.robotModelType, removeJointNames_));
  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  std::string pinocchioWorldFrame_ = "world";
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping pinocchioMapping(modelInfo);
  const vector_t q = pinocchioMapping.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
  const auto results = geometryVisualization_->getGeometryInterface().computeDistances(pinocchioInterface_);

  visualization_msgs::MarkerArray markerArray;

  constexpr size_t numMarkersPerResult = 5;

  visualization_msgs::Marker markerTemplate;
  markerTemplate.color = ros_msg_helpers::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = pinocchioWorldFrame_;
  markerTemplate.header.stamp = ros::Time::now();
  markerTemplate.pose.orientation = ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult, markerTemplate);

  //std::cout << "[GeometryInterfaceVisualization::publishDistances] results.size(): " << results.size() << std::endl;

  for (size_t i = 0; i < results.size(); ++i) 
  {
    // I apologize for the magic numbers, it's mostly just visualization numbers(so 0.02 scale corresponds rougly to 0.02 cm)

    for (size_t j = 0; j < numMarkersPerResult; ++j) 
    {
      markerArray.markers[numMarkersPerResult * i + j].ns = std::to_string(geometryInterface.getGeometryModel().collisionPairs[i].first) +
                                                            " - " +
                                                            std::to_string(geometryInterface.getGeometryModel().collisionPairs[i].second);
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
    markerArray.markers[numMarkersPerResult * i + 2].text = "obj " + std::to_string(geometryVisualization_->getGeometryInterface().getGeometryModel().collisionPairs[i].first);
    markerArray.markers[numMarkersPerResult * i + 3].id = numMarkersPerResult * i + 3;
    markerArray.markers[numMarkersPerResult * i + 3].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 3].pose.position = ros_msg_helpers::getPointMsg(results[i].nearest_points[1]);
    markerArray.markers[numMarkersPerResult * i + 3].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 3].text = "obj " + std::to_string(geometryVisualization_->getGeometryInterface().getGeometryModel().collisionPairs[i].second);
    markerArray.markers[numMarkersPerResult * i + 3].scale.z = 0.02;

    // Text above the arrow, denoting the distance
    markerArray.markers[numMarkersPerResult * i + 4].id = numMarkersPerResult * i + 4;
    markerArray.markers[numMarkersPerResult * i + 4].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 4].pose.position = ros_msg_helpers::getPointMsg((results[i].nearest_points[0] + results[i].nearest_points[1]) / 2.0);
    markerArray.markers[numMarkersPerResult * i + 4].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 4].text = "dist " + std::to_string(geometryVisualization_->getGeometryInterface().getGeometryModel().collisionPairs[i].first) + " - " +
                                                                                     std::to_string(geometryVisualization_->getGeometryInterface().getGeometryModel().collisionPairs[i].second) + 
                                                                                     ": " + std::to_string(results[i].min_distance);
    markerArray.markers[numMarkersPerResult * i + 4].scale.z = 0.02;
  }

  markerPublisher_.publish(markerArray);

  //std::cout << "[GeometryInterfaceVisualization::publishDistances] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
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
