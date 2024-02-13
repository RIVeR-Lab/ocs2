// LAST UPDATE: 2024.02.12
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

// needs to be included before boost
#include <pinocchio/multibody/geometry.hpp>

#include <ros/package.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>
#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include <ocs2_mobile_manipulator/MobileManipulatorVisualization.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) 
{
  //std::cout << "[MobileManipulatorObservationGeneration::main] START" << std::endl;

  // Initialize ros node
  ros::init(argc, argv, "distance_visualization");
  ros::NodeHandle nodeHandle;

  ros::MultiThreadedSpinner spinner(4);
  //ros::AsyncSpinner spinner(0);
  //spinner.start();
  
  // Get ROS parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);

  // read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  
  // Initialize local parameters
  RobotModelType robotModelType;
  std::vector<std::string> removeJointNames, armJointFrameNames, armJointNames;
  std::string sim, ns, worldFrameName, robotName, baseFrameName, armBaseFrame, eeFrame, collisionConstraintPoints, collisionCheckPoints,
              baseStateMsg, armStateMsg, baseControlMsg, armControlMsg, selfCollisionMsg, occupancyDistanceBaseMsg, occupancyDistanceArmMsg, pointsOnRobotMsg, octomapMsg,
              baseFrame_withNS, armBaseFrame_withNS, eeFrame_withNS;
  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  double maxDistance;
  bool recompileLibraries;

  /// NUA TODO: READ THIS IN CONFIG!
  bool printOutFlag = true;

  // Read parameters from task file
  loadData::loadPtreeValue<std::string>(pt, sim, "model_information.sim", printOutFlag);
  robotModelType = loadRobotType(taskFile, "model_information.robotModelType");
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, worldFrameName, "model_information.worldFrame", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, baseFrameName, "model_information.baseFrame", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg, "model_information.baseStateMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg, "model_information.armStateMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg, "model_information.baseControlMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg, "model_information.armControlMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, selfCollisionMsg, "model_information.selfCollisionMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceBaseMsg, "model_information.occupancyDistanceBaseMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceArmMsg, "model_information.occupancyDistanceArmMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, pointsOnRobotMsg, "model_information.pointsOnRobotMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, octomapMsg, "model_information.octomapMsg", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, collisionConstraintPoints, "model_information.collisionConstraintPoints", printOutFlag);
  loadData::loadPtreeValue<std::string>(pt, collisionCheckPoints, "model_information.collisionCheckPoints", printOutFlag);
  loadData::loadStdVector<std::string>(taskFile, "model_information.armJointFrameNames", armJointFrameNames, printOutFlag);
  loadData::loadStdVector<std::string>(taskFile, "model_information.armJointNames", armJointNames, printOutFlag);
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", selfCollisionObjectPairs, printOutFlag);
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", selfCollisionLinkPairs, printOutFlag);
  loadData::loadPtreeValue(pt, maxDistance, "extCollision.maxDistance", printOutFlag);
  loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", printOutFlag);

  // Print out parameter values
  if (printOutFlag)
  {
    std::cout << "[MobileManipulatorObservationGeneration::main] sim: " << sim << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] robotModelType: " << static_cast<int>(robotModelType) << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] worldFrameName: " << worldFrameName << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] robotName: " << robotName << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] baseFrame: " << baseFrameName << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] armBaseFrame: " << armBaseFrame << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] eeFrame: " << eeFrame << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] baseStateMsg: " << baseStateMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] armStateMsg: " << armStateMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] baseControlMsg: " << baseControlMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] armControlMsg: " << armControlMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] selfCollisionMsg: " << selfCollisionMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] occupancyDistanceBaseMsg: " << occupancyDistanceBaseMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] occupancyDistanceArmMsg: " << occupancyDistanceArmMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] pointsOnRobotMsg: " << pointsOnRobotMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] octomapMsg: " << octomapMsg << std::endl;
    std::cout << "[MobileManipulatorObservationGeneration::main] removeJoints: " << std::endl;
    for (const auto& name : removeJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorObservationGeneration::main] armJointFrameNames:" << std::endl;
    for (const auto& name : armJointFrameNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorObservationGeneration::main] jointNames: ";
    for (const auto& name : armJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorObservationGeneration::main] selfCollisionObjectPairs:" << std::endl;
    for (const auto& element : selfCollisionObjectPairs) 
    {
      std::cout << "[" << element.first << ", " << element.second << "]" << std::endl;
    }
    std::cout << "[MobileManipulatorObservationGeneration::main] selfCollisionLinkPairs:" << std::endl;
    for (const auto& element : selfCollisionLinkPairs) 
    {
      std::cout << "[" << element.first << ", " << element.second << "]" << std::endl;
    }
  }

  // Set object names
  std::string goal_frame_name;
  std::vector<std::string> obj_octomap_names;
  std::vector<std::string> mobiman_occ_obs_names;
  
  //nodeHandle.param<std::vector<std::string>>("/obj_octomap_names", obj_octomap_names, "");
  nodeHandle.getParam("/goal_frame_name", goal_frame_name);
  nodeHandle.getParam("/obj_octomap_names", obj_octomap_names);
  nodeHandle.getParam("/mobiman_occ_obs_names", mobiman_occ_obs_names);

  std::cout << "[MobileManipulatorObservationGeneration::main] goal_frame_name: " << goal_frame_name << std::endl;
  std::cout << "[MobileManipulatorObservationGeneration::main] obj_octomap_names size: " << obj_octomap_names.size() << std::endl;
  for (size_t i = 0; i < obj_octomap_names.size(); i++)
  {
    std::cout << i << " -> " << obj_octomap_names[i] << std::endl;
  }

  std::cout << "[MobileManipulatorObservationGeneration::main] mobiman_occ_obs_names size: " << mobiman_occ_obs_names.size() << std::endl;
  for (size_t i = 0; i < mobiman_occ_obs_names.size(); i++)
  {
    std::cout << i << " -> " << mobiman_occ_obs_names[i] << std::endl;
  }
  
  //std::cout << "[MobileManipulatorObservationGeneration::main] DEBUG_VISU" << std::endl;
  //while(1);

  // Adding namespace
  ns = nodeHandle.getNamespace();
  std::vector<std::string> armJointFrameNames_withNS_ = armJointFrameNames;
  baseFrame_withNS = baseFrameName;
  armBaseFrame_withNS = armBaseFrame;
  eeFrame_withNS = eeFrame;
  if (ns != "/")
  {
    baseFrame_withNS = ns + "/" + baseFrameName;
    armBaseFrame_withNS = ns + "/" + armBaseFrame;
    eeFrame_withNS = ns + "/" + eeFrame;
    goal_frame_name = ns + "/" + goal_frame_name;

    if (printOutFlag)
    {
      std::cout << "[MobileManipulatorObservationGeneration::main] ns: " << ns << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] armJointFrameNames_withNS_: " << std::endl;
    }
    for (auto& name : armJointFrameNames_withNS_) 
    {
      name = ns + "/" + name;
      if (printOutFlag)
      {
        std::cout << name << std::endl;
      }
    }

    armStateMsg = ns + armStateMsg;
    baseControlMsg = ns + baseControlMsg;
    armControlMsg = ns + armControlMsg;
    selfCollisionMsg = ns + selfCollisionMsg;
    occupancyDistanceBaseMsg = ns + occupancyDistanceBaseMsg;
    occupancyDistanceArmMsg = ns + occupancyDistanceArmMsg;
    pointsOnRobotMsg = ns + pointsOnRobotMsg;
    octomapMsg = ns + octomapMsg;

    for (size_t i = 0; i < obj_octomap_names.size(); i++)
    {
      obj_octomap_names[i] = ns + "/" + obj_octomap_names[i];
    }

    for (size_t i = 0; i < mobiman_occ_obs_names.size(); i++)
    {
      mobiman_occ_obs_names[i] = ns + "/" + mobiman_occ_obs_names[i];
    }
    
    if (printOutFlag)
    {
      std::cout << "[MobileManipulatorObservationGeneration::main] baseFrame_withNS: " << baseFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] armBaseFrame_withNS: " << armBaseFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] eeFrame_withNS: " << eeFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] goal_frame_name: " << goal_frame_name << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] armStateMsg: " << armStateMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] baseControlMsg: " << baseControlMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] armControlMsg: " << armControlMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] selfCollisionMsg: " << selfCollisionMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] occupancyDistanceBaseMsg: " << occupancyDistanceBaseMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] occupancyDistanceArmMsg: " << occupancyDistanceArmMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] pointsOnRobotMsgName: " << pointsOnRobotMsg << std::endl;
      std::cout << "[MobileManipulatorObservationGeneration::main] octomapMsg: " << octomapMsg << std::endl;
    
      std::cout << "[MobileManipulatorObservationGeneration::main] obj_octomap_names size: " << obj_octomap_names.size() << std::endl;
      for (size_t i = 0; i < obj_octomap_names.size(); i++)
      {
        std::cout << i << " -> " << obj_octomap_names[i] << std::endl;
      }

      std::cout << "[MobileManipulatorObservationGeneration::main] mobiman_occ_obs_names size: " << mobiman_occ_obs_names.size() << std::endl;
      for (size_t i = 0; i < mobiman_occ_obs_names.size(); i++)
      {
        std::cout << i << " -> " << mobiman_occ_obs_names[i] << std::endl;
      }
    }
  }

  //std::cout << "[MobileManipulatorObservationGeneration::main] DEBUG_VISU" << std::endl; 
  //while(1);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE createPinocchioInterface" << std::endl;
  // Create pinocchio interface
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  pinocchioInterfacePtr.reset(new PinocchioInterface(ocs2::mobile_manipulator::createPinocchioInterface(urdfFile, robotModelType, removeJointNames, worldFrameName, baseFrame_withNS)));

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE createRobotModelInfo" << std::endl;
  // Set Robot Model Info
  RobotModelInfo robotModelInfo = createRobotModelInfo(robotName,
                                                       robotModelType,
                                                       baseFrameName, 
                                                       armBaseFrame, 
                                                       eeFrame,
                                                       armJointFrameNames,
                                                       armJointNames);

  //std::cout << "[MobileManipulatorObservationGeneration::main] DEBUG_VISU" << std::endl; 
  //while(1);

  //std::shared_ptr<PinocchioGeometryInterface> geometryInterfacePtr;
  //geometryInterfacePtr.reset(new PinocchioGeometryInterface(*pinocchioInterfacePtr, selfCollisionLinkPairs, selfCollisionObjectPairs));
  
  //std::unique_ptr<GeometryInterfaceVisualization> visualizationInterfacePtr;
  //visualizationInterfacePtr.reset(new GeometryInterfaceVisualization(*pinocchioInterfacePtr, *geometryInterfacePtr, nodeHandle, baseFrameName));

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE pointsAndRadii" << std::endl;
  //// NUA TODO: GET INFO FROM TASK FILE AND INIT IN MobileManipulatorInterface!
  // Get points on robot parameters
  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nodeHandle.hasParam(collisionCheckPoints)) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle.getParam(collisionCheckPoints, collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      std::cout << "[MobileManipulatorObservationGeneration::main] collision_points parameter is not of type array." << std::endl;
    }
    
    //// NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    if (printOutFlag)
      std::cout << "[MobileManipulatorObservationGeneration::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        if (printOutFlag)
          std::cout << "[MobileManipulatorObservationGeneration::main] collision_points[" << i << "] parameter is not of type array." << std::endl;
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          if (printOutFlag)
            std::cout << "[MobileManipulatorObservationGeneration::main] collision_points[" << i << "][" << j << "] parameter is not of type array." << std::endl;
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          if (printOutFlag)
            std::cout << "[MobileManipulatorObservationGeneration::main] collision_points[" << i << "][" << j << "] does not have 2 elements." << std::endl;
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        if (printOutFlag)
          std::cout << "[MobileManipulatorObservationGeneration::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius << std::endl;
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorObservationGeneration::main] ERROR: collision_points is not defined!" << std::endl;
  }

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE PointsOnRobot" << std::endl;
  // Set PointsOnRobot object
  std::shared_ptr<PointsOnRobot> pointsOnRobotPtr;
  pointsOnRobotPtr.reset(new PointsOnRobot(pointsAndRadii));
  if (pointsOnRobotPtr->getNumOfPoints() > 0) 
  {
    pointsOnRobotPtr->initialize(*pinocchioInterfacePtr,
                                 MobileManipulatorPinocchioMapping(robotModelInfo),
                                 MobileManipulatorPinocchioMappingCppAd(robotModelInfo),
                                 robotModelInfo,
                                 "points_on_robot",
                                 libFolder,
                                 recompileLibraries,
                                 false);
    pointsOnRobotPtr->setNodeHandle(nodeHandle);
    pointsOnRobotPtr->setPointsOnRobotVisu();
    //pointsOnRobotPtr->setTimerPointsOnRobotVisu(0.01);
  }
  else
  {
    pointsOnRobotPtr = nullptr;
  }

  //std::cout << "[MobileManipulatorObservationGeneration::main] DEBUG_VISU" << std::endl; 
  //while(1);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE ExtMapUtility" << std::endl;
  // Set ExtMapUtility object
  std::shared_ptr<ExtMapUtility> emuPtr;
  emuPtr.reset(new ExtMapUtility());
  emuPtr->setWorldFrameName(worldFrameName);
  emuPtr->setPubCollisionInfoBase(occupancyDistanceBaseMsg);
  emuPtr->setPubCollisionInfoArm(occupancyDistanceArmMsg);
  emuPtr->setPubOccDistArrayVisu(occupancyDistanceArmMsg + "_visu");
  emuPtr->setPubOccDistArrayVisu2(occupancyDistanceBaseMsg + "_visu");
  emuPtr->setNodeHandle(nodeHandle);
  emuPtr->subscribeOctMsg(octomapMsg);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE subscribeObjectsOctMsg" << std::endl;
  
  for (size_t i = 0; i < obj_octomap_names.size(); i++)
  {
    emuPtr->subscribeObjectsOctMsg(obj_octomap_names[i]);
  }

  //distances_.resize(pointsOnRobotPtr_->getNumOfPoints());

  // Visualization
  bool activateSelfCollision = true;
  bool activateExtCollision = true;

  /// NUA TODO: READ FROM CONFIG!
  maxDistance = 5;

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE mobileManipulatorVisu_" << std::endl;
  MobileManipulatorVisualization mobileManipulatorVisu(nodeHandle, 
                                                       *pinocchioInterfacePtr,
                                                       worldFrameName,
                                                       ns,
                                                       baseFrameName,
                                                       urdfFile,
                                                       armStateMsg,
                                                       robotModelInfo,
                                                       activateSelfCollision,
                                                       activateExtCollision,
                                                       removeJointNames,
                                                       selfCollisionObjectPairs,
                                                       selfCollisionLinkPairs,
                                                       selfCollisionMsg,
                                                       pointsOnRobotPtr,
                                                       emuPtr,
                                                       maxDistance);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] AFTER mobileManipulatorVisu_" << std::endl; 

  mobileManipulatorVisu.setObjOctomapNames(obj_octomap_names);
  mobileManipulatorVisu.setMobimanOccObsNames(mobiman_occ_obs_names);
  mobileManipulatorVisu.updateStateIndexMap();
  mobileManipulatorVisu.setGoalFrameName(goal_frame_name);
  mobileManipulatorVisu.setGoalTrajectoryFrameName(baseFrame_withNS);
  mobileManipulatorVisu.setOccupancyInfoFrameName(baseFrame_withNS);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE launchVisualizerNode" << std::endl;
  mobileManipulatorVisu.launchVisualizerNode(nodeHandle);

  //double emu_dt = 0.01;
  //ros::Timer octUpdateTimer = nodeHandle.createTimer(ros::Duration(emu_dt), &MobileManipulatorVisualization::updateOctCallback, &mobileManipulatorVisu);
  
  //std::cout << "[MobileManipulatorObservationGeneration::main] DEBUG_VISU" << std::endl; 
  //while(1);

  /*
  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE distanceVisualizationCallback" << std::endl; 
  double dist_dt = 0.001;
  ros::Timer mainTimer = nodeHandle.createTimer(ros::Duration(dist_dt), &MobileManipulatorVisualization::distanceVisualizationCallback, &mobileManipulatorVisu);
  */

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] BEFORE mobimanObservationTimerCallback" << std::endl;
  double occ_info_dt = 0.1;
  mobileManipulatorVisu.setGoalTrajectoryQueueDt(occ_info_dt);
  mobileManipulatorVisu.setOccupancyInfoQueueDt(occ_info_dt);
  ros::Timer occupancyInfoTimer = nodeHandle.createTimer(ros::Duration(occ_info_dt), &MobileManipulatorVisualization::mobimanObservationTimerCallback, &mobileManipulatorVisu);

  if (printOutFlag)
    std::cout << "[MobileManipulatorObservationGeneration::main] END" << std::endl;

  spinner.spin();
  //ros::spin();

  return 0;
}