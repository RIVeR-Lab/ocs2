// LAST UPDATE: 2023.11.02
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
  //std::cout << "[MobileManipulatorDistanceVisualization::main] START" << std::endl;

  // Initialize ros node
  ros::init(argc, argv, "distance_visualization");
  ros::NodeHandle nodeHandle;

  //ros::MultiThreadedSpinner spinner(0);
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
  bool printOutFlag = false;

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
    std::cout << "[MobileManipulatorDistanceVisualization::main] sim: " << sim << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] robotModelType: " << static_cast<int>(robotModelType) << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] worldFrameName: " << worldFrameName << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] robotName: " << robotName << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] baseFrame: " << baseFrameName << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] armBaseFrame: " << armBaseFrame << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] eeFrame: " << eeFrame << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] baseStateMsg: " << baseStateMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] armStateMsg: " << armStateMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] baseControlMsg: " << baseControlMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] armControlMsg: " << armControlMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] selfCollisionMsg: " << selfCollisionMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] occupancyDistanceBaseMsg: " << occupancyDistanceBaseMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] occupancyDistanceArmMsg: " << occupancyDistanceArmMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] pointsOnRobotMsg: " << pointsOnRobotMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] octomapMsg: " << octomapMsg << std::endl;
    std::cout << "[MobileManipulatorDistanceVisualization::main] removeJoints: " << std::endl;
    for (const auto& name : removeJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorDistanceVisualization::main] armJointFrameNames:" << std::endl;
    for (const auto& name : armJointFrameNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorDistanceVisualization::main] jointNames: ";
    for (const auto& name : armJointNames) 
    {
      std::cout << name << std::endl;
    }
    std::cout << "[MobileManipulatorDistanceVisualization::main] selfCollisionObjectPairs:" << std::endl;
    for (const auto& element : selfCollisionObjectPairs) 
    {
      std::cout << "[" << element.first << ", " << element.second << "]" << std::endl;
    }
    std::cout << "[MobileManipulatorDistanceVisualization::main] selfCollisionLinkPairs:" << std::endl;
    for (const auto& element : selfCollisionLinkPairs) 
    {
      std::cout << "[" << element.first << ", " << element.second << "]" << std::endl;
    }
  }
  
  // Adding namespace
  ns = nodeHandle.getNamespace();
  std::vector<std::string> armJointFrameNames_withNS_ = armJointFrameNames;
  baseFrame_withNS = baseFrameName;
  armBaseFrame_withNS = armBaseFrame;
  eeFrame_withNS = eeFrame;
  if (ns != "")
  {
    baseFrame_withNS = ns + "/" + baseFrameName;
    armBaseFrame_withNS = ns + "/" + armBaseFrame;
    eeFrame_withNS = ns + "/" + eeFrame;

    if (printOutFlag)
    {
      std::cout << "[MobileManipulatorDistanceVisualization::main] ns: " << ns << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] armJointFrameNames_withNS_: " << std::endl;
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

    if (printOutFlag)
    {
      std::cout << "[MobileManipulatorDistanceVisualization::main] baseFrame_withNS: " << baseFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] armBaseFrame_withNS: " << armBaseFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] eeFrame_withNS: " << eeFrame_withNS << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] armStateMsg: " << armStateMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] baseControlMsg: " << baseControlMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] armControlMsg: " << armControlMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] selfCollisionMsg: " << selfCollisionMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] occupancyDistanceBaseMsg: " << occupancyDistanceBaseMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] occupancyDistanceArmMsg: " << occupancyDistanceArmMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] pointsOnRobotMsgName: " << pointsOnRobotMsg << std::endl;
      std::cout << "[MobileManipulatorDistanceVisualization::main] octomapMsg: " << octomapMsg << std::endl;
    }
  }

  //std::cout << "[MobileManipulatorDistanceVisualization::main] DEBUG_VISU" << std::endl; 
  //while(1);

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE createPinocchioInterface" << std::endl;
  // Create pinocchio interface
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  pinocchioInterfacePtr.reset(new PinocchioInterface(ocs2::mobile_manipulator::createPinocchioInterface(urdfFile, robotModelType, removeJointNames, worldFrameName, baseFrame_withNS)));

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE createRobotModelInfo" << std::endl;
  // Set Robot Model Info
  RobotModelInfo robotModelInfo = createRobotModelInfo(robotName,
                                                       robotModelType,
                                                       baseFrameName, 
                                                       armBaseFrame, 
                                                       eeFrame,
                                                       armJointFrameNames,
                                                       armJointNames);

  //std::cout << "[MobileManipulatorDistanceVisualization::main] DEBUG_VISU" << std::endl; 
  //while(1);

  //std::shared_ptr<PinocchioGeometryInterface> geometryInterfacePtr;
  //geometryInterfacePtr.reset(new PinocchioGeometryInterface(*pinocchioInterfacePtr, selfCollisionLinkPairs, selfCollisionObjectPairs));
  
  //std::unique_ptr<GeometryInterfaceVisualization> visualizationInterfacePtr;
  //visualizationInterfacePtr.reset(new GeometryInterfaceVisualization(*pinocchioInterfacePtr, *geometryInterfacePtr, nodeHandle, baseFrameName));

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE pointsAndRadii" << std::endl;
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
      ROS_WARN("[MobileManipulatorDistanceVisualization::main] collision_points parameter is not of type array.");
    }
    
    //// NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    if (printOutFlag)
      std::cout << "[MobileManipulatorDistanceVisualization::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorDistanceVisualization::main] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorDistanceVisualization::main] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorDistanceVisualization::main] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("[MobileManipulatorDistanceVisualization::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorDistanceVisualization::main] ERROR: collision_points is not defined!" << std::endl;
  }

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE PointsOnRobot" << std::endl;
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

  //std::cout << "[MobileManipulatorDistanceVisualization::main] DEBUG_VISU" << std::endl; 
  //while(1);

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE ExtMapUtility" << std::endl;
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

  /// NUA TODO: GENERALIZE AND READ FROM CONFIG!
  /*
  { 
    "oct_conveyor", 
    "oct_normal_pkg",
    "oct_long_pkg",
    "oct_longwide_pkg",
    "oct_red_cube",
    "oct_green_cube",
    "oct_blue_cube",
    "oct_bin"
  }
  */

  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE subscribeObjectsOctMsg" << std::endl;
  std::vector<std::string> obj_octomap_names = {"oct_conveyor"};
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
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE mobileManipulatorVisu_" << std::endl;
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
    std::cout << "[MobileManipulatorDistanceVisualization::main] AFTER mobileManipulatorVisu_" << std::endl; 

  mobileManipulatorVisu.setObjOctomapNames(obj_octomap_names);
  mobileManipulatorVisu.updateStateIndexMap();

  //double emu_dt = 0.01;
  //ros::Timer octUpdateTimer = nodeHandle.createTimer(ros::Duration(emu_dt), &MobileManipulatorVisualization::updateOctCallback, &mobileManipulatorVisu);
  
  //std::cout << "[MobileManipulatorDistanceVisualization::main] DEBUG_VISU" << std::endl; 
  //while(1);

  double dist_dt = 0.001;
  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] BEFORE createTimer" << std::endl; 
  ros::Timer distanceTimer = nodeHandle.createTimer(ros::Duration(dist_dt), &MobileManipulatorVisualization::distanceVisualizationCallback, &mobileManipulatorVisu);
  
  if (printOutFlag)
    std::cout << "[MobileManipulatorDistanceVisualization::main] END" << std::endl;

  //spinner.spin();
  ros::spin();

  return 0;
}
