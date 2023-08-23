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
  std::cout << "[MobileManipulatorDistanceVisualization::main] START" << std::endl;

  // Initialize ros node
  ros::init(argc, argv, "distance_visualization");
  ros::NodeHandle nodeHandle;

  ros::MultiThreadedSpinner spinner(0);
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
  std::string worldFrameName, robotName, baseFrameName, armBaseFrame, eeFrame, collisionConstraintPoints, collisionCheckPoints,
              baseStateMsg, armStateMsg, baseControlMsg, armControlMsg, occupancyDistanceMsg, octomapMsg;
  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  double maxDistance;
  bool recompileLibraries;
  bool printOutFlag_ = false;

  // Read parameters from task file
  robotModelType = loadRobotType(taskFile, "model_information.robotModelType");
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, worldFrameName, "model_information.worldFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, robotName, "model_information.robotName", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseFrameName, "model_information.baseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armBaseFrame, "model_information.armBaseFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseStateMsg, "model_information.baseStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armStateMsg, "model_information.armStateMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, baseControlMsg, "model_information.baseControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, armControlMsg, "model_information.armControlMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, occupancyDistanceMsg, "model_information.occupancyDistanceMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, octomapMsg, "model_information.octomapMsg", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionConstraintPoints, "model_information.collisionConstraintPoints", printOutFlag_);
  loadData::loadPtreeValue<std::string>(pt, collisionCheckPoints, "model_information.collisionCheckPoints", printOutFlag_);
  loadData::loadStdVector<std::string>(taskFile, "model_information.armJointFrameNames", armJointFrameNames, printOutFlag_);
  loadData::loadStdVector<std::string>(taskFile, "model_information.armJointNames", armJointNames, printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", selfCollisionObjectPairs, printOutFlag_);
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", selfCollisionLinkPairs, printOutFlag_);
  loadData::loadPtreeValue(pt, maxDistance, "extCollision.maxDistance", printOutFlag_);
  loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", printOutFlag_);

  // Print out parameter values
  std::cout << "[MobileManipulatorDistanceVisualization::main] robotModelType: " << static_cast<int>(robotModelType) << std::endl;
  std::cout << "[MobileManipulatorDistanceVisualization::main] worldFrameName: " << worldFrameName << std::endl;
  std::cout << "[MobileManipulatorDistanceVisualization::main] robotName: " << robotName << std::endl;
  std::cout << "[MobileManipulatorDistanceVisualization::main] baseFrame: " << baseFrameName << std::endl;
  std::cout << "[MobileManipulatorDistanceVisualization::main] armBaseFrame: " << armBaseFrame << std::endl;
  std::cout << "[MobileManipulatorDistanceVisualization::main] eeFrame: " << eeFrame << std::endl;
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

  // Create pinocchio interface
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  pinocchioInterfacePtr.reset(new PinocchioInterface(ocs2::mobile_manipulator::createPinocchioInterface(urdfFile, robotModelType, removeJointNames)));

  // Set Robot Model Info
  RobotModelInfo robotModelInfo = createRobotModelInfo(robotName,
                                                       robotModelType,
                                                       baseFrameName, 
                                                       armBaseFrame, 
                                                       eeFrame,
                                                       armJointFrameNames,
                                                       armJointNames);

  std::shared_ptr<PinocchioGeometryInterface> geometryInterfacePtr;
  geometryInterfacePtr.reset(new PinocchioGeometryInterface(*pinocchioInterfacePtr, selfCollisionLinkPairs, selfCollisionObjectPairs));
  
  std::unique_ptr<GeometryInterfaceVisualization> visualizationInterfacePtr;
  visualizationInterfacePtr.reset(new GeometryInterfaceVisualization(*pinocchioInterfacePtr, *geometryInterfacePtr, nodeHandle, baseFrameName));

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

  // Set ExtMapUtility object
  std::shared_ptr<ExtMapUtility> emuPtr;
  emuPtr.reset(new ExtMapUtility());
  emuPtr->setWorldFrameName(worldFrameName);
  emuPtr->setPubOccDistArrayVisu(occupancyDistanceMsg);
  emuPtr->setNodeHandle(nodeHandle);
  emuPtr->subscribeOctMsg(octomapMsg);

  //distances_.resize(pointsOnRobotPtr_->getNumOfPoints());

  // Visualization
  bool activateSelfCollision = true;
  bool activateExtCollision = true;
  std::cout << "[MobileManipulatorInterface::runMRT] BEFORE mobileManipulatorVisu_" << std::endl;
  MobileManipulatorVisualization mobileManipulatorVisu(nodeHandle, 
                                                       *pinocchioInterfacePtr,
                                                       worldFrameName,
                                                       baseFrameName,
                                                       urdfFile,
                                                       armStateMsg,
                                                       robotModelInfo,
                                                       activateSelfCollision,
                                                       activateExtCollision,
                                                       removeJointNames,
                                                       selfCollisionObjectPairs,
                                                       selfCollisionLinkPairs,
                                                       pointsOnRobotPtr,
                                                       emuPtr,
                                                       maxDistance);
  std::cout << "[MobileManipulatorInterface::runMRT] AFTER mobileManipulatorVisu_" << std::endl;

  mobileManipulatorVisu.updateStateIndexMap();

  //double emu_dt = 0.01;
  //ros::Timer octUpdateTimer = nodeHandle.createTimer(ros::Duration(emu_dt), &MobileManipulatorVisualization::updateOctCallback, &mobileManipulatorVisu);
  
  double dist_dt = 0.001;
  ros::Timer distanceTimer = nodeHandle.createTimer(ros::Duration(dist_dt), &MobileManipulatorVisualization::distanceVisualizationCallback, &mobileManipulatorVisu);

  /*
  ros::Rate r(100);
  while(ros::ok)
  {
    std::cout << "[MobileManipulatorDistanceVisualization::main] START LOOP" << std::endl;

    mobileManipulatorVisu.updateDistances(false);

    ros::spinOnce();
    r.sleep();

    std::cout << "[MobileManipulatorDistanceVisualization::main] END LOOP" << std::endl << std::endl;
  }
  */

  std::cout << "[MobileManipulatorDistanceVisualization::main] END" << std::endl;

  spinner.spin();
  //ros::spin();

  return 0;
}
