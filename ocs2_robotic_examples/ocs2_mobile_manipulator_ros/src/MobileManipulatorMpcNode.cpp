// LAST UPDATE: 2023.06.21
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include <ros/init.h>
#include <ros/package.h>

#include "ocs2_core/dynamics/MultiModelFunctions.h"
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

using namespace ocs2;
using namespace mobile_manipulator;
using namespace std;

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorMpcNode::main] START" << std::endl;

  //std::string robotName = "jackal_ur5";
  std::string robotModelName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotModelName + "_mpc");
  ros::NodeHandle nodeHandle;
  
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;

  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);

  std::cout << "[MobileManipulatorMpcNode::main] Loading task file: " << taskFile << std::endl;
  std::cout << "[MobileManipulatorMpcNode::main] Loading library folder: " << libFolder << std::endl;
  std::cout << "[MobileManipulatorMpcNode::main] Loading urdf file: " << urdfFile << std::endl;
  
  //// NUA TODO: GET INFO FROM TASK FILE AND INIT IN MobileManipulatorInterface!
  // Get points on robot parameters
  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nodeHandle.hasParam("/collision_points")) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle.getParam("/collision_points", collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorMpcNode::main] collision_points parameter is not of type array.");
    }
    
    // NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    std::cout << "[MobileManipulatorMpcNode::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorMpcNode::main] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorMpcNode::main] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorMpcNode::main] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("[MobileManipulatorMpcNode::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorMpcNode::main] ERROR: collision_points is not defined!" << std::endl;
  }

  // Robot interfaces
  /*
  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface_baseMotion" << std::endl;
  MobileManipulatorInterface interface_baseMotion(taskFile, libFolder, urdfFile, pointsAndRadii, 0);
  interface_baseMotion.launchNodes(nodeHandle);
  auto robotModelInfo_baseMotion = interface_baseMotion.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo_baseMotion);
  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface_baseMotion" << std::endl;

  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface_armMotion" << std::endl;
  MobileManipulatorInterface interface_armMotion(taskFile, libFolder, urdfFile, pointsAndRadii, 1);
  interface_armMotion.launchNodes(nodeHandle);
  auto robotModelInfo_armMotion = interface_armMotion.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo_armMotion);
  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface_armMotion" << std::endl;
  */
  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface_wholeBodyMotion" << std::endl;
  MobileManipulatorInterface interface_wholeBodyMotion(taskFile, libFolder, urdfFile, pointsAndRadii, 2);
  interface_wholeBodyMotion.launchNodes(nodeHandle);
  auto robotModelInfo_wholeBodyMotion = interface_wholeBodyMotion.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo_wholeBodyMotion);
  std::cout << "[MobileManipulatorMpcNode::main] END INIT interface_wholeBodyMotion" << std::endl;

  //std::cout << "[MobileManipulatorMpcNode::main] DEBUG INF" << std::endl;
  //while(1);

  // ROS ReferenceManager
  /*
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_baseMotion(new ocs2::RosReferenceManager(robotModelName, 
                                                                                                             interface_baseMotion.getReferenceManagerPtr(),
                                                                                                             interface_baseMotion.getRobotModelInfo()));
  rosReferenceManagerPtr_baseMotion->subscribe(nodeHandle);

  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_armMotion(new ocs2::RosReferenceManager(robotModelName, 
                                                                                                            interface_armMotion.getReferenceManagerPtr(),
                                                                                                            interface_armMotion.getRobotModelInfo()));
  rosReferenceManagerPtr_armMotion->subscribe(nodeHandle);
  */
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr_wholeBodyMotion(new ocs2::RosReferenceManager(robotModelName, 
                                                                                                                  interface_wholeBodyMotion.getReferenceManagerPtr(),
                                                                                                                  interface_wholeBodyMotion.getRobotModelInfo()));
  rosReferenceManagerPtr_wholeBodyMotion->subscribe(nodeHandle);

  // MPC
  /*
  ocs2::GaussNewtonDDP_MPC mpc_baseMotion(interface_baseMotion.mpcSettings(), 
                                          interface_baseMotion.ddpSettings(), 
                                          interface_baseMotion.getRollout(), 
                                          interface_baseMotion.getOptimalControlProblem(), 
                                          interface_baseMotion.getInitializer());
  mpc_baseMotion.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_baseMotion);

  ocs2::GaussNewtonDDP_MPC mpc_armMotion(interface_armMotion.mpcSettings(), 
                                         interface_armMotion.ddpSettings(), 
                                         interface_armMotion.getRollout(), 
                                         interface_armMotion.getOptimalControlProblem(), 
                                         interface_armMotion.getInitializer());
  mpc_armMotion.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_armMotion);
  */

  ocs2::GaussNewtonDDP_MPC mpc_wholeBodyMotion(interface_wholeBodyMotion.mpcSettings(), 
                                               interface_wholeBodyMotion.ddpSettings(), 
                                               interface_wholeBodyMotion.getRollout(), 
                                               interface_wholeBodyMotion.getOptimalControlProblem(), 
                                               interface_wholeBodyMotion.getInitializer());
  mpc_wholeBodyMotion.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr_wholeBodyMotion);

  // Launch MPC ROS node
  /*
  MPC_ROS_Interface mpcNode(mpc_baseMotion, 
                            mpc_armMotion, 
                            mpc_wholeBodyMotion, 
                            robotModelName);
  */
  MPC_ROS_Interface mpcNode(mpc_wholeBodyMotion, 
                            robotModelName);
  mpcNode.launchNodes(nodeHandle);

  std::cout << "[MobileManipulatorMpcNode::main] END" << std::endl;

  // Successful exit
  return 0;
}
