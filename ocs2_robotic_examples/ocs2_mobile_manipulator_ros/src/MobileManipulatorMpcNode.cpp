// LAST UPDATE: 2022.04.10
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

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorMpcNode::main] START" << std::endl;

  //std::string robotName = "jackal_ur5";
  std::string robotModel = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotModel + "_mpc");
  ros::NodeHandle nodeHandle;
  
  // Get node parameters
  string taskFile, libFolder, urdfFile;

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

  // Robot interface
  std::cout << "[MobileManipulatorMpcNode::main] START INIT MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile, pointsAndRadii);
  interface.launchNodes(nodeHandle);
  std::cout << "[MobileManipulatorMpcNode::main] END INIT MobileManipulatorInterface" << std::endl;

  auto robotModelInfo = interface.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo);

  //std::cout << "[MobileManipulatorMpcNode::main] DEBUG INF" << std::endl;
  //while(1);

  // ROS ReferenceManager
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(new ocs2::RosReferenceManager(robotModel, interface.getReferenceManagerPtr()));
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), 
                               interface.ddpSettings(), 
                               interface.getRollout(), 
                               interface.getOptimalControlProblem(), 
                               interface.getInitializer());

  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotModel);
  mpcNode.launchNodes(nodeHandle);

  std::cout << "[MobileManipulatorMpcNode::main] END" << std::endl;

  // Successful exit
  return 0;
}
