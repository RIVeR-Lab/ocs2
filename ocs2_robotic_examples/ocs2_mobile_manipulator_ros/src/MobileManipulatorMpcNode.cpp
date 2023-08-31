// LAST UPDATE: 2023.07.20
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

  // Robot interface
  std::cout << "[MobileManipulatorMpcNode::main] START INIT interface" << std::endl;
  MobileManipulatorInterface interface(nodeHandle, taskFile, libFolder, urdfFile);
  //interface.setNodeHandle(nodeHandle);

  interface.runMPC();

  /*
  interface.launchNodes(nodeHandle);
  std::cout << "[MobileManipulatorMpcNode::main] END INIT interface" << std::endl;

  size_t modelModeInt = 2;

  interface.setMPCProblem(modelModeInt, pointsAndRadii);

  auto robotModelInfo = interface.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo);

  //std::cout << "[MobileManipulatorMpcNode::main] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[MobileManipulatorMpcNode::main] BEFORE rosReferenceManagerPtr" << std::endl;
  // ROS ReferenceManager
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(new ocs2::RosReferenceManager(robotModelName, 
                                                                                                  interface.getReferenceManagerPtr(),
                                                                                                  interface.getRobotModelInfo()));
  
  std::cout << "[MobileManipulatorMpcNode::main] BEFORE rosReferenceManagerPtr subscribe" << std::endl;
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  std::cout << "[MobileManipulatorMpcNode::main] BEFORE mpc" << std::endl;
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), 
                              interface.ddpSettings(), 
                              interface.getRollout(), 
                              interface.getOptimalControlProblem(), 
                              interface.getInitializer());

  std::cout << "[MobileManipulatorMpcNode::main] BEFORE mpc setReferenceManager" << std::endl;
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  std::cout << "[MobileManipulatorMpcNode::main] BEFORE mpc mpcNode" << std::endl;
  MPC_ROS_Interface mpcNode(mpc, robotModelName);

  std::cout << "[MobileManipulatorMpcNode::main] BEFORE mpc mpcNode launchNodes" << std::endl;
  mpcNode.launchNodes(nodeHandle);

  modelModeInt = mpcNode.getModelModeInt();
  */

  /*
  size_t modelModeInt = 2;
  int iter = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "=====================================================" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "[MobileManipulatorMpcNode::main] START ITERATION: " << iter << std::endl;
    //std::cout << "[MobileManipulatorMpcNode::main] modelModeInt: " << modelModeInt << std::endl;

    interface.runMPC(modelModeInt);
    iter++;
 
    std::cout << "[MobileManipulatorMpcNode::main] END ITERATION: " << iter << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }
  */

  std::cout << "[MobileManipulatorMpcNode::main] END" << std::endl;

  // Successful exit
  return 0;
}
