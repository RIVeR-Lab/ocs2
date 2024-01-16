// LAST UPDATE: 2023.12.13
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

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorMPCNode::main] START" << std::endl;

  bool printOutFlag = false;

  // Initialize ros node
  ros::init(argc, argv, "mobile_manipulator_mpc_node");
  ros::NodeHandle nh;

  ros::MultiThreadedSpinner spinner(4);
  
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;

  nh.getParam("/taskFile", taskFile);
  nh.getParam("/libFolder", libFolder);
  nh.getParam("/urdfFile", urdfFile);

  if (printOutFlag)
  {
    std::cout << "[MobileManipulatorMPCNode::main] Loading task file: " << taskFile << std::endl;
    std::cout << "[MobileManipulatorMPCNode::main] Loading library folder: " << libFolder << std::endl;
    std::cout << "[MobileManipulatorMPCNode::main] Loading urdf file: " << urdfFile << std::endl;
  }

  // Robot interface
  int initModelModeInt = 2;
  std::string interfaceName = "MPC";
  std::cout << "[MobileManipulatorMPCNode::main] BEFORE MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface m4_interface(nh, taskFile, libFolder, urdfFile, initModelModeInt, interfaceName, printOutFlag);
  
  std::cout << "[MobileManipulatorMPCNode::main] BEFORE initializeMPC" << std::endl;
  m4_interface.initializeMPC();

  std::cout << "[MobileManipulatorMPCNode::main] BEFORE launchMPC" << std::endl;
  m4_interface.launchMPC();

  //double mpc_dt = 0.01;
  //ros::Timer mpcTimer = nh_interface.createTimer(ros::Duration(mpc_dt), &MobileManipulatorInterface::mpcCallback, &m4_interface);

  std::cout << "[MobileManipulatorMPCNode::main] END" << std::endl;

  spinner.spin(); // spin() will not return until the node has been shutdown
  //ros::spin();

  // Successful exit
  return 0;
}
