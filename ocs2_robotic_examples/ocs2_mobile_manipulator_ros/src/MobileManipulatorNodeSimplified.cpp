// LAST UPDATE: 2023.11.23
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
  //std::cout << "[MobileManipulatorNodeSimplified::main] START" << std::endl;

  bool printOutFlag = false;

  // Initialize ros node
  ros::init(argc, argv, "mobile_manipulator_node");
  ros::NodeHandle nh_interface;
  //ros::NodeHandle nh_mpc;
  //ros::NodeHandle nh_mrt;

  ros::MultiThreadedSpinner spinner(4);
  
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;

  nh_interface.getParam("/taskFile", taskFile);
  nh_interface.getParam("/libFolder", libFolder);
  nh_interface.getParam("/urdfFile", urdfFile);

  if (printOutFlag)
  {
    std::cout << "[MobileManipulatorNodeSimplified::main] Loading task file: " << taskFile << std::endl;
    std::cout << "[MobileManipulatorNodeSimplified::main] Loading library folder: " << libFolder << std::endl;
    std::cout << "[MobileManipulatorNodeSimplified::main] Loading urdf file: " << urdfFile << std::endl;
  }

  // Robot interface
  int initModelModeInt = 2;
  //std::cout << "[MobileManipulatorNodeSimplified::main] START interface" << std::endl;
  MobileManipulatorInterface m4_interface(nh_interface, taskFile, libFolder, urdfFile, initModelModeInt);

  double mpc_dt = 0.01;

  //std::cout << "[MobileManipulatorNode::main] BEFORE mpcTimer " << std::endl;
  ros::Timer mpcTimer = nh_interface.createTimer(ros::Duration(mpc_dt), &MobileManipulatorInterface::mpcCallback, &m4_interface);

  //m4_interface.calculateMPCTrajectory();

  std::cout << "[MobileManipulatorNodeSimplified::main] END" << std::endl;

  spinner.spin(); // spin() will not return until the node has been shutdown
  //ros::spin();

  // Successful exit
  return 0;
}
