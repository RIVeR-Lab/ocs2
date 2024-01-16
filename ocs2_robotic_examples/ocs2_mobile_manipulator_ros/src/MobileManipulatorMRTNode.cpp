// LAST UPDATE: 2024.01.16
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
  std::cout << "[MobileManipulatorMRTNode::main] START" << std::endl;

  bool printOutFlag = false;

  // Initialize ros node
  ros::init(argc, argv, "mobile_manipulator_mrt_node");
  ros::NodeHandle nh;

  ros::MultiThreadedSpinner spinner(4);
  
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;

  nh.getParam("/taskFile", taskFile);
  nh.getParam("/libFolder", libFolder);
  nh.getParam("/urdfFile", urdfFile);

  if (printOutFlag)
  {
    std::cout << "[MobileManipulatorMRTNode::main] Loading task file: " << taskFile << std::endl;
    std::cout << "[MobileManipulatorMRTNode::main] Loading library folder: " << libFolder << std::endl;
    std::cout << "[MobileManipulatorMRTNode::main] Loading urdf file: " << urdfFile << std::endl;
  }

  // Robot interface
  int initModelModeInt = 2;
  std::string interfaceName = "MRT";
  std::cout << "[MobileManipulatorMRTNode::main] START MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface m4_interface(nh, taskFile, libFolder, urdfFile, initModelModeInt, interfaceName, printOutFlag);

  std::cout << "[MobileManipulatorMRTNode::main] BEFORE initializeMRT" << std::endl;
  m4_interface.initializeMRT();

  std::cout << "[MobileManipulatorMRTNode::main] BEFORE launchMRT" << std::endl;
  //m4_interface.launchMRT();

  double mrt_dt = 0.05;
  ros::Timer mrtTimer = nh.createTimer(ros::Duration(mrt_dt), &MobileManipulatorInterface::mrtCallback, &m4_interface);

  std::cout << "[MobileManipulatorMRTNode::main] END" << std::endl;

  spinner.spin(); // spin() will not return until the node has been shutdown
  //ros::spin();

  // Successful exit
  return 0;
}
