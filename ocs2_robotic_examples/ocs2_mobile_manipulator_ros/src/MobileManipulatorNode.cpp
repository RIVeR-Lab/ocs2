// LAST UPDATE: 2023.08.24
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

//#include "ocs2_core/dynamics/MultiModelFunctions.h"
//#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
//#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
//#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

//#include <ocs2_mpc/SystemObservation.h>
//#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
//#include <ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h>
//#include <ocs2_mobile_manipulator_ros/MobileManipulatorGazeboVisualization.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorNode::main] START" << std::endl;

  //std::string robotName = "jackal_ur5";
  std::string robotModelName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotModelName + "_mpc");
  ros::NodeHandle nh_interface;
  //ros::NodeHandle nh_mpc;
  //ros::NodeHandle nh_mrt;

  ros::MultiThreadedSpinner spinner(4);
  
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;

  nh_interface.getParam("/taskFile", taskFile);
  nh_interface.getParam("/libFolder", libFolder);
  nh_interface.getParam("/urdfFile", urdfFile);

  std::cout << "[MobileManipulatorNode::main] Loading task file: " << taskFile << std::endl;
  std::cout << "[MobileManipulatorNode::main] Loading library folder: " << libFolder << std::endl;
  std::cout << "[MobileManipulatorNode::main] Loading urdf file: " << urdfFile << std::endl;

  // Robot interface
  int initModelModeInt = 2;
  std::cout << "[MobileManipulatorNode::main] START interface" << std::endl;
  MobileManipulatorInterface m4_interface(nh_interface, taskFile, libFolder, urdfFile, initModelModeInt);

  double mpc_dt = 0.01;
  double mrt_dt = 0.01;

  std::cout << "[MobileManipulatorNode::main] BEFORE mpcTimer " << std::endl;
  ros::Timer mpcTimer = nh_interface.createTimer(ros::Duration(mpc_dt), &MobileManipulatorInterface::mpcCallback, &m4_interface);
  std::cout << "[MobileManipulatorNode::main] AFTER mpcTimer" << std::endl;

  ros::Timer mrtTimer = nh_interface.createTimer(ros::Duration(mrt_dt), &MobileManipulatorInterface::mrtCallback, &m4_interface);

  std::cout << "[MobileManipulatorNode::main] END" << std::endl;

  spinner.spin(); // spin() will not return until the node has been shutdown
  //ros::spin();

  // Successful exit
  return 0;
}
