// LAST UPDATE: 2022.07.13
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

// External libraries:
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorGazeboVisualization.h>

using namespace ocs2;
using namespace mobile_manipulator;
using namespace std;

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorGazeboMRT::main] START" << std::endl;

  std::string robotName = "jackal_ur5";
  std::string robotModelName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotModelName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Get node parameters
  string taskFile, libFolder, urdfFile;

  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);

  std::cout << "[MobileManipulatorMpcNode::main] Loading task file: " << taskFile << std::endl;
  std::cout << "[MobileManipulatorMpcNode::main] Loading library folder: " << libFolder << std::endl;
  std::cout << "[MobileManipulatorMpcNode::main] Loading urdf file: " << urdfFile << std::endl;

  // Robot Interface
  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface interface(nodeHandle, taskFile, libFolder, urdfFile);
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER MobileManipulatorInterface" << std::endl;

  interface.runMRT();

  /*
  interface.setMPCProblem(modelModeInt, pointsAndRadii);

  auto robotModelInfo = interface.getRobotModelInfo();
  printRobotModelInfo(robotModelInfo);

  // MRT
  MRT_ROS_Interface mrt(robotModelInfo, robotModelName);
  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE initRollout" << std::endl;
  mrt.initRollout(&interface.getRollout());
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER initRollout" << std::endl;

  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE launchNodes" << std::endl;
  mrt.launchNodes(nodeHandle);
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER launchNodes" << std::endl;

  // Visualization
  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE ocs2_mm_visu" << std::endl;
  //std::shared_ptr<MobileManipulatorVisualization> ocs2_mm_visu(new MobileManipulatorVisualization(nodeHandle, interface));
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER ocs2_mm_visu" << std::endl;

  // MRT loop
  std::string worldFrameName = "world";
  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE mrt_loop" << std::endl;
  MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle, 
                              mrt, 
                              worldFrameName,
                              interface.getBaseStateMsg(),
                              interface.getArmStateMsg(),
                              interface.getBaseControlMsg(),
                              interface.getArmControlMsg(),
                              interface.mpcSettings().mrtDesiredFrequency_, 
                              interface.mpcSettings().mpcDesiredFrequency_);
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER mrt_loop" << std::endl;

  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE subscribeObservers" << std::endl;
  //mrt_loop.subscribeObservers({ocs2_mm_visu});
  //std::cout << "[MobileManipulatorGazeboMRT::main] AFTER subscribeObservers" << std::endl;

  // initial command
  vector_t initTarget(7);
  //initTarget.head(3) << -0.2, 0, 1.0;
  //initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  mrt_loop.getInitTarget(initTarget);   

  // Run mrt_loop
  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE run" << std::endl;
  mrt_loop.run(initTarget);

  modelModeInt = mrt.getModelModeInt();
  */

  /*
  size_t modelModeInt = 2;
  int iter = 0;
  while (ros::ok() && ros::master::check())
  {
    std::cout << "=====================================================" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "[MobileManipulatorGazeboMRT::main] START ITERATION: " << iter << std::endl;
    std::cout << "[MobileManipulatorGazeboMRT::main] modelModeInt: " << modelModeInt << std::endl;

    interface.runMRT(modelModeInt);

    iter++;
    std::cout << "[MobileManipulatorGazeboMRT::main] END ITERATION: " << iter << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }
  */

  // Successful exit
  return 0;
}
