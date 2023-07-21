// LAST UPDATE: 2023.07.21
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
  
  //// NUA TODO: GET INFO FROM TASK FILE AND INIT IN MobileManipulatorInterface!
  // Get points on robot parameters
  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nh_interface.hasParam("/collision_points")) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nh_interface.getParam("/collision_points", collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorNode::main] collision_points parameter is not of type array.");
    }
    
    //// NUA TODO: Get the point and radii info from task file! Also seperate base and arm!
    std::cout << "[MobileManipulatorNode::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorNode::main] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("[MobileManipulatorNode::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorNode::main] ERROR: collision_points is not defined!" << std::endl;
  }

  // Robot interface
  int initModelModeInt = 2;
  std::cout << "[MobileManipulatorNode::main] START interface" << std::endl;
  MobileManipulatorInterface m4_interface(nh_interface, taskFile, libFolder, urdfFile, pointsAndRadii, initModelModeInt);

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
