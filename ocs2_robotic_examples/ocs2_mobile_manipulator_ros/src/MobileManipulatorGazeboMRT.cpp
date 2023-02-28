/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

// External libraries:
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
//#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

// Custom libraries:
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Gazebo_Loop.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorGazeboVisualization.h>
//#include <ocs2_mobile_manipulator_interface.h>
//#include <ocs2_mobile_manipulator_visualization.h>
//#include <ocs2_mrt_loop.h>

using namespace ocs2;
using namespace mobile_manipulator;
using namespace std;

int main(int argc, char** argv) 
{
  std::cout << "[MobileManipulatorGazeboMRT::main] START" << std::endl;

  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;

  PointsOnRobot::points_radii_t pointsAndRadii(8);
  if (nodeHandle.hasParam("/collision_points")) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle.getParam("/collision_points", collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorGazeboMRT::main] collision_points parameter is not of type array.");
    }
    
    std::cout << "[MobileManipulatorGazeboMRT::main] pointsAndRadii:" << std::endl;
    for (int i = 0; i < collisionPoints.size(); i++) 
    {
      if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
      {
        ROS_WARN_STREAM("[MobileManipulatorGazeboMRT::main] collision_points[" << i << "] parameter is not of type array.");
      }

      for (int j = 0; j < collisionPoints[i].size(); j++) 
      {
        if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) 
        {
          ROS_WARN_STREAM("[MobileManipulatorGazeboMRT::main] collision_points[" << i << "][" << j << "] parameter is not of type array.");
        }

        if (collisionPoints[i][j].size() != 2) 
        {
          ROS_WARN_STREAM("[MobileManipulatorGazeboMRT::main] collision_points[" << i << "][" << j << "] does not have 2 elements.");
        }

        double segmentId = collisionPoints[i][j][0];
        double radius = collisionPoints[i][j][1];
        pointsAndRadii[i].push_back(pair_t(segmentId, radius));
        ROS_INFO_STREAM("[MobileManipulatorGazeboMRT::main] segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
      }
    }
  }
  else
  {
    std::cout << "[MobileManipulatorGazeboMRT::main] ERROR: collision_points is not defined!" << std::endl;
  }

  // Robot Interface
  std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile, pointsAndRadii);
  std::cout << "[MobileManipulatorGazeboMRT::main] AFTER MobileManipulatorInterface" << std::endl;

  // ManipulatorModelInfo
  ManipulatorModelInfo manipulatorModelInfo = interface.getManipulatorModelInfo();
  std::string baseFrameName = manipulatorModelInfo.baseFrame;
  if (modelTypeEnumToString(manipulatorModelInfo.manipulatorModelType) == "defaultManipulator")
  {
    baseFrameName = manipulatorModelInfo.armBaseFrame;
  }

  std::cout << "[MobileManipulatorGazeboMRT::main] manipulatorModelType: " << modelTypeEnumToString(manipulatorModelInfo.manipulatorModelType) << std::endl;
  std::cout << "[MobileManipulatorGazeboMRT::main] stateDim: " << manipulatorModelInfo.stateDim << std::endl;
  std::cout << "[MobileManipulatorGazeboMRT::main] inputDim: " << manipulatorModelInfo.inputDim << std::endl;
  std::cout << "[MobileManipulatorGazeboMRT::main] armDim: " << manipulatorModelInfo.armDim << std::endl;
  std::cout << "[MobileManipulatorGazeboMRT::main] dofNames: " << std::endl;
  for (int i = 0; i < manipulatorModelInfo.dofNames.size(); ++i)
  {
    std::cout << i << ": " << manipulatorModelInfo.dofNames[i] << std::endl;
  }

  size_t modalMode = 2;
  size_t baseStateDim = 3;
  size_t armStateDim = 6;

  // MRT
  MRT_ROS_Interface mrt(robotName, modalMode);
  mrt.setBaseStateDim(baseStateDim);
  mrt.setArmStateDim(armStateDim);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE ocs2_mm_visu" << std::endl;
  std::shared_ptr<OCS2_Mobile_Manipulator_Visualization> ocs2_mm_visu(new OCS2_Mobile_Manipulator_Visualization(nodeHandle, interface));
  std::cout << "[MobileManipulatorGazeboMRT::main] AFTER ocs2_mm_visu" << std::endl;

  // MRT loop
  MRT_ROS_Gazebo_Loop mrt_loop(nodeHandle, 
                               mrt, 
                               "world",
                               baseFrameName,
                               modelTypeEnumToString(manipulatorModelInfo.manipulatorModelType),
                               interface.getManipulatorModelInfo().stateDim, 
                               interface.getManipulatorModelInfo().inputDim, 
                               interface.getManipulatorModelInfo().dofNames, 
                               interface.mpcSettings().mrtDesiredFrequency_, 
                               interface.mpcSettings().mpcDesiredFrequency_);
  mrt_loop.subscribeObservers({ocs2_mm_visu});


  // initial state
  SystemObservation initObservation;
  //initObservation.state = interface.getInitialState();
  initObservation.state.setZero(interface.getManipulatorModelInfo().stateDim);
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
  initObservation.time = 0.0;

  std::cout << "[MobileManipulatorGazeboMRT::main] state size: " << initObservation.state.size() << std::endl;
  std::cout << "[MobileManipulatorGazeboMRT::main] input size: " << initObservation.input.size() << std::endl;
  

  // initial command
  vector_t initTarget(7);
  initTarget.head(3) << -0.2, 0, 0.8;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(0, 0, 1, 0).coeffs();
  const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  //std::cout << "[MobileManipulatorGazeboMRT::main] BEFORE INF" << std::endl;
  //while(1);

  // Run mrt_loop (loops while ros is ok)
  mrt_loop.run(initTargetTrajectories);
  //mrt_loop.runSinglePolicy(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
