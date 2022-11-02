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

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) 
{
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  auto t0_initnode = std::chrono::high_resolution_clock::now();
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;
  auto t1_initnode = std::chrono::high_resolution_clock::now();
  
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration initnode: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_initnode - t0_initnode).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // Get node parameters
  auto t0_getparam = std::chrono::high_resolution_clock::now();
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  auto t1_getparam = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration getparam: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_getparam - t0_getparam).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  
  // Robot interface
  auto t0_interface = std::chrono::high_resolution_clock::now();
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);
  auto t1_interface = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration interface: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_interface - t0_interface).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // ROS ReferenceManager
  auto t0_refmanager = std::chrono::high_resolution_clock::now();
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(new ocs2::RosReferenceManager(robotName, interface.getReferenceManagerPtr()));
  auto t1_refmanager = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration refmanager: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_refmanager - t0_refmanager).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  rosReferenceManagerPtr->subscribe(nodeHandle);
  
  // MPC
  auto t0_mpc = std::chrono::high_resolution_clock::now();
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), 
                               interface.ddpSettings(), 
                               interface.getRollout(),
                               interface.getOptimalControlProblem(), 
                               interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  auto t1_mpc = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration mpc: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_mpc - t0_mpc).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // Launch MPC ROS node
  auto t0_mpcnode = std::chrono::high_resolution_clock::now();
  MPC_ROS_Interface mpcNode(mpc, robotName);
  auto t1_mpcnode = std::chrono::high_resolution_clock::now();
  
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[mobile_manipulator_mpc_node::main] duration mpcnode: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_mpcnode - t0_mpcnode).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  mpcNode.launchNodes(nodeHandle);
  // Successful exit
  return 0;
}
