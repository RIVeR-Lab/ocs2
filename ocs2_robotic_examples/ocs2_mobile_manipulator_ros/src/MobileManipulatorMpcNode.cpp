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
  std::cout << "[MobileManipulatorMpcNode::main] START" << std::endl;

  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
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
  if (nodeHandle.hasParam("/voxblox_node/collision_points")) 
  {
    using pair_t = std::pair<double, double>;
    XmlRpc::XmlRpcValue collisionPoints;
    nodeHandle.getParam("/voxblox_node/collision_points", collisionPoints);

    if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) 
    {
      ROS_WARN("[MobileManipulatorMpcNode::main] collision_points parameter is not of type array.");
    }
    
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

  //std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServerPtr;
  //esdfCachingServerPtr.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));

  // Robot interface
  std::cout << "" << std::endl;
  std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
  std::cout << "[MobileManipulatorMpcNode::main] START INIT MobileManipulatorInterface" << std::endl;
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile, pointsAndRadii);
  std::cout << "[MobileManipulatorMpcNode::main] END INIT MobileManipulatorInterface" << std::endl;
  std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
  std::cout << "" << std::endl;

  //Eigen::VectorXd state(9);
  //state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  //scalar_t maxDistance_ = 10;

  //voxblox::EsdfServer voxblox_server_(ros::NodeHandle(), ros::NodeHandle("~"));

  /*
  while(1)
  {
    /*
    //if (interface.getEsdfCachingServerPtr()) 
    if (esdfCachingServerPtr) 
    {
      std::cout << "[MobileManipulatorMpcNode::main] updateInterpolator" << std::endl;
      esdfCachingServerPtr -> updateInterpolator();
    }
    * /

    Eigen::Matrix<scalar_t, -1, 1> distances_(interface.getPointsOnRobotPtr()->getNumOfPoints());
    Eigen::MatrixXd gradientsVoxblox_(interface.getPointsOnRobotPtr()->getNumOfPoints(), interface.getPointsOnRobotPtr()->getNumOfPoints() * 3);
    Eigen::Matrix<scalar_t, -1, -1> gradients_(interface.getPointsOnRobotPtr()->getNumOfPoints(), 9);
    vector_t violations;

    if (interface.getPointsOnRobotPtr()) 
    {
      violations.resize(interface.getPointsOnRobotPtr()->getNumOfPoints());
      gradientsVoxblox_.setZero();

      //std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP" << std::endl;
      //while(1){;}
      
      Eigen::VectorXd positionPointsOnRobot = interface.getPointsOnRobotPtr()->getPointsPositionCppAd(state);
      Eigen::MatrixXd jacobianPointsOnRobot = interface.getPointsOnRobotPtr()->getPointsJacobianCppAd(state);
      Eigen::VectorXd radii = interface.getPointsOnRobotPtr()->getRadii();

      std::cout << "[MobileManipulatorMpcNode::main] positionPointsOnRobot:" << std::endl << positionPointsOnRobot << std::endl;
      //std::cout << "[MobileManipulatorMpcNode::main] jacobianPointsOnRobot:" << std::endl << jacobianPointsOnRobot << std::endl;
      std::cout << "[MobileManipulatorMpcNode::main] radii:" << std::endl << radii << std::endl;

      assert(positionPointsOnRobot.size() % 3 == 0);
      int numPoints = interface.getPointsOnRobotPtr()->getNumOfPoints();

      std::cout << "[MobileManipulatorMpcNode::main] numPoints: " << numPoints << std::endl;

      double distance = 0.0;
      Eigen::Vector3f gradientVoxblox;
      for (int i = 0; i < numPoints; i++)
      {
        Eigen::Ref<Eigen::Matrix<scalar_t, 3, 1>> position = positionPointsOnRobot.segment<3>(i * 3);
        
        std::cout << "[MobileManipulatorMpcNode::main] " << i << ": (" << position(0) << ", " << position(1) << ", " << position(2) << ") -> " << distance << std::endl;

        if (!voxblox_server_.getEsdfMapPtr()) 
        {
          std::cout << "[MobileManipulatorMpcNode::main] NO EsdfMapPtr!" << std::endl;
          distance = 0.0;
        }
        
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) 
        {
          distance = 0;
        }

        if (distance > 0)
        {
          std::cout << "[MobileManipulatorMpcNode::main] (" << position(0) << ", " << position(1) << ", " << position(2) << ") -> " << distance << std::endl;
          distances_[i] = distance - radii(i);
        }
        else
        {
          std::cout << "[MobileManipulatorMpcNode::main] maxDistance_" << std::endl;
          distances_[i] = maxDistance_ - radii(i);
        }

        /*
        if (esdfCachingServerPtr->getInterpolator()->getInterpolatedDistanceGradient(position.cast<float>(), &distance, &gradientVoxblox)) 
        {
          distances_[i] = distance - radii(i);

          //std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP" << std::endl;
          //while(1){ros::spinOnce();}

          gradientsVoxblox_.block<1, 3>(i, 3 * i) = gradientVoxblox.transpose().cast<double>();

          std::cout << "[MobileManipulatorMpcNode::main] (" << position(0) << ", " << position(1) << ", " << position(2) << ") -> " << distance << std::endl;
          std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP 1" << std::endl;
          //while(1){ros::spinOnce();}
        } 
        else 
        {
          std::cout << "[MobileManipulatorMpcNode::main] maxDistance_" << std::endl;
          std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP 2" << std::endl;
          //while(1){ros::spinOnce();}
          distances_[i] = maxDistance_ - radii(i);
        }
        * /
      }

      //std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP 3" << std::endl;
      //while(1){ros::spinOnce();}

      assert(gradients_.rows() == gradientsVoxblox_.rows());
      assert(gradients_.cols() == jacobianPointsOnRobot.cols());
      gradients_ = gradientsVoxblox_ * jacobianPointsOnRobot;
    }
    else
    {
      std::cout << "[MobileManipulatorMpcNode::main] No points on robot!" << std::endl;
      violations = vector_t::Zero(1);
    }
    std::cout << "" << std::endl;

    ros::spinOnce();
  }
  */

  // ROS ReferenceManager
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(new ocs2::RosReferenceManager(robotName, interface.getReferenceManagerPtr()));
  rosReferenceManagerPtr->subscribe(nodeHandle);

  //std::cout << "[MobileManipulatorMpcNode::main] BEFORE INF LOOP" << std::endl;
  //while(1){;}

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), 
                               interface.ddpSettings(), 
                               interface.getRollout(), 
                               interface.getOptimalControlProblem(), 
                               interface.getInitializer());

  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.setEsdfCachingServer(interface.getEsdfCachingServerPtr());
  mpcNode.launchNodes(nodeHandle);

  std::cout << "[MobileManipulatorMpcNode::main] END" << std::endl;
  
  // Successful exit
  return 0;
}
