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

#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RosReferenceManager::RosReferenceManager(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
  : ReferenceManagerDecorator(std::move(referenceManagerPtr)), topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe(ros::NodeHandle& nodeHandle) 
{
  // ModeSchedule
  auto modeScheduleCallback = [this](const ocs2_msgs::mode_schedule::ConstPtr& msg) 
  {
    auto modeSchedule = ros_msg_conversions::readModeScheduleMsg(*msg);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  };
  modeScheduleSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mode_schedule>(topicPrefix_ + "mode_schedule", 1, modeScheduleCallback);

  // TargetTrajectories
  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) 
  {
    //std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] START" << std::endl;

    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

    /*
    std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] timeTrajectory size: " << targetTrajectories.timeTrajectory.size() << std::endl;
    for (size_t i = 0; i < targetTrajectories.timeTrajectory.size(); i++)
    {
      std::cout << i << " -> " << targetTrajectories.timeTrajectory[i] << std::endl;
    }
    std::cout << "------------" << std::endl;

    std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] stateTrajectory size: " << targetTrajectories.stateTrajectory[0].size() << std::endl;
    for (size_t i = 0; i < targetTrajectories.stateTrajectory[0].size(); i++)
    {
      std::cout << i << " -> " << targetTrajectories.stateTrajectory[0][i] << std::endl;
    }
    std::cout << "------------" << std::endl;

    std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] inputTrajectory size: " << targetTrajectories.inputTrajectory[0].size() << std::endl;
    for (size_t i = 0; i < targetTrajectories.inputTrajectory[0].size(); i++)
    {
      std::cout << i << " -> " << targetTrajectories.inputTrajectory[0][i] << std::endl;
    }
    std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] targetTrajectories size: " << std::endl;
    */

    //targetTrajectories.stateTrajectory[0] << -1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 1.0;

    referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));

    //std::cout << "[RosReferenceManager::subscribe::targetTrajectoriesCallback] END" << std::endl;
    //std::cout << "" << std::endl;
  };
  targetTrajectoriesSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>(topicPrefix_ + "mpc_target", 1, targetTrajectoriesCallback);

  // ModelMode
  /*
  auto modelModeCallback = [this](const std_msgs::UInt8::ConstPtr& msg) 
  {
    std::cout << "[RosReferenceManager::subscribe::modelModeCallback] START" << std::endl;

    modelModeInt_ = msg->data;

    std::cout << "[RosReferenceManager::subscribe::modelModeCallback] modelModeInt: " << modelModeInt_ << std::endl;

    std::cout << "[RosReferenceManager::subscribe::modelModeCallback] END" << std::endl;
    std::cout << "" << std::endl;
  };
  modelModeSubscriber_ = nodeHandle.subscribe<std_msgs::UInt8>(topicPrefix_ + "_model_mode", 1, modelModeCallback);
  */
}

}  // namespace ocs2
