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

#include <algorithm>

#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_BASE::MPC_BASE(mpc::Settings mpcSettings) : mpcSettings_(std::move(mpcSettings)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::reset() 
{
  initRun_ = true;
  mpcTimer_.reset();
  getSolverPtr()->reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_BASE::run(scalar_t currentTime, const vector_t& currentState) 
{
  /*
  std::cout << "" << std::endl;
  std::cout << "MPC_BASE::run -> currentTime: " << currentTime << std::endl;
  std::cout << "MPC_BASE::run -> finalTime: " << getSolverPtr() -> getFinalTime() << std::endl;
  std::cout << "-------------------" << std::endl;
  std::cout << "MPC_BASE::run -> currentState size: " << currentState.size() << std::endl;
  std::cout << "MPC_BASE::run -> currentState: " << std::endl;
  std::cout << "MPC_BASE::run -> position x: " << currentState[0] << std::endl;
  std::cout << "MPC_BASE::run -> position y: " << currentState[1] << std::endl;
  std::cout << "MPC_BASE::run -> heading: " << currentState[2] << std::endl;
  std::cout << "MPC_BASE::run -> joint_head_pan: " << currentState[3] << std::endl;
  std::cout << "MPC_BASE::run -> joint_head_tilt: " << currentState[4] << std::endl;
  std::cout << "MPC_BASE::run -> joint_lift: " << currentState[5] << std::endl;
  std::cout << "MPC_BASE::run -> joint_arm_l3: " << currentState[6] << std::endl;
  std::cout << "MPC_BASE::run -> joint_arm_l2: " << currentState[7] << std::endl;
  std::cout << "MPC_BASE::run -> joint_arm_l1: " << currentState[8] << std::endl;
  std::cout << "MPC_BASE::run -> joint_arm_l0: " << currentState[9] << std::endl;
  std::cout << "MPC_BASE::run -> joint_wrist_yaw: " << currentState[10] << std::endl;
  std::cout << "" << std::endl;
  */


  // check if the current time exceeds the solver final limit
  if (!initRun_ && currentTime >= getSolverPtr() -> getFinalTime()) 
  {
    std::cerr << "MPC_BASE::run -> WARNING: The MPC time-horizon is smaller than the MPC starting time.\n";
    std::cerr << "MPC_BASE::run -> currentTime: " << currentTime << "\t Controller finalTime: " << getSolverPtr() -> getFinalTime() << '\n';
    return false;
  }

  const scalar_t finalTime = currentTime + mpcSettings_.timeHorizon_;

  // display
  if (mpcSettings_.debugPrint_) 
  {
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n### MPC is called at time:  " << currentTime << " [s].";
    std::cerr << "\n### MPC final Time:         " << finalTime << " [s].";
    std::cerr << "\n### MPC time horizon:       " << mpcSettings_.timeHorizon_ << " [s].\n";
    mpcTimer_.startTimer();
  }

  std::cout << "MPC_BASE::run -> BEFORE calculateController" << std::endl;
  // calculate the MPC policy
  calculateController(currentTime, currentState, finalTime);

  std::cout << "MPC_BASE::run -> AFTER calculateController" << std::endl;

  // set initRun flag to false
  initRun_ = false;

  // display
  if (mpcSettings_.debugPrint_) 
  {
    mpcTimer_.endTimer();
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  return true;
}

}  // namespace ocs2
