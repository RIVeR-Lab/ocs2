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

#include "ocs2_oc/synchronized_module/ReferenceManager.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ReferenceManager::ReferenceManager(TargetTrajectories initialTargetTrajectories, ModeSchedule initialModeSchedule)
  : targetTrajectories_(std::move(initialTargetTrajectories)), modeSchedule_(std::move(initialModeSchedule)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) 
{
  //std::cout << "[ReferenceManager::preSolverRun] START" << std::endl;

  //std::cout << "[ReferenceManager::preSolverRun] targetTrajectories_ size: " << targetTrajectories_.get().size() << std::endl;

  targetTrajectories_.updateFromBuffer();
  modeSchedule_.updateFromBuffer();

  // NUA NOTE: Unnecessary?
  //modifyReferences(initTime, finalTime, initState, targetTrajectories_.get(), modeSchedule_.get());

  //std::cout << "[ReferenceManager::preSolverRun] END" << std::endl;
}

}  // namespace ocs2
