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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_ddp/GaussNewtonDDP.h>

#include "MPC_BASE.h"

namespace ocs2 {

/**
 * This is an MPC implementation with DDP (SLQ or ILQR) optimal control solvers.
 */
class MPC_DDP : public MPC_BASE {
 public:
  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDynamicsPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  MPC_DDP(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
          const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
          ddp::Settings ddpSettings, mpc::Settings mpcSettings, const CostFunctionBase* heuristicsFunctionPtr = nullptr);

  /** Default destructor. */
  ~MPC_DDP() override = default;

  GaussNewtonDDP* getSolverPtr() override { return ddpPtr_.get(); }

  const GaussNewtonDDP* getSolverPtr() const override { return ddpPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

 private:
  std::unique_ptr<GaussNewtonDDP> ddpPtr_;
};

}  // namespace ocs2