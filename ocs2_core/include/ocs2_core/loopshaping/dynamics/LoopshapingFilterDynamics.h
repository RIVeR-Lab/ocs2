/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <memory>

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include "ocs2_core/integration/Integrator.h"

namespace ocs2 {

class LoopshapingFilterDynamics {
 public:
  explicit LoopshapingFilterDynamics(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : loopshapingDefinition_(std::move(loopshapingDefinition)), integrator_(newIntegrator(IntegratorType::ODE45)) {
    filter_state_.setZero(loopshapingDefinition_->getInputFilter().getNumStates());
  }

  void integrate(scalar_t dt, const vector_t& input);

  void setFilterState(const vector_t& filter_state) { filter_state_ = filter_state; };
  const vector_t& getFilterState() const { return filter_state_; };

 private:
  vector_t computeFlowMap(scalar_t time, const vector_t& filter_state, const vector_t& input) const;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  vector_t filter_state_;
  std::unique_ptr<IntegratorBase> integrator_;
};

}  // namespace ocs2
