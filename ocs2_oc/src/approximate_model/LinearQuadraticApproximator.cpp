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

#include <iostream>
#include <chrono>

#include "ocs2_oc/approximate_model/LinearQuadraticApproximator.h"

#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void approximateIntermediateLQ(OptimalControlProblem& problem, const scalar_t time, const vector_t& state, const vector_t& input,
                               const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  preComputation.request(request, time, state, input);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = input.rows();
  modelData.dynamicsBias.setZero(state.rows());

  // Dynamics
  modelData.dynamicsCovariance = problem.dynamicsPtr->dynamicsCovariance(time, state, input);
  modelData.dynamics = problem.dynamicsPtr->linearApproximation(time, state, input, preComputation);

  // Cost
  modelData.cost = ocs2::approximateCost(problem, time, state, input);

  // Equality constraints
  modelData.stateEqConstraint = problem.stateEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);
  modelData.stateInputEqConstraint = problem.equalityConstraintPtr->getLinearApproximation(time, state, input, preComputation);

  // Lagrangians
  if (!problem.stateEqualityLagrangianPtr->empty()) {
    auto approx = problem.stateEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.stateInequalityLagrangianPtr->empty()) {
    auto approx = problem.stateInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.equalityLagrangianPtr->empty()) {
    modelData.cost +=
        problem.equalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputEq, preComputation);
  }
  if (!problem.inequalityLagrangianPtr->empty()) {
    modelData.cost +=
        problem.inequalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputIneq, preComputation);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void approximatePreJumpLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                          const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  preComputation.requestPreJump(request, time, state);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = 0;
  modelData.dynamicsBias.setZero(state.rows());

  // Jump map
  modelData.dynamics = problem.dynamicsPtr->jumpMapLinearApproximation(time, state, preComputation);

  // Pre-jump cost
  modelData.cost = approximateEventCost(problem, time, state);

  // state equality constraint
  modelData.stateEqConstraint = problem.preJumpEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);

  // Lagrangians
  if (!problem.preJumpEqualityLagrangianPtr->empty()) {
    auto approx = problem.preJumpEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.preJumpInequalityLagrangianPtr->empty()) {
    auto approx = problem.preJumpInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void approximateFinalLQ(OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                        const MultiplierCollection& multipliers, ModelData& modelData) {
  auto& preComputation = *problem.preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  preComputation.requestFinal(request, time, state);

  modelData.time = time;
  modelData.stateDim = state.rows();
  modelData.inputDim = 0;
  modelData.dynamicsBias = vector_t();

  // Dynamics
  modelData.dynamics = VectorFunctionLinearApproximation();

  // state equality constraint
  modelData.stateEqConstraint = problem.finalEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);

  // Final cost
  modelData.cost = approximateFinalCost(problem, time, state);

  // Lagrangians
  if (!problem.finalEqualityLagrangianPtr->empty()) {
    auto approx = problem.finalEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  if (!problem.finalInequalityLagrangianPtr->empty()) {
    auto approx = problem.finalInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state, const vector_t& input) 
{
  auto t0_settrajprecomp = std::chrono::high_resolution_clock::now();
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;
  auto t1_settrajprecomp = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeCost] duration set: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_settrajprecomp - t0_settrajprecomp).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // Compute and sum all costs
  auto t0_costPtr = std::chrono::high_resolution_clock::now();
  auto cost = problem.costPtr->getValue(time, state, input, targetTrajectories, preComputation);
  auto t1_costPtr = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeCost] duration costPtr: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_costPtr - t0_costPtr).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_softConstraintPtr = std::chrono::high_resolution_clock::now();
  cost += problem.softConstraintPtr->getValue(time, state, input, targetTrajectories, preComputation);
  auto t1_softConstraintPtr = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeCost] duration softConstraintPtr: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_softConstraintPtr - t0_softConstraintPtr).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateCostPtr = std::chrono::high_resolution_clock::now();
  cost += problem.stateCostPtr->getValue(time, state, targetTrajectories, preComputation);
  auto t1_stateCostPtr = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeCost] duration stateCostPtr: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateCostPtr - t0_stateCostPtr).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateSoftConstraintPtr = std::chrono::high_resolution_clock::now();
  cost += problem.stateSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);
  auto t1_stateSoftConstraintPtr = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeCost] duration stateSoftConstraintPtr: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateSoftConstraintPtr - t0_stateSoftConstraintPtr).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                                     const vector_t& input) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // get the state-input cost approximations
  auto cost = problem.costPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);

  if (!problem.softConstraintPtr->empty()) {
    cost += problem.softConstraintPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);
  }

  // get the state only cost approximations
  if (!problem.stateCostPtr->empty()) {
    auto stateCost = problem.stateCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  if (!problem.stateSoftConstraintPtr->empty()) {
    auto stateCost = problem.stateSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeEventCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.preJumpSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateEventCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.preJumpSoftConstraintPtr->empty()) {
    cost += problem.preJumpSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeFinalCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.finalSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateFinalCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.finalSoftConstraintPtr->empty()) {
    cost += problem.finalSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MetricsCollection computeIntermediateMetrics(OptimalControlProblem& problem, 
                                             const scalar_t time, 
                                             const vector_t& state,
                                             const vector_t& input, 
                                             const MultiplierCollection& multipliers) 
{
  auto& preComputation = *problem.preComputationPtr;

  MetricsCollection metrics;

  // Cost
  auto t0_computeCost = std::chrono::high_resolution_clock::now();
  metrics.cost = computeCost(problem, time, state, input);
  auto t1_computeCost = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration computeCost: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_computeCost - t0_computeCost).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // Equality constraints
  auto t0_stateEqConstraint = std::chrono::high_resolution_clock::now();
  metrics.stateEqConstraint = problem.stateEqualityConstraintPtr->getValue(time, state, preComputation);
  auto t1_stateEqConstraint = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateEqConstraint: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateEqConstraint - t0_stateEqConstraint).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateInputEqConstraint = std::chrono::high_resolution_clock::now();
  metrics.stateInputEqConstraint = problem.equalityConstraintPtr->getValue(time, state, input, preComputation);
  auto t1_stateInputEqConstraint = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateInputEqConstraint: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateInputEqConstraint - t0_stateInputEqConstraint).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  // Lagrangians
  auto t0_stateEqLagrangian = std::chrono::high_resolution_clock::now();
  metrics.stateEqLagrangian = problem.stateEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  auto t1_stateEqLagrangian = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateEqLagrangian: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateEqLagrangian - t0_stateEqLagrangian).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateIneqLagrangian = std::chrono::high_resolution_clock::now();
  metrics.stateIneqLagrangian = problem.stateInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);
  auto t1_stateIneqLagrangian = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateIneqLagrangian: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateIneqLagrangian - t0_stateIneqLagrangian).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateInputEqLagrangian = std::chrono::high_resolution_clock::now();
  metrics.stateInputEqLagrangian = problem.equalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputEq, preComputation);
  auto t1_stateInputEqLagrangian = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateInputEqLagrangian: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateInputEqLagrangian - t0_stateInputEqLagrangian).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_stateInputIneqLagrangian = std::chrono::high_resolution_clock::now();
  metrics.stateInputIneqLagrangian = problem.inequalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputIneq, preComputation);
  auto t1_stateInputIneqLagrangian = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics] duration stateInputIneqLagrangian: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_stateInputIneqLagrangian - t0_stateInputIneqLagrangian).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MetricsCollection computePreJumpMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                                        const MultiplierCollection& multipliers) {
  auto& preComputation = *problem.preComputationPtr;

  MetricsCollection metrics;

  // Cost
  metrics.cost = computeEventCost(problem, time, state);

  // Equality constraint
  metrics.stateEqConstraint = problem.preJumpEqualityConstraintPtr->getValue(time, state, preComputation);

  // Lagrangians
  metrics.stateEqLagrangian = problem.preJumpEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  metrics.stateIneqLagrangian = problem.preJumpInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MetricsCollection computeFinalMetrics(OptimalControlProblem& problem, const scalar_t time, const vector_t& state,
                                      const MultiplierCollection& multipliers) {
  auto& preComputation = *problem.preComputationPtr;

  MetricsCollection metrics;

  // Cost
  metrics.cost = computeFinalCost(problem, time, state);

  // Equality constraint
  metrics.stateEqConstraint = problem.finalEqualityConstraintPtr->getValue(time, state, preComputation);

  // Lagrangians
  metrics.stateEqLagrangian = problem.finalEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  metrics.stateIneqLagrangian = problem.finalInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);

  return metrics;
}

}  // namespace ocs2
