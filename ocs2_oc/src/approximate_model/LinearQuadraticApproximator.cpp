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

#include "ocs2_oc/approximate_model/LinearQuadraticApproximator.h"

#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void approximateIntermediateLQ(OptimalControlProblem& problem, 
                               const scalar_t time, 
                               const vector_t& state, 
                               const vector_t& input,
                               const MultiplierCollection& multipliers, 
                               ModelData& modelData) 
{
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] START" << std::endl;

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
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] START Cost" << std::endl;
  modelData.cost = ocs2::approximateCost(problem, time, state, input);
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] END Cost" << std::endl;

  // Equality constraints
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] START Equality constraints" << std::endl;
  modelData.stateEqConstraint = problem.stateEqualityConstraintPtr->getLinearApproximation(time, state, preComputation);
  modelData.stateInputEqConstraint = problem.equalityConstraintPtr->getLinearApproximation(time, state, input, preComputation);
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] END Equality constraints" << std::endl;

  // Lagrangians
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] START Lagrangians" << std::endl;
  if (!problem.stateEqualityLagrangianPtr->empty()) 
  {
    //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] stateEqualityLagrangianPtr" << std::endl;
    auto approx = problem.stateEqualityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateEq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }
  
  if (!problem.stateInequalityLagrangianPtr->empty()) 
  {
    //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] stateInequalityLagrangianPtr" << std::endl;
    auto approx = problem.stateInequalityLagrangianPtr->getQuadraticApproximation(time, state, multipliers.stateIneq, preComputation);
    modelData.cost.f += approx.f;
    modelData.cost.dfdx += approx.dfdx;
    modelData.cost.dfdxx += approx.dfdxx;
  }

  if (!problem.equalityLagrangianPtr->empty()) 
  {
    //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] equalityLagrangianPtr" << std::endl;
    modelData.cost += problem.equalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputEq, preComputation);
  }

  if (!problem.inequalityLagrangianPtr->empty()) 
  {
    //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] inequalityLagrangianPtr" << std::endl;
    modelData.cost += problem.inequalityLagrangianPtr->getQuadraticApproximation(time, state, input, multipliers.stateInputIneq, preComputation);
  }
  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] END Lagrangians" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::approximateIntermediateLQ] END" << std::endl << std::endl;
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
  //std::cout << "[LinearQuadraticApproximator::computeCost(4)] START" << std::endl;

  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // Compute and sum all costs
  auto cost = problem.costPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.softConstraintPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.stateCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.stateSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  //std::cout << "[LinearQuadraticApproximator::computeCost(4)] END" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeCost(const OptimalControlProblem& problem, 
                     const scalar_t& time, 
                     const vector_t& state, 
                     const vector_t& full_state, 
                     const vector_t& input) 
{
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] START" << std::endl;

  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // Compute and sum all costs
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] START costPtr" << std::endl;
  auto cost = problem.costPtr->getValue(time, state, input, targetTrajectories, preComputation);
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] END costPtr" << std::endl;
  
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] START softConstraintPtr" << std::endl;
  cost += problem.softConstraintPtr->getValue(time, state, input, targetTrajectories, preComputation);
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] END softConstraintPtr" << std::endl;

  std::cout << "[LinearQuadraticApproximator::computeCost(5)] START stateCostPtr" << std::endl;
  cost += problem.stateCostPtr->getValue(time, state, targetTrajectories, preComputation);
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] END stateCostPtr" << std::endl;

  std::cout << "[LinearQuadraticApproximator::computeCost(5)] START stateSoftConstraintPtr" << std::endl;
  cost += problem.stateSoftConstraintPtr->getValue(time, state, full_state, targetTrajectories, preComputation);
  std::cout << "[LinearQuadraticApproximator::computeCost(5)] END stateSoftConstraintPtr" << std::endl;

  std::cout << "[LinearQuadraticApproximator::computeCost(5)] END" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateCost(const OptimalControlProblem& problem, 
                                                     const scalar_t& time, 
                                                     const vector_t& state,
                                                     const vector_t& input) 
{
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // get the state-input cost approximations
  auto cost = problem.costPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);

  if (!problem.softConstraintPtr->empty()) 
  {
    cost += problem.softConstraintPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);
  }

  // get the state only cost approximations
  if (!problem.stateCostPtr->empty()) 
  {
    auto stateCost = problem.stateCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  if (!problem.stateSoftConstraintPtr->empty()) 
  {
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
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START" << std::endl;
  auto& preComputation = *problem.preComputationPtr;

  MetricsCollection metrics;

  // Cost
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START computeCost" << std::endl;
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] state size: " << state.size() << std::endl;
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] input size: " << input.size() << std::endl;
  metrics.cost = computeCost(problem, time, state, input);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END computeCost" << std::endl;

  // Equality constraints
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START stateEqualityConstraintPtr" << std::endl;
  metrics.stateEqConstraint = problem.stateEqualityConstraintPtr->getValue(time, state, preComputation);
  metrics.stateInputEqConstraint = problem.equalityConstraintPtr->getValue(time, state, input, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END stateEqualityConstraintPtr" << std::endl;

  // Lagrangians
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START stateEqLagrangian" << std::endl;
  metrics.stateEqLagrangian = problem.stateEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END stateEqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START stateIneqLagrangian" << std::endl;
  metrics.stateIneqLagrangian = problem.stateInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END stateIneqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START stateInputEqLagrangian" << std::endl;
  metrics.stateInputEqLagrangian = problem.equalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputEq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END stateInputEqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] START stateInputIneqLagrangian" << std::endl;
  metrics.stateInputIneqLagrangian = problem.inequalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputIneq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END stateInputIneqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(5)] END" << std::endl;

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MetricsCollection computeIntermediateMetrics(OptimalControlProblem& problem, 
                                             const scalar_t time, 
                                             const vector_t& state,
                                             const vector_t& full_state,
                                             const vector_t& input, 
                                             const MultiplierCollection& multipliers) 
{
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START" << std::endl;
  auto& preComputation = *problem.preComputationPtr;

  MetricsCollection metrics;

  // Cost
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START computeCost" << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] state size: " << state.size() << std::endl;
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] input size: " << input.size() << std::endl;
  metrics.cost = computeCost(problem, time, state, full_state, input);
  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END computeCost" << std::endl;

  // Equality constraints
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START stateEqualityConstraintPtr" << std::endl;
  metrics.stateEqConstraint = problem.stateEqualityConstraintPtr->getValue(time, state, preComputation);
  metrics.stateInputEqConstraint = problem.equalityConstraintPtr->getValue(time, state, input, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END stateEqualityConstraintPtr" << std::endl;

  // Lagrangians
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START stateEqLagrangian" << std::endl;
  metrics.stateEqLagrangian = problem.stateEqualityLagrangianPtr->getValue(time, state, multipliers.stateEq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END stateEqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START stateIneqLagrangian" << std::endl;
  metrics.stateIneqLagrangian = problem.stateInequalityLagrangianPtr->getValue(time, state, multipliers.stateIneq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END stateIneqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START stateInputEqLagrangian" << std::endl;
  metrics.stateInputEqLagrangian = problem.equalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputEq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END stateInputEqLagrangian" << std::endl;

  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] START stateInputIneqLagrangian" << std::endl;
  metrics.stateInputIneqLagrangian = problem.inequalityLagrangianPtr->getValue(time, state, input, multipliers.stateInputIneq, preComputation);
  //std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END stateInputIneqLagrangian" << std::endl;

  std::cout << "[LinearQuadraticApproximator::computeIntermediateMetrics(6)] END" << std::endl;

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
