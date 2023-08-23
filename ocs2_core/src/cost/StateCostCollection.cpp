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

#include <ocs2_core/cost/StateCostCollection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection::StateCostCollection(const StateCostCollection& other) : Collection<StateCost>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection* StateCostCollection::clone() const 
{
  return new StateCostCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateCostCollection::getValue(scalar_t time, 
                                       const vector_t& state, 
                                       const TargetTrajectories& targetTrajectories,
                                       const PreComputation& preComp) const 
{
  //std::cout << "[StateCostCollection::getValue(4)] START" << std::endl;

  scalar_t cost = 0.0;

  // accumulate cost terms
  for (const auto& costTerm : this->terms_) 
  {
    if (costTerm->isActive(time)) 
    {
      std::cout << "[StateCostCollection::getValue(4)] DEBUG INF" << std::endl;
      while(1);
      
      cost += costTerm->getValue(time, state, targetTrajectories, preComp);
    }
  }

  //std::cout << "[StateCostCollection::getValue(4)] END" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateCostCollection::getValue(scalar_t time, 
                                       const vector_t& state, 
                                       const vector_t& fullState, 
                                       const TargetTrajectories& targetTrajectories,
                                       const PreComputation& preComp) const 
{
  //std::cout << "[StateCostCollection::getValue(5)] START" << std::endl;
  scalar_t cost = 0.0;

  //std::cout << "[StateCostCollection::getValue(5)] this->terms_.size: " << this->terms_.size() << std::endl;

  // accumulate cost terms
  for (const auto& costTerm : this->terms_) 
  {
    if (costTerm->isActive(time)) 
    {
      //std::cout << "[StateCostCollection::getValue(5)] state size: " << state.size() << std::endl;
      //std::cout << "[StateCostCollection::getValue(5)] fullState size: " << fullState.size() << std::endl;
      cost += costTerm->getValue(time, state, fullState, targetTrajectories, preComp);
    }
  }

  //std::cout << "[StateCostCollection::getValue(5)] END" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateCostCollection::getQuadraticApproximation(scalar_t time, 
                                                                                    const vector_t& state,
                                                                                    const TargetTrajectories& targetTrajectories,
                                                                                    const PreComputation& preComp) const 
{
  std::cout << "[StateCostCollection::getQuadraticApproximation(4)] START" << std::endl;

  std::cout << "[StateCostCollection::getQuadraticApproximation(4)] DEBUG INF" << std::endl;
  while(1);

  const auto firstActive = std::find_if(terms_.begin(), terms_.end(), [time](const std::shared_ptr<StateCost>& costTerm) { return costTerm->isActive(time); });

  // No active terms (or terms is empty).
  if (firstActive == terms_.end()) 
  {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows());
  }

  // Initialize with first active term, accumulate potentially other active terms.
  auto cost = (*firstActive)->getQuadraticApproximation(time, state, targetTrajectories, preComp);
  std::for_each(std::next(firstActive), terms_.end(), [&](const std::shared_ptr<StateCost>& costTerm) 
  {
    if (costTerm->isActive(time)) 
    {
      const auto costTermApproximation = costTerm->getQuadraticApproximation(time, state, targetTrajectories, preComp);
      cost.f += costTermApproximation.f;
      cost.dfdx += costTermApproximation.dfdx;
      cost.dfdxx += costTermApproximation.dfdxx;
    }
  });

  // Make sure that input derivatives are empty
  cost.dfdu = vector_t();
  cost.dfduu = matrix_t();
  cost.dfdux = matrix_t();

  std::cout << "[StateCostCollection::getQuadraticApproximation(4)] END" << std::endl;

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateCostCollection::getQuadraticApproximation(scalar_t time, 
                                                                                    const vector_t& state,
                                                                                    const vector_t& fullState,
                                                                                    const TargetTrajectories& targetTrajectories,
                                                                                    const PreComputation& preComp) const 
{
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] START" << std::endl;

  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] terms_ size: " << terms_.size() << std::endl;

  const auto firstActive = std::find_if(terms_.begin(), terms_.end(), [time](const std::shared_ptr<StateCost>& costTerm) 
  { 
    return costTerm->isActive(time); 
  });

  // No active terms (or terms is empty).
  if (firstActive == terms_.end()) 
  {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows());
  }

  int ctr = 0;
  int ctr2 = 0;
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] START firstActive getQuadraticApproximation" << std::endl;
  // Initialize with first active term, accumulate potentially other active terms.
  auto cost = (*firstActive)->getQuadraticApproximation(time, state, fullState, targetTrajectories, preComp);
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] END firstActive getQuadraticApproximation" << std::endl;

  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] cost.f: " << cost.f << std::endl;

  std::for_each(std::next(firstActive), terms_.end(), [&](const std::shared_ptr<StateCost>& costTerm) 
  {
    //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] ctr2: " << ctr2 << std::endl;
    if (costTerm->isActive(time)) 
    {
      //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] ctr: " << ctr << std::endl;
      //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] START getQuadraticApproximation" << std::endl;
      const auto costTermApproximation = costTerm->getQuadraticApproximation(time, state, fullState, targetTrajectories, preComp);
      //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] END getQuadraticApproximation" << std::endl;

      cost.f += costTermApproximation.f;
      cost.dfdx += costTermApproximation.dfdx;
      cost.dfdxx += costTermApproximation.dfdxx;

      //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] " <<  ctr << " -> costTermApproximation.f: " << costTermApproximation.f << std::endl;
      //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] " <<  ctr << " -> cost.f: " << cost.f << std::endl;
      ctr++;
    }
    ctr2++;
  });
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] ctr: " << ctr << std::endl;
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] ctr2: " << ctr2 << std::endl;
  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] END getQuadraticApproximation" << std::endl << std::endl;

  // Make sure that input derivatives are empty
  cost.dfdu = vector_t();
  cost.dfduu = matrix_t();
  cost.dfdux = matrix_t();

  //std::cout << "[StateCostCollection::getQuadraticApproximation(5)] END" << std::endl;

  return cost;
}

}  // namespace ocs2
