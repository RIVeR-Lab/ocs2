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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorPreComputation::MobileManipulatorPreComputation(PinocchioInterface pinocchioInterface, const ManipulatorModelInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorPreComputation* MobileManipulatorPreComputation::clone() const {
  return new MobileManipulatorPreComputation(pinocchioInterface_, pinocchioMapping_.getManipulatorModelInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) 
{
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) 
  {
    return;
  }

  auto t0_getModel = std::chrono::high_resolution_clock::now();
  const auto& model = pinocchioInterface_.getModel();
  auto t1_getModel = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration getModel: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_getModel - t0_getModel).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_getData = std::chrono::high_resolution_clock::now();
  auto& data = pinocchioInterface_.getData();
  auto t1_getData = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration getData: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_getData - t0_getData).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  auto t0_getpinmap = std::chrono::high_resolution_clock::now();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);
  auto t1_getpinmap = std::chrono::high_resolution_clock::now();

  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration getpinmap: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_getpinmap - t0_getpinmap).count() << std::endl;
  std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

  if (request.contains(Request::Approximation)) 
  {
    auto t0_forwardKinematics = std::chrono::high_resolution_clock::now();
    pinocchio::forwardKinematics(model, data, q);
    auto t1_forwardKinematics = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration forwardKinematics: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_forwardKinematics - t0_forwardKinematics).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

    auto t0_updateFramePlacements = std::chrono::high_resolution_clock::now();
    pinocchio::updateFramePlacements(model, data);
    auto t1_updateFramePlacements = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration updateFramePlacements: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_updateFramePlacements - t0_updateFramePlacements).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

    auto t0_computeJointJacobians = std::chrono::high_resolution_clock::now();
    pinocchio::computeJointJacobians(model, data);
    auto t1_computeJointJacobians = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration computeJointJacobians: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_computeJointJacobians - t0_computeJointJacobians).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

    auto t0_updateGlobalPlacements = std::chrono::high_resolution_clock::now();
    pinocchio::updateGlobalPlacements(model, data);
    auto t1_updateGlobalPlacements = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration updateGlobalPlacements: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_updateGlobalPlacements - t0_updateGlobalPlacements).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  } 
  else 
  {
    auto t0_forwardKinematics = std::chrono::high_resolution_clock::now();
    pinocchio::forwardKinematics(model, data, q);
    auto t1_forwardKinematics = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration forwardKinematics: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_forwardKinematics - t0_forwardKinematics).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;

    auto t0_updateFramePlacements = std::chrono::high_resolution_clock::now();
    pinocchio::updateFramePlacements(model, data);
    auto t1_updateFramePlacements = std::chrono::high_resolution_clock::now();

    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "[MobileManipulatorPreComputation::MobileManipulatorPreComputation] duration updateFramePlacements: " << std::chrono::duration_cast<std::chrono::microseconds>(t1_updateFramePlacements - t0_updateFramePlacements).count() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace mobile_manipulator
}  // namespace ocs2
