//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_ddp/ContinuousTimeLqr.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  const auto& settings = modelSettings();
  const auto& costWeights = costSettings();

  // nominal values
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation = weightCompensatingInputs(getComModel(), stanceFlags, switched_model::vector3_t::Zero());
  const auto jointTorquesForWeightCompensation = torqueApproximation(
      getJointPositions(getInitialState()), toArray<scalar_t>(uSystemForWeightCompensation.head<3 * NUM_CONTACT_POINTS>()), kinematicModel);

  problemPtr_->preComputationPtr = createPrecomputation();

  // Cost terms
  problemPtr_->costPtr->add("MotionTrackingCost", createMotionTrackingCost());
  problemPtr_->stateSoftConstraintPtr->add("FootPlacementCost", createFootPlacementCost());
  problemPtr_->stateSoftConstraintPtr->add("CollisionAvoidanceCost", createCollisionAvoidanceCost());
  problemPtr_->softConstraintPtr->add("JointLimitCost", createJointLimitsSoftConstraint());
  problemPtr_->softConstraintPtr->add("TorqueLimitCost", createTorqueLimitsSoftConstraint(jointTorquesForWeightCompensation));

  // Dynamics
  problemPtr_->dynamicsPtr = createDynamics();

  // Per leg terms
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto& footName = feetNames[i];
    problemPtr_->equalityConstraintPtr->add(footName + "_ZeroForce", createZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EENormal", createFootNormalConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_EEVel", createEndEffectorVelocityConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_FrictionCone", createFrictionConeCost(i));
  }

  // Initialize cost to be able to query it
  ocs2::TargetTrajectories targetTrajectories({0.0}, {getInitialState()}, {uSystemForWeightCompensation});
  problemPtr_->targetTrajectoriesPtr = &targetTrajectories;

  getSwitchedModelModeScheduleManagerPtr()->setTargetTrajectories(targetTrajectories);
  getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 1.0, getInitialState());
  auto lqrSolution = ocs2::continuous_time_lqr::solve(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);
  lqrSolution.valueFunction *= 10.0;
  std::unique_ptr<ocs2::StateCost> terminalCost(new ocs2::QuadraticStateCost(lqrSolution.valueFunction));
  problemPtr_->finalCostPtr->add("lqr_terminal_cost", std::move(terminalCost));

  // Store cost approximation at nominal state input
  nominalCostApproximation_ = ocs2::approximateCost(*problemPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);

  // Reset, the target trajectories pointed to are local
  problemPtr_->targetTrajectoriesPtr = nullptr;

  initializerPtr_.reset(new ComKinoInitializer(getComModel(), *getSwitchedModelModeScheduleManagerPtr(),
                                               *problemPtr_->equalityConstraintPtr, *problemPtr_->preComputationPtr));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings()));
}

}  // namespace switched_model
