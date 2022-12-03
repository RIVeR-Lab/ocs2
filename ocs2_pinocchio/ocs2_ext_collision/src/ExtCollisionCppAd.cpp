/******************************************************************************
Copyright (c) 2020, Neset Unver Akmandor. All rights reserved.

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

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_ext_collision/ExtCollisionCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionCppAd::ExtCollisionCppAd(const PinocchioInterface& pinocchioInterface, 
                                     ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                                     std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                     scalar_t minimumDistance, 
                                     const std::string& modelName, 
                                     const std::string& modelFolder,
                                     bool recompileLibraries, 
                                     bool verbose)
  : extCollisionPinocchioGeometryInterface_(std::move(extCollisionPinocchioGeometryInterface)), pointsOnRobotPtr_(pointsOnRobotPtr) 
{
  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] START" << std::endl;

  PinocchioInterfaceCppAd pinocchioInterfaceAd = pinocchioInterface.toCppAd();

  setADInterfaces(pinocchioInterfaceAd, modelName, modelFolder);
  
  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] recompileLibraries: " << recompileLibraries << std::endl;

  if (recompileLibraries) 
  {
    cppAdInterface_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } 
  else 
  {
    cppAdInterface_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }

  /*
  if (recompileLibraries) 
  {
    cppAdInterfaceDistanceCalculation_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    cppAdInterfaceLinkPoints_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } 
  else 
  {
    cppAdInterfaceDistanceCalculation_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    cppAdInterfaceLinkPoints_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
  */

  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionCppAd::ExtCollisionCppAd(const ExtCollisionCppAd& rhs)
  : minimumDistance_(rhs.minimumDistance_),
    points_(rhs.points_),
    extCollisionPinocchioGeometryInterface_(rhs.extCollisionPinocchioGeometryInterface_),
    cppAdInterface_(new CppAdInterface(*rhs.cppAdInterface_)){}
    //cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
    //cppAdInterfaceLinkPoints_(new CppAdInterface(*rhs.cppAdInterfaceLinkPoints_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface) const 
{
  const std::vector<hpp::fcl::DistanceResult> distanceArray = extCollisionPinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                        const vector_t& q) const 
{
  /*
  const std::vector<hpp::fcl::DistanceResult> distanceArray = extCollisionPinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  vector_t pointsInWorldFrame(distanceArray.size() * numberOfParamsPerResult_);
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_) = distanceArray[i].nearest_points[0];
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3) = distanceArray[i].nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distanceArray[i].min_distance >= 0 ? 1.0 : -1.0;
  }
  */

  const auto f = cppAdInterface_ -> getFunctionValue(q);
  const auto dfdq = cppAdInterface_ -> getJacobian(q);

  //const auto pointsInLinkFrame = cppAdInterfaceLinkPoints_ -> getFunctionValue(q, pointsInWorldFrame);
  //const auto f = cppAdInterfaceDistanceCalculation_ -> getFunctionValue(q, pointsInLinkFrame);
  //const auto dfdq = cppAdInterfaceDistanceCalculation_ -> getJacobian(q, pointsInLinkFrame);

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
ad_vector_t ExtCollisionCppAd::computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                   const ad_vector_t& state,
                                                   const ad_vector_t& points) const 
{
  const auto& geometryModel = extCollisionPinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateGlobalPlacements(model, data);

  ad_vector_t pointsInLinkFrames = ad_vector_t::Zero(points.size());
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) 
  {
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    const auto joint1Position = data.oMi[joint1].translation();
    const auto joint1Orientation = matrixToQuaternion(data.oMi[joint1].rotation());
    const quaternion_t joint1OrientationInverse = joint1Orientation.conjugate();
    const vector3_t joint1PositionInverse = joint1OrientationInverse * -joint1Position;

    const auto joint2Position = data.oMi[joint2].translation();
    const auto joint2Orientation = matrixToQuaternion(data.oMi[joint2].rotation());
    const quaternion_t joint2OrientationInverse = joint2Orientation.conjugate();
    const vector3_t joint2PositionInverse = joint2OrientationInverse * -joint2Position;

    const ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    const ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_) = joint1PositionInverse;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_).noalias() += joint1OrientationInverse * point1;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_ + 3) = joint2PositionInverse;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_ + 3).noalias() += joint2OrientationInverse * point2;
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] =
        CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], ad_scalar_t(0.0), ad_scalar_t(1.0), ad_scalar_t(-1.0));
  }
  return pointsInLinkFrames;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
ad_vector_t ExtCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                     const ad_vector_t& state,
                                                     const ad_vector_t& points) const 
{
  const auto& geometryModel = extCollisionPinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateGlobalPlacements(model, data);

  ad_vector_t results = ad_vector_t::Zero(points.size() / numberOfParamsPerResult_);
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) 
  {
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    const ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    const auto joint1Position = data.oMi[joint1].translation();
    const auto joint1Orientation = matrixToQuaternion(data.oMi[joint1].rotation());
    const ad_vector_t point1InWorld = joint1Position + joint1Orientation * point1;

    const ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);
    const auto joint2Position = data.oMi[joint2].translation();
    const auto joint2Orientation = matrixToQuaternion(data.oMi[joint2].rotation());
    const ad_vector_t point2InWorld = joint2Position + joint2Orientation * point2;

    results[i] = points[i * numberOfParamsPerResult_ + 6] * (point2InWorld - point1InWorld).norm() - minimumDistance_;
  }
  return results;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ExtCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                        const std::string& modelName,
                                        const std::string& modelFolder) 
{
  std::cout << "[ExtCollisionCppAd::setADInterfaces] START" << std::endl;
  using ad_interface = ocs2::CppAdInterface;
  using ad_dynamic_vector_t = ad_interface::ad_vector_t;
  //using ad_scalar_t = ad_interface::ad_scalar_t;

  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;

  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixResult = computeState2MultiplePointsOnRobot(x, points_);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));

  /*
  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;
  const size_t numDistanceResults = extCollisionPinocchioGeometryInterface_.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };

  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, 
                                                              stateDim,
                                                              numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", 
                                                              modelFolder));

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = computeLinkPointsAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, 
                                                     stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, 
                                                     modelName + "_links_intermediate",
                                                     modelFolder));
  */
  std::cout << "[ExtCollisionCppAd::setADInterfaces] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<ad_scalar_t, 3, -1> ExtCollisionCppAd::computeState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, -1, 1>& state,  
                                                                                        const std::vector<std::vector<double>>& points) const 
{
  std::cout << "[ExtCollisionCppAd::computeState2MultiplePointsOnRobot] START" << std::endl;

  std::cout << "state size: " << state.size() << std::endl;


  int dim = 0;
  for (int i = 0; i < points.size(); i++) 
  {
    for (int j = 0; j < points[i].size(); j++) 
    {
      dim++;
    }
  }

  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }

  Eigen::Quaternion<ad_scalar_t> baseOrientation;
  baseOrientation.coeffs() = state.template head<7>().template head<4>();
  const Eigen::Matrix<ad_scalar_t, 3, 1>& basePosition = state.template head<7>().template tail<3>();
  const Eigen::Matrix<ad_scalar_t, 6, 1>& armState = state.template tail<6>();

  Eigen::Matrix<ad_scalar_t, 4, 4> worldXFrBase = Eigen::Matrix<ad_scalar_t, 4, 4>::Identity();
  worldXFrBase.template topLeftCorner<3, 3>() = baseOrientation.toRotationMatrix();
  worldXFrBase.template topRightCorner<3, 1>() = basePosition;

  Eigen::Matrix4d transformBase_X_ArmMount;
  Eigen::Matrix4d transformToolMount_X_Endeffector;

  std::cout << "[ExtCollisionCppAd::computeState2MultiplePointsOnRobot] END" << std::endl;

  return computeArmState2MultiplePointsOnRobot(armState, 
                                               points, 
                                               transformBase_X_ArmMount, 
                                               transformToolMount_X_Endeffector,
                                               worldXFrBase);
}

Eigen::Matrix<ad_scalar_t, 3, -1> ExtCollisionCppAd::computeArmState2MultiplePointsOnRobot(const Eigen::Matrix<ad_scalar_t, 6, 1>& state,  
                                                                                                              const std::vector<std::vector<double>>& points,
                                                                                                              const Eigen::Matrix4d& transformBase_X_ArmBase, 
                                                                                                              const Eigen::Matrix4d& transformToolMount_X_Endeffector,
                                                                                                              const Eigen::Matrix<ad_scalar_t, 4, 4>& transformWorld_X_Base) const
{
  std::cout << "[ExtCollisionCppAd::computeArmState2MultiplePointsOnRobot] START" << std::endl;

  int dim = 0;
  for (int i = 0; i < points.size(); i++) 
  {
    for (int j = 0; j < points[i].size(); j++) 
    {
      dim++;
    }
  }

  if (dim == 0) 
  {
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }

  Eigen::Matrix<ad_scalar_t, 3, -1> result(3, dim);
  int resultIndex = 0;
  int linkIndex;

  Eigen::Matrix<ad_scalar_t, 4, 4> transformWorld_X_Endeffector = transformWorld_X_Base;

  /*
  typedef typename iit::rbd::tpl::TraitSelector<ad_scalar_t>::Trait trait_t;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_arm_mount_X_fr_SHOULDER armMountXFrShoulderXFrShoulder;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_SHOULDER_X_fr_ARM shoulderXFrArm;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_ARM_X_fr_ELBOW armXFrElbow;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_ELBOW_X_fr_FOREARM elbowXFrForearm;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_FOREARM_X_fr_WRIST_1 forearmXFrWrist1;
  typename iit::mabi::tpl::HomogeneousTransforms<trait_t>::Type_fr_WRIST_1_X_fr_WRIST_2 wrist1XFrWrist2;
  */

  Eigen::Matrix<ad_scalar_t, 4, 4> nextStep = transformBase_X_ArmBase.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) 
  {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;


/*
  nextStep = armMountXFrShoulderXFrShoulder.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = shoulderXFrArm.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = armXFrElbow.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = elbowXFrForearm.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = forearmXFrWrist1.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = wrist1XFrWrist2.update(state);
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;

  nextStep = transformToolMount_X_Endeffector.cast<ad_scalar_t>();
  for (int i = 0; i < points[linkIndex].size(); i++) {
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = nextStep.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t)points[linkIndex][i];
    result.col(resultIndex++) = (transformWorld_X_Endeffector * directionVector).template head<3>();
  }
  linkIndex++;
  transformWorld_X_Endeffector = transformWorld_X_Endeffector * nextStep;
  */

  std::cout << "[ExtCollisionCppAd::computeArmState2MultiplePointsOnRobot] END" << std::endl;

  return result;
}

} /* namespace ocs2 */
