// LAST UPDATE: 2022.05.02
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
// [1] https://github.com/leggedrobotics/ocs2

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_self_collision/SelfCollisionCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(const PinocchioInterface& pinocchioInterface, 
                                       PinocchioGeometryInterface pinocchioGeometryInterface,
                                       const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                       RobotModelInfo& robotModelInfo,
                                       scalar_t minimumDistance, 
                                       const std::string& modelName, 
                                       const std::string& modelFolder,
                                       bool recompileLibraries, 
                                       bool verbose)
  : pinocchioGeometryInterface_(std::move(pinocchioGeometryInterface)), 
    robotModelInfo_(robotModelInfo),
    minimumDistance_(minimumDistance) 
{
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  setADInterfaces(pinocchioInterfaceCppAd, *mappingCppAdPtr, modelName, modelFolder);
  
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
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(const SelfCollisionCppAd& rhs)
    : pinocchioGeometryInterface_(rhs.pinocchioGeometryInterface_),
      cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
      cppAdInterfaceLinkPoints_(new CppAdInterface(*rhs.cppAdInterfaceLinkPoints_)),
      robotModelInfo_(rhs.robotModelInfo_),
      minimumDistance_(rhs.minimumDistance_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface) const 
{
  //std::cout << "[SelfCollisionCppAd::getValue] START" << std::endl;

  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  //std::cout << "[SelfCollisionCppAd::getValue] distanceArray" << std::endl;
  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;

    /*
    if(violations[i] > 1 || distanceArray[i].min_distance < 0)
    {
      std::cout << "[SelfCollisionCppAd::getValue] violations[i]: " << violations[i] << std::endl;
      std::cout << "[SelfCollisionCppAd::getValue] distanceArray[i].min_distance: " << distanceArray[i].min_distance << std::endl;
      std::cout << "[SelfCollisionCppAd::getValue] DEBUG INF" << std::endl;
      while(1);
    }

    if (bench_min_dist_ > violations[i])
    {
      bench_min_dist_ = violations[i];
    }

    if (bench_max_dist_ < violations[i])
    {
      bench_max_dist_ = violations[i];
    }
    */

    //std::cout << "[SelfCollisionCppAd::getValue] distanceArray[i].min_distance:" << i << " -> " << distanceArray[i].min_distance << std::endl;
    //std::cout << "[SelfCollisionCppAd::getValue] minimumDistance_:" << minimumDistance_ << std::endl;
    //std::cout << "[SelfCollisionCppAd::getValue] violations:" << i << " -> " << violations[i] << std::endl;
  }

  //std::cout << "[SelfCollisionCppAd::getValue] bench_min_dist_: " << bench_min_dist_ << std::endl;
  //std::cout << "[SelfCollisionCppAd::getValue] bench_max_dist_: " << bench_max_dist_ << std::endl;
  //std::cout << "[SelfCollisionCppAd::getValue] END" << std::endl;

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                         const vector_t& state) const 
{
  std::cout << "[SelfCollisionCppAd::getLinearApproximation(2)] START" << std::endl;

  std::cout << "[SelfCollisionCppAd::getLinearApproximation(2)] DEBUG INF" << std::endl;
  while(1);

  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] distanceArray.size(): " << distanceArray.size() << std::endl;

  vector_t pointsInWorldFrame(distanceArray.size() * numberOfParamsPerResult_);
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_) = distanceArray[i].nearest_points[0];
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3) = distanceArray[i].nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distanceArray[i].min_distance >= 0 ? 1.0 : -1.0;
  }

  const auto pointsInLinkFrame = cppAdInterfaceLinkPoints_->getFunctionValue(state, pointsInWorldFrame);
  const auto f = cppAdInterfaceDistanceCalculation_->getFunctionValue(state, pointsInLinkFrame);
  const auto dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(state, pointsInLinkFrame);

  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] pointsInWorldFrame size: " << pointsInWorldFrame.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] pointsInLinkFrame size: " << pointsInLinkFrame.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] f size: " << f.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq rows: " << dfdq.rows() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq cols: " << dfdq.cols() << std::endl;

  /*
  std::cout << "[SelfCollisionCppAd::getLinearApproximation] f: " << std::endl;
  for (size_t i = 0; i < f.size(); i++)
  {
    std::cout << i << " -> " << f(i) << std::endl;
  }

  std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq: " << std::endl;
  for (size_t i = 0; i < dfdq.rows(); i++)
  {
    std::cout << i << " -> ";
    for (size_t j = 0; j < dfdq.cols(); j++)
    {
      std::cout << f(i) << " ";
    }
    std::cout << "" << std::endl;
  }
  */

  std::cout << "[SelfCollisionCppAd::getLinearApproximation(2)] END" << std::endl << std::endl;

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                         const vector_t& state,
                                                                         const vector_t& fullState) const 
{
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation(3)] START" << std::endl;

  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  auto n_points_param = distanceArray.size() * numberOfParamsPerResult_;
  auto stateDim = fullState.size();

  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] distanceArray.size(): " << distanceArray.size() << std::endl;

  vector_t pointsInWorldFrame(n_points_param);
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_) = distanceArray[i].nearest_points[0];
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3) = distanceArray[i].nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distanceArray[i].min_distance >= 0 ? 1.0 : -1.0;
  }

  vector_t params(stateDim + n_points_param);
  params.head(stateDim) = fullState;
  params.tail(n_points_param) = pointsInWorldFrame;

  const auto pointsInLinkFrame = cppAdInterfaceLinkPoints_->getFunctionValue(state, params);
  const auto f = cppAdInterfaceDistanceCalculation_->getFunctionValue(state, params);
  const auto dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(state, params);

  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] pointsInWorldFrame size: " << pointsInWorldFrame.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] pointsInLinkFrame size: " << pointsInLinkFrame.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] f size: " << f.size() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq rows: " << dfdq.rows() << std::endl;
  //std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq cols: " << dfdq.cols() << std::endl;

  /*
  std::cout << "[SelfCollisionCppAd::getLinearApproximation] f: " << std::endl;
  for (size_t i = 0; i < f.size(); i++)
  {
    std::cout << i << " -> " << f(i) << std::endl;
  }

  std::cout << "[SelfCollisionCppAd::getLinearApproximation] dfdq: " << std::endl;
  for (size_t i = 0; i < dfdq.rows(); i++)
  {
    std::cout << i << " -> ";
    for (size_t j = 0; j < dfdq.cols(); j++)
    {
      std::cout << f(i) << " ";
    }
    std::cout << "" << std::endl;
  }
  */

  //std::cout << "[SelfCollisionCppAd::getLinearApproximation(3)] END" << std::endl << std::endl;

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                    const ad_vector_t& state,
                                                    const ad_vector_t& points) const 
{
  std::cout << "[SelfCollisionCppAd::computeLinkPointsAd(3)] START" << std::endl;

  std::cout << "[SelfCollisionCppAd::computeLinkPointsAd(3)] DEBUG INF" << std::endl;
  while(1);

  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
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
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] = CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], 
                                                                            ad_scalar_t(0.0), 
                                                                            ad_scalar_t(1.0), 
                                                                            ad_scalar_t(-1.0));
  }

  //std::cout << "[SelfCollisionCppAd::computeLinkPointsAd(3)] END" << std::endl;

  return pointsInLinkFrames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                    const ad_vector_t& state,
                                                    const ad_vector_t& fullState,
                                                    const ad_vector_t& points) const 
{
  //std::cout << "[SelfCollisionCppAd::computeLinkPointsAd(5)] START" << std::endl;

  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
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
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] = CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], 
                                                                            ad_scalar_t(0.0), 
                                                                            ad_scalar_t(1.0), 
                                                                            ad_scalar_t(-1.0));
  }

  //std::cout << "[SelfCollisionCppAd::computeLinkPointsAd(5)] END" << std::endl;

  return pointsInLinkFrames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                      const ad_vector_t& state,
                                                      const ad_vector_t& points) const 
{
  std::cout << "[SelfCollisionCppAd::distanceCalculationAd(3)] START" << std::endl;

  std::cout << "[SelfCollisionCppAd::distanceCalculationAd(3)] DEBUG INF" << std::endl;
  while(1);

  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
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

  //std::cout << "[SelfCollisionCppAd::distanceCalculationAd] DEBUG INF" << std::endl;
  //while(1);

  std::cout << "[SelfCollisionCppAd::distanceCalculationAd(3)] END" << std::endl;

  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                      const ad_vector_t& state,
                                                      const ad_vector_t& fullState,
                                                      const ad_vector_t& points) const 
{
  //std::cout << "[SelfCollisionCppAd::distanceCalculationAd(5)] START" << std::endl;

  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(state, fullState);

  pinocchio::forwardKinematics(model, data, q);
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

  //std::cout << "[SelfCollisionCppAd::distanceCalculationAd(5)] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[SelfCollisionCppAd::distanceCalculationAd(5)] END" << std::endl;

  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                         const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                         const std::string& modelName,
                                         const std::string& modelFolder) 
{
  const size_t modeStateDim = getModeStateDim(robotModelInfo_);
  const size_t stateDim = getStateDim(robotModelInfo_);
  const size_t numDistanceResults = pinocchioGeometryInterface_.getGeometryModel().collisionPairs.size();
  const size_t n_points_param = numDistanceResults * numberOfParamsPerResult_;
  const size_t paramDim = stateDim + n_points_param;

  /*
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
  */

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
  {
    const size_t stateDim = getStateDim(robotModelInfo_);
    auto x_full = p.head(stateDim);
    auto p_param = p.tail(n_points_param);

    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(pinocchioInterfaceAd, mappingCppAd, x, x_full, p_param);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };

  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, 
                                                              modeStateDim,
                                                              paramDim,
                                                              modelName + "_distance_intermediate", 
                                                              modelFolder));

  /*
  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = computeLinkPointsAd(pinocchioInterfaceAd, mappingCppAd, x, x_full, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, 
                                                     stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, 
                                                     modelName + "_links_intermediate",
                                                     modelFolder));
  */

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
  {
    const size_t stateDim = getStateDim(robotModelInfo_);
    auto x_full = p.head(stateDim);
    auto p_param = p.tail(n_points_param);
    
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = computeLinkPointsAd(pinocchioInterfaceAd, mappingCppAd, x, x_full, p_param);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, 
                                                     modeStateDim,
                                                     paramDim,
                                                     modelName + "_links_intermediate",
                                                     modelFolder));
}

} /* namespace ocs2 */
