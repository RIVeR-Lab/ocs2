// LAST UPDATE: 2023.08.22
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

#include <ocs2_ext_collision/ExtCollisionCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionCppAd::ExtCollisionCppAd(const PinocchioInterface& pinocchioInterface, 
                                     const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,  
                                     std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                     ocs2::scalar_t maxDistance,
                                     std::shared_ptr<ExtMapUtility> emuPtr,
                                     const std::string& modelName, 
                                     const std::string& modelFolder,
                                     bool recompileLibraries, 
                                     bool verbose)
  : pointsOnRobotPtr_(pointsOnRobotPtr),
    maxDistance_(maxDistance),
    distances_(pointsOnRobotPtr->getNumOfPoints()),
    emuPtr_(emuPtr)
{
  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] START" << std::endl;

  normalize_flag_ = true;

  PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  setADInterfaces(pinocchioInterfaceCppAd, *mappingCppAdPtr, modelName, modelFolder);

  if (recompileLibraries) 
  {
    cppAdInterfaceDistanceCalculation_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } 
  else 
  {
    cppAdInterfaceDistanceCalculation_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }

  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollisionCppAd::ExtCollisionCppAd(const ExtCollisionCppAd& rhs)
  : cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
    pointsOnRobotPtr_(rhs.pointsOnRobotPtr_),
    maxDistance_(rhs.maxDistance_),
    distances_(rhs.distances_),
    emuPtr_(rhs.emuPtr_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ExtCollisionCppAd::getNumPointsOnRobot() const
{
  size_t numPoints = pointsOnRobotPtr_->getNumOfPoints();
  return numPoints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface,
                                     const vector_t& state) const 
{
  std::cout << "[ExtCollisionCppAd::getValue(2)] START" << std::endl;
  
  std::cout << "[ExtCollisionCppAd::getValue(2)] DEBUG INF" << std::endl;
  while(1);

  vector_t violations;
  if (pointsOnRobotPtr_) 
  { 
    updateDistances(state, normalize_flag_);
    violations = distances_;

    //assert(gradients_.rows() == gradientsVoxblox_.rows());
    //assert(gradients_.cols() == jacobianPointsOnRobot.cols());
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getValue] ERROR: No points on robot!" << std::endl;
    violations = vector_t::Zero(1);
  }

  std::cout << "[ExtCollisionCppAd::getValue(2)] END" << std::endl;

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface,
                                     const vector_t& state,
                                     const vector_t& fullState) const 
{
  vector_t violations;
  if (pointsOnRobotPtr_) 
  { 
    updateDistances(state, fullState, normalize_flag_);
    violations = distances_;

    //assert(gradients_.rows() == gradientsVoxblox_.rows());
    //assert(gradients_.cols() == jacobianPointsOnRobot.cols());
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getValue] ERROR: No points on robot!" << std::endl;
    violations = vector_t::Zero(1);
  }

  //std::cout << "[ExtCollisionCppAd::getValue] violations: " << std::endl;
  //std::cout << violations << std::endl;

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                        const vector_t& state) const 
{
  std::cout << "[ExtCollisionCppAd::getLinearApproximation(2)] START" << std::endl;
  
  std::cout << "[ExtCollisionCppAd::getLinearApproximation(2)] DEBUG INF" << std::endl;
  while(1);

  vector_t distances;
  if (pointsOnRobotPtr_) 
  { 
    updateDistances(state, normalize_flag_);
    distances = distances_;
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getLinearApproximation(2)] ERROR: No points on robot!" << std::endl;
    distances = vector_t::Zero(1);
  }

  vector_t pointsInWorldFrame(distances.size() * numberOfParamsPerResult_);
  for (size_t i = 0; i < distances.size(); ++i) 
  {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).x() = p0_vec_[i].x;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).y() = p0_vec_[i].y;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).z() = p0_vec_[i].z;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).x() = p1_vec_[i].x;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).y() = p1_vec_[i].y;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).z() = p1_vec_[i].z;
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distances[i] >= 0 ? 1.0 : -1.0;
  }

  //const auto pointsInLinkFrame = cppAdInterfaceLinkPoints_ -> getFunctionValue(q, pointsInWorldFrame);
  const auto f = cppAdInterfaceDistanceCalculation_->getFunctionValue(state, pointsInWorldFrame);
  const auto dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(state, pointsInWorldFrame);

  /*
  std::cout << "[ExtCollisionCppAd::getLinearApproximation] f: " << std::endl;
  for (size_t i = 0; i < f.size(); i++)
  {
    std::cout << i << " -> " << f(i) << std::endl;
  }

  std::cout << "[ExtCollisionCppAd::getLinearApproximation] dfdq: " << std::endl;
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

  std::cout << "[ExtCollisionCppAd::getLinearApproximation(2)] END" << std::endl;

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                        const vector_t& state,
                                                                        const vector_t& fullState) const 
{
  //std::cout << "[ExtCollisionCppAd::getLinearApproximation(3)] START" << std::endl;

  vector_t distances;
  if (pointsOnRobotPtr_) 
  { 
    updateDistances(state, fullState, normalize_flag_);
    distances = distances_;
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getValue] ERROR: No points on robot!" << std::endl;
    distances = vector_t::Zero(1);
  }

  auto n_points_param = distances.size() * numberOfParamsPerResult_;
  auto stateDim = fullState.size();

  //std::cout << "[ExtCollisionCppAd::getLinearApproximation(3)] n_points_param: " << n_points_param << std::endl;
  //std::cout << "[ExtCollisionCppAd::getLinearApproximation(3)] stateDim: " << stateDim << std::endl;

  vector_t pointsInWorldFrame(n_points_param);
  for (size_t i = 0; i < distances.size(); ++i) 
  {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).x() = p0_vec_[i].x;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).y() = p0_vec_[i].y;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_).z() = p0_vec_[i].z;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).x() = p1_vec_[i].x;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).y() = p1_vec_[i].y;
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3).z() = p1_vec_[i].z;
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distances[i] >= 0 ? 1.0 : -1.0;
  }

  vector_t params(stateDim + n_points_param);
  params.head(stateDim) = fullState;
  params.tail(n_points_param) = pointsInWorldFrame;

  const auto f = cppAdInterfaceDistanceCalculation_->getFunctionValue(state, params);
  const auto dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(state, params);

  /*
  std::cout << "[ExtCollisionCppAd::getLinearApproximation] f: " << std::endl;
  for (size_t i = 0; i < f.size(); i++)
  {
    std::cout << i << " -> " << f(i) << std::endl;
  }

  std::cout << "[ExtCollisionCppAd::getLinearApproximation] dfdq: " << std::endl;
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

  //std::cout << "[ExtCollisionCppAd::getLinearApproximation(3)] DEBUG INF" << std::endl;
  //while(1);
  
  //std::cout << "[ExtCollisionCppAd::getLinearApproximation(3)] END" << std::endl << std::endl;

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ExtCollisionCppAd::updateDistances(const vector_t& state, bool normalize_flag) const
{
  std::cout << "[ExtCollisionCppAd::updateDistances(1)] START" << std::endl;

  std::cout << "[ExtCollisionCppAd::updateDistances(1)] DEBUG INF" << std::endl;
  while(1);

  int numPoints = pointsOnRobotPtr_->getNumOfPoints();
  Eigen::VectorXd positionPointsOnRobot = pointsOnRobotPtr_->getPointsPositionCppAd(state);
  Eigen::VectorXd radii = pointsOnRobotPtr_->getRadii();
  
  assert(positionPointsOnRobot.size() % 3 == 0);
  
  double distance;

  //vector<geometry_msgs::Point> p0_vec;
  //vector<geometry_msgs::Point> p1_vec;

  //p0_vec_.clear();
  //p1_vec_.clear();

  for (int i = 0; i < numPoints; i++)
  {
    Eigen::Ref<Eigen::Matrix<scalar_t, 3, 1>> position = positionPointsOnRobot.segment<3>(i * 3);
    
    geometry_msgs::Point p0;
    p0.x = position(0);
    p0.y = position(1);
    p0.z = position(2);

    geometry_msgs::Point p1;
    bool col_status;
    emuPtr_->getNearestOccupancyDist(position(0), 
                                     position(1), 
                                     position(2), 
                                     radii(i),
                                     maxDistance_,
                                     p1, 
                                     distance,
                                     col_status,
                                     false);
    if(col_status)
    {
      //p0_vec_.push_back(p0);
      //p1_vec_.push_back(p1);

      distances_[i] = distance - radii(i);

      if (normalize_flag)
      {
        distances_[i] /= maxDistance_ - radii(i);
      }
    }
    else
    {
      distances_[i] = maxDistance_;

      if (normalize_flag)
      {
        distances_[i] = 1;
      }
    }
  }

  //emuPtr_->fillOccDistanceArrayVisu(p0_vec_, p1_vec_);

  std::cout << "[ExtCollisionCppAd::updateDistances(1)] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ExtCollisionCppAd::updateDistances(const vector_t& state, const vector_t& fullState, bool normalize_flag) const
{
  //std::cout << "[ExtCollisionCppAd::updateDistances(2)] START" << std::endl;

  int numPoints = pointsOnRobotPtr_->getNumOfPoints();
  Eigen::VectorXd positionPointsOnRobot = pointsOnRobotPtr_->getPointsPositionCppAd(state, fullState);
  Eigen::VectorXd radii = pointsOnRobotPtr_->getRadii();

  assert(positionPointsOnRobot.size() % 3 == 0);
  
  double distance;
  p0_vec_.clear();
  p1_vec_.clear();

  timer1_.startTimer();
  pointsOnRobotPtr_->getPointsEigenToGeometryMsgsVec(positionPointsOnRobot, p0_vec_);
  timer1_.endTimer();

  timer2_.startTimer();
  std::vector<bool> col_status;
  std::vector<double> min_distances;
  emuPtr_->getNearestOccupancyDist(numPoints,
                                   positionPointsOnRobot,
                                   radii, 
                                   maxDistance_,
                                   p1_vec_, 
                                   min_distances,
                                   col_status,
                                   normalize_flag);

  for (size_t i = 0; i < min_distances.size(); i++)
  {
    distances_[i] = min_distances[i];
  }

  //std::cout << "[ExtCollisionCppAd::updateDistances(2)] numPoints: " << numPoints << std::endl;
  /*
  for (int i = 0; i < numPoints; i++)
  {
    Eigen::Ref<Eigen::Matrix<scalar_t, 3, 1>> position = positionPointsOnRobot.segment<3>(i * 3);
  
    //std::cout << i << " -> " << position(0) << ", " << position(1) << ", " << position(2) << std::endl;

    geometry_msgs::Point p0;
    p0.x = position(0);
    p0.y = position(1);
    p0.z = position(2);

    geometry_msgs::Point p1;
    bool collision = emuPtr_->getNearestOccupancyDist(position(0), 
                                                      position(1), 
                                                      position(2),
                                                      radii(i), 
                                                      maxDistance_,
                                                      p1, 
                                                      distance,
                                                      false);
    distances_[i] = distance;

    std::cout << "distance: " << distance << std::endl;
    //std::cout << "radii: " << radii(i) << std::endl;
    //std::cout << "distances_: " << distances_[i] << std::endl;
  }
  */
  timer2_.endTimer();

  //timer2_.startTimer();
  //emuPtr_->fillOccDistanceArrayVisu(p0_vec_, p1_vec_);
  //timer2_.endTimer();

  /*
  std::cout << "[ExtCollisionCppAd::updateDistances(2)] distances_ size: " << distances_.size() << std::endl;
  std::cout << distances_ << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer1_" << std::endl;
  std::cout << "###   Maximum : " << timer1_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer1_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer1_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;

  std::cout << "### MPC_ROS Benchmarking timer2_" << std::endl;
  std::cout << "###   Maximum : " << timer2_.getMaxIntervalInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Average : " << timer2_.getAverageInMilliseconds() << "[ms]" << std::endl;
  std::cout << "###   Latest  : " << timer2_.getLastIntervalInMilliseconds() << "[ms]" << std::endl;
  */

  //std::cout << "[ExtCollisionCppAd::updateDistances(2)] DEBUG INF" << std::endl;
  //while(1);

  //std::cout << "[ExtCollisionCppAd::updateDistances(2)] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ExtCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                     const ad_vector_t& state,
                                                     const ad_vector_t& points) const 
{
  std::cout << "[ExtCollisionCppAd::distanceCalculationAd(3)] START" << std::endl;

  std::cout << "[ExtCollisionCppAd::distanceCalculationAd(3)] DEBUG INF" << std::endl;
  while(1);

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_Base_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 1> p0_wrt_World_cppAd = transform_Base_wrt_World_cppAd.col(3);
  Eigen::Matrix<ad_scalar_t, 4, 1> p1_wrt_World_cppAd = transform_Base_wrt_World_cppAd.col(3);

  const ad_vector_t radii = pointsOnRobotPtr_->getRadii().cast<ad_scalar_t>();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();

  // Update Pinocchio
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateGlobalPlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[pointsOnRobotPtr_->getFrameIds()[0]].rotation();
  transform_Base_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[pointsOnRobotPtr_->getFrameIds()[0]].translation();

  Eigen::Matrix<ad_scalar_t, -1, 1> results(points.size() / numberOfParamsPerResult_, 1);

  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) 
  {
    p0_wrt_World_cppAd.template head<3>() = points.segment(i * numberOfParamsPerResult_, 3);
    p1_wrt_World_cppAd.template head<3>() = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    const ad_vector_t p0_wrt_base_cppAd = transform_Base_wrt_World_cppAd.inverse() * p0_wrt_World_cppAd;
    const ad_vector_t p1_wrt_base_cppAd = transform_Base_wrt_World_cppAd.inverse() * p1_wrt_World_cppAd;

    auto dist = (p1_wrt_base_cppAd - p0_wrt_base_cppAd).norm();
    dist -= radii(i);

    if (normalize_flag_)
    {
      dist /= maxDistance_ - radii(i);
    }

    results(i, 1) = points[i * numberOfParamsPerResult_ + 6] * dist;
  }
  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ExtCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                     const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                     const ad_vector_t& state,
                                                     const ad_vector_t& fullState,
                                                     const ad_vector_t& points) const 
{
  //std::cout << "[ExtCollisionCppAd::distanceCalculationAd(5)] START" << std::endl;

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_Base_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 1> p0_wrt_World_cppAd = transform_Base_wrt_World_cppAd.col(3);
  Eigen::Matrix<ad_scalar_t, 4, 1> p1_wrt_World_cppAd = transform_Base_wrt_World_cppAd.col(3);

  const ad_vector_t radii = pointsOnRobotPtr_->getRadii().cast<ad_scalar_t>();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(state, fullState);

  // Update Pinocchio
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[pointsOnRobotPtr_->getFrameIds()[0]].rotation();
  transform_Base_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[pointsOnRobotPtr_->getFrameIds()[0]].translation();

  Eigen::Matrix<ad_scalar_t, -1, 1> results(points.size() / numberOfParamsPerResult_, 1);

  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) 
  {
    p0_wrt_World_cppAd.template head<3>() = points.segment(i * numberOfParamsPerResult_, 3);
    p1_wrt_World_cppAd.template head<3>() = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    const ad_vector_t p0_wrt_base_cppAd = transform_Base_wrt_World_cppAd.inverse() * p0_wrt_World_cppAd;
    const ad_vector_t p1_wrt_base_cppAd = transform_Base_wrt_World_cppAd.inverse() * p1_wrt_World_cppAd;

    auto dist = (p1_wrt_base_cppAd - p0_wrt_base_cppAd).norm();
    dist -= radii(i);

    if (normalize_flag_)
    {
      dist /= maxDistance_ - radii(i);
    }

    results(i, 1) = points[i * numberOfParamsPerResult_ + 6] * dist;
  }

  //std::cout << "[ExtCollisionCppAd::distanceCalculationAd(5)] END" << std::endl;

  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ExtCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                        const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                        const std::string& modelName,
                                        const std::string& modelFolder)
{
  //std::cout << "[ExtCollisionCppAd::setADInterfaces] START" << std::endl;

  auto robotModelInfo = pointsOnRobotPtr_->getRobotModelInfo();
  auto modeStateDim = getModeStateDim(robotModelInfo);
  auto stateDim = getStateDim(robotModelInfo);
  const size_t numDistanceResults = distances_.size();
  const size_t n_points_param = numDistanceResults * numberOfParamsPerResult_;

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
    auto x_full = p.head(stateDim);
    auto p_param = p.tail(n_points_param);

    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(pinocchioInterfaceAd, mappingCppAd, x, x_full, p_param);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };

  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, 
                                                              modeStateDim,
                                                              stateDim + numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", 
                                                              modelFolder));

  //std::cout << "[ExtCollisionCppAd::setADInterfaces] END" << std::endl;
}

} /* namespace ocs2 */
