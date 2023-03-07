// LAST UPDATE: 2022.03.04
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
                                     ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                                     size_t modalMode,
                                     std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                     ocs2::scalar_t maxDistance,
                                     std::shared_ptr<ExtMapUtility> emuPtr,
                                     const std::string& modelName, 
                                     const std::string& modelFolder,
                                     bool recompileLibraries, 
                                     bool verbose)
  : extCollisionPinocchioGeometryInterface_(std::move(extCollisionPinocchioGeometryInterface)), 
    modalMode_(modalMode),
    pointsOnRobotPtr_(pointsOnRobotPtr),
    maxDistance_(maxDistance),
    distances_(pointsOnRobotPtr->getNumOfPoints()),
    emuPtr_(emuPtr)
{
  std::cout << "[ExtCollisionCppAd::ExtCollisionCppAd] START" << std::endl;

  PinocchioInterfaceCppAd pinocchioInterfaceAd = pinocchioInterface.toCppAd();
  setADInterfaces(pinocchioInterfaceAd, modelName, modelFolder);

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
  : extCollisionPinocchioGeometryInterface_(rhs.extCollisionPinocchioGeometryInterface_),
    cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
    modalMode_(rhs.modalMode_),
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

  /////// NUA TODO: ADD 6 DOF BASE VERSION!
  if (modalMode_ == 0)
  {
    numPoints = 1;
  }
  else if (modalMode_ == 1)
  {
    numPoints--;
  }

  return numPoints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface,
                                     const vector_t& state) const 
{
  vector_t violations;

  if (pointsOnRobotPtr_) 
  { 
    updateDistances(state);

    violations = distances_;

    //assert(gradients_.rows() == gradientsVoxblox_.rows());
    //assert(gradients_.cols() == jacobianPointsOnRobot.cols());
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getValue] ERROR: No points on robot!" << std::endl;
    violations = vector_t::Zero(1);
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                        const vector_t& q) const 
{
  //std::cout << "[ExtCollisionCppAd::getLinearApproximation] START" << std::endl;

  vector_t distances;
  if (pointsOnRobotPtr_) 
  { 
    updateDistances(q);
    distances = distances_;
  }
  else
  {
    std::cout << "[ExtCollisionCppAd::getValue] ERROR: No points on robot!" << std::endl;
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
  const auto f = cppAdInterfaceDistanceCalculation_ -> getFunctionValue(q, pointsInWorldFrame);
  const auto dfdq = cppAdInterfaceDistanceCalculation_ -> getJacobian(q, pointsInWorldFrame);

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

  //std::cout << "[ExtCollisionCppAd::getLinearApproximation] END" << std::endl << std::endl;

  return std::make_pair(f, dfdq);
}

void ExtCollisionCppAd::updateDistances(const vector_t& q) const
{
  int numPoints = pointsOnRobotPtr_->getNumOfPoints();
  Eigen::VectorXd positionPointsOnRobot = pointsOnRobotPtr_->getPointsPositionCppAd(q);
  Eigen::VectorXd radii = pointsOnRobotPtr_->getRadii();

  /////// NUA TODO: ADD 6 DOF BASE VERSION!
  if (modalMode_ == 0)
  {
    numPoints = 1;
    positionPointsOnRobot = positionPointsOnRobot.segment(0, 3);
    radii = radii.segment(0, 1);
  }
  else if (modalMode_ == 1)
  {
    numPoints--;
    positionPointsOnRobot = positionPointsOnRobot.segment(3, numPoints*3);
    radii = radii.segment(1, numPoints);
  }
  
  assert(positionPointsOnRobot.size() % 3 == 0);
  
  float distance;

  //vector<geometry_msgs::Point> p0_vec;
  //vector<geometry_msgs::Point> p1_vec;

  p0_vec_.clear();
  p1_vec_.clear();

  for (int i = 0; i < numPoints; i++)
  {
    Eigen::Ref<Eigen::Matrix<scalar_t, 3, 1>> position = positionPointsOnRobot.segment<3>(i * 3);
    
    geometry_msgs::Point p0;
    p0.x = position(0);
    p0.y = position(1);
    p0.z = position(2);
    p0_vec_.push_back(p0);

    geometry_msgs::Point p1;
    distance = emuPtr_->getNearestOccupancyDist2(position(0), position(1), position(2), p1, maxDistance_, false);
    p1_vec_.push_back(p1);

    distances_[i] = distance - radii(i);
  }

  emuPtr_->fillOccDistanceArrayVisu(p0_vec_, p1_vec_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ExtCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                                     const ad_vector_t& state,
                                                     const ad_vector_t& points) const 
{
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

    results(i, 1) = points[i * numberOfParamsPerResult_ + 6] * (p1_wrt_base_cppAd - p0_wrt_base_cppAd).norm() - radii(i);
  }
  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ExtCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                        const std::string& modelName,
                                        const std::string& modelFolder) 
{
  std::cout << "[ExtCollisionCppAd::setADInterfaces] START" << std::endl;

  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;
  const size_t numDistanceResults = distances_.size();

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

  std::cout << "[ExtCollisionCppAd::setADInterfaces] END" << std::endl;
}

} /* namespace ocs2 */
