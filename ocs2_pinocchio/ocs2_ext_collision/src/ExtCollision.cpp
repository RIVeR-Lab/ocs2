// LAST UPDATE: 2022.12.21
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_ext_collision/ExtCollision.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ExtCollision::ExtCollision(ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                           std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                           std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> voxbloxInterpolatorPtr)
  : extCollisionPinocchioGeometryInterface_(std::move(extCollisionPinocchioGeometryInterface)), 
    pointsOnRobotPtr_(pointsOnRobotPtr),
    voxbloxInterpolatorPtr_(voxbloxInterpolatorPtr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollision::getValue(PinocchioInterface& pinocchioInterface, 
                                const PinocchioStateInputMapping<scalar_t>& mapping, 
                                const vector_t& state) const 
{
  std::cout << "[ExtCollision::getValue] START" << std::endl;
  
  //Eigen::VectorXd points = pointsOnRobotPtr_->getPointsPosition(pinocchioInterface, mapping, state);
  Eigen::VectorXd points = pointsOnRobotPtr_->getPointsPositionCppAd(state);
  pointsOnRobotPtr_->publishPointsOnRobotVisu(pinocchioInterface, mapping, state);
  
  /*
  std::cout << "[ExtCollision::getValue] state" << std::endl;
  for (size_t i = 0; i < state.rows(); i++)
  {
    for (size_t j = 0; j < state.cols(); j++)
    {
      std::cout << i << ": " << state(i,j) << std::endl;
    } 
  }

  std::cout << "[ExtCollision::getValue] points" << std::endl;
  for (size_t i = 0; i < points.rows(); i++)
  {
    for (size_t j = 0; j < points.cols(); j++)
    {
      std::cout << i << ": " << points(i,j) << std::endl;
    } 
  }
  */
  
  const std::vector<hpp::fcl::DistanceResult> distanceArray = extCollisionPinocchioGeometryInterface_.computeDistances(pinocchioInterface);
  vector_t violations = vector_t::Zero(distanceArray.size());

  /*
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }
  */

  std::cout << "[ExtCollision::getValue] END" << std::endl;
  std::cout << "" << std::endl;

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollision::getLinearApproximation(const PinocchioInterface& pinocchioInterface) const 
{
  std::cout << "[ExtCollision::getLinearApproximation] START" << std::endl;

  const std::vector<hpp::fcl::DistanceResult> distanceArray = extCollisionPinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  const auto& model = pinocchioInterface.getModel();
  const auto& data = pinocchioInterface.getData();

  const auto& geometryModel = extCollisionPinocchioGeometryInterface_.getGeometryModel();

  vector_t f(distanceArray.size());
  matrix_t dfdq(distanceArray.size(), model.nq);

  /*
  for (size_t i = 0; i < distanceArray.size(); ++i) 
  {
    // Distance violation
    f[i] = distanceArray[i].min_distance - minimumDistance_;

    // Jacobian calculation
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    // We need to get the jacobian of the point on the first object; use the joint jacobian translated to the point
    const vector3_t joint1Position = data.oMi[joint1].translation();
    const vector3_t pt1Offset = distanceArray[i].nearest_points[0] - joint1Position;
    matrix_t joint1Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint1Jacobian);
    
    // Jacobians from pinocchio are given as
    // [ position jacobian ]
    // [ rotation jacobian ]
    const matrix_t pt1Jacobian = joint1Jacobian.topRows(3) - skewSymmetricMatrix(pt1Offset) * joint1Jacobian.bottomRows(3);

    // We need to get the jacobian of the point on the second object; use the joint jacobian translated to the point
    const vector3_t joint2Position = data.oMi[joint2].translation();
    const vector3_t pt2Offset = distanceArray[i].nearest_points[1] - joint2Position;
    matrix_t joint2Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint2Jacobian);
    const matrix_t pt2Jacobian = joint2Jacobian.topRows(3) - skewSymmetricMatrix(pt2Offset) * joint2Jacobian.bottomRows(3);

    // To get the (approximate) jacobian of the distance, get the difference between the two nearest point jacobians, then multiply by the
    // vector from point to point
    const matrix_t differenceJacobian = pt2Jacobian - pt1Jacobian;
    
    // TODO(perry): is there a way to calculate a correct jacobian for the case of distanceVector = 0?
    const vector3_t distanceVector = distanceArray[i].min_distance > 0
                                         ? (distanceArray[i].nearest_points[1] - distanceArray[i].nearest_points[0]).normalized()
                                         : (distanceArray[i].nearest_points[0] - distanceArray[i].nearest_points[1]).normalized();
    dfdq.row(i).noalias() = distanceVector.transpose() * differenceJacobian;
  }  // end of i loop
  */

  std::cout << "[ExtCollision::getLinearApproximation] END" << std::endl;

  return {f, dfdq};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ExtCollision::getNumPointsOnRobot() const
{
  return pointsOnRobotPtr_->getNumOfPoints();
}

}  // namespace ocs2
