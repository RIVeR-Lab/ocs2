// LAST UPDATE: 2022.03.04
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
                           ocs2::scalar_t maxDistance,
                           std::shared_ptr<ExtMapUtility> emuPtr,
                           size_t modalMode,
                           size_t stateDim)
  : extCollisionPinocchioGeometryInterface_(std::move(extCollisionPinocchioGeometryInterface)), 
    pointsOnRobotPtr_(pointsOnRobotPtr),
    maxDistance_(maxDistance),
    distances_(pointsOnRobotPtr->getNumOfPoints()),
    emuPtr_(emuPtr),
    modalMode_(modalMode)
{
  //updateModalDim(modalMode, armStateDim);

  gradientsVoxblox_.resize(pointsOnRobotPtr->getNumOfPoints(), pointsOnRobotPtr->getNumOfPoints() * 3);
  gradients_.resize(pointsOnRobotPtr->getNumOfPoints(), stateDim); 
}

/////// NUA TODO: IMPLEMENTATION IS NOT COMPLETE!
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ExtCollision::getValue(PinocchioInterface& pinocchioInterface, 
                                const PinocchioStateInputMapping<scalar_t>& mapping, 
                                const vector_t& state) const 
{
  //std::cout << "[ExtCollision::getValue] START" << std::endl;
  //std::cout << "[ExtCollision::getValue] state size: " << state.size() << std::endl;
  //std::cout << "[ExtCollision::getValue] maxDistance_: " << maxDistance_ << std::endl;

  std::cout << "[ExtCollision::getValue] DEBUG INF" << std::endl;
  while(1);
  
  vector_t violations;

  Eigen::VectorXd points = pointsOnRobotPtr_->getPointsPositionCppAd(state);
  
  if (pointsOnRobotPtr_) 
  { 
    int numPoints = pointsOnRobotPtr_->getNumOfPoints();

    violations.resize(numPoints);
    gradientsVoxblox_.setZero();

    Eigen::VectorXd positionPointsOnRobot = pointsOnRobotPtr_->getPointsPositionCppAd(state);
    Eigen::MatrixXd jacobianPointsOnRobot = pointsOnRobotPtr_->getPointsJacobianCppAd(state);
    Eigen::VectorXd radii = pointsOnRobotPtr_->getRadii();
    
    assert(positionPointsOnRobot.size() % 3 == 0);
    
    double distance;
    Eigen::Vector3f gradientVoxblox;

    p0_vec_.clear();
    p1_vec_.clear();

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
        p0_vec_.push_back(p0);
        p1_vec_.push_back(p1);

        distances_[i] = distance - radii(i);

        if (normalize_flag_)
        {
          distances_[i] /= maxDistance_ - radii(i);
        }
      }
      else
      {
        distances_[i] = maxDistance_;

        if (normalize_flag_)
        {
          distances_[i] = 1;
        }
      }
    }

    //emuPtr_->fillOccDistanceArrayVisu(p0_vec_, p1_vec_);

    violations = distances_;

    assert(gradients_.rows() == gradientsVoxblox_.rows());
    assert(gradients_.cols() == jacobianPointsOnRobot.cols());
    gradients_ = gradientsVoxblox_ * jacobianPointsOnRobot;
  }
  else
  {
    std::cout << "[ExtCollision::getValue] No points on robot!" << std::endl;
    violations = vector_t::Zero(1);
  }

  /*
  std::cout << "[ExtCollision::getValue] state" << std::endl;
  for (size_t i = 0; i < state.rows(); i++)
  {
    for (size_t j = 0; j < state.cols(); j++)
    {
      std::cout << i << ": " << state(i,j) << std::endl;
    } 
  }
  */

  //std::cout << "[ExtCollision::getValue] END" << std::endl;

  return violations;
}

/////// NUA TODO: IMPLEMENTATION IS NOT COMPLETE!
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> ExtCollision::getLinearApproximation(const PinocchioInterface& pinocchioInterface) const 
{
  //std::cout << "[ExtCollision::getLinearApproximation] START" << std::endl;

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

  //std::cout << "[ExtCollision::getLinearApproximation] END" << std::endl;

  //dfdq = gradients_.transpose() * distances_

  return {f, dfdq};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ExtCollision::getNumPointsOnRobot() const
{
  return pointsOnRobotPtr_->getNumOfPoints();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void ExtCollision::updateModalDim(size_t modalMode, size_t armStateDim)
{
  // NUA TODO: ADD 6 DOF BASE VERSION!
  switch (modalMode)
  {
    case 0:
      modalBaseStateDim_ = 3;
      modalArmStateDim_ = 0;
      modalStateDim_ = modalBaseStateDim_ + modalArmStateDim_;
      
      modalBaseInputDim_ = 2;
      modalArmInputDim_ = 0;
      modalInputDim_ = modalBaseInputDim_ + modalArmInputDim_;
      break;

    case 1:
      modalBaseStateDim_ = 0;
      modalArmStateDim_ = armStateDim;
      modalStateDim_ = modalBaseStateDim_ + modalArmStateDim_;

      modalBaseInputDim_ = 0;
      modalArmInputDim_ = armStateDim;
      modalInputDim_ = modalBaseInputDim_ + modalArmInputDim_;
      break;
    
    case 2:
      modalBaseStateDim_ = 3;
      modalArmStateDim_ = armStateDim;
      modalStateDim_ = modalBaseStateDim_ + modalArmStateDim_;

      modalBaseInputDim_ = 2;
      modalArmInputDim_ = armStateDim;
      modalInputDim_ = modalBaseInputDim_ + modalArmInputDim_;
      break;

    default:
      modalBaseStateDim_ = 3;
      modalArmStateDim_ = armStateDim;
      modalStateDim_ = modalBaseStateDim_ + modalArmStateDim_;

      modalBaseInputDim_ = 2;
      modalArmInputDim_ = armStateDim;
      modalInputDim_ = modalBaseInputDim_ + modalArmInputDim_;

      std::cout << "[ExtCollision::updateModalDim] WARNING: Undefined modal mode: " << modalMode << std:: endl;

      break;
  }
}
*/

}  // namespace ocs2
