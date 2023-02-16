// LAST UPDATE: 2022.02.03
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// REFERENCES:
//

#pragma once

//#include <voxblox/interpolator/interpolator.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_ext_collision/ExtCollisionPinocchioGeometryInterface.h>
#include <ocs2_ext_collision/PointsOnRobot.h>

#include <ocs2_ext_collision/ext_map_utility.h>

namespace ocs2 {

class ExtCollision {
  public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

    /**
     * Constructor
     *
     * @param [in] extCollisionPinocchioGeometryInterface: pinocchio geometry interface of the robot model
     */
    ExtCollision(ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                 std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                 ocs2::scalar_t maxDistance,
                 std::shared_ptr<ExtMapUtility> emuPtr);

    /**
     * TODO: Add desription!
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @return: The differences between the distance of each collision pair and the minimum distance
     */
    vector_t getValue(PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping, const vector_t& state) const;

    /**
     * TODO: Add desription!
     *
     * @note Requires updated forwardKinematics(), updateGlobalPlacements() and computeJointJacobians() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @param [in] extCollisionPinocchioGeometryInterface: pinocchio geometry interface of the robot model
     * @return: The pair of the distance violation and the first derivative of the distance against q
     */
    std::pair<vector_t, matrix_t> getLinearApproximation(const PinocchioInterface& pinocchioInterface) const;

    /**
     * TODO: Add desription!
     *
     *
     */
    size_t getNumPointsOnRobot() const;

    //void setDistanceAndGradient() const;

  private:
    ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface_;

    std::shared_ptr<const PointsOnRobot> pointsOnRobotPtr_;
    //std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> voxbloxInterpolatorPtr_;

    scalar_t mu_;
    scalar_t delta_;
    ocs2::scalar_t maxDistance_;

    mutable Eigen::Matrix<scalar_t, -1, 1> distances_;
    mutable Eigen::MatrixXd gradientsVoxblox_;
    mutable Eigen::Matrix<scalar_t, -1, -1> gradients_;
    std::shared_ptr<ExtMapUtility> emuPtr_;
};

}  // namespace ocs2
