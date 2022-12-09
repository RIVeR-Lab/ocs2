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

#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_ext_collision/ExtCollisionPinocchioGeometryInterface.h>
#include <ocs2_ext_collision/PointsOnRobot.h>

namespace ocs2 {

class ExtCollision {
  public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

    /**
     * Constructor
     *
     * @param [in] extCollisionPinocchioGeometryInterface: pinocchio geometry interface of the robot model
     */
    ExtCollision(ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, std::shared_ptr<PointsOnRobot> pointsOnRobotPtr);

    /**
     * TODO: Add desription!
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @return: The differences between the distance of each collision pair and the minimum distance
     */
    vector_t getValue(const PinocchioInterface& pinocchioInterface, const vector_t& state) const;

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

  private:
    ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface_;

    std::shared_ptr<const PointsOnRobot> pointsOnRobotPtr_;
};

}  // namespace ocs2
