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

#pragma once

#include "ocs2_core/dynamics/MultiModelFunctions.h"
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "ocs2_self_collision/PinocchioGeometryInterface.h"

namespace ocs2 {

class SelfCollisionCppAd 
{
  public:
    using vector3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<ad_scalar_t>;

    /**
     * Constructor
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
     * @param [in] minimumDistance: minimum allowed distance between each collision pair
     * @param [in] modelName : name of the generate model library
     * @param [in] modelFolder : folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
     *                                  available.
     * @param [in] verbose : print information.
     */
    SelfCollisionCppAd(const PinocchioInterface& pinocchioInterface, 
                      PinocchioGeometryInterface pinocchioGeometryInterface,
                      const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                      ocs2::RobotModelInfo& robotModelInfo,
                      scalar_t minimumDistance, 
                      const std::string& modelName, 
                      const std::string& modelFolder = "/tmp/ocs2",
                      bool recompileLibraries = true, 
                      bool verbose = true);

    /** Default destructor */
    ~SelfCollisionCppAd() = default;

    /** Copy constructor */
    SelfCollisionCppAd(const SelfCollisionCppAd& rhs);

    /** Get the number of collision pairs */
    size_t getNumCollisionPairs() const 
    { 
      return pinocchioGeometryInterface_.getNumCollisionPairs(); 
    }

    /**
     * Evaluate the distance violation
     * Computes the distance results of all collision pairs through PinocchioGeometryInterface
     * and the violation compared with the specified minimum distance.
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @return: the differences between the distance of each collision pair and the minimum distance
     */
    vector_t getValue(const PinocchioInterface& pinocchioInterface) const;

    /**
     * Evaluate the linear approximation of the distance function
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @param [in] q: pinocchio coordinates
     * @return: the pair of the distance violation and the first derivative of the distance against q
     */
    std::pair<vector_t, matrix_t> getLinearApproximation(const PinocchioInterface& pinocchioInterface, const vector_t& state) const;

    std::pair<vector_t, matrix_t> getLinearApproximation(const PinocchioInterface& pinocchioInterface, const vector_t& state, const vector_t& fullState) const;

  private:
    /**
     * Sets all the required CppAdCodeGenInterfaces
     */
    void setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                         const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd, 
                         const std::string& modelName, 
                         const std::string& modelFolder);

    // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
    // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
    // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
    ad_vector_t computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state, const ad_vector_t& points) const;

    ad_vector_t computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                    const ad_vector_t& state,
                                    const ad_vector_t& fullState,
                                    const ad_vector_t& points) const;

    // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
    // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
    // Returns a vector that is of length |collisionPairs|
    ad_vector_t distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                      const ad_vector_t& state,
                                      const ad_vector_t& points) const;

    ad_vector_t distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                      const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                      const ad_vector_t& state,
                                      const ad_vector_t& fullState,
                                      const ad_vector_t& points) const;

    // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
    const size_t numberOfParamsPerResult_ = 7;

    RobotModelInfo robotModelInfo_;

    std::unique_ptr<CppAdInterface> cppAdInterfaceDistanceCalculation_;
    std::unique_ptr<CppAdInterface> cppAdInterfaceLinkPoints_;

    PinocchioGeometryInterface pinocchioGeometryInterface_;
    scalar_t minimumDistance_;

    mutable scalar_t bench_min_dist_ = 100000;
    mutable scalar_t bench_max_dist_ = -100000;
};

} /* namespace ocs2 */
