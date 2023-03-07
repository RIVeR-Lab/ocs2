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

#pragma once

//#include <voxblox/interpolator/interpolator.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ocs2_ext_collision/ExtCollisionPinocchioGeometryInterface.h"
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>

namespace ocs2 {

class ExtCollisionCppAd 
{
  public:
    using vector3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<ad_scalar_t>;

    using ad_interface = ocs2::CppAdInterface;
    using ad_dynamic_vector_t = ad_interface::ad_vector_t;
    //using ad_scalar_t = ad_interface::ad_scalar_t;

    /**
     * Constructor
     * NUA TODO: UPDATE!
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @param [in] extCollisionPinocchioGeometryInterface: pinocchio geometry interface of the robot model
     * @param [in] minimumDistance: minimum allowed distance between each collision pair
     * @param [in] modelName : name of the generate model library
     * @param [in] modelFolder : folder to save the model library files to
     * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose : print information.
     */
    ExtCollisionCppAd(const PinocchioInterface& pinocchioInterface, 
                      ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface,
                      size_t modalMode,
                      std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                      ocs2::scalar_t maxDistance,
                      std::shared_ptr<ExtMapUtility> emuPtr,
                      const std::string& modelName, 
                      const std::string& modelFolder = "/tmp/ocs2",
                      bool recompileLibraries = true, 
                      bool verbose = true);

    /** Default destructor */
    ~ExtCollisionCppAd() = default;

    /** Copy constructor */
    ExtCollisionCppAd(const ExtCollisionCppAd& rhs);

    /** NUA TODO: Description! */
    std::shared_ptr<PointsOnRobot> getPointsOnRobotPtr() 
    { 
      return pointsOnRobotPtr_;
    }

    /** NUA TODO: Description! */
    void setPointsOnRobotPtr(std::shared_ptr<PointsOnRobot> newPointsOnRobotPtr) 
    { 
      pointsOnRobotPtr_ = newPointsOnRobotPtr;
    }

    /**
     * TODO: Add desription!
     *
     *
     */
    size_t getNumPointsOnRobot() const;

    /**
     * Evaluate the distance violation
     * Computes the distance results of all collision pairs through ExtCollisionPinocchioGeometryInterface
     * and the violation compared with the specified minimum distance.
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @return: the differences between the distance of each collision pair and the minimum distance
     */
    vector_t getValue(const PinocchioInterface& pinocchioInterface,
                      const vector_t& state) const;

    /**
     * Evaluate the linear approximation of the distance function
     *
     * @note Requires updated forwardKinematics() on pinocchioInterface.
     *
     * @param [in] pinocchioInterface: pinocchio interface of the robot model
     * @param [in] q: pinocchio coordinates
     * @return: the pair of the distance violation and the first derivative of the distance against q
     */
    std::pair<vector_t, matrix_t> getLinearApproximation(const PinocchioInterface& pinocchioInterface, const vector_t& q) const;

    /** NUA TODO: Description! */
    void updateDistances(const vector_t& q) const;

  private:
    // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
    // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
    // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
    ad_vector_t computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state, const ad_vector_t& points) const;
    
    // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
    // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
    // Returns a vector that is of length |collisionPairs|
    ad_vector_t distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, 
                                      const ad_vector_t& state,
                                      const ad_vector_t& points) const;

    /**
     * Sets all the required CppAdCodeGenInterfaces
     */
    void setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const std::string& modelName, const std::string& modelFolder);

    // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
    const size_t numberOfParamsPerResult_ = 7;

    //std::shared_ptr<ocs2::CppAdInterface> cppAdInterface_;
    std::unique_ptr<CppAdInterface> cppAdInterfaceDistanceCalculation_;
    //std::unique_ptr<CppAdInterface> cppAdInterfaceLinkPoints_;

    ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface_;
    
    //std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> interpolator_;
    //scalar_t minimumDistance_;
    //std::vector<std::vector<double>> points_;

    // NUA TODO: CLEAN BEFORE!
    size_t modalMode_;
    std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_;
    ocs2::scalar_t maxDistance_;

    mutable Eigen::Matrix<scalar_t, -1, 1> distances_;
    mutable vector<geometry_msgs::Point> p0_vec_;
    mutable vector<geometry_msgs::Point> p1_vec_;

    //mutable Eigen::MatrixXd gradientsVoxblox_;
    //mutable Eigen::Matrix<scalar_t, -1, -1> gradients_;
    std::shared_ptr<ExtMapUtility> emuPtr_;
};

} /* namespace ocs2 */
