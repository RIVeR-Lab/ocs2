// LAST UPDATE: 2022.04.10
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

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_ext_collision/ExtCollisionCppAd.h>
#include <ocs2_ext_collision/PointsOnRobot.h>

namespace ocs2 {

/**
 * This class provides the CppAD variant of the external-collision constraints. Therefore no pre-computation is required. The class has
 * two constructors. The constructor with an additional argument, "updateCallback", is meant for cases that PinocchioStateInputMapping
 * requires extra update calls on PinocchioInterface, such as the centroidal model mapping (refer to CentroidalModelPinocchioMapping).
 *
 * See also ExtCollisionConstraint, which uses analytical computation and caching.
 */
class ExtCollisionConstraintCppAd final : public StateConstraint 
{
  public:
    using update_pinocchio_interface_callback = std::function<void(const vector_t& state, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface)>;

    /**
     * Constructor
     * NUA TODO: UPDATE!
     * @param [in] pinocchioInterface: Pinocchio interface of the robot model.
     * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
     * @param [in] extCollisionPinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
     * @param [in] modelName: Name of the generated model library.
     * @param [in] modelFolder: Folder to save the model library files to.
     * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
     *                                 available.
     * @param [in] verbose: If true, print information. Otherwise, no information is printed.
     */
    ExtCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, 
                                const PinocchioStateInputMapping<scalar_t>& mapping,
                                const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                //ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                //size_t modalMode,
                                std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                ocs2::scalar_t maxDistance,
                                std::shared_ptr<ExtMapUtility> emuPtr,
                                const std::string& modelName, 
                                const std::string& modelFolder = "/tmp/ocs2", 
                                bool recompileLibraries = true,
                                bool verbose = true);

    /**
     * Constructor
     * NUA TODO: UPDATE!
     * @param [in] pinocchioInterface: Pinocchio interface of the robot model.
     * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
     * @param [in] extCollisionPinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
     * @param [in] updateCallback: In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
     *                             use this callback (no need to call pinocchio::forwardKinematics).
     * @param [in] modelName: Name of the generated model library.
     * @param [in] modelFolder: Folder to save the model library files to.
     * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
     *                                 available.
     * @param [in] verbose: If true, print information. Otherwise, no information is printed.
     */
    ExtCollisionConstraintCppAd(PinocchioInterface pinocchioInterface, 
                                const PinocchioStateInputMapping<scalar_t>& mapping,
                                const PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                //ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                //size_t modalMode,
                                std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                ocs2::scalar_t maxDistance,
                                std::shared_ptr<ExtMapUtility> emuPtr,
                                update_pinocchio_interface_callback updateCallback, 
                                const std::string& modelName,
                                const std::string& modelFolder = "/tmp/ocs2", 
                                bool recompileLibraries = true, 
                                bool verbose = true);

    ~ExtCollisionConstraintCppAd() override = default;

    ExtCollisionConstraintCppAd* clone() const override 
    { 
      return new ExtCollisionConstraintCppAd(*this); 
    }

    size_t getNumConstraints(scalar_t time) const override;

    /** Get the extrenal-collision distance values */
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation&) const override;

    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& fullState, const PreComputation&) const override;

    /** Get the external collision distance approximation */
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const PreComputation&) const override;

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                             const vector_t& state, 
                                                             const vector_t& fullState, 
                                                             const PreComputation&) const override;

  private:
    ExtCollisionConstraintCppAd(const ExtCollisionConstraintCppAd& rhs);

    mutable PinocchioInterface pinocchioInterface_;
    ExtCollisionCppAd extCollisionCppAd_;
    std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
    update_pinocchio_interface_callback updateCallback_;
};

}  // namespace ocs2
