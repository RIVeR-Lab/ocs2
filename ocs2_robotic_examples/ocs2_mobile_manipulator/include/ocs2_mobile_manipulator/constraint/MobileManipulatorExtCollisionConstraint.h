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

#pragma once

#include <memory>

#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_ext_collision/ExtCollisionConstraint.h>
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorExtCollisionConstraint final : public ExtCollisionConstraint 
{
  public:
    MobileManipulatorExtCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                            ExtCollisionPinocchioGeometryInterface extCollisionPinocchioGeometryInterface, 
                                            std::shared_ptr<PointsOnRobot> pointsOnRobotPtr,
                                            ocs2::scalar_t maxDistance,
                                            std::shared_ptr<ExtMapUtility> emuPtr)
      : ExtCollisionConstraint(mapping, 
                               std::move(extCollisionPinocchioGeometryInterface), 
                               pointsOnRobotPtr,
                               maxDistance,
                               emuPtr) {}
    
    ~MobileManipulatorExtCollisionConstraint() override = default;
    
    MobileManipulatorExtCollisionConstraint(const MobileManipulatorExtCollisionConstraint& other) = default;
    
    MobileManipulatorExtCollisionConstraint* clone() const
    { 
      return new MobileManipulatorExtCollisionConstraint(*this); 
    }

    const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override 
    {
      //std::cout << "[MobileManipulatorExtCollisionConstraint::getPinocchioInterface] START" << std::endl;
      
      return cast<MobileManipulatorPreComputation>(preComputation).getPinocchioInterface();
    }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
