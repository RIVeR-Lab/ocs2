/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <string>
#include <XmlRpcValue.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

//#include <voxblox/interpolator/interpolator.h>

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
//#include <ocs2_ros_interfaces/EsdfCachingServer.hpp>

#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_ext_collision/ExtCollisionConstraint.h>
#include <ocs2_ext_collision/ExtCollisionConstraintCppAd.h>
#include <ocs2_ext_collision/PointsOnRobot.h>
#include <ocs2_ext_collision/ext_map_utility.h>

#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorExtCollisionConstraint.h"
#include "ocs2_mobile_manipulator/cost/QuadraticInputCost.h"
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"
#include "ocs2_mobile_manipulator/dynamics/WheelBasedMobileManipulatorDynamics.h"

namespace ocs2 {
namespace mobile_manipulator {

//using typename voxblox::EsdfCachingVoxel;
//using voxblox::Interpolator;

/**
 * Mobile Manipulator Robot Interface class
 */
class MobileManipulatorInterface final : public RobotInterface 
{
  public:
    /**
     * Constructor
     *
     * @note Creates directory for generated library into if it does not exist.
     * @throw Invalid argument error if input task file or urdf file does not exist.
     *
     * @param [in] taskFile: The absolute path to the configuration file for the MPC.
     * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
     * @param [in] urdfFile: The absolute path to the URDF file for the robot.
     */
    MobileManipulatorInterface(const std::string& taskFile, 
                               const std::string& libraryFolder, 
                               const std::string& urdfFile,
                               PointsOnRobot::points_radii_t pointsAndRadii = std::vector<std::vector<std::pair<double, double>>>());

    const vector_t& getInitialState()
    { 
      return initialState_;
    }

    ddp::Settings& ddpSettings() 
    { 
      return ddpSettings_; 
    }

    mpc::Settings& mpcSettings() 
    {
      return mpcSettings_; 
    }

    const OptimalControlProblem& getOptimalControlProblem() const override 
    { 
      return problem_; 
    }

    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override 
    { 
      return referenceManagerPtr_; 
    }

    const Initializer& getInitializer() const override 
    { 
      return *initializerPtr_; 
    }

    const RolloutBase& getRollout() const 
    { 
      return *rolloutPtr_; 
    }

    const PinocchioInterface& getPinocchioInterface() const 
    { 
      return *pinocchioInterfacePtr_; 
    }

    const ManipulatorModelInfo& getManipulatorModelInfo() const 
    { 
      return manipulatorModelInfo_; 
    }

    std::shared_ptr<PointsOnRobot> getPointsOnRobotPtr() 
    { 
      return pointsOnRobotPtr_;
    }

    /*
    std::shared_ptr<voxblox::EsdfCachingServer> getEsdfCachingServerPtr() 
    { 
      return esdfCachingServerPtr_;
    }
    */

    /*
    std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> getVoxbloxInterpolatorPtr()
    {
      return voxbloxInterpolatorPtr_;
    }
    */

    void setPointsOnRobotPtr(std::shared_ptr<PointsOnRobot> newPointsOnRobotPtr) 
    { 
      pointsOnRobotPtr_ = newPointsOnRobotPtr;
    }

    /*
    void setEsdfCachingServerPtr(std::shared_ptr<voxblox::EsdfCachingServer> newEsdfCachingServerPtr) 
    { 
      esdfCachingServerPtr_ = newEsdfCachingServerPtr;
    }
    */

    /*
    void setVoxbloxInterpolatorPtr(std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> newVoxbloxInterpolatorPtr)
    {
      voxbloxInterpolatorPtr_ = newVoxbloxInterpolatorPtr;
    }
    */

   void launchNodes(ros::NodeHandle& nodeHandle);

  private:
    std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);
    
    std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, 
                                                        const std::string& taskFile,
                                                        const std::string& prefix, 
                                                        bool useCaching, 
                                                        const std::string& libraryFolder,
                                                        bool recompileLibraries);
    
    std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, 
                                                          const std::string& taskFile,
                                                          const std::string& urdfFile, 
                                                          const std::string& prefix, 
                                                          bool useCaching,
                                                          const std::string& libraryFolder, 
                                                          bool recompileLibraries);

    std::unique_ptr<StateCost> getExtCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                         const std::string& taskFile,
                                                         const std::string& urdfFile, 
                                                         const std::string& prefix, 
                                                         bool useCaching,
                                                         const std::string& libraryFolder, 
                                                         bool recompileLibraries);

    std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;

    OptimalControlProblem problem_;
    std::shared_ptr<ReferenceManager> referenceManagerPtr_;

    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> initializerPtr_;

    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    ManipulatorModelInfo manipulatorModelInfo_;

    vector_t initialState_;

    std::shared_ptr<PointsOnRobot> pointsOnRobotPtr_;
    //std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServerPtr_;
    //std::shared_ptr<voxblox::Interpolator<voxblox::EsdfCachingVoxel>> voxbloxInterpolatorPtr_;
    std::shared_ptr<ExtMapUtility> emuPtr_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
