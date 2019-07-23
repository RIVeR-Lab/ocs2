#include "ocs2_double_slit_example/DoubleSlitInterface.h"
#include <ocs2_oc/pi_solver/PI_Settings.h>
#include <ros/package.h>

namespace ocs2 {
namespace double_slit {

DoubleSlitInterface::DoubleSlitInterface(const std::string& taskFileFolderName)
    : barrierLowerEnd_(-0.5), barrierUpperEnd_(0.5), barrierTimePos_(50.0) {
  taskFile_ = ros::package::getPath("ocs2_double_slit_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cout << "Loading task file: " << taskFile_ << std::endl;

  loadSettings(taskFile_);
  setupOptimizer(taskFile_);
}

void DoubleSlitInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadInitialState(taskFile, initialState_);

  /*
   * Dynamics
   */
  linearSystemDynamicsPtr_.reset(new DoubleSlitDynamics());

  /*
   * Cost function
   */
  loadScalar(taskFile, "systemParameters.barrierTimePosition", barrierTimePos_);
  loadScalar(taskFile, "systemParameters.barrierLowerEnd", barrierLowerEnd_);
  loadScalar(taskFile, "systemParameters.barrierUpperEnd", barrierUpperEnd_);
  loadEigenMatrix(taskFile, "Q", qM_);
  loadEigenMatrix(taskFile, "R", rM_);
  loadEigenMatrix(taskFile, "Q_final", qMFinal_);
  xNominal_ = dim_t::state_vector_t::Zero();
  uNominal_ = dim_t::input_vector_t::Zero();

  std::cerr << "Q:  \n" << qM_ << std::endl;
  std::cerr << "R:  \n" << rM_ << std::endl;
  std::cerr << "Q_final:\n" << qMFinal_ << std::endl;
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  auto V = [this](const state_vector_t& x, double t) { return x.dot(this->qM_ * x) + doubleSlitPotentialWall(x, t); };
  auto r = [](const state_vector_t&, double) { return input_vector_t::Zero(); };
  auto Phi = [this](const state_vector_t& x) { return x.dot(this->qMFinal_ * x); };
  input_vector_t uNominal;
  uNominal.setZero();

  costPtr_.reset(new DoubleSlitBarrierCost(rM_, uNominal, V, r, Phi));

  /*
   * Constraints
   */
  linearSystemConstraintPtr_.reset(new DoubleSlitConstraint);

  /*
   * Initialization
   */
  linearSystemOperatingPointPtr_.reset(new DoubleSlitOperatingPoint(initialState_, dim_t::input_vector_t::Zero()));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  scalar_t timeHorizon;
  definePartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
}

void DoubleSlitInterface::setupOptimizer(const std::string& taskFile) {
  mpcSettings_.loadSettings(taskFile);

  PI_Settings piSettings;
  piSettings.loadSettings(taskFile);

  piMpcPtr_.reset(new pi_mpc_t(linearSystemDynamicsPtr_, std::move(costPtr_), *linearSystemConstraintPtr_, partitioningTimes_, mpcSettings_,
                               std::move(piSettings)));
}

DoubleSlitInterface::scalar_t DoubleSlitInterface::doubleSlitPotentialWall(dim_t::state_vector_t x, scalar_t t) const {
  if ((std::abs(t - barrierTimePos_) < 0.1) and x(0) > barrierLowerEnd_ and x(0) < barrierUpperEnd_) {
    return std::numeric_limits<scalar_t>::infinity();
  }
  return 0.0;
}

}  // namespace double_slit
}  // namespace ocs2