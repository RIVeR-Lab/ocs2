// LAST UPDATE: 2022.04.12
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
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_ext_collision/PointsOnRobot.h>

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
PointsOnRobot::PointsOnRobot(const PointsOnRobot::points_radii_t& points_radii)
  : points_(), radii_()
{
  std::cout << "[PointsOnRobot::PointsOnRobot] START" << std::endl;

  const auto& pointsAndRadii = points_radii;

  int numPoints = 0;
  for (const auto& segments : pointsAndRadii) 
  {
    std::vector<double> pointsOnSegment;
    for (const auto& pointRadius : segments) 
    {
      pointsOnSegment.push_back(pointRadius.first);
      numPoints++;
    }
    points_.push_back(pointsOnSegment);
  }

  int idx = 0;
  radii_ = Eigen::VectorXd(numPoints);
  for (const auto& segments : pointsAndRadii) 
  {
    for (const auto& pointRadius : segments) 
    {
      radii_(idx++) = pointRadius.second;
    }
  }

  //ros::NodeHandle nh;
  //nh_ = nh;
  //pointsOnRobot_visu_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("points_on_robot", 1);

  std::cout << "[PointsOnRobot::PointsOnRobot] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_),
    cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)),
    robotModelInfo_(rhs.robotModelInfo_),
    //base_link_name_(rhs.base_link_name_),
    //arm_mount_link_name_(rhs.arm_mount_link_name_),
    //ee_link_name_(rhs.ee_link_name_),
    frameNames_(rhs.frameNames_),
    frameIds_(rhs.frameIds_),
    points_on_robot_(rhs.points_on_robot_),
    nh_(rhs.nh_),
    pointsOnRobot_visu_(rhs.pointsOnRobot_visu_),
    pointsOnRobot_visu_pub_(rhs.pointsOnRobot_visu_pub_){}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::initialize(ocs2::PinocchioInterface& pinocchioInterface,
                               const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                               const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                               const ocs2::RobotModelInfo& robotModelInfo,
                               const std::string& modelName, 
                               const std::string& modelFolder, 
                               bool recompileLibraries, 
                               bool verbose)
{
  //std::cout << "[PointsOnRobot::initialize] START" << std::endl;

  robotModelInfo_ = robotModelInfo;

  //base_link_name_ = base_link_name;
  //arm_mount_link_name_ = arm_mount_link_name; 
  //ee_link_name_ = ee_link_name;

  switch (robotModelInfo_.robotModelType)
  {
    case ocs2::RobotModelType::MobileBase:
    {
      frameNames_.push_back(robotModelInfo_.mobileBase.baseFrame);
      break;
    }
      
    case ocs2::RobotModelType::RobotArm:
    {
      frameNames_.push_back(robotModelInfo_.robotArm.baseFrame);
      for(auto dof_name : robotModelInfo.robotArm.jointFrameNames)
      {
        frameNames_.push_back(dof_name);
      }
      frameNames_.push_back(robotModelInfo.robotArm.eeFrame);
      break;
    }
    
    case ocs2::RobotModelType::MobileManipulator:
    {
      frameNames_.push_back(robotModelInfo_.mobileBase.baseFrame);
      frameNames_.push_back(robotModelInfo_.robotArm.baseFrame);
      for(auto dof_name : robotModelInfo.robotArm.jointFrameNames)
      {
        frameNames_.push_back(dof_name);
      }
      frameNames_.push_back(robotModelInfo.robotArm.eeFrame);
      break;
    }
    
    default:
      throw std::invalid_argument("[PointsOnRobot::initialize] ERROR: Invalid robot model type!");
      break;
  }

  //std::cout << "[PointsOnRobot::initialize] frameNames_: " << std::endl;
  for (size_t i = 0; i < frameNames_.size(); i++)
  {
    std::cout << i << " -> " << frameNames_[i] << std::endl;
  }

  //std::cout << "[PointsOnRobot::initialize] DEBUG INF" << std::endl;
  //while(1);

  // Set link Ids of joints
  for (const auto& bodyName : frameNames_) 
  {
    //std::cout << "[PointsOnRobot::initialize] START getBodyId" << std::endl;
    auto id =  pinocchioInterface.getModel().getBodyId(bodyName);
    frameIds_.push_back(id);

    //std::cout << bodyName << " -> " << id << std::endl;
  }

  //std::cout << "[PointsOnRobot::initialize] START PinocchioInterfaceCppAd" << std::endl;
  // CppAD interface
  ocs2::PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  //std::cout << "[PointsOnRobot::initialize] START PinocchioStateInputMapping" << std::endl;
  // pinocchioInterface to mapping
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  //std::cout << "[PointsOnRobot::initialize] START setADInterfaces" << std::endl;
  setADInterfaces(pinocchioInterface,
                  pinocchioInterfaceCppAd, 
                  *mappingCppAdPtr, 
                  modelName, 
                  modelFolder);

  
  if (recompileLibraries) 
  {
    //std::cout << "[PointsOnRobot::initialize] START createModels" << std::endl;
    createModels(verbose);
  } 
  else 
  {
    //std::cout << "[PointsOnRobot::initialize] START loadModelsIfAvailable" << std::endl;
    loadModelsIfAvailable(verbose);
  }

  //std::cout << "[PointsOnRobot::initialize] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::VectorXd PointsOnRobot::getPointsPositionCppAd(const Eigen::VectorXd& state) const
{
  std::cout << "[PointsOnRobot::getPointsPositionCppAd] START" << std::endl;
  std::cout << "[PointsOnRobot::getPointsPositionCppAd] DEBUG INF" << std::endl;
  while(1);

  points_on_robot_ = cppAdInterface_->getFunctionValue(state);

  fillPointsOnRobotVisu();

  std::cout << "[PointsOnRobot::getPointsPositionCppAd] END" << std::endl;

  return points_on_robot_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::VectorXd PointsOnRobot::getPointsPositionCppAd(const Eigen::VectorXd& state, const Eigen::VectorXd& fullState) const
{
  //std::cout << "[PointsOnRobot::getPointsPositionCppAd] START" << std::endl;

  points_on_robot_ = cppAdInterface_->getFunctionValue(state, fullState);

  //std::cout << "[PointsOnRobot::getPointsPositionCppAd] points_on_robot_ size: " << points_on_robot_.size() << std::endl;
  //std::cout << points_on_robot_ << std::endl;

  fillPointsOnRobotVisu();
  //std::cout << "[PointsOnRobot::getPointsPositionCppAd] END" << std::endl;

  return points_on_robot_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::MatrixXd PointsOnRobot::getPointsJacobianCppAd(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getJacobian(state);
}

ocs2::RobotModelInfo PointsOnRobot::getRobotModelInfo() const
{
  return robotModelInfo_;
}

void PointsOnRobot::getPointsEigenToGeometryMsgsVec(Eigen::VectorXd& pointsEigen, std::vector<geometry_msgs::Point>& pointsGeometryMsgsVec)
{
  //std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] START" << std::endl;

  pointsGeometryMsgsVec.clear();
  int numPoints = getNumOfPoints();

  for (size_t i = 0; i < numPoints; i++)
  {
    Eigen::Ref<Eigen::Matrix<ocs2::scalar_t, 3, 1>> position = pointsEigen.segment<3>(i * 3);
    
    geometry_msgs::Point pgm;
    pgm.x = position(0);
    pgm.y = position(1);
    pgm.z = position(2);
    pointsGeometryMsgsVec.push_back(pgm);

    /*
    std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] i: " << i << std::endl;
    std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] pgm.x: " << pgm.x << std::endl;
    std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] pgm.y: " << pgm.y << std::endl;
    std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] pgm.z: " << pgm.z << std::endl << std::endl;
    */
  }

  //std::cout << "[PointsOnRobot::getPointsEigenToGeometryMsgsVec] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::VectorXd PointsOnRobot::getRadii() const 
{
  return radii_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int PointsOnRobot::getNumOfPoints() const 
{
  return radii_.size();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int PointsOnRobot::getDimPoints() const
{
  int dim = 0;
  for (int i = 0; i < points_.size(); i++) 
  {
    for (int j = 0; j < points_[i].size(); j++) 
    {
      dim++;
    }
  }
  return dim;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::string PointsOnRobot::getBaseFrameName() const
{
  return robotModelInfo_.mobileBase.baseFrame;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
std::vector<size_t> PointsOnRobot::getFrameIds() const
{
  return frameIds_;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = nh;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::setPointsOnRobotVisu()
{
  pointsOnRobot_visu_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("points_on_robot", 100);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::setTimerPointsOnRobotVisu(double dt)
{
  timer_ = nh_.createTimer(ros::Duration(dt), &PointsOnRobot::publishPointsOnRobotVisu, this);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::fillPointsOnRobotVisu() const
{
  //std::cout << "[PointsOnRobot::fillPointsOnRobotVisu] START" << std::endl;
  //pointsOnRobot_visu_.markers.resize(radii_.size());

  Eigen::VectorXd points_on_robot = points_on_robot_;
  pointsOnRobot_visu_.markers.clear();

  for (int i = 0; i < radii_.size(); i++) 
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::Type::SPHERE;
    marker.id = i;
    //marker.action = 0;
    marker.scale.x = radii_[i] * 2;
    marker.scale.y = radii_[i] * 2;
    marker.scale.z = radii_[i] * 2;
    marker.pose.position.x = points_on_robot(3 * i + 0);
    marker.pose.position.y = points_on_robot(3 * i + 1);
    marker.pose.position.z = points_on_robot(3 * i + 2);

    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.color.a = 0.3;
    marker.color.r = 0.5;
    marker.color.b = 0.0;
    marker.color.g = 0.0;

    //marker.frame_locked = true;
    marker.header.frame_id = "world";
    marker.header.seq = i;
    marker.header.stamp = ros::Time::now();

    //pointsOnRobot_visu_.markers[i] = marker;
    pointsOnRobot_visu_.markers.push_back(marker);
  }

  //std::cout << "[PointsOnRobot::fillPointsOnRobotVisu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::publishPointsOnRobotVisu()
{
  //std::cout << "[PointsOnRobot::publishPointsOnRobotVisu] START" << std::endl;
  
  pointsOnRobot_visu_pub_.publish(pointsOnRobot_visu_);

  //std::cout << "[PointsOnRobot::publishPointsOnRobotVisu] END" << std::endl;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::publishPointsOnRobotVisu(const ros::TimerEvent& e)
{
  pointsOnRobot_visu_pub_.publish(pointsOnRobot_visu_);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::Quaternion<ocs2::scalar_t> PointsOnRobot::EulerToQuaternion(const ocs2::scalar_t& yaw, 
                                                                   const ocs2::scalar_t& pitch, 
                                                                   const ocs2::scalar_t& roll) const
{
  Eigen::Quaternion<ocs2::scalar_t> quat;
  quat = Eigen::AngleAxis<ocs2::scalar_t>(roll, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxis<ocs2::scalar_t>(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxis<ocs2::scalar_t>(yaw, Eigen::Vector3d::UnitZ());
  
  std::cout << "[PointsOnRobot::EulerToQuaternion] Quaternion" << std::endl << quat.coeffs() << std::endl;

  return quat;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::Matrix<ocs2::scalar_t, 3, 1> PointsOnRobot::QuaternionToEuler(Eigen::Quaternion<ocs2::scalar_t>& quat) const
{
  Eigen::Matrix<ocs2::scalar_t, 3, 1> euler;
  euler = quat.toRotationMatrix().eulerAngles(3,2,1);
  
  std::cout << "[PointsOnRobot::QuaternionToEuler] Euler from quaternion in yaw, pitch, roll"<< std::endl << euler << std::endl;

  return euler;
}

//// NUA TODO: The function is specific to a robot model!  Generalize it!
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                                                const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                                                const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const
{
  std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] START" << std::endl;

  std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] DEBUG INF" << std::endl;
  while(1);
  
  int dim = getDimPoints();
  if (dim == 0) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] WHY ZERO?" << std::endl;
    while(1){;}
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ad_scalar_t, 3, -1> pointsOnRobot_matrix(3, dim);

  //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] dim: " << dim << std::endl;
  
  //updateTransformsPinocchio(pinocchioInterfaceCppAd, mappingCppAd, stateCppAd);

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_Base_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();

  // Update Pinocchio
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(stateCppAd);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[0]].rotation();
  transform_Base_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[0]].translation();

  // Arm Base wrt World
  transform_ArmMount_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[1]].rotation();
  transform_ArmMount_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[1]].translation();

  // J1 wrt World
  transform_J1_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[2]].rotation();
  transform_J1_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[2]].translation();

  // J2 wrt World
  transform_J2_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[3]].rotation();
  transform_J2_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[3]].translation();

  // J3 wrt World
  transform_J3_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[4]].rotation();
  transform_J3_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[4]].translation();

  // J4 wrt World
  transform_J4_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[5]].rotation();
  transform_J4_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[5]].translation();

  // J5 wrt World
  transform_J5_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[6]].rotation();
  transform_J5_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[6]].translation();

  // J6 wrt World
  transform_J6_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[7]].rotation();
  transform_J6_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[7]].translation();

  // Tool Mount wrt World
  transform_ToolMount_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[8]].rotation();
  transform_ToolMount_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[8]].translation();

  // Arm Base wrt Base
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_Base_cppAd = transform_Base_wrt_World_cppAd.inverse() * transform_ArmMount_wrt_World_cppAd;

  // J1 wrt Arm Base
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_ArmMount_cppAd = transform_ArmMount_wrt_World_cppAd.inverse() * transform_J1_wrt_World_cppAd;

  // J2 wrt J1
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_J1_cppAd = transform_J1_wrt_World_cppAd.inverse() * transform_J2_wrt_World_cppAd;

  // J3 wrt J2
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_J2_cppAd = transform_J2_wrt_World_cppAd.inverse() * transform_J3_wrt_World_cppAd;

  // J4 wrt J3
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_J3_cppAd = transform_J3_wrt_World_cppAd.inverse() * transform_J4_wrt_World_cppAd;

  // J5 wrt J4
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_J4_cppAd = transform_J4_wrt_World_cppAd.inverse() * transform_J5_wrt_World_cppAd;

  // J6 wrt J5
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_J5_cppAd = transform_J5_wrt_World_cppAd.inverse() * transform_J6_wrt_World_cppAd;

  // Tool Mount wrt J6
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_J6_cppAd = transform_J6_wrt_World_cppAd.inverse() * transform_ToolMount_wrt_World_cppAd;

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_tmp = transform_Base_wrt_World_cppAd;

  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = transform_J1_wrt_ArmMount_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 1 and joint 2
  next_transform = transform_J2_wrt_J1_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 2 and joint 3
  next_transform = transform_J3_wrt_J2_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 3 and joint 4
  next_transform = transform_J4_wrt_J3_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 4 and joint 5
  next_transform = transform_J5_wrt_J4_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 5 and joint 6
  next_transform = transform_J6_wrt_J5_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 6 and tool mount
  next_transform = transform_ToolMount_wrt_J6_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }

  return pointsOnRobot_matrix;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                                                const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                                                const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd,
                                                                                                const Eigen::Matrix<ad_scalar_t, -1, 1>& fullStateCppAd) const
{
  //// NUA TODO: The function is specific to a robot model!  Generalize it!

  int dim = getDimPoints();
  if (dim == 0) 
  {
    std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] WHY ZERO?" << std::endl;
    while(1);
    return Eigen::Matrix<ad_scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ad_scalar_t, 3, -1> pointsOnRobot_matrix(3, dim);

  //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] dim: " << dim << std::endl;
  
  //updateTransformsPinocchio(pinocchioInterfaceCppAd, mappingCppAd, stateCppAd);

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_Base_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_World_cppAd = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();

  // Update Pinocchio
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(stateCppAd, fullStateCppAd);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[0]].rotation();
  transform_Base_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[0]].translation();

  // Arm Base wrt World
  transform_ArmMount_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[1]].rotation();
  transform_ArmMount_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[1]].translation();

  // J1 wrt World
  transform_J1_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[2]].rotation();
  transform_J1_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[2]].translation();

  // J2 wrt World
  transform_J2_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[3]].rotation();
  transform_J2_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[3]].translation();

  // J3 wrt World
  transform_J3_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[4]].rotation();
  transform_J3_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[4]].translation();

  // J4 wrt World
  transform_J4_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[5]].rotation();
  transform_J4_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[5]].translation();

  // J5 wrt World
  transform_J5_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[6]].rotation();
  transform_J5_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[6]].translation();

  // J6 wrt World
  transform_J6_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[7]].rotation();
  transform_J6_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[7]].translation();

  // Tool Mount wrt World
  transform_ToolMount_wrt_World_cppAd.topLeftCorner(3,3) = data.oMf[frameIds_[8]].rotation();
  transform_ToolMount_wrt_World_cppAd.topRightCorner(3,1) = data.oMf[frameIds_[8]].translation();

  // Arm Base wrt Base
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ArmMount_wrt_Base_cppAd = transform_Base_wrt_World_cppAd.inverse() * transform_ArmMount_wrt_World_cppAd;

  // J1 wrt Arm Base
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J1_wrt_ArmMount_cppAd = transform_ArmMount_wrt_World_cppAd.inverse() * transform_J1_wrt_World_cppAd;

  // J2 wrt J1
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J2_wrt_J1_cppAd = transform_J1_wrt_World_cppAd.inverse() * transform_J2_wrt_World_cppAd;

  // J3 wrt J2
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J3_wrt_J2_cppAd = transform_J2_wrt_World_cppAd.inverse() * transform_J3_wrt_World_cppAd;

  // J4 wrt J3
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J4_wrt_J3_cppAd = transform_J3_wrt_World_cppAd.inverse() * transform_J4_wrt_World_cppAd;

  // J5 wrt J4
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J5_wrt_J4_cppAd = transform_J4_wrt_World_cppAd.inverse() * transform_J5_wrt_World_cppAd;

  // J6 wrt J5
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_J6_wrt_J5_cppAd = transform_J5_wrt_World_cppAd.inverse() * transform_J6_wrt_World_cppAd;

  // Tool Mount wrt J6
  Eigen::Matrix<ad_scalar_t, 4, 4> transform_ToolMount_wrt_J6_cppAd = transform_J6_wrt_World_cppAd.inverse() * transform_ToolMount_wrt_World_cppAd;

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ad_scalar_t, 4, 4> transform_tmp = transform_Base_wrt_World_cppAd;

  // Points between base and arm mount
  Eigen::Matrix<ad_scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = transform_J1_wrt_ArmMount_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 1 and joint 2
  next_transform = transform_J2_wrt_J1_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 2 and joint 3
  next_transform = transform_J3_wrt_J2_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 3 and joint 4
  next_transform = transform_J4_wrt_J3_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 4 and joint 5
  next_transform = transform_J5_wrt_J4_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 5 and joint 6
  next_transform = transform_J6_wrt_J5_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 6 and tool mount
  next_transform = transform_ToolMount_wrt_J6_cppAd;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobotCppAd] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ad_scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * (ad_scalar_t) points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }

  return pointsOnRobot_matrix;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::setADInterfaces(ocs2::PinocchioInterface& pinocchioInterface,
                                    ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                    const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                    const std::string& modelName,
                                    const std::string& modelFolder)
{
  using ad_interface = ocs2::CppAdInterface;
  using ad_dynamic_vector_t = ad_interface::ad_vector_t;
  using ad_scalar_t = ad_interface::ad_scalar_t;

  int numPoints = 0;
  for (int i = 0; i < points_.size(); i++) 
  {
    numPoints += points_[i].size();
  }
  assert(numPoints == radii_.size());

  const size_t modeStateDim = ocs2::getModeStateDim(robotModelInfo_);
  const size_t stateDim = ocs2::getStateDim(robotModelInfo_);

  /*
  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixPointsOnRobot = computeState2PointsOnRobotCppAd(pinocchioInterfaceAd, mappingCppAd, x);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixPointsOnRobot.data(), matrixPointsOnRobot.size());
  };
  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));
  */

  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& x_full, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixPointsOnRobot = computeState2PointsOnRobotCppAd(pinocchioInterfaceAd, mappingCppAd, x, x_full);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixPointsOnRobot.data(), matrixPointsOnRobot.size());
  };
  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 modeStateDim,
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::createModels(bool verbose) 
{
  cppAdInterface_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void PointsOnRobot::loadModelsIfAvailable(bool verbose) 
{
  cppAdInterface_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}
