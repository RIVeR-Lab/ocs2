// LAST UPDATE: 2022.01.06
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PointsOnRobot::PointsOnRobot(const PointsOnRobot& rhs)
  : points_(rhs.points_),
    radii_(rhs.radii_),
    cppAdInterface_(new ocs2::CppAdInterface(*rhs.cppAdInterface_)),
    base_link_name_(rhs.base_link_name_),
    arm_mount_link_name_(rhs.arm_mount_link_name_),
    ee_link_name_(rhs.ee_link_name_),
    frameNames_(rhs.frameNames_),
    frameIds_(rhs.frameIds_),
    points_on_robot_(rhs.points_on_robot_),
    nh_(rhs.nh_),
    pointsOnRobot_visu_(rhs.pointsOnRobot_visu_),
    pointsOnRobot_visu_pub_(rhs.pointsOnRobot_visu_pub_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::initialize(ocs2::PinocchioInterface& pinocchioInterface,
                               const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                               const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                               const std::string& modelName, 
                               const std::string& modelFolder, 
                               bool recompileLibraries, 
                               bool verbose,
                               const std::string& base_link_name, 
                               const std::string& arm_mount_link_name,  
                               const std::string& ee_link_name,
                               const std::vector<std::string>& jointParentFrameNames)
{
  std::cout << "[PointsOnRobot::initialize] START" << std::endl;

  base_link_name_ = base_link_name;
  arm_mount_link_name_ = arm_mount_link_name; 
  ee_link_name_ = ee_link_name;

  //std::cout << "[PointsOnRobot::initialize] frameNames_: " << std::endl;
  frameNames_.push_back(base_link_name);
  frameNames_.push_back(arm_mount_link_name);
  for(auto dof_name : jointParentFrameNames)
  {
    frameNames_.push_back(dof_name);
  }
  frameNames_.push_back(ee_link_name);

  for (size_t i = 0; i < frameNames_.size(); i++)
  {
    std::cout << i << ": "  << frameNames_[i] << std::endl;
  }
  
  // Set link Ids of joints
  for (const auto& bodyName : frameNames_) 
  {
    auto id =  pinocchioInterface.getModel().getBodyId(bodyName);
    frameIds_.push_back(id);

    //std::cout << bodyName << " -> " << id << std::endl;
  }

  // Set fixed transforms
  //setFixedTransformsTf2();

  // CppAD interface
  ocs2::PinocchioInterfaceCppAd pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // pinocchioInterface to mapping
  std::unique_ptr<ocs2::PinocchioStateInputMapping<ad_scalar_t>> mappingCppAdPtr(mappingCppAd.clone());
  mappingCppAdPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  setADInterfaces(pinocchioInterface,
                  pinocchioInterfaceCppAd, 
                  *mappingCppAdPtr, 
                  modelName, 
                  modelFolder);

  if (recompileLibraries) 
  {
    createModels(verbose);
  } 
  else 
  {
    loadModelsIfAvailable(verbose);
  }

  std::cout << "[PointsOnRobot::initialize] END" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
Eigen::VectorXd PointsOnRobot::getPointsPosition(ocs2::PinocchioInterface& pinocchioInterface, 
                                                  const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                  const Eigen::VectorXd& state) const
{
  //std::cout << "[PointsOnRobot::getPointPosition] START" << std::endl;

  updateTransformsPinocchio(pinocchioInterface, mapping, state);

  Eigen::Matrix<ocs2::scalar_t, 3, -1> pointsOnRobot_matrix = computeState2PointsOnRobot(pinocchioInterface, mapping, state);
  points_on_robot_ = Eigen::Map<Eigen::Matrix<ocs2::scalar_t, -1, 1>>(pointsOnRobot_matrix.data(), pointsOnRobot_matrix.size()); 

  //std::cout << "[PointsOnRobot::getPointPosition] END" << std::endl;

  return points_on_robot_;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::getPointsPositionCppAd(const Eigen::VectorXd& state) const
{
  //std::cout << "[PointsOnRobot::getPointsPositionCppAd] START" << std::endl;

  points_on_robot_ = cppAdInterface_->getFunctionValue(state);

  fillPointsOnRobotVisu();
  //std::cout << "[PointsOnRobot::getPointsPositionCppAd] END" << std::endl;

  return points_on_robot_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::MatrixXd PointsOnRobot::getPointsJacobianCppAd(const Eigen::VectorXd& state) const 
{
  return cppAdInterface_->getJacobian(state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd PointsOnRobot::getRadii() const 
{
  return radii_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int PointsOnRobot::getNumOfPoints() const 
{
  return radii_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = nh;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
    marker.scale.x = radii_[i] * 1;
    marker.scale.y = radii_[i] * 1;
    marker.scale.z = radii_[i] * 1;
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::publishPointsOnRobotVisu()
{
  pointsOnRobot_visu_pub_.publish(pointsOnRobot_visu_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::publishPointsOnRobotVisu(const ros::TimerEvent& e)
{
  pointsOnRobot_visu_pub_.publish(pointsOnRobot_visu_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::publishPointsOnRobotVisu(double dt)
{
  pointsOnRobot_visu_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("points_on_robot", 10);
  timer_ = nh_.createTimer(ros::Duration(dt), &PointsOnRobot::publishPointsOnRobotVisu, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<ocs2::scalar_t, 3, 1> PointsOnRobot::QuaternionToEuler(Eigen::Quaternion<ocs2::scalar_t>& quat) const
{
  Eigen::Matrix<ocs2::scalar_t, 3, 1> euler;
  euler = quat.toRotationMatrix().eulerAngles(3,2,1);
  
  std::cout << "[PointsOnRobot::EulerToQuaternion] Euler from quaternion in yaw, pitch, roll"<< std::endl << euler << std::endl;

  return euler;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void PointsOnRobot::setFixedTransformsTf2()
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      //std::cout << "[PointsOnRobot::setFixedTransformsTf2] base_link_name_: " << base_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::setFixedTransformsTf2] arm_mount_link_name_: " << arm_mount_link_name_ << std::endl;
      while(!tfBuffer.canTransform(base_link_name_, arm_mount_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(base_link_name_, arm_mount_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ArmMount_wrt_Base_ = Eigen::Matrix4d::Identity();
    transform_ArmMount_wrt_Base_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ArmMount_wrt_Base_(0, 3) = transformStamped.transform.translation.x;
    transform_ArmMount_wrt_Base_(1, 3) = transformStamped.transform.translation.y;
    transform_ArmMount_wrt_Base_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransformsTf2] baseToArmMount_: " << std::endl << transform_ArmMount_wrt_Base_ << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::setFixedTransformsTf2] ee_link_name_: " << ee_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::setFixedTransformsTf2] J6: " << frameNames_[frameNames_.size()-2] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[frameNames_.size()-2], ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[frameNames_.size()-2], ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::setFixedTransformsTf2] wrist2ToEETransform_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  }
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void PointsOnRobot::updateTransformsTf2Relative() const
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] arm_mount_link_name_: " << arm_mount_link_name_ << std::endl;
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[2]: " << frameNames_[2] << std::endl;
      while(!tfBuffer.canTransform(arm_mount_link_name_, frameNames_[2], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(arm_mount_link_name_, frameNames_[2], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J1_wrt_ArmMount_ = Eigen::Matrix4d::Identity();
    transform_J1_wrt_ArmMount_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J1_wrt_ArmMount_(0, 3) = transformStamped.transform.translation.x;
    transform_J1_wrt_ArmMount_(1, 3) = transformStamped.transform.translation.y;
    transform_J1_wrt_ArmMount_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J1_wrt_ArmMount_: " << std::endl << transform_J1_wrt_ArmMount_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[3]: " << frameNames_[3] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[2], frameNames_[3], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[2], frameNames_[3], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J2_wrt_J1_ = Eigen::Matrix4d::Identity();
    transform_J2_wrt_J1_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J2_wrt_J1_(0, 3) = transformStamped.transform.translation.x;
    transform_J2_wrt_J1_(1, 3) = transformStamped.transform.translation.y;
    transform_J2_wrt_J1_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J2_wrt_J1_: " << std::endl << transform_J2_wrt_J1_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[4]: " << frameNames_[4] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[3], frameNames_[4], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[3], frameNames_[4], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J3_wrt_J2_ = Eigen::Matrix4d::Identity();
    transform_J3_wrt_J2_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J3_wrt_J2_(0, 3) = transformStamped.transform.translation.x;
    transform_J3_wrt_J2_(1, 3) = transformStamped.transform.translation.y;
    transform_J3_wrt_J2_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J3_wrt_J2_: " << std::endl << transform_J3_wrt_J2_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[5]: " << frameNames_[5] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[4], frameNames_[5], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[4], frameNames_[5], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J4_wrt_J3_ = Eigen::Matrix4d::Identity();
    transform_J4_wrt_J3_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J4_wrt_J3_(0, 3) = transformStamped.transform.translation.x;
    transform_J4_wrt_J3_(1, 3) = transformStamped.transform.translation.y;
    transform_J4_wrt_J3_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J4_wrt_J3_: " << std::endl << transform_J4_wrt_J3_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[6]: " << dofParentLinkNames_[4] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[5], frameNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[5], frameNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J5_wrt_J4_ = Eigen::Matrix4d::Identity();
    transform_J5_wrt_J4_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J5_wrt_J4_(0, 3) = transformStamped.transform.translation.x;
    transform_J5_wrt_J4_(1, 3) = transformStamped.transform.translation.y;
    transform_J5_wrt_J4_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J5_wrt_J4_: " << std::endl << transform_J5_wrt_J4_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] frameNames_[7]: " << dofParentLinkNames_[5] << std::endl;
      while(!tfBuffer.canTransform(frameNames_[6], frameNames_[7], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[6], frameNames_[7], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_J6_wrt_J5_ = Eigen::Matrix4d::Identity();
    transform_J6_wrt_J5_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_J6_wrt_J5_(0, 3) = transformStamped.transform.translation.x;
    transform_J6_wrt_J5_(1, 3) = transformStamped.transform.translation.y;
    transform_J6_wrt_J5_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_J6_wrt_J5_: " << std::endl << transform_J6_wrt_J5_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] ee_link_name_: " << ee_link_name_ << std::endl;
      while(!tfBuffer.canTransform(frameNames_[7], ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform(frameNames_[7], ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    transform_ToolMount_wrt_J6_ = Eigen::Matrix4d::Identity();
    transform_ToolMount_wrt_J6_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    transform_ToolMount_wrt_J6_(0, 3) = transformStamped.transform.translation.x;
    transform_ToolMount_wrt_J6_(1, 3) = transformStamped.transform.translation.y;
    transform_ToolMount_wrt_J6_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2Relative] transform_ToolMount_wrt_J6_: " << std::endl << transform_ToolMount_wrt_J6_ << std::endl;
  }
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void PointsOnRobot::updateTransformsTf2World() const
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[0], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[0], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_Base_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_Base_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_Base_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_Base_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_Base_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_Base_wrt_World_: " << std::endl << tf2_transform_Base_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[1], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[1], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ArmMount_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_ArmMount_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ArmMount_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ArmMount_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ArmMount_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_ArmMount_wrt_World_: " << std::endl << tf2_transform_ArmMount_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      while(!tfBuffer.canTransform("world", frameNames_[2], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[2], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J1_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J1_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J1_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J1_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J1_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J1_wrt_World_: " << std::endl << tf2_transform_J1_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] frameNames_[3]: " << frameNames_[3] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[3], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[3], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J2_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J2_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J2_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J2_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J2_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J2_wrt_World_: " << std::endl << tf2_transform_J2_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] frameNames_[4]: " << frameNames_[4] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[4], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[4], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());

      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J3_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J3_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J3_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J3_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J3_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J3_wrt_World_: " << std::endl << tf2_transform_J3_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] frameNames_[5]: " << frameNames_[5] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[5], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[5], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J4_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J4_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J4_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J4_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J4_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J4_wrt_World_: " << std::endl << tf2_transform_J4_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] frameNames_[6]: " << dofParentLinkNames_[4] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[6], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[6], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J5_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J5_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J5_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J5_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J5_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J5_wrt_World_: " << std::endl << tf2_transform_J5_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] frameNames_[7]: " << dofParentLinkNames_[5] << std::endl;
      while(!tfBuffer.canTransform("world", frameNames_[7], ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", frameNames_[7], ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_J6_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_J6_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_J6_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_J6_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_J6_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransformsTf2World] tf2_transform_J6_wrt_World_: " << std::endl << tf2_transform_J6_wrt_World_ << std::endl;
    //std::cout << "--------------------&&--------------------" << std::endl;
  }

  {
    try 
    {
      //std::cout << "[PointsOnRobot::updateTransformsTf2World] ee_link_name_: " << ee_link_name_ << std::endl;
      while(!tfBuffer.canTransform("world", ee_link_name_, ros::Time(0), ros::Duration(5.0))){;};
      transformStamped = tfBuffer.lookupTransform("world", ee_link_name_, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException& ex) 
    {
      ROS_ERROR("%s", ex.what());
      
      while(1){;}
      throw;
    }
    Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    tf2_transform_ToolMount_wrt_World_ = Eigen::Matrix4d::Identity();
    tf2_transform_ToolMount_wrt_World_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    tf2_transform_ToolMount_wrt_World_(0, 3) = transformStamped.transform.translation.x;
    tf2_transform_ToolMount_wrt_World_(1, 3) = transformStamped.transform.translation.y;
    tf2_transform_ToolMount_wrt_World_(2, 3) = transformStamped.transform.translation.z;

    //std::cout << "[PointsOnRobot::updateTransforms] tf2_transform_ToolMount_wrt_World_: " << std::endl << tf2_transform_ToolMount_wrt_World_ << std::endl;
  }
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void PointsOnRobot::updateTransformsPinocchio(ocs2::PinocchioInterface& pinocchioInterface,
                                              const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                              const Eigen::VectorXd& state) const
{
  //std::cout << "[PointsOnRobot::updateTransformsPinocchio] START" << std::endl;

  transform_Base_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ArmMount_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J1_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J2_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J3_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J4_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J5_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_J6_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();
  transform_ToolMount_wrt_World_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity();

  // Base wrt world
  const ocs2::scalar_t yaw = state[2];
  const ocs2::scalar_t pitch = 0.0;
  const ocs2::scalar_t roll = 0.0;

  Eigen::Quaternion<ocs2::scalar_t> baseOrientation = EulerToQuaternion(yaw, pitch, roll);
  Eigen::Matrix<ocs2::scalar_t, 3, 1> basePosition;
  basePosition << state[0], state[1], 0.0;

  transform_Base_wrt_World_.topLeftCorner(3,3) = baseOrientation.toRotationMatrix();
  transform_Base_wrt_World_.topRightCorner(3,1) = basePosition;

  // Update Pinocchio
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[0]].rotation();
  transform_Base_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[0]].translation();

  // Arm Base wrt World
  transform_ArmMount_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[1]].rotation();
  transform_ArmMount_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[1]].translation();

  // J1 wrt World
  transform_J1_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[2]].rotation();
  transform_J1_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[2]].translation();

  // J2 wrt World
  transform_J2_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[3]].rotation();
  transform_J2_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[3]].translation();

  // J3 wrt World
  transform_J3_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[4]].rotation();
  transform_J3_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[4]].translation();

  // J4 wrt World
  transform_J4_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[5]].rotation();
  transform_J4_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[5]].translation();

  // J5 wrt World
  transform_J5_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[6]].rotation();
  transform_J5_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[6]].translation();

  // J6 wrt World
  transform_J6_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[7]].rotation();
  transform_J6_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[7]].translation();

  // Tool Mount wrt World
  transform_ToolMount_wrt_World_.topLeftCorner(3,3) = data.oMf[frameIds_[8]].rotation();
  transform_ToolMount_wrt_World_.topRightCorner(3,1) = data.oMf[frameIds_[8]].translation();

  // Arm Base wrt Base
  transform_ArmMount_wrt_Base_ = transform_Base_wrt_World_.inverse() * transform_ArmMount_wrt_World_;

  // J1 wrt Arm Base
  transform_J1_wrt_ArmMount_ = transform_ArmMount_wrt_World_.inverse() * transform_J1_wrt_World_;

  // J2 wrt J1
  transform_J2_wrt_J1_ = transform_J1_wrt_World_.inverse() * transform_J2_wrt_World_;

  // J3 wrt J2
  transform_J3_wrt_J2_ = transform_J2_wrt_World_.inverse() * transform_J3_wrt_World_;

  // J4 wrt J3
  transform_J4_wrt_J3_ = transform_J3_wrt_World_.inverse() * transform_J4_wrt_World_;

  // J5 wrt J4
  transform_J5_wrt_J4_ = transform_J4_wrt_World_.inverse() * transform_J5_wrt_World_;

  // J6 wrt J5
  transform_J6_wrt_J5_ = transform_J5_wrt_World_.inverse() * transform_J6_wrt_World_;

  // Tool Mount wrt J6
  transform_ToolMount_wrt_J6_ = transform_J6_wrt_World_.inverse() * transform_ToolMount_wrt_World_;

  //std::cout << "[PointsOnRobot::updateTransformsPinocchio] END" << std::endl;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
void PointsOnRobot::updateTransformsPinocchio(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                              const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                              const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const
{
  //std::cout << "[PointsOnRobot::updateTransformsPinocchio(CppAD)] START" << std::endl;

  transform_Base_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_ArmMount_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J1_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J2_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J3_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J4_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J5_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_J6_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();
  transform_ToolMount_wrt_World_cppAd_ = Eigen::Matrix<ocs2::scalar_t, 4, 4>::Identity().cast<ad_scalar_t>();

  // Update Pinocchio
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const auto q = mappingCppAd.getPinocchioJointPosition(stateCppAd);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // Base wrt World
  transform_Base_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[0]].rotation();
  transform_Base_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[0]].translation();

  // Arm Base wrt World
  transform_ArmMount_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[1]].rotation();
  transform_ArmMount_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[1]].translation();

  // J1 wrt World
  transform_J1_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[2]].rotation();
  transform_J1_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[2]].translation();

  // J2 wrt World
  transform_J2_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[3]].rotation();
  transform_J2_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[3]].translation();

  // J3 wrt World
  transform_J3_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[4]].rotation();
  transform_J3_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[4]].translation();

  // J4 wrt World
  transform_J4_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[5]].rotation();
  transform_J4_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[5]].translation();

  // J5 wrt World
  transform_J5_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[6]].rotation();
  transform_J5_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[6]].translation();

  // J6 wrt World
  transform_J6_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[7]].rotation();
  transform_J6_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[7]].translation();

  // Tool Mount wrt World
  transform_ToolMount_wrt_World_cppAd_.topLeftCorner(3,3) = data.oMf[frameIds_[8]].rotation();
  transform_ToolMount_wrt_World_cppAd_.topRightCorner(3,1) = data.oMf[frameIds_[8]].translation();

  // Arm Base wrt Base
  transform_ArmMount_wrt_Base_cppAd_ = transform_Base_wrt_World_cppAd_.inverse() * transform_ArmMount_wrt_World_cppAd_;

  // J1 wrt Arm Base
  transform_J1_wrt_ArmMount_cppAd_ = transform_ArmMount_wrt_World_cppAd_.inverse() * transform_J1_wrt_World_cppAd_;

  // J2 wrt J1
  transform_J2_wrt_J1_cppAd_ = transform_J1_wrt_World_cppAd_.inverse() * transform_J2_wrt_World_cppAd_;

  // J3 wrt J2
  transform_J3_wrt_J2_cppAd_ = transform_J2_wrt_World_cppAd_.inverse() * transform_J3_wrt_World_cppAd_;

  // J4 wrt J3
  transform_J4_wrt_J3_cppAd_ = transform_J3_wrt_World_cppAd_.inverse() * transform_J4_wrt_World_cppAd_;

  // J5 wrt J4
  transform_J5_wrt_J4_cppAd_ = transform_J4_wrt_World_cppAd_.inverse() * transform_J5_wrt_World_cppAd_;

  // J6 wrt J5
  transform_J6_wrt_J5_cppAd_ = transform_J5_wrt_World_cppAd_.inverse() * transform_J6_wrt_World_cppAd_;

  // Tool Mount wrt J6
  transform_ToolMount_wrt_J6_cppAd_ = transform_J6_wrt_World_cppAd_.inverse() * transform_ToolMount_wrt_World_cppAd_;

  //std::cout << "[PointsOnRobot::updateTransformsPinocchio(CppAD)] END" << std::endl;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
Eigen::Matrix<ocs2::scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobot(ocs2::PinocchioInterface& pinocchioInterface,
                                                                               const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
                                                                               const Eigen::Matrix<ocs2::scalar_t, -1, 1>& state) const
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

  int dim = getDimPoints();
  if (dim == 0) 
  {

    return Eigen::Matrix<ocs2::scalar_t, 3, -1>(3, 0);
  }
  Eigen::Matrix<ocs2::scalar_t, 3, -1> pointsOnRobot_matrix(3, dim);

  //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] dim: " << dim << std::endl;
  
  updateTransformsPinocchio(pinocchioInterface, mapping, state);

  int linkIndex = 0;
  int pointIndex = 0;

  Eigen::Matrix<ocs2::scalar_t, 4, 4> transform_tmp = transform_Base_wrt_World_;

  // Points between base and arm mount
  Eigen::Matrix<ocs2::scalar_t, 4, 4> next_transform = transform_ArmMount_wrt_Base_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between arm mount and joint 1
  next_transform = transform_J1_wrt_ArmMount_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 1 and joint 2
  next_transform = transform_J2_wrt_J1_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 2 and joint 3
  next_transform = transform_J3_wrt_J2_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 3 and joint 4
  next_transform = transform_J4_wrt_J3_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 4 and joint 5
  next_transform = transform_J5_wrt_J4_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 5 and joint 6
  next_transform = transform_J6_wrt_J5_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }
  linkIndex++;
  transform_tmp = transform_tmp * next_transform;

  // Points between joint 6 and tool mount
  next_transform = transform_ToolMount_wrt_J6_;
  for (int i = 0; i < points_[linkIndex].size(); i++) 
  {
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] linkIndex: " << linkIndex << std::endl;
    //std::cout << "[PointsOnRobot::computeState2PointsOnRobot] pointIndex: " << pointIndex << std::endl;
    Eigen::Matrix<ocs2::scalar_t, 4, 1> directionVector = next_transform.col(3);
    directionVector.template head<3>() = directionVector.template head<3>() * points_[linkIndex][i];
    pointsOnRobot_matrix.col(pointIndex++) = (transform_tmp * directionVector).template head<3>();
  }

  return pointsOnRobot_matrix;
}
*/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<PointsOnRobot::ad_scalar_t, 3, -1> PointsOnRobot::computeState2PointsOnRobotCppAd(ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                                                const ocs2::PinocchioStateInputMapping<ad_scalar_t>& mappingCppAd,
                                                                                                const Eigen::Matrix<ad_scalar_t, -1, 1>& stateCppAd) const
{
  // NUA TODO: The function is specific to a robot model!  Generalize it!

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;

  auto state2MultiplePointsAd = [&, this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) 
  {
    Eigen::Matrix<ad_scalar_t, 3, -1> matrixPointsOnRobot = computeState2PointsOnRobotCppAd(pinocchioInterfaceAd, mappingCppAd, x);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixPointsOnRobot.data(), matrixPointsOnRobot.size());
  };

  cppAdInterface_.reset(new ocs2::CppAdInterface(state2MultiplePointsAd, 
                                                 stateDim, 
                                                 modelName + "_intermediate", 
                                                 modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::createModels(bool verbose) 
{
  cppAdInterface_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PointsOnRobot::loadModelsIfAvailable(bool verbose) 
{
  cppAdInterface_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
}
