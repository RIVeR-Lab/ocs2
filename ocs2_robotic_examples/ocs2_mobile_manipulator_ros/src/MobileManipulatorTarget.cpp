// LAST UPDATE: 2022.07.25
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

#include <ocs2_ros_interfaces/command/TargetTrajectoriesGazebo.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>

using namespace ocs2;
using namespace std;

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, 
                                                const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) 
{
  // time trajectory
  const scalar_array_t timeTrajectory{observation.time};
  
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  
  // input trajectory
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) 
{
  cout << "[MobileManipulatorTarget::main] START" << endl;

  const std::string robotMode = "mobile_manipulator";
  ros::init(argc, argv, robotMode + "_target");
  
  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener tflistener;

  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE AND SET PARAMETERS
  std::string world_frame_name, gz_model_msg_name, robot_name, drop_target_name;
  std::vector<std::string> name_pkgs_ign, name_pkgs_man, scan_data_path_pkgs_ign, scan_data_path_pkgs_man, target_names;
  double map_resolution;
  
  pnh.param<std::string>("/world_frame_name", world_frame_name, "");
  pnh.param<std::string>("/gz_model_msg_name", gz_model_msg_name, "");
  robot_name = "mobiman";
  target_names = {"red_cube"};
  drop_target_name = "bin_4_dropping_task";
  //target_names = {"normal_pkg","long_pkg","longwide_pkg","red_cube","green_cube","blue_cube"};

  //cout << "[MobileManipulatorTarget::main] DEBUG INF" << endl;
  //while(1);

  // Gazebo
  TargetTrajectoriesGazebo gu(nh, robotMode, gz_model_msg_name, robot_name, target_names, drop_target_name, &goalPoseToTargetTrajectories);
  gu.initializeInteractiveMarkerTarget();
  gu.initializeInteractiveMarkerAutoTarget();
  //gu.initializeInteractiveMarkerDropTarget();
  gu.initializeInteractiveMarkerModelMode();

  ros::Rate r(100);
  while(ros::ok)
  {
    //gu.updateObservationAndTarget();
    //gu.updateTarget();
    gu.publishTargetVisu();
    gu.publishGraspFrame();

    ros::spinOnce();
    r.sleep();
  }

  // Interactive Marker
  //TargetTrajectoriesInteractiveMarker targetPoseCommand(nh, robotMode, &goalPoseToTargetTrajectories);
  //targetPoseCommand.publishInteractiveMarker();

  cout << "[MobileManipulatorTarget::main] END" << endl;

  // Successful exit
  return 0;
}
