// LAST UPDATE: 2024.01.18
//
// AUTHOR: Neset Unver Akmandor
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
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
  //cout << "[MobileManipulatorTarget::main] START" << endl;

  ros::init(argc, argv, "mobile_manipulator_target");
  
  ros::MultiThreadedSpinner spinner(4);

  // INITIALIZE TRANSFORM LISTENER
  tf::TransformListener tflistener;

  // INITIALIZE THE MAIN ROS NODE HANDLE
  ros::NodeHandle nh;

  // INITIALIZE THE ROS NODE HANDLE FOR PARAMETERS
  ros::NodeHandle pnh("~");

  // INITIALIZE AND SET PARAMETERS
  std::string ns, topicPrefix, taskFile, world_frame_name, gz_model_msg_name, robot_name, drop_target_name;
  std::vector<std::string> name_pkgs_ign, name_pkgs_man, scan_data_path_pkgs_ign, scan_data_path_pkgs_man, target_names;
  double map_resolution, dummy_goal_pos_x, dummy_goal_pos_y, dummy_goal_pos_z, dummy_goal_ori_r, dummy_goal_ori_p, dummy_goal_ori_y;
  bool drlFlag, printOutFlag = true;

  ns = nh.getNamespace();
  cout << "[MobileManipulatorTarget::main] ns: " << ns << endl;
  topicPrefix = "/mobile_manipulator_";
  if (ns != "/")
  {
    topicPrefix = ns;
  }
  cout << "[MobileManipulatorTarget::main] topicPrefix: " << topicPrefix << endl;

  robot_name = "mobiman";
  target_names = {"red_cube"};
  drop_target_name = "bin_4_dropping_task";
  //target_names = {"normal_pkg","long_pkg","longwide_pkg","red_cube","green_cube","blue_cube"};

  //cout << "[MobileManipulatorTarget::main] DEBUG INF" << endl;
  //while(1);

  nh.getParam("/taskFile", taskFile);
  nh.getParam("/dummy_goal_pos_x", dummy_goal_pos_x);
  nh.getParam("/dummy_goal_pos_y", dummy_goal_pos_y);
  nh.getParam("/dummy_goal_pos_z", dummy_goal_pos_z);
  nh.getParam("/dummy_goal_ori_r", dummy_goal_ori_r);
  nh.getParam("/dummy_goal_ori_p", dummy_goal_ori_p);
  nh.getParam("/dummy_goal_ori_y", dummy_goal_ori_y);
  nh.getParam("/flag_drl", drlFlag);

  // read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  loadData::loadPtreeValue(pt, world_frame_name, "model_information.worldFrame", printOutFlag);
  //loadData::loadPtreeValue(pt, drlFlag, "model_settings.drlFlag", printOutFlag);

  /// NUA NOTE: LEFT HEREEEEEEEEE: CHECK IF DUMMY GOAL COMES RIGHT, ALSO SET THE GOAL TO RED CUBE!!!
  if (printOutFlag)
  {
    cout << "[MobileManipulatorTarget::main] world_frame_name: " << world_frame_name << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_pos_x: " << dummy_goal_pos_x << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_pos_y: " << dummy_goal_pos_y << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_pos_z: " << dummy_goal_pos_z << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_ori_r: " << dummy_goal_ori_r << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_ori_p: " << dummy_goal_ori_p << endl;
    cout << "[MobileManipulatorTarget::main] dummy_goal_ori_y: " << dummy_goal_ori_y << endl;
    cout << "[MobileManipulatorTarget::main] drlFlag: " << drlFlag << endl;
  }

  // Set TargetTrajectoriesGazebo
  TargetTrajectoriesGazebo gu(nh, ns, topicPrefix, gz_model_msg_name, robot_name, target_names, drop_target_name, &goalPoseToTargetTrajectories);
  gu.updateDummyGoal(dummy_goal_pos_x, dummy_goal_pos_y, dummy_goal_pos_z, dummy_goal_ori_r, dummy_goal_ori_p, dummy_goal_ori_y);

  if (drlFlag)
  {
    cout << "[MobileManipulatorTarget::main] DRL MODE IS ON!" << endl;
    gu.setTaskMode(1);
  }
  else
  {
    cout << "[MobileManipulatorTarget::main] MANUAL MODE IS ON!" << endl;
    gu.initializeInteractiveMarkerTarget();
    gu.initializeInteractiveMarkerAutoTarget();
    gu.initializeInteractiveMarkerModelMode();
    gu.setTaskMode(0);
  }
  
  gu.setTargetToEEPose();

  /*
  //ros::Rate r(100);
  while(ros::ok)
  {
    //gu.updateObservationAndTarget();
    gu.updateGoal(true);
    gu.publishTargetTrajectories();
    gu.publishGoalVisu();
    gu.publishTargetVisu();
    gu.publishGraspFrame();
    gu.publishDropFrame();

    //ros::spinOnce();
    //r.sleep();
  }
  */

  double update_dt = 0.01;
  ros::Timer updateTimer = nh.createTimer(ros::Duration(update_dt), &TargetTrajectoriesGazebo::updateCallback, &gu);

  spinner.spin();

  // Interactive Marker
  //TargetTrajectoriesInteractiveMarker targetPoseCommand(nh, robotMode, &goalPoseToTargetTrajectories);
  //targetPoseCommand.publishInteractiveMarker();

  //cout << "[MobileManipulatorTarget::main] END" << endl;

  // Successful exit
  return 0;
}
