// LAST UPDATE: 2023.05.02
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
// 
// TODO:

// --CUSTOM LIBRARIES--
#include "ocs2_ext_collision/ext_map_utility.h"

ExtMapUtility::ExtMapUtility()
{
  tflistener = new tf::TransformListener;
  world_frame_name = "";
  map_name = "";
  map_frame_name = "";
  sensor_pc2_msg_name = "";
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;
  x_range.clear();
  y_range.clear();
  z_range.clear();
  map_resolution_ = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 0;
  map_server_dt = 0;
  skip_cnt = 0;
  oct_msg.header.frame_id = world_frame_name;
  pc_msg.header.frame_id = world_frame_name;
  pc2_msg.header.frame_id = world_frame_name;
}

ExtMapUtility::ExtMapUtility(ros::NodeHandle& nh,
                             string oct_msg_name,
                             string pub_name_oct_msg)
{
  nh_ = nh;

  sub_oct_msg_ = nh.subscribe(oct_msg_name, 5, &ExtMapUtility::octMsgCallback, this);
  //sub_map = nh.subscribe(robot_param.local_map_msg, 5, &Tentabot::mapCallback, this);

  nh.advertise<octomap_msgs::Octomap>(pub_name_oct_msg, 10);
}

ExtMapUtility::ExtMapUtility(NodeHandle& nh,
                             NodeHandle& pnh,
                             string new_world_frame_name,
                             string gz_model_msg,
                             vector<string> frame_name_pkgs_ign, 
                             vector<string> frame_name_pkgs_man,
                             vector<sensor_msgs::PointCloud2> pc2_msg_gz_pkgs_ign,
                             vector<sensor_msgs::PointCloud2> pc2_msg_gz_pkgs_man,
                             double map_resolution)
{
  nh_ = nh;
  tflistener = new tf::TransformListener;

  //esdf_server_ptr_.reset(new voxblox::EsdfServer(nh, pnh));
  //esdf_server_ptr_->setClearFlag(true);

  world_frame_name = new_world_frame_name;

  frame_name_pkgs_ign_ = frame_name_pkgs_ign;
  frame_name_pkgs_man_ = frame_name_pkgs_man;

  pc2_msg_gz_pkgs_ign_ = pc2_msg_gz_pkgs_ign;
  pc2_msg_gz_pkgs_man_ = pc2_msg_gz_pkgs_man;

  map_resolution_ = map_resolution;
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  // Subscribers
  sub_gz_model_ = nh_.subscribe(gz_model_msg, 5, &ExtMapUtility::gazeboModelCallback, this);

  // Publishers
  pub_pc2_msg_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan", 100);
  pub_oct_msg_ = nh_.advertise<octomap_msgs::Octomap>("octomap_scan", 100);

  pub_pc2_msg_gz_pkg_ign_conveyor_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_conveyor", 10);
  pub_pc2_msg_gz_pkg_ign_red_cube_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_red_cube", 10);
  pub_pc2_msg_gz_pkg_ign_green_cube_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_green_cube", 10);
  pub_pc2_msg_gz_pkg_ign_blue_cube_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_blue_cube", 10);
  pub_pc2_msg_gz_pkg_man_normal_pkg_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_normal_pkg", 10);
  pub_pc2_msg_gz_pkg_man_long_pkg_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_long_pkg", 10);
  pub_pc2_msg_gz_pkg_man_longwide_pkg_ = nh_.advertise<sensor_msgs::PointCloud2>("pc2_scan_longwide_pkg", 10);

  pub_occ_distance_visu_ = nh_.advertise<visualization_msgs::Marker>("occupancy_distance", 10);
  pub_occ_distance_array_visu_ = nh_.advertise<visualization_msgs::MarkerArray>("occupancy_distance_array", 100);
}

ExtMapUtility::ExtMapUtility(NodeHandle& nh, 
                       string new_map_name,
                       string new_sensor_pc2_msg_name, 
                       string new_sensor_laser_msg_name)
{
  nh_ = nh;
  tflistener = new tf::TransformListener;

  map_name = new_map_name;

  world_frame_name = "";
  map_frame_name = "";
  sensor_pc2_msg_name = new_sensor_pc2_msg_name;
  sensor_pc2_direction = "";
  sensor_pc2_frame_name = "";
  
  sensor_pc2_pose.position.x = 0;
  sensor_pc2_pose.position.y = 0;
  sensor_pc2_pose.position.z = 0;
  sensor_pc2_pose.orientation.x = 0;
  sensor_pc2_pose.orientation.y = 0;
  sensor_pc2_pose.orientation.z = 0;
  sensor_pc2_pose.orientation.w = 0;
  map_pose = sensor_pc2_pose;

  sensor_laser_msg_name = new_sensor_laser_msg_name;
  
  x_range.clear();
  y_range.clear();
  z_range.clear();
  
  map_resolution_ = 0;
  pc_resolution_scale = 0;
  max_occupancy_belief_value = 100;
  map_server_dt = 0;
  skip_cnt = 0;
  
  oct_msg.header.frame_id = world_frame_name;
  pc_msg.header.frame_id = world_frame_name;
  pc2_msg.header.frame_id = world_frame_name;

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (PointCloud2)
  sub_pc2 = nh.subscribe(sensor_pc2_msg_name, 5, &ExtMapUtility::pc2Callback, this);

  // SUBSCRIBE TO THE OCCUPANCY SENSOR DATA (LaserScan)
  sub_laser = nh.subscribe(sensor_laser_msg_name, 5, &ExtMapUtility::laserCallback, this);

  // NUA TODO: MAP SERVICE
  //ros::ServiceServer service_reset_map_utility = nh.advertiseService("reset_map_utility", &ExtMapUtility::reset_map_utility, this);

  pub_oct_msg_ = nh.advertise<octomap_msgs::Octomap>("octomap_" + map_name, 10);
  pc_msg_pub = nh.advertise<sensor_msgs::PointCloud>("PC_" + map_name, 10);
  pc2_msg_pub = nh.advertise<sensor_msgs::PointCloud2>("PC2_" + map_name, 10);
  debug_array_visu_pub = nh.advertise<visualization_msgs::MarkerArray>("debug_array_" + map_name, 10);
  debug_visu_pub = nh.advertise<visualization_msgs::Marker>("debug_" + map_name, 10);
}

ExtMapUtility::ExtMapUtility(const ExtMapUtility& mu)
{
  nh_ = mu.nh_;
  tflistener = mu.tflistener;
  world_frame_name = mu.world_frame_name;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution_ = mu.map_resolution_;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  pub_oct_msg_ = mu.pub_oct_msg_;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
}

ExtMapUtility::~ExtMapUtility()
{
  //ROS_INFO( "Calling Destructor for ExtMapUtility..." );
  delete[] tflistener;
  //delete oct;
}

ExtMapUtility& ExtMapUtility::operator = (const ExtMapUtility& mu) 
{
  nh_ = mu.nh_;
  tflistener = mu.tflistener;
  world_frame_name = mu.world_frame_name;
  map_name = mu.map_name;
  map_frame_name = mu.map_frame_name;
  sensor_pc2_msg_name = mu.sensor_pc2_msg_name;
  sensor_pc2_direction = mu.sensor_pc2_direction;
  sensor_pc2_frame_name = mu.sensor_pc2_frame_name;
  measured_sensor_pc2_pose = mu.measured_sensor_pc2_pose;
  measured_map_pose = mu.measured_map_pose;
  sensor_pc2_pose = mu.sensor_pc2_pose;
  map_pose = mu.map_pose;
  x_range = mu.x_range;
  y_range = mu.y_range;
  z_range = mu.z_range;
  bbx_x_max = mu.bbx_x_max;
  bbx_x_min = mu.bbx_x_min;
  bbx_y_max = mu.bbx_y_max;
  bbx_y_min = mu.bbx_y_min;
  bbx_z_max = mu.bbx_z_max;
  bbx_z_min = mu.bbx_z_min;
  crop_x_max = mu.crop_x_max;
  crop_x_min = mu.crop_x_min;
  crop_y_max = mu.crop_y_max;
  crop_y_min = mu.crop_y_min;
  crop_z_max = mu.crop_z_max;
  crop_z_min = mu.crop_z_min;
  map_resolution_ = mu.map_resolution_;
  pc_resolution_scale = mu.pc_resolution_scale;
  max_occupancy_belief_value = mu.max_occupancy_belief_value;
  map_server_dt = mu.map_server_dt;
  skip_cnt = mu.skip_cnt;
  oct = mu.oct;
  oct_msg = mu.oct_msg;
  oct_pc = mu.oct_pc;
  measured_pc_msg = mu.measured_pc_msg;
  measured_pc2_msg = mu.measured_pc2_msg;
  pc_msg = mu.pc_msg;
  pc2_msg = mu.pc2_msg;
  debug_array_visu = mu.debug_array_visu;
  debug_visu = mu.debug_visu;
  pub_oct_msg_ = mu.pub_oct_msg_;
  pc_msg_pub = mu.pc_msg_pub;
  pc2_msg_pub = mu.pc2_msg_pub;
  debug_array_visu_pub = mu.debug_array_visu_pub;
  debug_visu_pub = mu.debug_visu_pub;
  return *this;
}

string ExtMapUtility::getWorldFrameName()
{
  return world_frame_name;
}

string ExtMapUtility::getMapName()
{
  return map_name;
}

string ExtMapUtility::getMapFrameName()
{
  return map_frame_name;
}

string ExtMapUtility::getSensorPC2MsgName()
{
  return sensor_pc2_msg_name;
}

string ExtMapUtility::getSensorPC2Direction()
{
  return sensor_pc2_direction;
}

string ExtMapUtility::getSensorPC2FrameName()
{
  return sensor_pc2_frame_name;
}

double ExtMapUtility::getSensorPC2MinRange()
{
  return sensor_pc2_min_range;
}

double ExtMapUtility::getSensorPC2MaxRange()
{
  return sensor_pc2_max_range;
}

double ExtMapUtility::getSensorPC2MaxYaw()
{
  return sensor_pc2_max_yaw;
}

double ExtMapUtility::getSensorPC2MaxPitch()
{
  return sensor_pc2_max_pitch;
}

string ExtMapUtility::getSensorLaserMsgName()
{
  return sensor_laser_msg_name;
}

float ExtMapUtility::getSensorLaserMaxRange()
{
  return sensor_laser_max_range;
}

geometry_msgs::Pose ExtMapUtility::getMeasuredSensorPC2Pose()
{
  return measured_sensor_pc2_pose;
}

geometry_msgs::Pose ExtMapUtility::getSensorPC2Pose()
{
  return sensor_pc2_pose;
}

geometry_msgs::Pose ExtMapUtility::getMeasuredMapPose()
{
  return measured_map_pose;
}

geometry_msgs::Pose ExtMapUtility::getMapPose()
{
  return map_pose;
}

vector<double> ExtMapUtility::getXRange()
{
  return x_range;
}

vector<double> ExtMapUtility::getYRange()
{
  return y_range;
}

vector<double> ExtMapUtility::getZRange()
{
  return z_range;
}

double ExtMapUtility::getBBxXMax()
{
  return bbx_x_max;
}

double ExtMapUtility::getBBxXMin()
{
  return bbx_x_min;
}

double ExtMapUtility::getBBxYMax()
{
  return bbx_y_max;
}

double ExtMapUtility::getBBxYMin()
{
  return bbx_y_min;
}

double ExtMapUtility::getBBxZMax()
{
  return bbx_z_max;
}

double ExtMapUtility::getBBxZMin()
{
  return bbx_z_min;
}

double ExtMapUtility::getCropXMax()
{
  return crop_x_max;
}

double ExtMapUtility::getCropXMin()
{
  return crop_x_min;
}

double ExtMapUtility::getCropYMax()
{
  return crop_y_max;
}

double ExtMapUtility::getCropYMin()
{
  return crop_y_min;
}

double ExtMapUtility::getCropZMax()
{
  return crop_z_max;
}

double ExtMapUtility::getCropZMin()
{
  return crop_z_min;
}

bool ExtMapUtility::getFilterGround()
{
  return filter_ground;
}

double ExtMapUtility::getFilterGroundThreshold()
{
  return filter_ground_threshold;
}

double ExtMapUtility::getMapResolution()
{
  return map_resolution_;
}

double ExtMapUtility::getPCResolutionScale()
{
  return pc_resolution_scale;
}

double ExtMapUtility::getMaxOccupancyBeliefValue()
{
  return max_occupancy_belief_value;
}

double ExtMapUtility::getMapServerDt()
{
  return map_server_dt;
}

bool ExtMapUtility::getLocalMapFlag()
{
  return local_map_flag;
}

bool ExtMapUtility::getDynamicFlag()
{
  return dynamic_flag;
}

int ExtMapUtility::getSkipCntResetSensorRange()
{
  return skip_cnt_reset_sensor_range;
}

shared_ptr<octomap::ColorOcTree> ExtMapUtility::getOct()
{
  return oct;
}

octomap_msgs::Octomap& ExtMapUtility::getOctMsg()
{
  return oct_msg;
}

octomap::Pointcloud& ExtMapUtility::getOctPC()
{
  return oct_pc;
}

sensor_msgs::PointCloud& ExtMapUtility::getMeasuredPCMsg()
{
  return measured_pc_msg;
}

sensor_msgs::PointCloud2& ExtMapUtility::getMeasuredPC2Msg()
{
  return measured_pc2_msg;
}

sensor_msgs::PointCloud& ExtMapUtility::getPCMsg()
{
  return pc_msg;
}

sensor_msgs::PointCloud2& ExtMapUtility::getPC2Msg()
{
  return pc2_msg;
}

ros::Publisher ExtMapUtility::getOctMsgPub()
{
  return pub_oct_msg_;
}

ros::Publisher ExtMapUtility::getPCMsgPub()
{
  return pc_msg_pub;
}

ros::Publisher ExtMapUtility::getPC2MsgPub()
{
  return pc2_msg_pub;
}

void ExtMapUtility::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = nh;
}

void ExtMapUtility::setWorldFrameName(string new_world_frame_name)
{
  world_frame_name = new_world_frame_name;
}

void ExtMapUtility::setMapName(string new_map_name)
{
  map_name = new_map_name;
}

void ExtMapUtility::setMapFrameName(string new_map_frame_name)
{
  map_frame_name = new_map_frame_name;
}

void ExtMapUtility::setSensorPC2MsgName(string new_sensor_pc2_msg_name)
{
  sensor_pc2_msg_name = new_sensor_pc2_msg_name;
}

void ExtMapUtility::setSensorPC2Direction(string new_sensor_pc2_direction)
{
  sensor_pc2_direction = new_sensor_pc2_direction;
}

void ExtMapUtility::setSensorPC2FrameName(string new_sensor_pc2_frame_name)
{
  sensor_pc2_frame_name = new_sensor_pc2_frame_name;
}

void ExtMapUtility::setSensorPC2MinRange(double new_sensor_pc2_min_range)
{
  sensor_pc2_min_range = new_sensor_pc2_min_range;
}

void ExtMapUtility::setSensorPC2MaxRange(double new_sensor_pc2_max_range)
{
  sensor_pc2_max_range = new_sensor_pc2_max_range;
}

void ExtMapUtility::setSensorPC2MaxYaw(double new_sensor_pc2_max_yaw)
{
  sensor_pc2_max_yaw = new_sensor_pc2_max_yaw;
}

void ExtMapUtility::setSensorPC2MaxPitch(double new_sensor_pc2_max_pitch)
{
  sensor_pc2_max_pitch = new_sensor_pc2_max_pitch;
}

void ExtMapUtility::setSensorLaserMsgName(string new_sensor_laser_msg_name)
{
  sensor_laser_msg_name = new_sensor_laser_msg_name;
}

void ExtMapUtility::setSensorLaserMaxRange(float new_sensor_laser_max_range)
{
  sensor_laser_max_range = new_sensor_laser_max_range;
}

void ExtMapUtility::setMeasuredSensorPC2Pose(geometry_msgs::Pose new_measured_sensor_pc2_pose)
{
  measured_sensor_pc2_pose = new_measured_sensor_pc2_pose;
}

void ExtMapUtility::setSensorPC2Pose(geometry_msgs::Pose new_sensor_pc2_pose)
{
  sensor_pc2_pose = new_sensor_pc2_pose;
}

void ExtMapUtility::setMeasuredMapPose(geometry_msgs::Pose new_measured_map_pose)
{
  measured_map_pose = new_measured_map_pose;
}

void ExtMapUtility::setMapPose(geometry_msgs::Pose new_map_pose)
{
  map_pose = new_map_pose;
}

void ExtMapUtility::setXRange(double x0, double x1)
{
  x_range[0] = x0;
  x_range[1] = x1;
}

void ExtMapUtility::setYRange(double y0, double y1)
{
  y_range[0] = y0;
  y_range[1] = y1;
}

void ExtMapUtility::setZRange(double z0, double z1)
{
  z_range[0] = z0;
  z_range[1] = z1;
}

void ExtMapUtility::setBBxXMax(double new_bbx_x_max)
{
  bbx_x_max = new_bbx_x_max;
}

void ExtMapUtility::setBBxXMin(double new_bbx_x_min)
{
  bbx_x_min = new_bbx_x_min;
}

void ExtMapUtility::setBBxYMax(double new_bbx_y_max)
{
  bbx_y_max = new_bbx_y_max;
}

void ExtMapUtility::setBBxYMin(double new_bbx_y_min)
{
  bbx_y_min = new_bbx_y_min;
}

void ExtMapUtility::setBBxZMax(double new_bbx_z_max)
{
  bbx_z_max = new_bbx_z_max;
}

void ExtMapUtility::setBBxZMin(double new_bbx_z_min)
{
  bbx_z_min = new_bbx_z_min;
}

void ExtMapUtility::setCropXMax(double new_crop_x_max)
{
  crop_x_max = new_crop_x_max;
}

void ExtMapUtility::setCropXMin(double new_crop_x_min)
{
  crop_x_min = new_crop_x_min;
}

void ExtMapUtility::setCropYMax(double new_crop_y_max)
{
  crop_y_max = new_crop_y_max;
}

void ExtMapUtility::setCropYMin(double new_crop_y_min)
{
  crop_y_min = new_crop_y_min;
}

void ExtMapUtility::setCropZMax(double new_crop_z_max)
{
  crop_z_max = new_crop_z_max;
}

void ExtMapUtility::setCropZMin(double new_crop_z_min)
{
  crop_z_min = new_crop_z_min;
}

void ExtMapUtility::setFilterGround(bool new_filter_ground)
{
  filter_ground =  new_filter_ground;
}

void ExtMapUtility::setFilterGroundThreshold(double new_filter_ground_threshold)
{
  filter_ground_threshold =  new_filter_ground_threshold;
}

void ExtMapUtility::setMapResolution(double map_resolution)
{
  map_resolution_ = map_resolution;

  //oct = new ColorOcTree(map_resolution_);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);
}

void ExtMapUtility::setPCResolutionScale(double new_pc_resolution_scale)
{
  pc_resolution_scale = new_pc_resolution_scale;
}

void ExtMapUtility::setMaxOccupancyBeliefValue(double new_max_occupancy_belief_value)
{
  max_occupancy_belief_value = new_max_occupancy_belief_value;
}

void ExtMapUtility::setMapServerDt(double new_map_server_dt)
{
  map_server_dt = new_map_server_dt;
}

void ExtMapUtility::setLocalMapFlag(bool new_local_map_flag)
{
  local_map_flag = new_local_map_flag;
}

void ExtMapUtility::setDynamicFlag(bool new_dynamic_flag)
{
  dynamic_flag = new_dynamic_flag;
}

void ExtMapUtility::setSkipCntResetSensorRange(double new_skip_cnt_reset_sensor_range)
{
  skip_cnt_reset_sensor_range = new_skip_cnt_reset_sensor_range;
}

void ExtMapUtility::setOctPC(octomap::Pointcloud& new_oct_pc)
{
  oct_pc = new_oct_pc;
}

void ExtMapUtility::setMeasuredPCMsg(sensor_msgs::PointCloud& new_measured_pc_msg)
{
  measured_pc_msg = new_measured_pc_msg;
}

void ExtMapUtility::setMeasuredPC2Msg(sensor_msgs::PointCloud2& new_measured_pc2_msg)
{
  measured_pc2_msg = new_measured_pc2_msg;
}

void ExtMapUtility::setPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg = new_pc_msg;
}

void ExtMapUtility::setPC2Msg(sensor_msgs::PointCloud2& new_pc2_msg)
{
  pc2_msg = new_pc2_msg;
}

void ExtMapUtility::setFrameNamePkgsIgn(vector<string> frame_name_pkgs_ign)
{
  frame_name_pkgs_ign_ = frame_name_pkgs_ign;
}

void ExtMapUtility::setFrameNamePkgsMan(vector<string> frame_name_pkgs_man)
{
  frame_name_pkgs_man_ = frame_name_pkgs_man;
}

void ExtMapUtility::setPubOctMsg(string pub_name_oct_msg)
{
  pub_oct_msg_ = nh_.advertise<octomap_msgs::Octomap>(pub_name_oct_msg, 10);
}

void ExtMapUtility::setPubOccDistVisu(string pub_name_occ_dist_visu)
{
  pub_occ_distance_visu_ = nh_.advertise<visualization_msgs::Marker>(pub_name_occ_dist_visu, 10);
}

void ExtMapUtility::setPubOccDistArrayVisu(string pub_name_occ_dist_array_visu)
{
  pub_occ_distance_array_visu_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_name_occ_dist_array_visu, 100);
}

void ExtMapUtility::createTimerPubOctDistVisu(ros::Duration dt)
{
  timer_pub_oct_dist_visu_ = nh_.createTimer(dt, &ExtMapUtility::publishOccDistanceVisu, this);
}

double ExtMapUtility::randdouble(double from, double to)
{
  double f = (double)rand() / RAND_MAX;
  return from + f * (to - from);  
}

bool ExtMapUtility::isInBBx(double px, double py, double pz, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (px >= min_x) && (px < max_x) && (py >= min_y) && (py < max_y) && (pz >= min_z) && (pz < max_z);
}

bool ExtMapUtility::isInBBx(geometry_msgs::Point po, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (po.x >= min_x) && (po.x < max_x) && (po.y >= min_y) && (po.y < max_y) && (po.z >= min_z) && (po.z < max_z);
}

bool ExtMapUtility::isInBBx(tf::Vector3 po, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
  return (po.x() >= min_x) && (po.x() < max_x) && (po.y() >= min_y) && (po.y() < max_y) && (po.z() >= min_z) && (po.z() < max_z);
}

bool ExtMapUtility::isInBBx(tf::Vector3 po, tf::Vector3 mini, tf::Vector3 maxi)
{
  return (po.x() >= mini.x()) && (po.x() < maxi.x()) && (po.y() >= mini.y()) && (po.y() < maxi.y()) && (po.z() >= mini.z()) && (po.z() < maxi.z());
}

bool ExtMapUtility::isInBBx(tf::Vector3 po, octomap::point3d mini, octomap::point3d maxi)
{
  return (po.x() >= mini.x()) && (po.x() < maxi.x()) && (po.y() >= mini.y()) && (po.y() < maxi.y()) && (po.z() >= mini.z()) && (po.z() < maxi.z());
}

string ExtMapUtility::createFileName()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
  std::string str(buffer);

  return str;
}

double ExtMapUtility::find_Euclidean_distance(geometry_msgs::Point p1, tf::Vector3 p2)
{
  return ( sqrt( pow(p1.x - p2.x(), 2) + pow(p1.y - p2.y(), 2) + pow(p1.z - p2.z(), 2) ) );
}

double ExtMapUtility::find_Euclidean_distance(geometry_msgs::Point p1, geometry_msgs::Point p2) const
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

double ExtMapUtility::find_Euclidean_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
  return ( sqrt( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2) ) );
}

void ExtMapUtility::resetMap()
{
  //oct = new ColorOcTree(map_resolution_);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  fillOctMsgFromOct();
}

void ExtMapUtility::transformPoint(string frame_from,
                                string frame_to,
                                geometry_msgs::Point& p_to_msg)
{
  tf::Point p_from_tf;
  geometry_msgs::Point p_from_msg;
  p_from_msg.x = 0;
  p_from_msg.y = 0;
  p_from_msg.z = 0;
  tf::pointMsgToTF(p_from_msg, p_from_tf);
  tf::Stamped<tf::Point> p_from_stamped_tf(p_from_tf, ros::Time(0), frame_from);
  tf::Stamped<tf::Point> p_to_stamped_tf;
  geometry_msgs::PointStamped p_to_stamped_msg;

  try
  {
    tflistener -> transformPoint(frame_to, p_from_stamped_tf, p_to_stamped_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_INFO("Tentabot::transformPoint -> Couldn't get transform!");
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  tf::pointStampedTFToMsg(p_to_stamped_tf, p_to_stamped_msg);
  p_to_msg = p_to_stamped_msg.point;
}

void ExtMapUtility::createColorOcTree(double map_resolution, sensor_msgs::PointCloud& new_pc, vector<int> color_RGB)
{
  map_resolution_ = map_resolution;
  //oct = new ColorOcTree(map_resolution);
  oct = std::make_shared<octomap::ColorOcTree>(map_resolution_);

  int pcd_size = new_pc.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    oct -> updateNode(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z, true);
    oct -> setNodeColor(oct -> coordToKey(new_pc.points[i].x, new_pc.points[i].y, new_pc.points[i].z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
}

vector<geometry_msgs::Point32> ExtMapUtility::extract_pc_from_node_center(geometry_msgs::Point center)
{
  double half_vdim = 0.5 * map_resolution_;
  if (pc_resolution_scale <= 0)
  {
    ROS_WARN("ExtMapUtility::extract_pc_from_node_center -> pc_resolution_scale has not set. It is set to 1.");
    pc_resolution_scale = 1;
  }
  double pc_resolution = pc_resolution_scale * map_resolution_;
  vector<geometry_msgs::Point32> opc;
  geometry_msgs::Point minp;
  geometry_msgs::Point maxp;
  minp.x = center.x - half_vdim;                    // (-x, -y, -z)
  minp.y = center.y - half_vdim;
  minp.z = center.z - half_vdim;
  maxp.x = center.x + half_vdim;                    // (+x, +y, +z)
  maxp.y = center.y + half_vdim;
  maxp.z = center.z + half_vdim;

  for(double i = minp.x; i < maxp.x; i += pc_resolution)
  {
    for(double j = minp.y; j < maxp.y; j += pc_resolution)
    {
      for(double k = minp.z; k < maxp.z; k += pc_resolution)
      {
        geometry_msgs::Point32 newp;
        newp.x = i;
        newp.y = j;
        newp.z = k;
        opc.push_back(newp);

        if(i == minp.x)
        {
          geometry_msgs::Point32 newpx;
          newpx.x = maxp.x;
          newpx.y = j;
          newpx.z = k;
          opc.push_back(newpx);
        }
        
        if(j == minp.y)
        {
          geometry_msgs::Point32 newpy;
          newpy.x = i;
          newpy.y = maxp.y;
          newpy.z = k;
          opc.push_back(newpy);
        }

        if(k == minp.z)
        {
          geometry_msgs::Point32 newpz;
          newpz.x = i;
          newpz.y = j;
          newpz.z = maxp.z;
          opc.push_back(newpz);
        }
        
        if(i == minp.x && j == minp.y)
        {
          geometry_msgs::Point32 newpxy;
          newpxy.x = maxp.x;
          newpxy.y = maxp.y;
          newpxy.z = k;
          opc.push_back(newpxy);
        }      

        if(i == minp.x && k == minp.z)
        {
          geometry_msgs::Point32 newpxz;
          newpxz.x = maxp.x;
          newpxz.y = j;
          newpxz.z = maxp.z;
          opc.push_back(newpxz);
        }
        
        if(j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpyz;
          newpyz.x = i;
          newpyz.y = maxp.y;
          newpyz.z = maxp.z;
          opc.push_back(newpyz);
        }
        
        if(i == minp.x && j == minp.y && k == minp.z)
        {
          geometry_msgs::Point32 newpxyz;
          newpxyz.x = maxp.x;
          newpxyz.y = maxp.y;
          newpxyz.z = maxp.z;
          opc.push_back(newpxyz);
        }
      }
    }
  }
  return opc;
}

void ExtMapUtility::fillOct(sensor_msgs::PointCloud& pc_msg)
{
  oct -> clear();

  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void ExtMapUtility::fillOctFromMeasuredPCMsg()
{
  oct -> clear();

  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct -> updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
}

void ExtMapUtility::fillOctMsgFromOct()
{
  oct_msg.data.clear();
  oct_msg.header.frame_id = world_frame_name;
  oct_msg.binary = false;
  oct_msg.id = map_name;
  oct_msg.resolution = map_resolution_;
  octomap_msgs::fullMapToMsg(*oct, oct_msg);
}

void ExtMapUtility::fillPCMsgFromOct()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
  {
    geometry_msgs::Point32 op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    pc_msg.points.push_back(op);
  }
  pc_msg.header.frame_id = world_frame_name;
}

void ExtMapUtility::fillPCMsgFromOctByResolutionScale()
{
  pc_msg.points.clear();
  
  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
  {
    geometry_msgs::Point op;
    op.x = it.getCoordinate().x();
    op.y = it.getCoordinate().y();
    op.z = it.getCoordinate().z();

    vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(op);
    for(int i = 0; i < opc.size(); i++)
    {
      pc_msg.points.push_back(opc[i]);
    }
  }
  pc_msg.header.frame_id = world_frame_name;
}

void ExtMapUtility::fillOccDistanceVisu(geometry_msgs::Point& p0, geometry_msgs::Point& p1) const
{
  occ_distance_visu_.ns = "occupancy_distance";
  occ_distance_visu_.id = 1;
  occ_distance_visu_.type = visualization_msgs::Marker::ARROW;
  occ_distance_visu_.action = visualization_msgs::Marker::ADD;
  occ_distance_visu_.pose.orientation.w = 1.0;
  occ_distance_visu_.scale.x = 0.03;
  occ_distance_visu_.scale.y = 0.05;
  occ_distance_visu_.scale.z = 0.05;
  occ_distance_visu_.color.r = 1.0;
  occ_distance_visu_.color.g = 0.0;
  occ_distance_visu_.color.b = 1.0;
  occ_distance_visu_.color.a = 1.0;

  occ_distance_visu_.points.clear();
  occ_distance_visu_.points.push_back(p0);
  occ_distance_visu_.points.push_back(p1);

  occ_distance_visu_.header.frame_id = world_frame_name;
  occ_distance_visu_.header.seq++;
  occ_distance_visu_.header.stamp = ros::Time::now();
}

void ExtMapUtility::fillOccDistanceArrayVisu(vector<geometry_msgs::Point>& p0_vec, vector<geometry_msgs::Point>& p1_vec) const
{
  visualization_msgs::MarkerArray occ_distance_array_visu;
  for (size_t i = 0; i < p0_vec.size(); i++)
  {
    visualization_msgs::Marker occ_distance_visu;
    occ_distance_visu.ns = "occupancy_distance_" + i;
    occ_distance_visu.id = i;
    occ_distance_visu.type = visualization_msgs::Marker::ARROW;
    occ_distance_visu.action = visualization_msgs::Marker::ADD;
    occ_distance_visu.pose.orientation.w = 1.0;
    occ_distance_visu.scale.x = 0.03;
    occ_distance_visu.scale.y = 0.05;
    occ_distance_visu.scale.z = 0.05;
    occ_distance_visu.color.r = 1.0;
    occ_distance_visu.color.g = 0.0;
    occ_distance_visu.color.b = 1.0;
    occ_distance_visu.color.a = 1.0;

    occ_distance_visu.points.clear();
    occ_distance_visu.points.push_back(p0_vec[i]);
    occ_distance_visu.points.push_back(p1_vec[i]);

    occ_distance_visu.header.frame_id = world_frame_name;
    //occ_distance_visu.header.seq++;
    occ_distance_visu.header.stamp = ros::Time::now();

    occ_distance_array_visu.markers.push_back(occ_distance_visu);
  }
  occ_distance_array_visu_ = occ_distance_array_visu;
}

void ExtMapUtility::fillDebugArrayVisu(vector<tf::Vector3>& v)
{
  debug_array_visu.markers.clear();

  for(int i = 0; i < v.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.ns = "point" + to_string(i);
    marker.id = i;
    marker.header.frame_id = world_frame_name;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = v[i].x();
    marker.pose.position.y = v[i].y();
    marker.pose.position.z = v[i].z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    debug_array_visu.markers.push_back(marker); 
  }
}

void ExtMapUtility::fillDebugVisu(vector<tf::Vector3>& v)
{
  debug_visu.points.clear();

  debug_visu.ns = "points";
  debug_visu.id = 1;
  debug_visu.header.frame_id = world_frame_name;
  debug_visu.type = visualization_msgs::Marker::POINTS;
  debug_visu.action = visualization_msgs::Marker::ADD;
  debug_visu.pose.orientation.w = 1.0;
  debug_visu.scale.x = 0.04;
  debug_visu.scale.y = 0.04;
  debug_visu.scale.z = 0.04;
  debug_visu.color.r = 1.0;
  debug_visu.color.g = 1.0;
  debug_visu.color.b = 0.0;
  debug_visu.color.a = 1.0;

  debug_visu.points.clear();
  for(int i = 0; i < v.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = v[i].x();
    p.y = v[i].y();
    p.z = v[i].z();

    debug_visu.points.push_back(p); 
  }
}

void ExtMapUtility::insertToOctFromOctPC(octomap::point3d& sensor_pc2_origin)
{
  oct -> insertPointCloud(oct_pc, sensor_pc2_origin, sensor_pc2_max_range, false, true);
}

void ExtMapUtility::addToOct(sensor_msgs::PointCloud& pc_msg)
{
  for(int i = 0; i < pc_msg.points.size(); i++)
  {
    oct -> updateNode(pc_msg.points[i].x, pc_msg.points[i].y, pc_msg.points[i].z, true);
  }
}

void ExtMapUtility::addToOct(sensor_msgs::PointCloud2& pc2_msg)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      oct -> updateNode(*iter_x, *iter_y, *iter_z, true);
    }
  }
}

void ExtMapUtility::addToOctFromMeasuredPCMsg()
{
  for(int i = 0; i < measured_pc_msg.points.size(); i++)
  {
    oct -> updateNode(measured_pc_msg.points[i].x, measured_pc_msg.points[i].y, measured_pc_msg.points[i].z, true);
  }
  fillOctMsgFromOct();
}

void ExtMapUtility::addToPCMsg(sensor_msgs::PointCloud& new_pc_msg)
{
  pc_msg.header = new_pc_msg.header;
  pc_msg.channels = new_pc_msg.channels;
  pc_msg.points.insert(end(pc_msg.points), begin(new_pc_msg.points), end(new_pc_msg.points));
}

void ExtMapUtility::addToPCMsg(geometry_msgs::Point32 new_point)
{
  pc_msg.points.push_back(new_point);
}

void ExtMapUtility::clearMeasuredPCMsg()
{
  measured_pc_msg.points.clear();
}

void ExtMapUtility::clearPCMsg()
{
  pc_msg.points.clear();
}

bool ExtMapUtility::isOccupied(double x, double y, double z)
{
  OcTreeNode* node = oct -> search(x, y, z);
  if(node)
  {
    return oct -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

bool ExtMapUtility::isOccupied(geometry_msgs::Point po)
{
  OcTreeNode* node = oct -> search(po.x, po.y, po.z);
  if(node)
  {
    return oct -> isNodeOccupied(node);
  }
  else
  {
    return false;
  }
}

void ExtMapUtility::setSensorRangeQuery(float query_point_resolution)
{
  // SET THE QUERY POINT (BOX)
  fcl::Box<float> query(query_point_resolution, query_point_resolution, query_point_resolution);
  query_sharedPtr = std::make_shared< fcl::Box<float> > (query);
}

void ExtMapUtility::constructCameraSensorRange()
{
  double pi = 3.141592653589793;
  tf::Transform aligner;
  aligner.setIdentity();
  tf::Quaternion aligner_q(0, 0, 0, 1);
  if (sensor_pc2_direction == "y")
  {
    //cout << "ExtMapUtility::constructCameraSensorRange -> Sensor is in y direction!" << endl;
    aligner_q.setRPY(0, 0, 0.5*pi);
  }
  else if (sensor_pc2_direction == "z")
  {
    //cout << "ExtMapUtility::constructCameraSensorRange -> Sensor is in z direction!" << endl;
    aligner_q.setRPY(0, -0.5*pi, 0);
  }

  aligner.setRotation(aligner_q);
  
  // SET THE CAMERA SENSOR RANGE AS POLYTOPE
  tf::Vector3 vert;

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v0( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v1( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v2( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_min_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_min_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v3( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v4( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(-sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(-sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v5( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v6( vert.x(), vert.y(), vert.z() );

  vert.setValue(  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * cos(sensor_pc2_max_yaw), 
                  sensor_pc2_max_range * cos(-sensor_pc2_max_pitch) * sin(sensor_pc2_max_yaw),
                  sensor_pc2_max_range * sin(-sensor_pc2_max_pitch) );
  vert = aligner * vert;
  fcl::Vector3<float> v7( vert.x(), vert.y(), vert.z() );

  std::vector< fcl::Vector3<float> > vertices;
  auto vertices_sharedPtr = std::make_shared< std::vector< fcl::Vector3<float> > > (vertices);

  vertices_sharedPtr.get()->push_back(v0);
  vertices_sharedPtr.get()->push_back(v1);
  vertices_sharedPtr.get()->push_back(v2);
  vertices_sharedPtr.get()->push_back(v3);
  vertices_sharedPtr.get()->push_back(v4);
  vertices_sharedPtr.get()->push_back(v5);
  vertices_sharedPtr.get()->push_back(v6);
  vertices_sharedPtr.get()->push_back(v7);

  vector<int> f0 = {4,0,1,2,3};
  vector<int> f1 = {4,4,5,1,0};
  vector<int> f2 = {4,7,6,5,4};
  vector<int> f3 = {4,3,2,6,7};
  vector<int> f4 = {4,4,0,3,7};
  vector<int> f5 = {4,1,5,6,2};

  vector<int> faces;
  auto faces_sharedPtr = std::make_shared< std::vector<int> > (faces);

  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f0.begin(), f0.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f1.begin(), f1.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f2.begin(), f2.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f3.begin(), f3.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f4.begin(), f4.end());
  faces_sharedPtr.get()->insert(faces_sharedPtr.get()->end(), f5.begin(), f5.end());

  int num_faces = 6;
  bool throw_if_invalid = true;

  fcl::Convex<float> sensor_pc2_range_hull(vertices_sharedPtr, num_faces, faces_sharedPtr, throw_if_invalid);

  sensor_pc2_range_sharedPtr = std::make_shared< fcl::Convex<float> > (sensor_pc2_range_hull);
}

void ExtMapUtility::constructLaserSensorRange(float sensor_laser_z_range)
{
  // SET THE LASER SENSOR COVERAGE AS CYLINDER
  fcl::Cylinder<float> sensor_laser_hull(sensor_laser_max_range, sensor_laser_z_range);
  sensor_laser_range_sharedPtr = std::make_shared< fcl::Cylinder<float> > (sensor_laser_hull);
}

bool ExtMapUtility::isInSensorCameraRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the camera sensor range collision obj
    fcl::CollisionObjectf* sensor_pc2_range_collision_obj = new fcl::CollisionObjectf(sensor_pc2_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_pc2_range_result.clear();

    fcl::collide(sensor_pc2_range_collision_obj, query_collision_obj, sensor_pc2_range_request, sensor_pc2_range_result);

    delete query_collision_obj;
    delete sensor_pc2_range_collision_obj;
    
    return sensor_pc2_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

bool ExtMapUtility::isInSensorLaserRange(tf::Vector3 query, bool on_flag)
{
  if (on_flag)
  {
    // Set the query collision obj
    fcl::Vector3f query_T(query.x(), query.y(), query.z());
    fcl::Transform3f query_tr = fcl::Transform3f::Identity();
    query_tr.translation() = query_T;
    fcl::CollisionObjectf* query_collision_obj = new fcl::CollisionObjectf(query_sharedPtr, query_tr);

    // Set the laser sensor range collision obj
    fcl::CollisionObjectf* laser_sensor_collision_obj = new fcl::CollisionObjectf(sensor_laser_range_sharedPtr, fcl::Transform3f::Identity());

    // Perform collision tests
    sensor_laser_range_result.clear();

    fcl::collide(laser_sensor_collision_obj, query_collision_obj, sensor_laser_range_request, sensor_laser_range_result);

    delete query_collision_obj;
    delete laser_sensor_collision_obj;
    
    return sensor_laser_range_result.isCollision();
  }
  else
  {
    return false;
  }
}

bool ExtMapUtility::isInCube(geometry_msgs::Point po, geometry_msgs::Point center, double rad)
{
  return (po.x >= center.x - rad) && (po.x <= center.x + rad) && (po.y >= center.y - rad) && (po.y <= center.y + rad) && (po.z >= center.z - rad) && (po.z <= center.z + rad);
}

bool ExtMapUtility::isOccupiedByGoal(double x, double y, double z, vector<geometry_msgs::Pose> goal)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  double free_rad = 2 * map_resolution_;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

bool ExtMapUtility::isOccupiedByGoal(geometry_msgs::Point po, vector<geometry_msgs::Pose> goal)
{
  double free_rad = 2 * map_resolution_;

  for(int i = 0; i < goal.size(); i++)
  {
    if( isInCube(po, goal[i].position, free_rad) )
    {
      return true;
    }
  }
  return false;
}

void ExtMapUtility::addStaticObstacleByResolutionScale2PCMsg(geometry_msgs::Point po)
{
  vector<geometry_msgs::Point32> opc = extract_pc_from_node_center(po);
  for(int i = 0; i < opc.size(); i++)
  {
    pc_msg.points.push_back(opc[i]);
  }
  pc_msg.header.frame_id = world_frame_name;
}

bool ExtMapUtility::addStaticObstacle(double x, 
                                   double y, 
                                   double z, 
                                   bool constraint_flag, 
                                   vector<geometry_msgs::Pose> goal, 
                                   geometry_msgs::Point robot_center, 
                                   double robot_free_rad, 
                                   vector<int> color_RGB)
{
  geometry_msgs::Point po;
  po.x = x;
  po.y = y;
  po.z = z;

  if( constraint_flag && (isOccupied(x, y, z) || isOccupiedByGoal(x, y, z, goal) || isInCube(po, robot_center, robot_free_rad + map_resolution_)) )
  {
    return false;
  }
  else
  {
    oct -> updateNode(x, y, z, true);
    oct -> setNodeColor(oct -> coordToKey(x, y, z), color_RGB[0], color_RGB[1], color_RGB[2]);
    fillOctMsgFromOct();
    addStaticObstacleByResolutionScale2PCMsg(po);
    return true;
  }
}

vector<bool> ExtMapUtility::addStaticObstacle(sensor_msgs::PointCloud& pcd, 
                                           bool constraint_flag, 
                                           vector<geometry_msgs::Pose> goal, 
                                           geometry_msgs::Point robot_center, 
                                           double robot_free_rad, 
                                           vector<int> color_RGB)
{
  vector<bool> pc_add_result;
      
  int pcd_size = pcd.points.size();

  for(int i = 0; i < pcd_size; i++)
  {
    pc_add_result.push_back( addStaticObstacle(pcd.points[i].x, pcd.points[i].y, pcd.points[i].z, constraint_flag, goal, robot_center, robot_free_rad, color_RGB) );
  }
  return pc_add_result;
}

void ExtMapUtility::createRandomStaticObstacleMap(int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(x_range[0], x_range[1]);
    rp.position.y = randdouble(y_range[0], y_range[1]);
    rp.position.z = randdouble(z_range[0], z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution_)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(x_range[0], x_range[1]);
      rp.position.y = randdouble(y_range[0], y_range[1]);
      rp.position.z = randdouble(z_range[0], z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);  
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

void ExtMapUtility::createRandomStaticObstacleMap(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  oct -> clear();

  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution_)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
  }
  fillOctMsgFromOct();
  fillPCMsgFromOct();
}

void ExtMapUtility::addRandomStaticObstacle(vector<double> new_x_range, vector<double> new_y_range, vector<double> new_z_range, int num, bool constraint_flag, vector<geometry_msgs::Pose> goal, geometry_msgs::Point robot_center, double robot_free_rad, vector<int> color_RGB)
{
  for(int i = 0; i < num ; i++)
  {
    geometry_msgs::Pose rp;
    rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
    rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
    rp.position.z = randdouble(new_z_range[0], new_z_range[1]);

    while( isOccupied(rp.position) || (constraint_flag && isOccupiedByGoal(rp.position, goal) || isInCube(rp.position, robot_center, robot_free_rad + map_resolution_)) )    // check for duplicates, goal and initial robot occupancy
    {
      rp.position.x = randdouble(new_x_range[0], new_x_range[1]);
      rp.position.y = randdouble(new_y_range[0], new_y_range[1]);
      rp.position.z = randdouble(new_z_range[0], new_z_range[1]);
    }

    oct -> updateNode(rp.position.x, rp.position.y, rp.position.z, true);
    oct -> setNodeColor(oct -> coordToKey(rp.position.x, rp.position.y, rp.position.z), color_RGB[0], color_RGB[1], color_RGB[2]);
    addStaticObstacleByResolutionScale2PCMsg(rp.position);
  }
  fillOctMsgFromOct();
}

void ExtMapUtility::createRandomMapSet(string mapset_name, int map_cnt, int map_occupancy_count)
{
  vector<double> goal_x_range;
  goal_x_range.push_back(x_range[0] + 4);
  goal_x_range.push_back(x_range[1] - 4);

  vector<double> goal_y_range;
  goal_y_range.push_back(y_range[0] + 4);
  goal_y_range.push_back(y_range[1] - 4);

  vector<double> goal_z_range;
  goal_z_range.push_back(z_range[0] + 4);
  goal_z_range.push_back(z_range[1] - 4);
  
  for (int i = 0; i < map_cnt; i++)
  {
    createRandomStaticObstacleMap(map_occupancy_count);
    saveMap("mapset/" + mapset_name + "/map" + to_string(i));
  }
}

void ExtMapUtility::crop(sensor_msgs::PointCloud2& cloud_in, octomap::point3d lowerBound, octomap::point3d upperBound, octomap::Pointcloud& cloud_out, bool keep_in)
{
  cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  float x,y,z;

  min_x = lowerBound(0); min_y = lowerBound(1); min_z = lowerBound(2);
  max_x = upperBound(0); max_y = upperBound(1); max_z = upperBound(2);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      if( (*iter_x >= min_x) && (*iter_x <= max_x) &&
          (*iter_y >= min_y) && (*iter_y <= max_y) &&
          (*iter_z >= min_z) && (*iter_z <= max_z) )
      {
        if(keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
      else
      {
        if(!keep_in)
        {
          cloud_out.push_back(*iter_x, *iter_y, *iter_z);
        }
      }
    }
  }
}

void ExtMapUtility::cropOctPCFromPC2Msg(octomap::point3d lowerBound, octomap::point3d upperBound, bool keep_in)
{
  oct_pc.clear();

  /*
  if (pc2_msg.data.size() > 0 && laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve( (pc2_msg.data.size() + laser_pc2_msg.data.size()) / (pc2_msg.point_step + laser_pc2_msg.point_step) );
  }
  else if(pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);
  }
  else if(laser_pc2_msg.data.size() > 0)
  {
    oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);
  }
  */

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);

            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
        else
        {
          if(!keep_in)
          { 
            if(filter_ground)
            {
              if (*iter_z > filter_ground_threshold)
              {
                oct_pc.push_back(*iter_x, *iter_y, *iter_z);
              }
            }
            else
            {
              oct_pc.push_back(*iter_x, *iter_y, *iter_z);
            }
          }
        }
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        op_wrt_world.setValue(*iter_x, *iter_y, *iter_z);
        op_wrt_map = transform_map_wrt_world.inverse() * op_wrt_world;

        if( isInBBx(op_wrt_map, lowerBound, upperBound) )
        {
          if(keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
        else
        {
          if(!keep_in)
          {
            oct_pc.push_back(*iter_x, *iter_y, *iter_z);
          }
        }
      }
    }
  }
}

void ExtMapUtility::updateOctPC()
{
  oct_pc.clear();

  if(pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(pc2_msg.data.size() / pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }

  if(laser_pc2_msg.data.size() > 0)
  {
    //oct_pc.reserve(laser_pc2_msg.data.size() / laser_pc2_msg.point_step);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_pc2_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_pc2_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_pc2_msg, "z");

    tf::Vector3 op_wrt_world;
    tf::Vector3 op_wrt_map;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      // Check if the point is invalid
      if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
      {
        oct_pc.push_back(*iter_x, *iter_y, *iter_z);
      }
    }
  }
}

void ExtMapUtility::updateOct()
{
  if (!octUpdateFlag_)
  {
    oct = std::shared_ptr<octomap::ColorOcTree> (dynamic_cast<octomap::ColorOcTree*> (octomap_msgs::msgToMap(oct_msg)));
  }
  octUpdateFlag_ = true;
}

void ExtMapUtility::updateOct(sensor_msgs::PointCloud2& pc2_msg)
{
  oct->clear();
  addToOct(pc2_msg);

  fillOctMsgFromOct();
}

void ExtMapUtility::subscribeOctMsg(string oct_msg_name)
{
  //std::cout << "[ExtMapUtility::subscribeOctMsg] oct_msg_name: " << oct_msg_name << std::endl;
  sub_oct_msg_ = nh_.subscribe(oct_msg_name, 10, &ExtMapUtility::octMsgCallback, this);
}

void ExtMapUtility::pointcloud2ToOctPc2(const sensor_msgs::PointCloud2& cloud_pc2, octomap::Pointcloud& cloud_octomap)
{
  cloud_octomap.reserve(cloud_pc2.data.size() / cloud_pc2.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_pc2, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Check if the point is invalid
    if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
    {
      cloud_octomap.push_back(*iter_x, *iter_y, *iter_z);
    }
  }
}

void ExtMapUtility::publishOctMsg()
{
  std::cout << "[ExtMapUtility::publishOctMsg] START" << std::endl;
  oct_msg.header.frame_id = world_frame_name;
  //oct_msg.header.seq++;
  oct_msg.header.stamp = ros::Time::now();

  pub_oct_msg_.publish(oct_msg);

  std::cout << "[ExtMapUtility::publishOctMsg] END" << std::endl;
}

void ExtMapUtility::publishPCMsg()
{
  pc_msg.header.frame_id = world_frame_name;
  pc_msg.header.seq++;
  //pc_msg.header.stamp = ros::Time(0);
  pc_msg.header.stamp = ros::Time::now();
  pc_msg_pub.publish(pc_msg);
}

void ExtMapUtility::publishPC2Msg()
{
  pc2_msg.header.frame_id = world_frame_name;
  pc2_msg.header.seq++;
  //pc2_msg.header.stamp = ros::Time(0);
  pc2_msg.header.stamp = ros::Time::now();
  pc2_msg_pub.publish(pc2_msg);
}

void ExtMapUtility::publishPC2MsgGzPkgIgn(int index_pkg_ign)
{
  pc2_msg_gz_pkgs_ign_[index_pkg_ign].header.frame_id = frame_name_pkgs_ign_[index_pkg_ign];
  pc2_msg_gz_pkgs_ign_[index_pkg_ign].header.seq++;
  pc2_msg_gz_pkgs_ign_[index_pkg_ign].header.stamp = ros::Time::now();

  switch (index_pkg_ign)
  {
    case 0:
      pub_pc2_msg_gz_pkg_ign_conveyor_.publish(pc2_msg_gz_pkgs_ign_[index_pkg_ign]);
      break;

    case 1:
      pub_pc2_msg_gz_pkg_ign_red_cube_.publish(pc2_msg_gz_pkgs_ign_[index_pkg_ign]);
      break;

    case 2:
      pub_pc2_msg_gz_pkg_ign_green_cube_.publish(pc2_msg_gz_pkgs_ign_[index_pkg_ign]);
      break;

    case 3:
      pub_pc2_msg_gz_pkg_ign_blue_cube_.publish(pc2_msg_gz_pkgs_ign_[index_pkg_ign]);
      break;
    
    default:
      break;
  }
}

void ExtMapUtility::publishPC2MsgGzPkgMan(int index_pkg_man)
{
  pc2_msg_gz_pkgs_man_[index_pkg_man].header.frame_id = frame_name_pkgs_man_[index_pkg_man];
  pc2_msg_gz_pkgs_man_[index_pkg_man].header.seq++;
  pc2_msg_gz_pkgs_man_[index_pkg_man].header.stamp = ros::Time::now();
  
  switch (index_pkg_man)
  {
    case 0:
      pub_pc2_msg_gz_pkg_man_normal_pkg_.publish(pc2_msg_gz_pkgs_man_[index_pkg_man]);
      break;

    case 1:
      pub_pc2_msg_gz_pkg_man_long_pkg_.publish(pc2_msg_gz_pkgs_man_[index_pkg_man]);
      break;

    case 2:
      pub_pc2_msg_gz_pkg_man_longwide_pkg_.publish(pc2_msg_gz_pkgs_man_[index_pkg_man]);
      break;
    
    default:
      break;
  }
}

void ExtMapUtility::publishOccDistanceVisu()
{
  visualization_msgs::Marker occ_distance_visu = occ_distance_visu_;
  pub_occ_distance_visu_.publish(occ_distance_visu);
}

void ExtMapUtility::publishOccDistanceVisu(const ros::TimerEvent& event)
{
  visualization_msgs::Marker occ_distance_visu = occ_distance_visu_;
  pub_occ_distance_visu_.publish(occ_distance_visu);
}

void ExtMapUtility::publishOccDistanceVisu(geometry_msgs::Point p0, geometry_msgs::Point p1)
{
  fillOccDistanceVisu(p0, p1);
  pub_occ_distance_visu_.publish(occ_distance_visu_);
}

void ExtMapUtility::publishOccDistanceArrayVisu()
{
  //visualization_msgs::MarkerArray occ_distance_array_visu = occ_distance_array_visu_;
  pub_occ_distance_array_visu_.publish(occ_distance_array_visu_);
}

void ExtMapUtility::publishOccDistanceArrayVisu(vector<geometry_msgs::Point> p0_vec, vector<geometry_msgs::Point> p1_vec)
{
  fillOccDistanceArrayVisu(p0_vec, p1_vec);
  pub_occ_distance_array_visu_.publish(occ_distance_array_visu_);
}

void ExtMapUtility::publishDebugArrayVisu()
{
  for (int i = 0; i < debug_array_visu.markers.size(); ++i)
  {
    debug_array_visu.markers[i].header.seq++;
    //debug_array_visu.markers[i].header.stamp = ros::Time(0);
    debug_array_visu.markers[i].header.stamp = ros::Time::now();
  }
  debug_array_visu_pub.publish(debug_array_visu);
}

void ExtMapUtility::publishDebugVisu()
{
  debug_visu.header.seq++;
  //debug_visu.header.stamp = ros::Time(0);
  debug_visu.header.stamp = ros::Time::now();
  debug_visu_pub.publish(debug_visu);
}

void ExtMapUtility::printDataSize()
{
  cout << "" << endl;
  cout << "map_utility::printDataSize -> oct: " << oct->size() << endl;
  cout << "map_utility::printDataSize -> oct_msg: " << oct_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> oct_pc: " << oct_pc.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc_msg: " << measured_pc_msg.points.size() << endl;
  //cout << "map_utility::printDataSize -> measured_pc2_msg: " << measured_pc2_msg.data.size() << endl;
  //cout << "map_utility::printDataSize -> measured_laser_msg: " << measured_laser_msg.ranges.size() << endl;
  //cout << "map_utility::printDataSize -> pc_msg: " << pc_msg.points.size() << endl;
  cout << "map_utility::printDataSize -> pc2_msg: " << pc2_msg.data.size() << endl;
  cout << "map_utility::printDataSize -> laser_pc2_msg: " << laser_pc2_msg.data.size() << endl;
  cout << "" << endl;
}

void ExtMapUtility::saveMap(string filename)
{
  if(filename == "")
  {
    filename = createFileName();
  }

  ofstream map_file;
  string tentabot_path = ros::package::getPath("tentabot") + "/";
  map_file.open (tentabot_path + "dataset/map_utility/" + filename + ".csv");

  if( map_file.is_open() )
  {
    map_file << "map_name," + map_name + "\n";
    map_file << "world_frame_name," + world_frame_name + "\n";
    map_file << "x_range," + to_string(x_range[0]) + "," + to_string(x_range[1]) + "\n";
    map_file << "y_range," + to_string(y_range[0]) + "," + to_string(y_range[1]) + "\n";
    map_file << "z_range," + to_string(z_range[0]) + "," + to_string(z_range[1]) + "\n";
    map_file << "map_resolution," + to_string(map_resolution_) + "\n";
    map_file << "pc_resolution_scale," + to_string(pc_resolution_scale) + "\n";
    map_file << "max_occupancy_belief_value," + to_string(max_occupancy_belief_value) + "\n";

    map_file << "map,\n";

    for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); ++it)
    {
      map_file << to_string(it.getCoordinate().x()) + "," + to_string(it.getCoordinate().y()) + "," + to_string(it.getCoordinate().z()) + "\n";
    }
    map_file.close();
  }
  else
  {
    ROS_WARN("ExtMapUtility::saveMap -> Unable to open map file to save.");
  }
}

// NUA TODO: get the benchmark path
void ExtMapUtility::loadMap(string filename)
{
  string tentabot_path = ros::package::getPath("tentabot") + "/";
  ifstream map_file(tentabot_path + "dataset/map_utility/" + filename + ".csv");

  if( map_file.is_open() )
  {
    ROS_INFO_STREAM("" << filename << " is loading from the file...");

    oct -> clear();

    string line = "";
    bool map_flag = false;
    while( getline(map_file, line) )
    {
      vector<string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(","));

      if(vec[0] == "map")
      {
        map_flag = true;
        continue;
      }

      if(map_flag)
      {          
        addStaticObstacle( atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()) );
      }
    }

    // Close the File
    map_file.close();
    cout << filename + " is loaded!" << endl;
  }
  else
  {
    ROS_WARN_STREAM("ExtMapUtility::loadMap -> Unable to open " << filename << " file to load.");
  }
}

void ExtMapUtility::mapPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //cout << "ExtMapUtility::mapPoseCallback -> Incoming data..." << endl;
  measured_map_pose = *msg;
}

void ExtMapUtility::mapOdometryCallback(const nav_msgs::Odometry& msg)
{
  //cout << "ExtMapUtility::mapOdometryCallback -> Incoming data..." << endl;
  measured_map_pose = msg.pose.pose;
}

void ExtMapUtility::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //cout << "ExtMapUtility::pc2Callback -> Incoming data..." << endl;
  sensor_pc2_frame_name = msg -> header.frame_id;
  //cout << "ExtMapUtility::pc2Callback -> sensor_pc2_frame_name: " << sensor_pc2_frame_name << endl;

  try
  {
    tflistener -> waitForTransform(world_frame_name, msg -> header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tflistener -> lookupTransform(world_frame_name, msg -> header.frame_id, ros::Time(0), measured_transform_sensor_pc2_wrt_world);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("ExtMapUtility::pc2Callback -> Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
  }

  // SET MEASURED SENSOR POSE
  measured_sensor_pc2_pose.position.x = measured_transform_sensor_pc2_wrt_world.getOrigin().x();
  measured_sensor_pc2_pose.position.y = measured_transform_sensor_pc2_wrt_world.getOrigin().y();
  measured_sensor_pc2_pose.position.z = measured_transform_sensor_pc2_wrt_world.getOrigin().z();
  tf::quaternionTFToMsg(measured_transform_sensor_pc2_wrt_world.getRotation(), measured_sensor_pc2_pose.orientation);

  // SET MEASURED PC2
  pcl_ros::transformPointCloud(world_frame_name, measured_transform_sensor_pc2_wrt_world, *msg, measured_pc2_msg);
}

void ExtMapUtility::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //cout << "[ExtMapUtility::laserCallback] Incoming data..." << endl;
  sensor_laser_frame_name = msg -> header.frame_id;
  //cout << "[ExtMapUtility::laserCallback] sensor_laser_frame_name: " << sensor_laser_frame_name << endl;
  measured_laser_msg = *msg;
}

void ExtMapUtility::octMsgCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  //cout << "[ExtMapUtility::octMsgCallback] START" << endl;

  //cout << "[ExtMapUtility::octMsgCallback] BEFORE oct" << endl;
  //oct = std::shared_ptr<octomap::ColorOcTree> (dynamic_cast<octomap::ColorOcTree*> (octomap_msgs::msgToMap(*msg)));
  //cout << "[ExtMapUtility::octMsgCallback] AFTER oct" << endl;
  //fillOctMsgFromOct();
  oct_msg = *msg;
  //cout << "[ExtMapUtility::octMsgCallback] AFTER oct_msg" << endl;

  initOctMsgFlag_ =  true;

  //publishOccDistanceVisu();
  //publishOccDistanceArrayVisu();

  //cout << "[ExtMapUtility::octMsgCallback] END" << endl;
}

void ExtMapUtility::gazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  gazebo_msgs::ModelStates ms = *msg;

  bool initFlag = true;
  sensor_msgs::PointCloud2 pc2_msg_scan;
  octomap::Pointcloud oct_pc2_scan;

  transform_pkgs_ign_.clear();
  transform_pkgs_man_.clear();

  for (size_t i = 0; i < ms.name.size(); i++)
  {
    //esdf_server_ptr_->clear();
    for (size_t j = 0; j < frame_name_pkgs_ign_.size(); j++)
    {
      if (ms.name[i] == frame_name_pkgs_ign_[j])
      {
        tf::Transform transform_pkg_ign;

        sensor_msgs::PointCloud2 pc2_msg_gz_pkgs_ign_wrt_world;
        transform_pkg_ign.setIdentity();
        transform_pkg_ign.setOrigin(tf::Vector3(ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z));
        transform_pkg_ign.setRotation(tf::Quaternion(ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w));
      
        pcl_ros::transformPointCloud(world_frame_name, transform_pkg_ign, pc2_msg_gz_pkgs_ign_[j], pc2_msg_gz_pkgs_ign_wrt_world);

        static tf::TransformBroadcaster br_gz_pkg_ign;
        br_gz_pkg_ign.sendTransform(tf::StampedTransform(transform_pkg_ign, ros::Time::now(), world_frame_name, frame_name_pkgs_ign_[j]));

        //publishPC2MsgGzPkgIgn(j);
        if (initFlag)
        {
          pc2_msg_scan = pc2_msg_gz_pkgs_ign_wrt_world;
          initFlag = false;
        }
        else
        {
          pcl::concatenatePointCloud(pc2_msg_scan, pc2_msg_gz_pkgs_ign_wrt_world, pc2_msg_scan);
        }

        transform_pkgs_ign_.push_back(transform_pkg_ign);
      }
    }

    for (size_t j = 0; j < frame_name_pkgs_man_.size(); j++)
    {
      if (ms.name[i] == frame_name_pkgs_man_[j])
      {
        tf::Transform transform_pkg_man;

        transform_pkg_man.setIdentity();
        transform_pkg_man.setOrigin(tf::Vector3(ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z));
        transform_pkg_man.setRotation(tf::Quaternion(ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w));
      
        static tf::TransformBroadcaster br_gz_pkg_man;
        br_gz_pkg_man.sendTransform(tf::StampedTransform(transform_pkg_man, ros::Time::now(), world_frame_name, frame_name_pkgs_man_[j]));

        //publishPC2MsgGzPkgMan(j);

        transform_pkgs_man_.push_back(transform_pkg_man);
      }
    }
  }

  updateOct(pc2_msg_scan);

  pc2_msg_scan.header.frame_id = world_frame_name;
  pc2_msg_scan.header.seq++;
  pc2_msg_scan.header.stamp = ros::Time::now();
  pub_pc2_msg_scan_.publish(pc2_msg_scan);

  publishOctMsg();
}

void ExtMapUtility::update_states()
{
  //map_pose = measured_map_pose;
  sensor_pc2_pose = measured_sensor_pc2_pose;

  try
  {
    // scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment
    tflistener -> waitForTransform(world_frame_name, map_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name, map_frame_name, ros::Time(0), transform_map_wrt_world);

    tflistener -> waitForTransform(world_frame_name, sensor_pc2_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
    tflistener -> lookupTransform(world_frame_name, sensor_pc2_frame_name, ros::Time(0), transform_sensor_pc2_wrt_world);

    if (sensor_laser_frame_name != "")
    {
      tflistener -> waitForTransform(world_frame_name, sensor_laser_frame_name, ros::Time::now(), ros::Duration(map_server_dt));
      tflistener -> lookupTransform(world_frame_name, sensor_laser_frame_name, ros::Time(0), transform_sensor_laser_wrt_world);
      sensor_laser_projector.transformLaserScanToPointCloud(world_frame_name, measured_laser_msg, laser_pc2_msg, *tflistener);
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("[ExtMapUtility::reset_map_utility] Couldn't get transform!");
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("world_frame_name: %s", world_frame_name.c_str());
    ROS_ERROR("map_frame_name: %s", map_frame_name.c_str());
    ROS_ERROR("sensor_pc2_frame_name: %s", sensor_pc2_frame_name.c_str());
    ROS_ERROR("sensor_laser_frame_name: %s", sensor_laser_frame_name.c_str());
  }

  pc2_msg = measured_pc2_msg;
}

void ExtMapUtility::update_map()
{
  // DELETE MAP IN THE SENSOR RANGE
  tf::Vector3 query_wrt_world;
  tf::Vector3 query_wrt_map;
  tf::Vector3 query_wrt_sensor_pc2;
  tf::Vector3 query_wrt_sensor_laser;

  for(octomap::ColorOcTree::iterator it = oct -> begin(); it != oct -> end(); it++)
  {
    query_wrt_world.setValue(it.getX(), it.getY(), it.getZ());
    query_wrt_map = transform_map_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_pc2 = transform_sensor_pc2_wrt_world.inverse() * query_wrt_world;
    query_wrt_sensor_laser = transform_sensor_laser_wrt_world.inverse() * query_wrt_world;

    if (  ( !isInBBx(query_wrt_map, bbx_x_min, bbx_x_max, bbx_y_min, bbx_y_max, bbx_z_min, bbx_z_max) ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorCameraRange(query_wrt_sensor_pc2, sensor_pc2_msg_name != "") ) ||
          ( dynamic_flag && skip_cnt == skip_cnt_reset_sensor_range && isInSensorLaserRange(query_wrt_sensor_laser, sensor_laser_msg_name != "") ) )
    {
      oct -> deleteNode(it.getKey());
    }
  }

  // CROP RECENT POINTCLOUD2 DATA AND KEEP AS OCTOMAP PC
  octomap::point3d lowerBound(crop_x_min, crop_y_min, crop_z_min);
  octomap::point3d upperBound(crop_x_max, crop_y_max, crop_z_max);
  cropOctPCFromPC2Msg(lowerBound, upperBound, false);
  
  // INSERT RECENT OCTOMAP PC INTO THE MAP
  point3d sensor_pc2_origin(sensor_pc2_pose.position.x, sensor_pc2_pose.position.y, sensor_pc2_pose.position.z);
  insertToOctFromOctPC(sensor_pc2_origin);

  // UPDATE OCTOMAP MSG FROM THE MAP
  fillOctMsgFromOct();

  if (skip_cnt == skip_cnt_reset_sensor_range)
  {
    skip_cnt = 0;
  }
  else
  {
    skip_cnt++;
  }
}

/*
bool ExtMapUtility::reset_map_utility(tentabot::reset_map_utility::Request &req, tentabot::reset_map_utility::Response &res)
{
  cout << "[ExtMapUtility::reset_map_utility] Map is NOT reset! parity: " << req.parity << endl;

  oct -> clear();
  fillOctMsgFromOct();

  cout << "[ExtMapUtility::reset_map_utility] Map is reset!" << endl;

  res.success = true;
  return true;
}
*/

void ExtMapUtility::sensorMsgToOctomapCallback(const ros::TimerEvent& e)
{
  if (!local_map_flag)
  {
    oct -> clear();
    fillOctMsgFromOct();
  }

  update_states();

  update_map();

  publishOctMsg();
}

void getPointOnLine(geometry_msgs::Point& origin, geometry_msgs::Point& end, double dist)
{
  geometry_msgs::Point end_tmp = end;

  double alpha = (end_tmp.y - origin.y) * (end_tmp.y - origin.y) / ((end_tmp.x - origin.x) * (end_tmp.x - origin.x));
  double beta = (end_tmp.z - origin.z) * (end_tmp.z - origin.z) / ((end_tmp.x - origin.x) * (end_tmp.x - origin.x));

  end.x = dist / sqrt(1 + alpha + beta) + origin.x;

  double m = (end.x - origin.x) / (end_tmp.x - origin.x);

  end.y = m * (end_tmp.y - origin.y) + origin.y;
  end.z = m * (end_tmp.z - origin.z) + origin.z;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool ExtMapUtility::getNearestOccupancyDist(double x, 
                                            double y, 
                                            double z, 
                                            double radii, 
                                            double max_dist, 
                                            geometry_msgs::Point& min_p, 
                                            double& min_p_dist, 
                                            bool normalize_flag) const
{
  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] START" << std::endl;

  Eigen::Affine3d world_transform_eigen, query_tranform_eigen;
  world_transform_eigen.setIdentity();
  query_tranform_eigen.setIdentity();
  fcl::Vector3d query_translation(x, y, z);
  query_tranform_eigen.translation() = query_translation;

  fcl::Transform3d world_transform_fcl, query_tranform_fcl;
  robot_collision_checking::FCLInterface::transform2fcl(world_transform_eigen, world_transform_fcl);
  robot_collision_checking::FCLInterface::transform2fcl(query_tranform_eigen, query_tranform_fcl);

  worldFCLCollisionGeometryPtr_ = robot_collision_checking::FCLInterface::createCollisionGeometry(oct_msg);
  worldFCLCollisionObjectPtr_ = std::make_shared<fcl::CollisionObjectd>(worldFCLCollisionGeometryPtr_, world_transform_fcl);

  queryFCLCollisionGeometryPtr_ = std::make_shared<fcl::Sphered>(0.1);
  //std::shared_ptr<fcl::CollisionGeometryd> queryFCLCollisionGeometryPtr(new fcl::Sphered(0.1));

  FCLCollisionObjectPtr queryFCLCollisionObjectPtr = std::make_shared<fcl::CollisionObjectd>(queryFCLCollisionGeometryPtr_, query_tranform_fcl);
  //queryFCLCollisionObjectPtr_ = std::make_shared<fcl::CollisionObjectd>(queryFCLCollisionGeometryPtr, query_tranform_fcl);

  fcl::DistanceRequestd dist_req;
  dist_req.enable_nearest_points = true;
  dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

  fcl::DistanceResultd dist_result;
  dist_result.nearest_points[0].setZero();
  dist_result.nearest_points[1].setZero();

  fcl::distance(worldFCLCollisionObjectPtr_.get(),
                queryFCLCollisionObjectPtr.get(),
                dist_req,
                dist_result);

  Eigen::Vector3d world_min_p;
  world_min_p.x() = dist_result.nearest_points[0].x();
  world_min_p.y() = dist_result.nearest_points[0].y();
  world_min_p.z() = dist_result.nearest_points[0].z();

  //Eigen::Vector3d query_min_p;
  //query_min_p.x() = dist_result.nearest_points[1].x();
  //query_min_p.y() = dist_result.nearest_points[1].y();
  //query_min_p.z() = dist_result.nearest_points[1].z();

  min_p.x = world_min_p.x();
  min_p.y = world_min_p.y();
  min_p.z = world_min_p.z();

  min_p_dist = dist_result.min_distance;

  /*
  if (min_p_dist > max_dist)
  {
    geometry_msgs::Point end_tmp;
    end_tmp.x = min_p.x;
    end_tmp.y = min_p.y;
    end_tmp.z = min_p.z;

    double alpha = (end_tmp.y - y) * (end_tmp.y - y) / ((end_tmp.x - x) * (end_tmp.x - x));
    double beta = (end_tmp.z - z) * (end_tmp.z - z) / ((end_tmp.x - x) * (end_tmp.x - x));

    min_p.x = x + max_dist / sqrt(1 + alpha + beta);

    double m = (min_p.x - x) / (end_tmp.x - x);

    if (m < 0)
    {
      min_p.x = x - max_dist / sqrt(1 + alpha + beta);
      m = (min_p.x - x) / (end_tmp.x - x);
    }

    min_p.y = y + m * (end_tmp.y - y);
    min_p.z = z + m * (end_tmp.z - z);

    min_p_dist = max_dist;
  }
  */

  double min_dist = dist_result.min_distance - radii;

  if (normalize_flag)
  {
    min_dist /= max_dist - radii;
  }

  /*
  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] world_min_p x: " << world_min_p.x() << std::endl;
  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] world_min_p y: " << world_min_p.y() << std::endl;
  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] world_min_p z: " << world_min_p.z() << std::endl;

  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] query_min_p x: " << query_min_p.x() << std::endl;
  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] query_min_p y: " << query_min_p.y() << std::endl;
  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] query_min_p z: " << query_min_p.z() << std::endl;

  std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] min_dist: " << min_dist << std::endl;
  */

  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(8)] END" << std::endl << std::endl;

  return true;
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
bool ExtMapUtility::getNearestOccupancyDist(std::vector<geometry_msgs::Point>& query_points, 
                                            Eigen::VectorXd& radii, 
                                            double max_dist, 
                                            std::vector<geometry_msgs::Point>& min_points, 
                                            std::vector<double>& min_distances, 
                                            bool normalize_flag) const
{
  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] START" << std::endl;

  min_points.clear();
  min_distances.clear();

  Eigen::Affine3d world_transform_eigen, query_tranform_eigen;
  world_transform_eigen.setIdentity();
  query_tranform_eigen.setIdentity();

  fcl::Transform3d world_transform_fcl, query_tranform_fcl;
  robot_collision_checking::FCLInterface::transform2fcl(world_transform_eigen, world_transform_fcl);

  worldFCLCollisionGeometryPtr_ = robot_collision_checking::FCLInterface::createCollisionGeometry(oct_msg);
  worldFCLCollisionObjectPtr_ = std::make_shared<fcl::CollisionObjectd>(worldFCLCollisionGeometryPtr_, world_transform_fcl);

  std::shared_ptr<fcl::CollisionGeometryd> queryFCLCollisionGeometryPtr(new fcl::Sphered(0.1));

  for (size_t i = 0; i < query_points.size(); i++)
  {
    fcl::Vector3d query_translation(query_points[i].x, query_points[i].y, query_points[i].z);
    query_tranform_eigen.translation() = query_translation;
    robot_collision_checking::FCLInterface::transform2fcl(query_tranform_eigen, query_tranform_fcl);

    FCLCollisionObjectPtr queryFCLCollisionObjectPtr = std::make_shared<fcl::CollisionObjectd>(queryFCLCollisionGeometryPtr, query_tranform_fcl);

    fcl::DistanceRequestd dist_req;
    dist_req.enable_nearest_points = true;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    fcl::DistanceResultd dist_result;
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();

    fcl::distance(worldFCLCollisionObjectPtr_.get(),
                  queryFCLCollisionObjectPtr.get(),
                  dist_req,
                  dist_result);

    Eigen::Vector3d world_min_p;
    world_min_p.x() = dist_result.nearest_points[0].x();
    world_min_p.y() = dist_result.nearest_points[0].y();
    world_min_p.z() = dist_result.nearest_points[0].z();

    //Eigen::Vector3d query_min_p;
    //query_min_p.x() = dist_result.nearest_points[1].x();
    //query_min_p.y() = dist_result.nearest_points[1].y();
    //query_min_p.z() = dist_result.nearest_points[1].z();

    geometry_msgs::Point min_p;
    min_p.x = world_min_p.x();
    min_p.y = world_min_p.y();
    min_p.z = world_min_p.z();

    double min_dist = dist_result.min_distance - radii(i);

    if (normalize_flag)
    {
      min_dist /= max_dist - radii(i);
    }

    /*
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] world_min_p x: " << world_min_p.x() << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] world_min_p y: " << world_min_p.y() << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] world_min_p z: " << world_min_p.z() << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] query_min_p x: " << query_min_p.x() << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] query_min_p y: " << query_min_p.y() << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] query_min_p z: " << query_min_p.z() << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] min_dist: " << min_dist << std::endl;
    */

    min_points.push_back(min_p);
    min_distances.push_back(min_dist);
  }

  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(6)] END" << std::endl << std::endl;

  return true;
}

Eigen::Vector3d ExtMapUtility::calculateInteriorPoint(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double d) 
{
  // Calculate the vector from x to y
  Eigen::Vector3d V = p2 - p1;

  // Calculate the normalized vector
  Eigen::Vector3d N = V.normalized();

  // Calculate the interior point
  Eigen::Vector3d interiorPoint = p1 + d * N;

  return interiorPoint;
}

bool ExtMapUtility::getNearestOccupancyDist(int numPoints,
                                            Eigen::VectorXd& positionPointsOnRobot, 
                                            Eigen::VectorXd& radii, 
                                            double max_dist, 
                                            std::vector<geometry_msgs::Point>& min_points, 
                                            std::vector<double>& min_distances, 
                                            bool normalize_flag)
{
  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] START" << std::endl;

  min_points.clear();
  min_distances.clear();
  bool collisionFlag = false;

  Eigen::Affine3d world_transform_eigen, query_tranform_eigen;
  world_transform_eigen.setIdentity();
  query_tranform_eigen.setIdentity();

  fcl::Transform3d world_transform_fcl, query_tranform_fcl;
  robot_collision_checking::FCLInterface::transform2fcl(world_transform_eigen, world_transform_fcl);

  worldFCLCollisionGeometryPtr_ = robot_collision_checking::FCLInterface::createCollisionGeometry(oct_msg);
  worldFCLCollisionObjectPtr_ = std::make_shared<fcl::CollisionObjectd>(worldFCLCollisionGeometryPtr_, world_transform_fcl);

  //std::shared_ptr<fcl::CollisionGeometryd> queryFCLCollisionGeometryPtr(new fcl::Sphered(0.001));

  for (size_t i = 0; i < numPoints; i++)
  {
    Eigen::Ref<Eigen::Matrix<ocs2::scalar_t, 3, 1>> position = positionPointsOnRobot.segment<3>(i * 3);

    fcl::Vector3d query_translation(position(0), position(1), position(2));
    query_tranform_eigen.translation() = query_translation;
    std::shared_ptr<fcl::CollisionGeometryd> queryFCLCollisionGeometryPtr(new fcl::Sphered(radii[i]));
    robot_collision_checking::FCLInterface::transform2fcl(query_tranform_eigen, query_tranform_fcl);

    FCLCollisionObjectPtr queryFCLCollisionObjectPtr = std::make_shared<fcl::CollisionObjectd>(queryFCLCollisionGeometryPtr, query_tranform_fcl);

    fcl::DistanceRequestd dist_req;
    dist_req.enable_nearest_points = true;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    fcl::DistanceResultd dist_result;
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();

    fcl::distance(worldFCLCollisionObjectPtr_.get(),
                  queryFCLCollisionObjectPtr.get(),
                  dist_req,
                  dist_result);

    //Eigen::Vector3d world_min_p;
    //world_min_p.x() = dist_result.nearest_points[0].x();
    //world_min_p.y() = dist_result.nearest_points[0].y();
    //world_min_p.z() = dist_result.nearest_points[0].z();

    //Eigen::Vector3d query_min_p;
    //query_min_p.x() = dist_result.nearest_points[1].x();
    //query_min_p.y() = dist_result.nearest_points[1].y();
    //query_min_p.z() = dist_result.nearest_points[1].z();

    geometry_msgs::Point min_p;
    min_p.x = dist_result.nearest_points[0].x();
    min_p.y = dist_result.nearest_points[0].y();
    min_p.z = dist_result.nearest_points[0].z();

    double min_dist = dist_result.min_distance;
    //double min_dist = dist_result.min_distance - radii(i);

    //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] radii(i): " << radii(i) << std::endl;
    //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] min_dist: " << min_dist << std::endl;
    //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] max_dist: " << max_dist << std::endl;
    
    /*
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] min_p.x: " << min_p.x << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(0): " << position(0) << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] min_p.y: " << min_p.y << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(1): " << position(1) << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] min_p.z: " << min_p.z << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(2): " << position(2) << std::endl;
    */

    
    if (min_dist < 0)
    {
      min_dist = 0;
      collisionFlag = true;
    }
    else if (min_dist > max_dist)
    {
      Eigen::Vector3d int_p = calculateInteriorPoint(Eigen::Vector3d(position(0), position(1), position(2)), Eigen::Vector3d(min_p.x, min_p.y, min_p.z), max_dist);

      min_p.x = int_p.x();
      min_p.y = int_p.y();
      min_p.z = int_p.z();

      min_dist = max_dist;
    }

    if (normalize_flag)
    {
      min_dist /= max_dist;
    }

    /*
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] world_min_p.x(): " << min_p.x << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(0): " << position(0) << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] world_min_p.y(): " << min_p.y << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(1): " << position(1) << std::endl;

    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] world_min_p.z(): " << min_p.z << std::endl;
    std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] position(2): " << position(2) << std::endl << std::endl;
    */

    //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] AFTER min_dist: " << min_dist << std::endl << std::endl;

    min_points.push_back(min_p);
    min_distances.push_back(min_dist);
  }

  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] DEBUG INF" << std::endl << std::endl;
  //while(1);

  //std::cout << "[ExtMapUtility::getNearestOccupancyDist(7)] END" << std::endl << std::endl;

  return collisionFlag;
}

/*
bool ExtMapUtility::getNearestOccupancyDistSrv(mobiman_simulation::getNearestOccDist::Request &req, 
                                            mobiman_simulation::getNearestOccDist::Response &res)
{
  //std::cout << "[ExtMapUtility::getNearestOccupancyDistSrv] START" << std::endl;

  res.distance = getNearestOccupancyDist2(req.x, req.y, req.z);

  if (res.distance < 0)
  {
    //std::cout << "[ExtMapUtility::getNearestOccupancyDistSrv] FALSE END" << std::endl << std::endl;
    return false;
  }
  else
  {
    //std::cout << "[ExtMapUtility::getNearestOccupancyDistSrv] TRUE END" << std::endl << std::endl;
    return true;
  }

  return true;
}
*/
