#include <grouper_lidar/grouper_lidar.h>

using namespace grouper_lidar;

GrouperLidar::GrouperLidar()
    :nh_priv_("~"),
     motor_name_("")
{
  nh_priv_.param<std::string>("motor_name_", motor_name_, "radar_tilt");
  initGrouperLidar();
}

GrouperLidar::~GrouperLidar()
{
//TODO:destructor
}

bool GrouperLidar::grouperLidarLoop(void)
{
  switch(operating_state_)
  {
    case IDLE:
      changeMotorSpeed(MOTOR_MOVING_SPEED);
      moveMotor(0);
      log_enable = false;
    break;
    case INIT:
      if (motorReachGoal())
      {
//        present_scan_line_ = 0;
        operating_state_=CW;
//        log_enable = true;
        changeMotorSpeed(MOTOR_OPERATING_SPEED);
        moveMotor(MOTOR_BOARD_CW-MOTOR_OVER_EXCEED);
      }
      else
      {
        changeMotorSpeed(MOTOR_MOVING_SPEED);
        moveMotor(MOTOR_BOARD_CCW+MOTOR_OVER_EXCEED);
      }
    break;
    case CW:
      if(motorReachGoal())
      {
//        sendImageArchive();
        operating_state_=CCW;
//        log_enable = true;
        reampleImage();
        memset(scan_up2date_flag_,false,sizeof(scan_up2date_flag_));
        changeMotorSpeed(MOTOR_OPERATING_SPEED);
        moveMotor(MOTOR_BOARD_CCW+MOTOR_OVER_EXCEED);
      }
      else
      {
        moveMotor(MOTOR_BOARD_CW-MOTOR_OVER_EXCEED);
      }
    break;
    case CCW:
      if(motorReachGoal())
      {
        sendImageArchive();
        operating_state_=CW;
//        log_enable = true;
        reampleImage();
        memset(scan_up2date_flag_,false,sizeof(scan_up2date_flag_));
        changeMotorSpeed(MOTOR_OPERATING_SPEED);
        moveMotor(MOTOR_BOARD_CW-MOTOR_OVER_EXCEED);
      }
      else
      {
        moveMotor(MOTOR_BOARD_CCW+MOTOR_OVER_EXCEED);
      }
    break;
    default:
      
    break;
  }
}

void GrouperLidar::initGrouperLidar(void)
{
  operating_state_ = IDLE;
  motor_goal_position_ = 0;
  motor_present_position_ = 0;
  memset(ranges_matrix_,0,sizeof(ranges_matrix_));
  memset(intensities_matrix_,0,sizeof(intensities_matrix_));
  memset(scan_position_matrix_,0,sizeof(scan_position_matrix_));
  memset(ranges_img,0,sizeof(ranges_img));
  memset(intensities_img,0,sizeof(intensities_img));
  memset(scan_up2date_flag_,false,sizeof(scan_up2date_flag_));
//  present_scan_line_ = 0;
  msg_seq = 0;

//  scan_goal_position_=0;
  log_enable=false;

  motor_control_ = nh_.serviceClient<dynamixel_control_msgs::MotorControl>("dynamixel_control/motor_control");

  motor_state_ = nh_.subscribe("dynamixel_control/motor_state", 10, &GrouperLidar::motorStateCallback, this);
  lidar_scan_ = nh_.subscribe("scan", 10, &GrouperLidar::lidarScanCallback, this);

  grouper_depthmap_ = nh_.advertise<sensor_msgs::Image>("grouper_lidar/depthmap_image", 10);
  grouper_intensity_ = nh_.advertise<sensor_msgs::Image>("grouper_lidar/intensity_image", 10);
  grouper_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>( "grouper_lidar/point_cloud", 10);
  grouper_capture_ = nh_.advertise<std_msgs::Header>( "image_archive", 10);
  

  grouper_depthmap_control_ = nh_.advertiseService("grouper_lidar/capture_control", &GrouperLidar::grouperLidarControlCallback, this);

  float scan_start_ = getMotorValue(MOTOR_BOARD_CCW);
  float scan_end_ = getMotorValue(MOTOR_BOARD_CW);
  
  for(int i=0;i<SCAN_LINES;i++)
  {
    scan_goal_position_[i] = scan_start_-i*(scan_start_-scan_end_)/(SCAN_LINES-1);
//    ROS_INFO("initGL %d",scan_goal_position_[i]);
//    scan_up2date_flag_[i] = false;
  }
  
  for(int i=0;i<MOTOR_LOG_WINDOW;i++)
  {
    motor_log_time_[i] = ros::Time(0.);
    motor_log_position_[i] = 0;
  }

  motor_srv_.request.motor_name = "cam_pan";
  motor_srv_.request.control_type = "position";
  motor_srv_.request.unit = "rad";
  motor_srv_.request.value = 0.39;
  motor_control_.call(motor_srv_);
  motor_srv_.request.motor_name = "cam_tilt";
  motor_srv_.request.control_type = "position";
  motor_srv_.request.unit = "rad";
  motor_srv_.request.value = 0;
  motor_control_.call(motor_srv_);
}

void GrouperLidar::moveMotor(double value)
{
  motor_srv_.request.motor_name = motor_name_;
  motor_srv_.request.control_type = "position";
  motor_srv_.request.unit = "rad";
  motor_srv_.request.value = value;
  if (motor_control_.call(motor_srv_))
  {
    ROS_DEBUG("motor to %f", motor_srv_.response.value);
    motor_goal_position_ = motor_srv_.response.value;
  }
  else
  {
    ROS_ERROR("Failed to call motor service");
  }
}

double GrouperLidar::getMotorValue(double value)
{
  motor_srv_.request.motor_name = motor_name_;
  motor_srv_.request.control_type = "getposition";
  motor_srv_.request.unit = "rad";
  motor_srv_.request.value = value;
  if (motor_control_.call(motor_srv_))
  {
    return motor_srv_.response.value;
  }
  else
  {
    ROS_ERROR("Failed to call motor service");
    return 0;
  }
}

void GrouperLidar::changeMotorSpeed(float value)
{
  motor_srv_.request.motor_name = motor_name_;
  motor_srv_.request.control_type = "velocity";
  motor_srv_.request.unit = "raw";
  motor_srv_.request.value = value;
  if (motor_control_.call(motor_srv_))
  {
    return;
  }
  else
  {
    ROS_ERROR("Failed to call motor service");
    return;
  }
}

float GrouperLidar::estimatePosition(ros::Time msg_time)
{
  if(motor_log_time_[MOTOR_LOG_WINDOW-1].toSec() == 0)
  {
    return motor_log_position_[0];
  }
  else
  {
    ROS_INFO("%f %f %f",motor_log_position_[0],motor_log_position_[MOTOR_LOG_WINDOW-1],((motor_log_position_[0]-motor_log_position_[MOTOR_LOG_WINDOW-1])*(msg_time-motor_log_time_[0]).toSec()/(motor_log_time_[0]-motor_log_time_[MOTOR_LOG_WINDOW-1]).toSec())+motor_log_position_[0]);
    return ((motor_log_position_[0]-motor_log_position_[MOTOR_LOG_WINDOW-1])*(msg_time-motor_log_time_[0]).toSec()/(motor_log_time_[0]-motor_log_time_[MOTOR_LOG_WINDOW-1]).toSec())+motor_log_position_[0];
  }
}

bool GrouperLidar::motorReachGoal()
{
  return (abs(estimatePosition(ros::Time::now())-motor_goal_position_) < MOTOR_TOLERANT);
}

int GrouperLidar::GetClosestLine(float motor_pos)
{
  int result;
  result = 0;
  for(int i=0;i<SCAN_LINES;i++)
  {
    if(abs(motor_pos-scan_goal_position_[i]) < abs(motor_pos-scan_goal_position_[result]))
    {
      result = i;
    }
  }
  return result;
}

void GrouperLidar::reampleImage()
{
  for(int i=0;i<SCAN_LINES;i++)
  {
    if(!scan_up2date_flag_[i])
    {
      ROS_INFO("%d missing data",i);
      if(i-1>0 && scan_up2date_flag_[i-1])
      {
        memcpy(ranges_img[i],ranges_img[i-1],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i-1],OUTPUT_COL*sizeof(float));
      }
      else if (i+1<SCAN_LINES && scan_up2date_flag_[i+1])
      {
        memcpy(ranges_img[i],ranges_img[i+1],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i+1],OUTPUT_COL*sizeof(float));
      }
      else if (i-2>0 && scan_up2date_flag_[i-2])
      {
        memcpy(ranges_img[i],ranges_img[i-2],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i-2],OUTPUT_COL*sizeof(float));
      }
      else if (i+2<SCAN_LINES && scan_up2date_flag_[i+2])
      {
        memcpy(ranges_img[i],ranges_img[i+2],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i+2],OUTPUT_COL*sizeof(float));
      }
      else if (i-3>0 && scan_up2date_flag_[i-3])
      {
        memcpy(ranges_img[i],ranges_img[i-3],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i-3],OUTPUT_COL*sizeof(float));
      }
      else if (i+3<SCAN_LINES && scan_up2date_flag_[i+3])
      {
        memcpy(ranges_img[i],ranges_img[i+3],OUTPUT_COL*sizeof(float));
        memcpy(intensities_img[i],intensities_img[i+3],OUTPUT_COL*sizeof(float));
      }
      else
      {
        ROS_ERROR("Resample failed at line %d",i);
      }
    }
  }
  output1Frame();
}

void GrouperLidar::LogLidar(int scan_line_)
{
  ROS_INFO("Log line %d",scan_line_);
  memcpy(ranges_matrix_[scan_line_],lidar_ranges_,LIDAR_POINTS*sizeof(float));
  memcpy(intensities_matrix_[scan_line_],lidar_intensities_,LIDAR_POINTS*sizeof(float));
  scan_position_matrix_[scan_line_] = motor_present_position_;
  for(int i=0; i<OUTPUT_COL; i++)
  {
    ranges_img[scan_line_][OUTPUT_COL-i-1]=ranges_matrix_[scan_line_][OUTPUT_OFFSET+i*OUTPUT_STEP];
    intensities_img[scan_line_][OUTPUT_COL-i-1]=intensities_matrix_[scan_line_][OUTPUT_OFFSET+i*OUTPUT_STEP];
  }
}

void GrouperLidar::output1Frame()
{
  sensor_msgs::Image depthmap_msg;
  depthmap_msg.header.seq = msg_seq;
  depthmap_msg.header.stamp = ros::Time::now();
  depthmap_msg.header.frame_id = "depthmap";
  depthmap_msg.height = OUTPUT_ROW;
  depthmap_msg.width = OUTPUT_COL;
  depthmap_msg.encoding = "32FC1";
  depthmap_msg.is_bigendian = false;
  depthmap_msg.step = OUTPUT_COL*sizeof(float);
  depthmap_msg.data.resize(OUTPUT_ROW*depthmap_msg.step);
  memcpy(&depthmap_msg.data[0], &ranges_img[0][0], OUTPUT_ROW*depthmap_msg.step);
  grouper_depthmap_.publish(depthmap_msg);

  sensor_msgs::Image intensity_msg;  
  intensity_msg.header.seq = msg_seq;
  intensity_msg.header.stamp = ros::Time::now();
  intensity_msg.header.frame_id = "intensity";
  intensity_msg.height = OUTPUT_ROW;
  intensity_msg.width = OUTPUT_COL;
  intensity_msg.encoding = "32FC1";
  intensity_msg.is_bigendian = false;
  intensity_msg.step = OUTPUT_COL*sizeof(float);
  intensity_msg.data.resize(OUTPUT_ROW*intensity_msg.step);
  memcpy(&intensity_msg.data[0], &intensities_img[0][0], OUTPUT_ROW*intensity_msg.step);
  grouper_intensity_.publish(intensity_msg);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::PointCloud< pcl::PointXYZI > pc;
  pc.width  = LIDAR_POINTS;
  pc.height = SCAN_LINES;
  pc.is_dense = false;
  pc.points.resize( pc.width * pc.height );
  for(int i = 0; i < pc.height; ++i ) { 
    const double phi = (-1)*PI + scan_position_matrix_[i]*(2*PI)/MOTOR_POINTS;
    const double x_mod = LIDAR_HEIGHT*cos(phi+0.5*PI);
    const double z_mod = LIDAR_HEIGHT*sin(phi+0.5*PI);
    for(int j = 0; j < pc.width; ++j ) {
      const int k = pc.width * i + j;
      const double theta = (LIDAR_ANGLE_MIN) + j*((LIDAR_ANGLE_MAX)-(LIDAR_ANGLE_MIN))/(LIDAR_POINTS-1);
      const double x_pre = ranges_matrix_[i][j]*cos(theta);
      pc.points[k].x = x_pre*cos(phi)+x_mod;
      pc.points[k].y = ranges_matrix_[i][j]*sin(theta);
      pc.points[k].z = x_pre*sin(phi)+z_mod;
      pc.points[k].intensity = intensities_matrix_[i][j];
    }
  }
  pcl::toROSMsg(pc, pc_msg);
  pc_msg.header.seq = msg_seq;
  pc_msg.header.stamp = ros::Time::now();
  pc_msg.header.frame_id = "pc";
  grouper_pointcloud_.publish(pc_msg);

  msg_seq++;
}

bool GrouperLidar::grouperLidarControlCallback(grouper_lidar_msgs::GrouperLidarControl::Request  &req,
                                               grouper_lidar_msgs::GrouperLidarControl::Response &res)
{
  res.ans=false;
  if(req.command=="start")
  {
    memset(scan_up2date_flag_,false,sizeof(scan_up2date_flag_));
    changeMotorSpeed(MOTOR_MOVING_SPEED);
    moveMotor(MOTOR_BOARD_CCW+MOTOR_OVER_EXCEED);
    operating_state_ = INIT;
    res.ans = true;
  }
  if(req.command=="stop")
  {
    operating_state_ = IDLE;
    res.ans = true;
  }
  if(req.command=="reset")
  {
    moveMotor(0);
    operating_state_ = IDLE;
    res.ans = true;
  }
  return res.ans;
}

void GrouperLidar::motorStateCallback(const dynamixel_control_msgs::MotorState::ConstPtr& msg)
{
  if(msg->header.frame_id==motor_name_)
  {
    for(int i=MOTOR_LOG_WINDOW-1;i>0;i--)
    {
      motor_log_time_[i] = motor_log_time_[i-1];
      motor_log_position_[i] = motor_log_position_[i-1];
    }
    motor_log_time_[0] = msg->header.stamp;
    motor_log_position_[0] = msg->present_position;

//    motor_present_position_=msg->present_position;
    ROS_DEBUG("motor present %f", motor_present_position_);
  }
}

void GrouperLidar::lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  memcpy(&lidar_ranges_[0],&msg->ranges[0],LIDAR_POINTS*sizeof(float));
  memcpy(&lidar_intensities_[0],&msg->intensities[0],LIDAR_POINTS*sizeof(float));

  motor_present_position_ = estimatePosition(msg->header.stamp);

  int scan_line_;
  scan_line_ = GetClosestLine(motor_present_position_);
  if(!scan_up2date_flag_[scan_line_])
  {
    if(abs(motor_present_position_-scan_goal_position_[scan_line_])<=SCAN_TOLERANT)
    {
      LogLidar(scan_line_);
      scan_up2date_flag_[scan_line_]=true;
    }
    else
    {
      ROS_INFO("out of tolerant");
    }
  }
  else
  {
    if(abs(motor_present_position_-scan_goal_position_[scan_line_]) < abs(scan_position_matrix_[scan_line_]-scan_goal_position_[scan_line_]))
    {
      LogLidar(scan_line_);
    }
  }
  output1Frame();
}

void GrouperLidar::sendImageArchive()
{
  std_msgs::Header msg_;
  msg_.seq = msg_seq;
  msg_.stamp = ros::Time::now();
  msg_.frame_id = "grouper_lidar_image_archive";
  grouper_capture_.publish(msg_);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "grouper_lidar");
  GrouperLidar grouper_lidar;

  ros::Rate loop_rate(50);

  ROS_INFO("Grouper Lidar Ready.");
  while (ros::ok())
  {
    grouper_lidar.grouperLidarLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
