#ifndef GROUPER_LIDAR_H
#define GROUPER_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamixel_control_msgs/MotorState.h>
#include <dynamixel_control_msgs/MotorControl.h>
#include <grouper_lidar_msgs/GrouperLidarControl.h>

//#ifndef PI
#define PI 3.1415927
//#endif
#define MOTOR_BOARD_CW  -0.49
#define MOTOR_BOARD_CCW 0.49
#define MOTOR_OVER_EXCEED 0.06
#define MOTOR_TOLERANT 10
#define MOTOR_POINTS 4096
#define SCAN_LINES 225
#define SCAN_TOLERANT 3
#define LIDAR_ANGLE_MIN -2.356194
#define LIDAR_ANGLE_MAX 2.356194
#define LIDAR_POINTS 1081
#define LIDAR_HEIGHT 0.093
#define OUTPUT_OFFSET 340
#define OUTPUT_STEP 1
#define OUTPUT_ROW 225
#define OUTPUT_COL 400
#define MOTOR_LOG_WINDOW 5
#define MOTOR_MOVING_SPEED 20
#define MOTOR_OPERATING_SPEED 2

namespace grouper_lidar
{
enum GROUPER_LIDAR_STATE
{
  IDLE,
  INIT,
  CW,
  CCW
};
class GrouperLidar
{
 public:
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::ServiceClient motor_control_;
  ros::ServiceServer grouper_depthmap_control_;
  ros::Publisher grouper_depthmap_;
  ros::Publisher grouper_intensity_;
  ros::Publisher grouper_pointcloud_;
  ros::Publisher grouper_capture_;
  ros::Subscriber motor_state_;
  ros::Subscriber lidar_scan_;
  
  std::string motor_name_;
  GROUPER_LIDAR_STATE operating_state_;

  dynamixel_control_msgs::MotorControl motor_srv_;

  float motor_goal_position_;
  float motor_present_position_;

  float scan_goal_position_[SCAN_LINES];
  float scan_position_matrix_[SCAN_LINES];
  bool scan_up2date_flag_[SCAN_LINES];
  
  float lidar_ranges_[LIDAR_POINTS];
  float lidar_intensities_[LIDAR_POINTS];
  
  bool log_enable;
  
  float ranges_matrix_[SCAN_LINES][LIDAR_POINTS];
  float intensities_matrix_[SCAN_LINES][LIDAR_POINTS];

  float ranges_img[OUTPUT_ROW][OUTPUT_COL];
  float intensities_img[OUTPUT_ROW][OUTPUT_COL];


  uint32_t msg_seq;

  ros::Time motor_log_time_[MOTOR_LOG_WINDOW];
  float motor_log_position_[MOTOR_LOG_WINDOW];

//  int present_scan_line_;
  
  
 public:
  GrouperLidar();
  ~GrouperLidar();
  bool grouperLidarLoop(void);
 private:
  void initGrouperLidar(void);
  void resetMotorPosition(void);
  void moveMotor(double value);
  double getMotorValue(double value);
  void changeMotorSpeed(float value);
  float estimatePosition(ros::Time msg_time);
  bool motorReachGoal();
  int GetClosestLine(float motor_pos);
  void LogLidar(int scan_line);
  void output1Frame();
  void reampleImage();
  bool grouperLidarControlCallback(grouper_lidar_msgs::GrouperLidarControl::Request  &req,
                                   grouper_lidar_msgs::GrouperLidarControl::Response &res);
  void motorStateCallback(const dynamixel_control_msgs::MotorState::ConstPtr& msg);
  void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  
  void sendImageArchive();
};
}

#endif //GROUPER_LIDAR_H
