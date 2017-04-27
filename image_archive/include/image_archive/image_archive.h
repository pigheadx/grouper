#ifndef IMAGE_ARCHIVE_H
#define IMAGE_ARCHIVE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <iomanip>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace image_archive
{
class ImageArchive
{
 public:
 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_0_;
  image_transport::Subscriber image_sub_1_;
  image_transport::Subscriber image_sub_2_;
  image_transport::Subscriber image_sub_3_;
  image_transport::Subscriber image_sub_4_;
  cv_bridge::CvImagePtr cv_ptr_0_;
  cv_bridge::CvImagePtr cv_ptr_1_;
  cv_bridge::CvImagePtr cv_ptr_2_;
  cv_bridge::CvImagePtr cv_ptr_3_;
  cv_bridge::CvImagePtr cv_ptr_4_;
  ros::Subscriber capture_;
  int seq_;
  bool capture_flag_0_;
  bool capture_flag_1_;
  bool capture_flag_2_;
  bool capture_flag_3_;
  bool capture_flag_4_;

 public:
  ImageArchive();
  ~ImageArchive();
  void initImageArchive();
 private:
  void captureCallback(const std_msgs::Header::ConstPtr& msg);
  void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
  void imageCallback4(const sensor_msgs::ImageConstPtr& msg);
  void saveAsPNG(std::string filename, cv::Mat image);
  void saveAsYAML(std::string filename, cv::Mat image, std::string encoding);
};

}

#endif //IMAGE_ARCHIVE_H
