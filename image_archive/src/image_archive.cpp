#include "image_archive/image_archive.h"

using namespace image_archive;

ImageArchive::ImageArchive()
    : it_(nh_)
{
  ROS_INFO("init");
  initImageArchive();
}

ImageArchive::~ImageArchive()
{
}

void ImageArchive::initImageArchive()
{
  seq_ = 0;
  capture_flag_0_ = false;
  capture_flag_1_ = false;
  capture_flag_2_ = false;
  capture_flag_3_ = false;
  capture_flag_4_ = false;
  capture_ = nh_.subscribe("image_archive", 10, &ImageArchive::captureCallback,this);
  image_sub_0_ = it_.subscribe("grouper_lidar/depthmap_image", 10, &ImageArchive::imageCallback0, this);
  image_sub_1_ = it_.subscribe("grouper_lidar/intensity_image", 10, &ImageArchive::imageCallback1, this);
  image_sub_2_ = it_.subscribe("zed/left/image_raw_color", 10, &ImageArchive::imageCallback2, this);
  image_sub_3_ = it_.subscribe("zed/right/image_raw_color", 10, &ImageArchive::imageCallback3, this);
  image_sub_4_ = it_.subscribe("zed/depth/depth_registered", 10, &ImageArchive::imageCallback4, this);
}

void ImageArchive::captureCallback(const std_msgs::Header::ConstPtr& msg)
{
  capture_flag_0_ = true;
  capture_flag_1_ = true;
  capture_flag_2_ = true;
  capture_flag_3_ = true;
  capture_flag_4_ = true;
  seq_++;
}

void ImageArchive::imageCallback0(const sensor_msgs::ImageConstPtr& msg)
{
  if(capture_flag_0_)
  {
    try
    {
      cv_ptr_0_ = cv_bridge::toCvCopy(msg, msg->encoding);

      static std::ostringstream ss;
//      ROS_INFO("save");
      ss.str("");
//      ROS_INFO("save ss");
      ss << "lidar-depth-";
//      ROS_INFO("save fpre");
      ss << seq_;
//      ROS_INFO("save seq");
//      ROS_INFO("%s",ss.str().c_str());
      ss << ".yml";
      saveAsYAML(ss.str(), cv_ptr_0_->image, cv_ptr_0_->encoding);
      capture_flag_0_ = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}
void ImageArchive::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  if(capture_flag_1_)
  {
    try
    {
      cv_ptr_1_ = cv_bridge::toCvCopy(msg, msg->encoding);

      static std::ostringstream ss;
//      ROS_INFO("save");
      ss.str("");
//      ROS_INFO("save ss");
      ss << "lidar-intensity-";
//      ROS_INFO("save fpre");
      ss << seq_;
//      ROS_INFO("save seq");
//      ROS_INFO("%s",ss.str().c_str());
      ss << ".yml";
      saveAsYAML(ss.str(), cv_ptr_1_->image, cv_ptr_1_->encoding);
      capture_flag_1_ = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}
void ImageArchive::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  if(capture_flag_2_)
  {
    try
    {
      cv_ptr_2_ = cv_bridge::toCvCopy(msg, msg->encoding);

      static std::ostringstream ss;
//      ROS_INFO("save");
      ss.str("");
//      ROS_INFO("save ss");
      ss << "zed-left-";
//      ROS_INFO("save fpre");
      ss << seq_;
//      ROS_INFO("save seq");
//      ROS_INFO("%s",ss.str().c_str());
      ss << ".jpg";
      saveAsPNG(ss.str(), cv_ptr_2_->image);
      capture_flag_2_ = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}
void ImageArchive::imageCallback3(const sensor_msgs::ImageConstPtr& msg)
{
  if(capture_flag_3_)
  {
    try
    {
      cv_ptr_3_ = cv_bridge::toCvCopy(msg, msg->encoding);

      static std::ostringstream ss;
//      ROS_INFO("save");
      ss.str("");
//      ROS_INFO("save ss");
      ss << "zed-right-";
//      ROS_INFO("save fpre");
      ss << seq_;
//      ROS_INFO("save seq");
//      ROS_INFO("%s",ss.str().c_str());
      ss << ".jpg";
      saveAsPNG(ss.str(), cv_ptr_3_->image);
      capture_flag_3_ = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}
void ImageArchive::imageCallback4(const sensor_msgs::ImageConstPtr& msg)
{
  if(capture_flag_4_)
  {
    try
    {
      cv_ptr_4_ = cv_bridge::toCvCopy(msg, msg->encoding);

      static std::ostringstream ss;
//      ROS_INFO("save");
      ss.str("");
//      ROS_INFO("save ss");
      ss << "zed-depth-";
//      ROS_INFO("save fpre");
      ss << seq_;
//      ROS_INFO("save seq");
//      ROS_INFO("%s",ss.str().c_str());
      ss << ".yml";
      saveAsYAML(ss.str(), cv_ptr_4_->image, cv_ptr_4_->encoding);
      capture_flag_4_ = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}












void ImageArchive::saveAsPNG(std::string filename, cv::Mat image)
{
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
  try
  {
    cv::imwrite(filename, image, compression_params);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Exception converting image to PNG format: %s\n", e.what());
    return;
  }
}

void ImageArchive::saveAsYAML(std::string filename, cv::Mat image, std::string encoding)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "encoding" << encoding;
  fs << "image" << image;
  fs.release();
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_archive");

  time_t timep;
  struct tm *p;
  time(&timep);
  p = localtime(&timep);
  std::ostringstream oss;
//  oss << "log/";
  oss << 1900+p->tm_year << '-';
  oss << 1+p->tm_mon << '-';
  oss << p->tm_mday << '-';
  oss << p->tm_hour << '-';
  oss << p->tm_min << '-';
  oss << p->tm_sec;
  ROS_INFO("%s",oss.str().c_str());
  chdir("/home/phx/dev/ros/log");
  int mkdirResult = mkdir(oss.str().c_str(), 0777);
//  ROS_INFO("mkdirResult %d",mkdirResult);
//  ROS_INFO("2");
  int chdirResult = chdir(oss.str().c_str());
//  ROS_INFO("chdirResult %d",chdirResult);
  ROS_INFO("before ia");

  ImageArchive image_archive_;

  ros::spin();
//  ROS_INFO("0");
//  return 0;
}
