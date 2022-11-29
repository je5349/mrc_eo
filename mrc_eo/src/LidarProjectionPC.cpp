#include "mrc_eo/LidarProjectionPC.hpp"

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

namespace mrc_eo {

LidarProjectionPC::LidarProjectionPC(ros::NodeHandle nh)
    : nodeHandle_(nh), lidar_sub(){
  ROS_INFO("[Lidar Projection] started.");

  init();
}

LidarProjectionPC::~LidarProjectionPC() {
}

void LidarProjectionPC::init() {
  ROS_INFO("[Lidar Projection] init().");

  std::string lidarTopicName;
  int lidarQueueSize;
  std::string lidarProjectionPCTopicName;
  int lidarProjectionPCImageQueueSize;
  bool lidarProjectionPCImageLatch;

  nodeHandle_.param("subscribers/lidar_pc/topic", lidarTopicName, std::string("/kitti/velo/pointcloud"));
  nodeHandle_.param("subscribers/lidar_pc/queue_size", lidarQueueSize, 1);
  nodeHandle_.param("publishers/projection_pc_image/topic", lidarProjectionPCTopicName, std::string("projected_image_pc"));
  nodeHandle_.param("publishers/projection_pc_image/queue_size", lidarProjectionPCImageQueueSize, 1);
  nodeHandle_.param("publishers/projection_pc_image/latch", lidarProjectionPCImageLatch, true);

  lidar_sub = nodeHandle_.subscribe(lidarTopicName, lidarQueueSize, &LidarProjectionPC::LidarCallback, this);

  pub = nodeHandle_.advertise<sensor_msgs::Image> (lidarProjectionPCTopicName, lidarProjectionPCImageQueueSize, lidarProjectionPCImageLatch);

  height_ = 375;
  width_ = 1242;

}

int LidarProjectionPC::normalize_data(double val, int min, int max, int scale)
{
  return ((max - val) / (max - min))*scale;
}

void LidarProjectionPC::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  //ROS_INFO_STREAM("At Callback");

  sensor_msgs::PointCloud2 ROS_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *in_cloud);

  //ROS_INFO_STREAM("copy point cloud");

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat r(3,3,cv::DataType<double>::type);
  cv::Mat tr(3,4,cv::DataType<double>::type);
  
  // Projection matrix (3 X 4)
  p.at<double>(0,0) = 7.215377000000e+02;    p.at<double>(0,1) = 0.000000000000e+00;    p.at<double>(0,2) = 6.095593000000e+02;    p.at<double>(0,3) = 0.000000000000e+0;
  p.at<double>(1,0) = 0.00000000;    p.at<double>(1,1) = 7.215377000000e+02;    p.at<double>(1,2) = 1.728540000000e+02;    p.at<double>(1,3) = 0.000000000000e+0;  
  p.at<double>(2,0) = 0.00000000;    p.at<double>(2,1) = 0.00000000;    p.at<double>(2,2) = 1.00000000;    p.at<double>(2,3) = 0.000000000000e+0;  

  // Rotation matrix (3 X 3)
  r.at<double>(0,0) = 9.999239000000e-01;    r.at<double>(0,1) = 9.837760000000e-03;    r.at<double>(0,2) = -7.445048000000e-03;
  r.at<double>(1,0) = -9.869795000000e-033;    r.at<double>(1,1) = 9.999421000000e-01;    r.at<double>(1,2) = -4.278459000000e-03;
  r.at<double>(2,0) = 7.402527000000e-03;    r.at<double>(2,1) = 4.351614000000e-03;    r.at<double>(2,2) = 9.999631000000e-01;

  // Tr_velo_to_cam (3 X 4)
  tr.at<double>(0,0) = 7.533745000000e-03;    tr.at<double>(0,1) = -9.999714000000e-01;    tr.at<double>(0,2) = -6.166020000000e-04;    tr.at<double>(0,3) = -4.069766000000e-03;
  tr.at<double>(1,0) = 1.480249000000e-02;    tr.at<double>(1,1) = 7.280733000000e-04;    tr.at<double>(1,2) = -9.998902000000e-01;    tr.at<double>(1,3) = -7.631618000000e-02;
  tr.at<double>(2,0) = 9.998621000000e-01;    tr.at<double>(2,1) = 7.523790000000e-03;    tr.at<double>(2,2) = 1.480755000000e-02;    tr.at<double>(2,3) = -2.717806000000e-01;
  
  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat Z(4,1,cv::DataType<double>::type);
  cv::Mat F(3,1,cv::DataType<double>::type);
  double D;

  cv::Mat HSV_empty_img, result;
  cv::Mat empty_img(height_, width_, CV_8UC3,  cv::Scalar(255,255,255));

  cvtColor(empty_img, HSV_empty_img, cv::COLOR_BGR2HSV);

  cv::Point pt;
  
  // access point on everything
  for(auto it=in_cloud->begin(); it!=in_cloud->end(); ++it)
  {
    if(it->x >=0)
    {
      X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
      Y = r * tr * X;
      Z.at<double>(0,0) = Y.at<double>(0,0); Z.at<double>(1,0) = Y.at<double>(1,0); Z.at<double>(2,0) = Y.at<double>(2,0); Z.at<double>(3,0) = 1;
      F = p * Z;

      pt.x = F.at<double>(0,0) / F.at<double>(0,2);
      pt.y = F.at<double>(1,0) / F.at<double>(0,2);
      
      D = sqrt(pow(it->x,2)+pow(it->y,2)+pow(it->z,2));

      int color = normalize_data(D, 1, 70, 120);

      float val = it->x;
      float intensity = it-> intensity;

      // put the point on Image
      cv::circle(HSV_empty_img, pt, 2., cv::Scalar(color, 255, 255), -1);
    }
  }

  cvtColor(HSV_empty_img, result, cv::COLOR_HSV2BGR);

  sensor_msgs::ImagePtr projection_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();

  pub.publish(projection_img_msg);
}

} /* namespace mrc_eo*/