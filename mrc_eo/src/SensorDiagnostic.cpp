#include <ros/ros.h>

#include <mrc_eo/SensorDiagnostic.hpp>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <mrc_eo_msgs/Diagnostic.h>

namespace mrc_eo {

SensorDiagnostic::SensorDiagnostic(ros::NodeHandle nh)
    : nodeHandle_(nh), cam_sub(), lidar_sub() {
  ROS_INFO("Sensor Diagnostic started.");

  init();
}

SensorDiagnostic::~SensorDiagnostic(){
}

void SensorDiagnostic::init() {
  ROS_INFO("[Sensor Diagnostic] init().");

  std::string lidarTopicName;
  int lidarQueueSize;
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string diagnosticTopicName;
  int diagnosticQueueSize;
  bool diagnosticLatch;

  nodeHandle_.param("subscribers/lidar_pc/topic", lidarTopicName, std::string("/kitti/velo/pointcloud"));
  nodeHandle_.param("subscribers/lidar_pc/queue_size", lidarQueueSize, 1);
  nodeHandle_.param("subscribers/camera_image/topic", cameraTopicName, std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/camera_image/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/diagnostic/topic", diagnosticTopicName, std::string("/diagnostic"));
  nodeHandle_.param("publishers/diagnostic/queue_size", diagnosticQueueSize, 1);
  nodeHandle_.param("publishers/diagnostic/latch", diagnosticLatch, true);

  lidar_sub = nodeHandle_.subscribe(lidarTopicName, lidarQueueSize, &SensorDiagnostic::LidarCallback, this);
  cam_sub = nodeHandle_.subscribe(cameraTopicName, cameraQueueSize, &SensorDiagnostic::CameraCallback, this);

  diagnostic_pub = nodeHandle_.advertise<mrc_eo_msgs::Diagnostic>(diagnosticTopicName, diagnosticQueueSize, diagnosticLatch);

}

void SensorDiagnostic::CameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //printf("I get the image data \n");
  cam_time = ros::Time::now().toSec();
}

void SensorDiagnostic::LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //printf("I get the point cloud data \n");
  lidar_time = ros::Time::now().toSec();
}

void SensorDiagnostic::Diagnostic()
{
  mrc_eo_msgs::Diagnostic diagnostic_msg;
  diagnostic_msg.cam_dtc = "None";
  diagnostic_msg.lidar_dtc = "None";

  double current_time;
  current_time = ros::Time::now().toSec();
   
  //printf("cam time : %f \n", cam_time);
  //printf("lidar time : %f \n", lidar_time);

  diagnostic_msg.header.stamp = ros::Time::now();
  diagnostic_msg.header.frame_id = "diagnostic";

  if (abs(current_time - cam_time) > 3) {
  diagnostic_msg.cam_dtc = "Fault";
  }
  else {
  diagnostic_msg.cam_dtc = "Normal";
  }

  if (abs(current_time - lidar_time) > 3) {
  diagnostic_msg.lidar_dtc = "Fault";
  }
  else {
  diagnostic_msg.lidar_dtc = "Normal";
  }

  diagnostic_pub.publish(diagnostic_msg);

}

}
