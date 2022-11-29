#include <string>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <mrc_eo_msgs/Diagnostic.h>
#include <mrc_eo_msgs/BoundingBox.h>
#include <mrc_eo_msgs/BoundingBoxes.h>
#include <mrc_eo_msgs/ObjectCount.h>
#include <mrc_eo_msgs/FusionResult.h>

#include <mrc_eo/FusionResult.hpp>

namespace mrc_eo {

FusionResult::FusionResult(ros::NodeHandle nh)
    : nodeHandle_(nh), cam_count_sub(), cam_bb_sub(), lidar_count_sub(), lidar_bb_sub(), diagnostic_sub() {
  ROS_INFO("Fusion Result started.");

  init();
}

FusionResult::~FusionResult(){
}

void FusionResult::init() {
  ROS_INFO("[Fusion Result] init().");

  std::string objectDetectorCamTopicName;
  int objectDetectorCamQueueSize;
  std::string boundingBoxesCamTopicName;
  int boundingBoxesCamQueueSize;
  std::string objectDetectorLidarTopicName;
  int objectDetectorLidarQueueSize;
  std::string boundingBoxesLidarTopicName;
  int boundingBoxesLidarQueueSize;
  std::string diagnosticTopicName;
  int diagnosticQueueSize;
  std::string objectDetectionTopicName;
  int objectDetectionQueueSize;
  bool objectDetectionLatch;



  nodeHandle_.param("subscribers/cam_object_detector/topic", objectDetectorCamTopicName, std::string("/cam_found_object"));
  nodeHandle_.param("subscribers/cam_object_detector/queue_size", objectDetectorCamQueueSize, 1);
  nodeHandle_.param("subscribers/cam_bounding_boxes/topic", boundingBoxesCamTopicName, std::string("/cam_bounding_boxes"));
  nodeHandle_.param("subscribers/cam_bounding_boxes/queue_size", boundingBoxesCamQueueSize, 1);
  nodeHandle_.param("subscribers/lidar_object_detector/topic", objectDetectorLidarTopicName, std::string("/lidar_found_object"));
  nodeHandle_.param("subscribers/lidar_object_detector/queue_size", objectDetectorLidarQueueSize, 1);
  nodeHandle_.param("subscribers/lidar_bounding_boxes/topic", boundingBoxesLidarTopicName, std::string("/lidar_bounding_boxes"));
  nodeHandle_.param("subscribers/lidar_bounding_boxes/queue_size", boundingBoxesLidarQueueSize, 1);
  nodeHandle_.param("subscribers/diagnostic/topic", diagnosticTopicName, std::string("/diagnostic"));
  nodeHandle_.param("subscribers/diagnostic/queue_size", diagnosticQueueSize, 1);
  nodeHandle_.param("publishers/object_detection/topic", objectDetectionTopicName, std::string("/Object_detection"));
  nodeHandle_.param("publishers/object_detection/queue_size", objectDetectionQueueSize, 1);
  nodeHandle_.param("publishers/object_detection/latch", objectDetectionLatch, true);

  cam_count_sub = nodeHandle_.subscribe(objectDetectorCamTopicName, objectDetectorCamQueueSize, &FusionResult::CamCountCallback, this);
  cam_bb_sub = nodeHandle_.subscribe(boundingBoxesCamTopicName, boundingBoxesCamQueueSize, &FusionResult::CamBBCallback, this);
 lidar_count_sub = nodeHandle_.subscribe(objectDetectorLidarTopicName, objectDetectorLidarQueueSize, &FusionResult::LidarCountCallback, this);
  lidar_bb_sub = nodeHandle_.subscribe(boundingBoxesLidarTopicName, boundingBoxesLidarQueueSize, &FusionResult::LidarBBCallback, this);
  diagnostic_sub = nodeHandle_.subscribe(diagnosticTopicName, diagnosticQueueSize, &FusionResult::DiagonsitcCallback, this);
  fusionresult_pub = nodeHandle_.advertise<mrc_eo_msgs::FusionResult>(objectDetectionTopicName, objectDetectionQueueSize, objectDetectionLatch);
}

void FusionResult::CamCountCallback(const mrc_eo_msgs::ObjectCountConstPtr& msg)
{
  //printf("CamCountCallback");
  cam_obj.header.stamp = msg->header.stamp;
  cam_obj.header.frame_id = msg->header.frame_id;
  cam_obj.objectCount = msg->count;
}
void FusionResult::CamBBCallback(const mrc_eo_msgs::BoundingBoxesConstPtr& msg)
{
  //printf("CamBBCallback");
  if (cam_obj.objectCount != 0)
  {
    cam_obj.bounding_boxes = msg->bounding_boxes;
  }
  
}
void FusionResult::LidarCountCallback(const mrc_eo_msgs::ObjectCountConstPtr& msg)
{
  //printf("LidarCountCallback");
  lidar_obj.header.stamp = msg->header.stamp;
  lidar_obj.header.frame_id = msg->header.frame_id;
  lidar_obj.objectCount = msg->count;
}

void FusionResult::LidarBBCallback(const mrc_eo_msgs::BoundingBoxesConstPtr& msg)
{
  //printf("LidarBBCallback");
  if (lidar_obj.objectCount != 0)
  {
    lidar_obj.bounding_boxes = msg->bounding_boxes;
  }
}
void FusionResult::DiagonsitcCallback(const mrc_eo_msgs::DiagnosticConstPtr& msg)
{
  //printf("DiagonsitcCallback");
  cam_dtc = msg->cam_dtc;
  lidar_dtc = msg->lidar_dtc;
}

void FusionResult::FusionResultPub()
{

  std::string CondionNormal = "Normal";

  if (cam_dtc == CondionNormal) {
  fusion_obj = cam_obj;
  fusionresult_pub.publish(fusion_obj);
  
  }
  else if ((cam_dtc != CondionNormal) && (lidar_dtc == CondionNormal)) {
  fusion_obj = lidar_obj;
  fusionresult_pub.publish(fusion_obj);
  }
}


}
