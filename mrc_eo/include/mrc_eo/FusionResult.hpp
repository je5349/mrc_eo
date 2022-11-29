#pragma once

#include <iostream>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <mrc_eo_msgs/Diagnostic.h>
#include <mrc_eo_msgs/BoundingBox.h>
#include <mrc_eo_msgs/BoundingBoxes.h>
#include <mrc_eo_msgs/ObjectCount.h>
#include <mrc_eo_msgs/FusionResult.h>

namespace mrc_eo {

class FusionResult {
 public:
  /*!
   * Constructor.
   */
  explicit FusionResult(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~FusionResult();

  /*!
   * Publish Object detection result
   */
  void FusionResultPub();

 private:
  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Subscribe Camera detection result
   */
  void CamCountCallback(const mrc_eo_msgs::ObjectCountConstPtr& msg);

  void CamBBCallback(const mrc_eo_msgs::BoundingBoxesConstPtr& msg);

  /*!
   * Subscribe Lidar detection result
   */
  void LidarCountCallback(const mrc_eo_msgs::ObjectCountConstPtr& msg);

  void LidarBBCallback(const mrc_eo_msgs::BoundingBoxesConstPtr& msg);

  /*!
   * Subscribe Diagnostic data
   */
  void DiagonsitcCallback(const mrc_eo_msgs::DiagnosticConstPtr& msg);

  ros::NodeHandle nodeHandle_;

  ros::Subscriber cam_count_sub, cam_bb_sub, lidar_count_sub, lidar_bb_sub, diagnostic_sub;

  ros::Publisher fusionresult_pub;

  mrc_eo_msgs::FusionResult cam_obj, lidar_obj, fusion_obj;

  std::string cam_dtc, lidar_dtc;

};

} /* namespace mrc_eo*/
