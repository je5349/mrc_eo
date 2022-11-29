#pragma once

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <mrc_eo_msgs/Diagnostic.h>

namespace mrc_eo {

class SensorDiagnostic {
 public:
  /*!
   * Constructor.
   */
  explicit SensorDiagnostic(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~SensorDiagnostic();

   /*!
   * Publish diagnostic msg
   */
  void Diagnostic();

 private:
  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Subscribe Camera data
   */
  void CameraCallback(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * Subscribe Lidar data
   */
  void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nodeHandle_;

  ros::Subscriber cam_sub, lidar_sub;

  ros::Publisher diagnostic_pub;

  double cam_time;
  double lidar_time;

};

} /* namespace mrc_eo*/
