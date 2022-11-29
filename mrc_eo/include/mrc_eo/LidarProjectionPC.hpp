#pragma once

#include <iostream>

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

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
        sensor_msgs::Image> SyncPolicy;

class LidarProjectionPC {
 public:
  /*!
   * Constructor.
   */
  explicit LidarProjectionPC(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~LidarProjectionPC();

 private:
  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Return the normalized data from depth
   */
  int normalize_data(double val, int min, int max, int scale);

  /*!
   * Make projection image when get Lidar data
   */
  void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nodeHandle_;

  ros::Subscriber lidar_sub;
 
  ros::Publisher pub;

  int height_, width_;
};

} /* namespace mrc_eo*/
