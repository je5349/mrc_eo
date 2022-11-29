#pragma once

// c++
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// mrc_eo_msgs
#include <mrc_eo_msgs/BoundingBox.h>
#include <mrc_eo_msgs/BoundingBoxes.h>
#include <mrc_eo_msgs/CheckForObjectsAction.h>
#include <mrc_eo_msgs/ObjectCount.h>
#include "../../../darknet/src/blas.h"

// Darknet.
#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "mrc_eo/image_interface.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}

extern "C" void ipl_into_image(IplImage* src, image im);
extern "C" image ipl_to_image(IplImage* src);
// extern "C" void show_image_cv(image p, const char* name, IplImage* disp);

namespace mrc_eo {

//! Bounding box of the detected object.
typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

typedef struct {
  IplImage* image;
  std_msgs::Header header;
} IplImageWithHeader_;

} /* namespace mrc_eo*/
