#include <mrc_eo/YoloObjectDetector.hpp>
#include "mrc_eo/LidarObjectDetector.hpp"

#include <X11/Xlib.h>

#include <algorithm>
#include <cmath>
#include <string>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_lidar = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace mrc_eo {

char* cfg_lidar;
char* weights_lidar;
char* data_lidar;
char** detectionNames_lidar;

LidarObjectDetector::LidarObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh), imageTransport_(nodeHandle_), numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0) {
  ROS_INFO("[LidarObjectDetector] started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

LidarObjectDetector::~LidarObjectDetector() {
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool LidarObjectDetector::readParameters() {
  // Load common parameters.
  nodeHandle_.param("image_view/lidar_enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/lidar_wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/lidar_enable_console_output", enableConsoleOutput_, false);

  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[LidarObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[LidarObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  nodeHandle_.param("lidar_yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void LidarObjectDetector::init() {
  ROS_INFO("[LidarObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("lidar_yolo_model/threshold/value", thresh, (float)0.3);

  // Path to weights file.
  nodeHandle_.param("lidar_yolo_model/weight_file/name", weightsModel, std::string("yolov3-tiny.weights"));
  nodeHandle_.param("lidar_weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights_lidar = new char[weightsPath.length() + 1];
  strcpy(weights_lidar, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("lidar_yolo_model/config_file/name", configModel, std::string("yolov3-tiny.cfg"));
  nodeHandle_.param("lidar_config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg_lidar = new char[configPath.length() + 1];
  strcpy(cfg_lidar, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_lidar;
  dataPath += "/data";
  data_lidar = new char[dataPath.length() + 1];
  strcpy(data_lidar, dataPath.c_str());

  // Get classes.
  detectionNames_lidar = (char**)realloc((void*)detectionNames_lidar, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames_lidar[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames_lidar[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg_lidar, weights_lidar, data_lidar, thresh, detectionNames_lidar, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
  yoloThread_ = std::thread(&LidarObjectDetector::yolo, this);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  nodeHandle_.param("subscribers/lidar_image/topic", cameraTopicName, std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/lidar_image/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/lidar_object_detector/topic", objectDetectorTopicName, std::string("lidar_found_object"));
  nodeHandle_.param("publishers/lidar_object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/lidar_object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/lidar_bounding_boxes/topic", boundingBoxesTopicName, std::string("lidar_bounding_boxes"));
  nodeHandle_.param("publishers/lidar_bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/lidar_bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/lidar_detection_image/topic", detectionImageTopicName, std::string("lidar_detection_image"));
  nodeHandle_.param("publishers/lidar_detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/lidar_detection_image/latch", detectionImageLatch, true);

  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &LidarObjectDetector::cameraCallback, this);
  objectPublisher_ =
      nodeHandle_.advertise<mrc_eo_msgs::ObjectCount>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
  boundingBoxesPublisher_ =
      nodeHandle_.advertise<mrc_eo_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ =
      nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/lidar_reading/name", checkForObjectsActionName, std::string("lidar_check_for_objects"));
  checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(boost::bind(&LidarObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&LidarObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();
}

void LidarObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_DEBUG("[LidarObjectDetector] USB image received.");

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }

  return;
}

void LidarObjectDetector::checkForObjectsActionGoalCB() {
  ROS_DEBUG("[LidarObjectDetector] Start check for objects action.");

  boost::shared_ptr<const mrc_eo_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_);
      actionId_ = imageActionPtr->id;
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

void LidarObjectDetector::checkForObjectsActionPreemptCB() {
  ROS_DEBUG("[LidarObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool LidarObjectDetector::isCheckingForObjects() const {
  return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested());
}

bool LidarObjectDetector::publishDetectionImage(const cv::Mat& detectionImage) {
  if (detectionImagePublisher_.getNumSubscribers() < 1) return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "lidar_detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

// double LidarObjectDetector::getWallTime()
// {
//   struct timeval time;
//   if (gettimeofday(&time, NULL)) {
//     return 0;
//   }
//   return (double) time.tv_sec + (double) time.tv_usec * .000001;
// }

int LidarObjectDetector::sizeNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      count += l.outputs;
    }
  }
  return count;
}

void LidarObjectDetector::rememberNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection* LidarObjectDetector::avgPredictions(network* net, int* nboxes) {
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for (j = 0; j < demoFrame_; ++j) {
    axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  // detection* dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  detection* dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes, 1);
  return dets;
}

void* LidarObjectDetector::detectInThread() {
  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float* X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float* prediction = network_predict(*net_, X);

  rememberNetwork(net_);
  detection* dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  if (enableConsoleOutput_) {
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nObject Detection Result with Lidar\n");
    printf("\nFPS:%.1f\n", fps_);
    printf("Objects:\n\n");
  }

  image display = buff_[(buffIndex_ + 2) % 3];
  // draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, 1);
  draw_detections_v3(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, 1);

  // extract the bounding boxes and send them to ROS
  int i, j;
  int count = 0;
  for (i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) {
      if (dets[i].prob[j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = dets[i].prob[j];
          count++;
        }
      }
    }
  }

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) {
    roiBoxes_[0].num = 0;
  } else {
    roiBoxes_[0].num = count;
  }

  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

void* LidarObjectDetector::fetchInThread() {
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
    IplImage* ROS_img = imageAndHeader.image;
    ipl_into_image(ROS_img, buff_[buffIndex_]);
    headerBuff_[buffIndex_] = imageAndHeader.header;
    buffId_[buffIndex_] = actionId_;
  }
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  return 0;
}

void* LidarObjectDetector::displayInThread(void* ptr) {
  show_image_cv(buff_[(buffIndex_ + 1) % 3], "YOLO V3 tiny");
  int c = cv::waitKey(waitKeyDelay_);
  if (c != -1) c = c % 256;
  if (c == 27) {
    demoDone_ = 1;
    return 0;
  } else if (c == 82) {
    demoThresh_ += .02;
  } else if (c == 84) {
    demoThresh_ -= .02;
    if (demoThresh_ <= .02) demoThresh_ = .02;
  } else if (c == 83) {
    demoHier_ += .02;
  } else if (c == 81) {
    demoHier_ -= .02;
    if (demoHier_ <= .0) demoHier_ = .0;
  }
  return 0;
}

void* LidarObjectDetector::displayLoop(void* ptr) {
  while (1) {
    displayInThread(0);
  }
}

void* LidarObjectDetector::detectLoop(void* ptr) {
  while (1) {
    detectInThread();
  }
}

void LidarObjectDetector::setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay,
                                      char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) {
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image** alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  ROS_INFO("YOLO V3 tiny with Lidar\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void LidarObjectDetector::yolo() {
  const auto wait_duration = std::chrono::milliseconds(2000);
  while (!getImageStatus()) {
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float**)calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i) {
    predictions_[i] = (float*)calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float*)calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (mrc_eo::RosBox_*)calloc(l.w * l.h * l.n, sizeof(mrc_eo::RosBox_));

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
    IplImage* ROS_img = imageAndHeader.image;
    buff_[0] = ipl_to_image(ROS_img);
    headerBuff_[0] = imageAndHeader.header;
  }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

  int count = 0;

  if (!demoPrefix_ && viewImage_) {
    cv::namedWindow("YOLO V3 tiny", cv::WINDOW_NORMAL);
    if (fullScreen_) {
      cv::setWindowProperty("YOLO V3 tiny", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
      cv::moveWindow("YOLO V3 tiny", 0, 0);
      cv::resizeWindow("YOLO V3 tiny", 640, 480);
    }
  }

  demoTime_ = what_time_is_it_now();

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&LidarObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&LidarObjectDetector::detectInThread, this);

    if (!demoPrefix_) {
      fps_ = 1. / (what_time_is_it_now() - demoTime_);
      demoTime_ = what_time_is_it_now();
      if (viewImage_) {
        generate_image(buff_[(buffIndex_ + 1) % 3], ipl_);
        displayInThread(0);
      } else {
        generate_image(buff_[(buffIndex_ + 1) % 3], ipl_);
      }

      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }
}

IplImageWithHeader_ LidarObjectDetector::getIplImageWithHeader() {
  IplImage* ROS_img = new IplImage(camImageCopy_);
  IplImageWithHeader_ header = {.image = ROS_img, .header = imageHeader_};
  return header;
}

bool LidarObjectDetector::getImageStatus(void) {
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool LidarObjectDetector::isNodeRunning(void) {
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* LidarObjectDetector::publishInThread() {
  // Publish image.
  cv::Mat cvImage = cv::cvarrToMat(ipl_);
  if (!publishDetectionImage(cv::Mat(cvImage))) {
    ROS_DEBUG("Detection image has not been broadcasted.");
  }

  // Publish bounding boxes and detection result.
  int num = roiBoxes_[0].num;
  if (num > 0 && num <= 100) {
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (roiBoxes_[i].Class == j) {
          rosBoxes_[j].push_back(roiBoxes_[i]);
          rosBoxCounter_[j]++;
        }
      }
    }

    mrc_eo_msgs::ObjectCount msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "lidar_detection";
    msg.count = num;
    objectPublisher_.publish(msg);

    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) {
        mrc_eo_msgs::BoundingBox boundingBox;

        for (int j = 0; j < rosBoxCounter_[i]; j++) {
          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

/////////////////////////////////////////////////////////////////////////
          // calculate distance
          std::string label_car = "Car";
          float dist = 0;

          if (label_car == classLabels_[i]) {          
            float top_ori = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;
            float bottom_ori = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
            float left_ori = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
            float right_ori = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
            //printf("top_ori : %f, bottom_ori : %f, left_ori : %f, right_ori : %f \n", top_ori, bottom_ori, left_ori, right_ori);

            int tmp = floor(top_ori+0.5);
            int top = std::max(0, tmp);
            tmp = floor(bottom_ori+0.5);
            int bottom = std::min(frameHeight_, tmp);
            tmp = floor(left_ori+0.5);
            int left = std::max(0, tmp);
            tmp = floor(right_ori+0.5);
            int right = std::min(frameWidth_, tmp);
            
            int width = abs(right - left);
            int height = abs(top - bottom);           
            printf("top : %d, bottom : %d, left : %d, right : %d\n", top, bottom, left, right);
            printf("width : %i, height : %i \n", width, height);
                      
            int car_original_width = 72; // in inch
            int car_original_height = 60; // in inch
            int f = 645.24; // focal length

            //calculate distance with width
            dist = (car_original_width * f)/width;
            dist = dist/39.37007847; // inch to m
            printf("calculate distance with width : %lf\n", dist);
            
            //calculate distance with height
	    dist = (car_original_height * f)/height;
            dist = dist/39.37007847; //inch to cm
            printf("calculate distance with height : %f\n", dist);
            printf("\n");
          }
/////////////////////////////////////////////////////////////////////////
          boundingBox.Class = classLabels_[i];
          boundingBox.id = i;
          boundingBox.probability = rosBoxes_[i][j].prob;
          boundingBox.xmin = xmin;
          boundingBox.ymin = ymin;
          boundingBox.xmax = xmax;
          boundingBox.ymax = ymax;
          boundingBox.dist = dist;
          boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
        }
      }
    }
    boundingBoxesResults_.header.stamp = ros::Time::now();
    boundingBoxesResults_.header.frame_id = "lidar_detection";
    boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];
    boundingBoxesPublisher_.publish(boundingBoxesResults_);
  } else {
    mrc_eo_msgs::ObjectCount msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "lidar_detection";
    msg.count = 0;
    objectPublisher_.publish(msg);
  }
  if (isCheckingForObjects()) {
    ROS_DEBUG("[LidarObjectDetector] check for objects in image.");
    mrc_eo_msgs::CheckForObjectsResult objectsActionResult;
    objectsActionResult.id = buffId_[0];
    objectsActionResult.bounding_boxes = boundingBoxesResults_;
    checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
  }
  boundingBoxesResults_.bounding_boxes.clear();
  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  return 0;
}

} /* namespace mrc_eo*/
