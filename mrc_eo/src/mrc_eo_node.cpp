#include <ros/ros.h>
#include <mrc_eo/SensorDiagnostic.hpp>
#include <mrc_eo/LidarProjection.hpp>
#include <mrc_eo/LidarProjectionPC.hpp>
#include <mrc_eo/LidarProjectionOri.hpp>
#include <mrc_eo/CamObjectDetector.hpp>
#include <mrc_eo/LidarObjectDetector.hpp>
#include <mrc_eo/FusionResult.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrc_eo");
  ros::NodeHandle nodeHandle("~");

  mrc_eo::SensorDiagnostic SensorDiagnostic(nodeHandle);
  mrc_eo::LidarProjectionPC LidarProjectionPC(nodeHandle);
  mrc_eo::CamObjectDetector CamObjectDetector(nodeHandle);
  mrc_eo::LidarObjectDetector LidarObjectDetector(nodeHandle);
  mrc_eo::FusionResult FusionResult(nodeHandle);

  //mrc_eo::LidarProjection LidarProjection(nodeHandle);         // projection(image+pc)
  //mrc_eo::LidarProjectionOri LidarProjectionOri(nodeHandle);   // projection(image+pc, pc)

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    SensorDiagnostic.Diagnostic();
    FusionResult.FusionResultPub();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
