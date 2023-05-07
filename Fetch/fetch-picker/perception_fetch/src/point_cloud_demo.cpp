#include "perception_fetch/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/Marker.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "seg_point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher segment_pub =
      nh.advertise<visualization_msgs::Marker>("segment_cloud", 1, true);
  perception_fetch::Segmenter segmenter(segment_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception_fetch::Segmenter::Callback, &segmenter);
  ros::spin();
  return 0;
}
