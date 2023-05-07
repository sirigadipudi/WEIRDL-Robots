#include "perception_fetch/crop.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception_fetch::Cropper cropper(crop_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception_fetch::Cropper::Callback, &cropper);
  ros::spin();
  return 0;
}
