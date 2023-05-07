#include "perception_fetch/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception_fetch {

Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}


void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  // ROS_INFO("Got point cloud with %ld points in frame %s", cloud->size(), cloud->header.frame_id.c_str());

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  PointCloudC::Ptr output_cloud(new PointCloudC());
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, -2.0);
  ros::param::param("crop_min_y", min_y, 0.1);
  ros::param::param("crop_min_z", min_z, -0.1);
  ros::param::param("crop_max_x", max_x, 0.2);
  ros::param::param("crop_max_y", max_y, 0.22);
  ros::param::param("crop_max_z", max_z, 0.0);

  double tx, ty, tz, rx, ry, rz, rw;

  ros::param::param("crop_tx", tx, 0.0);
  ros::param::param("crop_ty", ty, 0.0);
  ros::param::param("crop_tz", tz, 0.0);

  ros::param::param("crop_rx", rx, 0.0);
  ros::param::param("crop_ry", ry, 0.0);
  ros::param::param("crop_rz", rz, 0.0);
  ros::param::param("crop_rw", rw, 0.0);
  // ROS_INFO("Set crop box at  tx %f, ty %f, tz %f", tx, ty, tz);
  // ROS_INFO("Set crop box ori at  rx %f, ry %f, rz %f, rw %f", rx, ry, rz, rw);


  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  Eigen::Vector3f trans(tx, ty, tz);
  Eigen::Quaternionf quat (rx, ry, rz, rw);
  Eigen::Vector3f rot = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  // rot[2] = 0;
  // Eigen::Vector3f rot(0.0f, 0.0f, 0.0f);
  // std::cout << "Roll: " << rot[0] * 180.0 / M_PI << std::endl;
  // std::cout << "Pitch: " << rot[1] * 180.0 / M_PI << std::endl;
  // std::cout << "Yaw: " << rot[2] * 180.0 / M_PI << std::endl;
  // ROS_INFO("Set crop box ori at %d, %d", rot[0], rot[1]);
  
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setTranslation(trans);
  crop.setRotation(rot);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*output_cloud);
  
  // pcl::VoxelGrid<PointC> vox;
  // vox.setInputCloud(cropped_cloud);
  // vox.filter(*output_cloud);
  
  // calc CoM of obj
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*output_cloud, min_pcl, max_pcl);
  // ROS_INFO("min: %f, max: %f", min_pcl.y, max_pcl.y);
  
  // ROS_INFO("Cropped and Downsampled to %ld points", output_cloud->size());

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*output_cloud, msg_out);
  pub_.publish(msg_out);
}
}
