#include "perception_fetch/segmentation.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception_fetch {
  Segmenter::Segmenter(const ros::Publisher& pub) : marker_pub_(pub) {}

  void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  // ROS_INFO("object cloud frame_id %s\n", msg.header.frame_id.c_str());

  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  // ROS_INFO("Got point cloud with %ld points in frame %s", cloud->size(), cloud->header.frame_id.c_str());
  std::vector<pcl::PointIndices> object_indices;
  SegmentBinObjects(cloud, &object_indices);
  float largest_scale = -1.0;
  visualization_msgs::Marker largest_object_marker;
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.filter(*object_cloud);
    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = msg.header.frame_id;
    object_marker.type = visualization_msgs::Marker::CUBE;

    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                              &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;

    // Keep track of the largest object marker
    float object_scale = object_marker.scale.x * object_marker.scale.y * object_marker.scale.z;
    if (object_scale > largest_scale) {
      largest_scale = object_scale;
      largest_object_marker = object_marker;
    }
  }
  // Publish only the largest object marker
  marker_pub_.publish(largest_object_marker);
}

  void Segmenter::SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices) {
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 7000);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    // euclid.setIndices(inside_bin_indices); 
    // << TODO: is cloud already cropped or do we get indices for the crop?
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < indices->size(); ++i) {;
      size_t cluster_size = indices->at(i).indices.size();
      if(cluster_size < min_size) {
        min_size = cluster_size;
      } 
      if(cluster_size > max_size){
        max_size = cluster_size;
      }
    }

    // ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
            //  indices->size(), min_size, max_size);
    

  }

  void Segmenter::GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::Pose* pose, 
    geometry_msgs::Vector3* dimensions){
        // Compute the minimum and maximum coordinates of the point cloud
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        

        // Compute the center of the bounding box
        float center_x = (max_pt.x + min_pt.x) / 2;
        float center_y = (max_pt.y + min_pt.y) / 2;
        float center_z = (max_pt.z + min_pt.z) / 2;
        // ROS_INFO("Center x %f, center y %f, center_z %f", center_x, center_y, center_z);
        // Compute the dimensions of the bounding box
        float width = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;
        float depth = max_pt.z - min_pt.z;

        // Set the pose of the bounding box
        pose->position.x = center_x;
        pose->position.y = center_y;
        pose->position.z = center_z;
        pose->orientation.x = 0;
        pose->orientation.y = 0;
        pose->orientation.z = 0;
        pose->orientation.w = 1;

        // Set the dimensions of the bounding box
        dimensions->x = width;
        dimensions->y = height;
        dimensions->z = depth;
  }
}
