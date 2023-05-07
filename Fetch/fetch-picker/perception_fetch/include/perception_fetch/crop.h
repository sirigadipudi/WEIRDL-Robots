#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception_fetch {
class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

  private:
  ros::Publisher pub_;
};
}  // namespace perception
