#include <cmath>
#include <chrono>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "pointcloud_interfaces/srv/camera_params.hpp"
#include "pointcloud_interfaces/msg/camera_state.hpp"

struct PointXYZ {
    float x;
    float y;
    float z;

    float padding;

    PointXYZ operator+(PointXYZ b) {
      PointXYZ sum;

      sum.x = this->x + b.x;
      sum.y = this->y + b.y;
      sum.z = this->z + b.z;

      return sum;
    }

    PointXYZ operator+(std::array<float, 3> b) {
      PointXYZ sum;

      sum.x = this->x + b[0];
      sum.y = this->y + b[1];
      sum.z = this->z + b[2];

      return sum;
    }

    PointXYZ operator-(PointXYZ b) {
      PointXYZ difference;

      difference.x = this->x - b.x;
      difference.y = this->y - b.y;
      difference.z = this->z - b.z;

      return difference;
    }

    PointXYZ operator-(std::array<float, 3> b) {
      PointXYZ difference;

      difference.x = this->x - b[0];
      difference.y = this->y - b[1];
      difference.z = this->z - b[2];

      return difference;
    }
};

class GenPointCloudNode : public rclcpp::Node {
  public:
    GenPointCloudNode(float hfov, float vfov, float near, float far);
  private:
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void subscriber_camera_state_callback(const pointcloud_interfaces::msg::CameraState::SharedPtr msg);

    void gen_pointcloud(const cv_bridge::CvImagePtr image_ptr);
    void updateCameraState(const pointcloud_interfaces::msg::CameraState::SharedPtr msg);
    void updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg);
    void remove_boundary_values();
    std::array<float, 4> calc_hamilton_product(std::array<float, 4> a, std::array<float, 4> b);
    void rotate_point(std::array<float, 4> quaternion, PointXYZ &point, std::array<float, 4> inverseQuaternion);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    rclcpp::Subscription<pointcloud_interfaces::msg::CameraState>::SharedPtr subCameraState;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    sensor_msgs::msg::PointCloud2 pointcloudMsg;
    pointcloud_interfaces::msg::CameraState camState;

    unsigned int imageWidth;
    unsigned int imageHeight;

    float nearPlane;
    float farPlane;

    const float PI = 3.141592f;
    const float toRadians = PI / 180.0f;

    float hFov;
    float vFov;

    const unsigned int AXIS_COUNT;
    const float SCALE_FACTOR;

    std::vector<float> u;
    std::vector<float> v;

    std::vector<PointXYZ> flatData;
};