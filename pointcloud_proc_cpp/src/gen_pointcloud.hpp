#include <cmath>
#include <chrono>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "pointcloud_interfaces/srv/camera_params.hpp"

struct PointXYZ {
    float x;
    float y;
    float z;

    float padding;
};

class GenPointCloudNode : public rclcpp::Node {
  public:
    GenPointCloudNode(float hfov, float vfov, float near, float far);
  private:
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void gen_pointcloud(const cv_bridge::CvImagePtr image_ptr);
    void updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg);
    void remove_boundary_values();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    sensor_msgs::msg::PointCloud2 pointcloudMsg;

    unsigned int imageWidth;
    unsigned int imageHeight;

    float nearPlane;
    float farPlane;

    const float PI = 3.141592f;
    const float toRadians = PI / 180.0f;

    float hFov;
    float vFov;

    const unsigned int AXIS_COUNT;

    std::vector<float> u;
    std::vector<float> v;

    std::vector<PointXYZ> flatData;
};