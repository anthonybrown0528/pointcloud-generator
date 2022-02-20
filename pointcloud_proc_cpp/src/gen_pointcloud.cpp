#include <cmath>
#include <chrono>
#include <cstring>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "pointcloud_interfaces/msg/image_cam_info.hpp"
#include "pointcloud_interfaces/srv/camera_params.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class GenPointCloudNode : public rclcpp::Node {
  public:
    GenPointCloudNode(float hfov, float vfov, float near, float far) : Node("gen_pointcloud"), AXIS_COUNT(3) {
      subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
      publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 2);

      //Clipping planes of simulated camera
      nearPlane = near;
      farPlane = far;

      //FOV of simulated camera
      hFov = hfov * toRadians;
      vFov = vfov * toRadians;

      pointcloudMsg.is_bigendian = false;
      pointcloudMsg.is_dense = false;

      sensor_msgs::msg::PointField newField;
      newField.count = 1;
      newField.datatype = newField.FLOAT32;

      //Structure of x-coordinate data
      newField.name = "x";
      newField.offset = 0;

      pointcloudMsg.fields.push_back(newField);

      //Structure of x-coordinate data
      newField.name = "y";
      newField.offset = newField.offset + sizeof(float);

      pointcloudMsg.fields.push_back(newField);

      //Structure of x-coordinate data
      newField.name = "z";
      newField.offset = newField.offset + sizeof(float);

      pointcloudMsg.fields.push_back(newField);

      pointcloudMsg.point_step = pointcloudMsg.fields.size() * sizeof(float);
    }

  private:
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
      this->imageWidth = msg->width;
      this->imageHeight = msg->height;

      pointcloudMsg.header.frame_id = msg->header.frame_id;
      pointcloudMsg.header.stamp = this->get_clock()->now();

      pointcloudMsg.height = 1;
      pointcloudMsg.width = imageWidth * imageHeight;

      pointcloudMsg.row_step = pointcloudMsg.point_step * pointcloudMsg.width;

      //Define UV coordinates that map to depth image
      if(this->u.size() != this->imageWidth || this->v.size() != this->imageHeight) {
        this->u = std::vector<float>(this->imageWidth);
        this->v = std::vector<float>(this->imageHeight);

        for(unsigned int i = 0; i < this->u.size(); i++) {
        this->u[i] = i / (float)(this->imageWidth - 1);
        this->u[i] = 0.5 - u[i];
        this->u[i] *= -1;
        }

        for(unsigned int i = 0; i < this->v.size(); i++) {
          this->v[i] = i / (float)(this->imageHeight - 1);
          this->v[i] = 0.5 - v[i];
          this->v[i] *= -1;
        }

        flatData = std::vector<float>(imageWidth * imageHeight * AXIS_COUNT);
      }

      const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);
      image_ptr->image.convertTo(image_ptr->image, CV_32F, 1.0f / 255);

      gen_pointcloud(image_ptr);

      uint8_t* data = reinterpret_cast<uint8_t*>(&(flatData[0]));
      std::vector<uint8_t> dataVector(data, data + flatData.size() * sizeof(float));

      pointcloudMsg.data = dataVector;

      RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
      publisher->publish(pointcloudMsg);
    }

    void gen_pointcloud(const cv_bridge::CvImagePtr image_ptr) {
      for(unsigned int i = 0; i < this->imageHeight; i++) {
        for(unsigned int j = 0; j < this->imageWidth; j++) {
          image_ptr->image.at<float>(i, j) = this->nearPlane / (image_ptr->image.at<float>(i, j) * this->nearPlane / this->farPlane - image_ptr->image.at<float>(i, j) + 1);
        }
      }

      for(unsigned int i = 0; i < this->imageHeight; i++) {
        std::vector<std::vector<float>> row(this->imageWidth);

        for(unsigned int j = 0; j < this->imageWidth; j++) {
          this->flatData[i * this->imageWidth * AXIS_COUNT + j * AXIS_COUNT + 0] = image_ptr->image.at<float>(i, j) * this->u[j] * tan(this->hFov / 2.0f);
          this->flatData[i * this->imageWidth * AXIS_COUNT + j * AXIS_COUNT + 1] = image_ptr->image.at<float>(i, j) * this->v[i] * tan(this->vFov / 2.0f);
          this->flatData[i * this->imageWidth * AXIS_COUNT + j * AXIS_COUNT + 2] = image_ptr->image.at<float>(i, j);
        }
      }
    }

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

    std::vector<float> flatData;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  float hFov;
  float vFov;

  float nearPlane;
  float farPlane;

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("camera_params_client");
  rclcpp::Client<pointcloud_interfaces::srv::CameraParams>::SharedPtr client =
    node->create_client<pointcloud_interfaces::srv::CameraParams>("camera/params");

  auto request = std::make_shared<pointcloud_interfaces::srv::CameraParams::Request>();
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    hFov = result.get()->hfov;
    vFov = result.get()->vfov;

    nearPlane = result.get()->near;
    farPlane = result.get()->far;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Retrieved simulated camera parameters");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::spin(std::make_shared<GenPointCloudNode>(hFov, vFov, nearPlane, farPlane));

  rclcpp::shutdown();
  return 0;
}