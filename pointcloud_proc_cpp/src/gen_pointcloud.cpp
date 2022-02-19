#include <cmath>
#include <cstring>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "pointcloud_interfaces/msg/image_cam_info.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

class GenPointCloudNode : public rclcpp::Node {
  public:
    GenPointCloudNode() : Node("gen_pointcloud") {
      subscriber = this->create_subscription<pointcloud_interfaces::msg::ImageCamInfo>("image_cam_info", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
    }

  private:
    void subscriber_callback(const pointcloud_interfaces::msg::ImageCamInfo::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received message");

      const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg->image);
      std::vector<std::vector<std::vector<float>>> points = gen_pointcloud(image_ptr);
      RCLCPP_INFO(this->get_logger(), "Finished generating pointcloud");
      RCLCPP_INFO(this->get_logger(), std::to_string(points[0][0][2]));
    }

    std::vector<std::vector<std::vector<float>>> gen_pointcloud(const cv_bridge::CvImagePtr image_ptr) {
      const float PI = 3.141592f;
      const float toRadians = PI / 180.0f;

      const int WIDTH = 1280;
      const int HEIGHT = 720;

      const float hfov = 110.0f;
      const float vfov = 70.0f;

      float u[WIDTH];
      float v[HEIGHT];

      float img_float[HEIGHT][WIDTH];
      std::vector<std::vector<std::vector<float>>> points;

      for(unsigned int i = 0; i < HEIGHT; i++) {
        for(unsigned int j = 0; j < WIDTH; j++) {
          img_float[i][j] = image_ptr->image.at<uchar>(i, j) / 255.0f;
        }
      }

      for(unsigned int i = 0; i < sizeof(u) / sizeof(float); i++) {
        u[i] = i / (float)(WIDTH - 1);
        u[i] = 0.5 - u[i];
        u[i] *= -1;
      }

      for(unsigned int i = 0; i < sizeof(v) / sizeof(float); i++) {
        v[i] = i / (float)(HEIGHT - 1);
        v[i] = 0.5 - v[i];
        v[i] *= -1;
      }

      for(unsigned int i = 0; i < HEIGHT; i++) {
        std::vector<std::vector<float>> row;

        for(unsigned int j = 0; j < WIDTH; j++) {
          std::vector<float> point(3);

          point[0] = img_float[i][j] * u[j] * tan(hfov * toRadians);
          point[1] = img_float[i][j] * v[i] * tan(vfov * toRadians);
          point[2] = img_float[i][j];

          row.push_back(point);
        }

        points.push_back(row);
      }

      return points;
    }

    rclcpp::Subscription<pointcloud_interfaces::msg::ImageCamInfo>::SharedPtr subscriber;    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenPointCloudNode>());

  rclcpp::shutdown();
  return 0;
}