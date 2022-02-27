#include "gen_pointcloud.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  float hFov;
  float vFov;

  float nearPlane;
  float farPlane;

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("camera_params_client");
  rclcpp::Client<pointcloud_interfaces::srv::CameraParams>::SharedPtr client =
    node->create_client<pointcloud_interfaces::srv::CameraParams>("camera_params");

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