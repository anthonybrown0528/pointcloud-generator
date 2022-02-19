#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"

#include "pointcloud_interfaces/msg/image_cam_info.hpp"

class ReadImageNode : public rclcpp::Node {
    public:
        ReadImageNode() : Node("read_depth_image") {
            subscriber = this->create_subscription<pointcloud_interfaces::msg::ImageCamInfo>("image_cam_info", 2, std::bind(&ReadImageNode::sub_callback, this, std::placeholders::_1));
        }

    private:
        void sub_callback(pointcloud_interfaces::msg::ImageCamInfo::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received message");

            cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg->image);
            cv::imshow("Image Window", image_ptr->image);
            cv::waitKey(33);

            RCLCPP_INFO(this->get_logger(), std::to_string(image_ptr->image.at<uchar>(50, 50)));
        }

        rclcpp::Subscription<pointcloud_interfaces::msg::ImageCamInfo>::SharedPtr subscriber; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadImageNode>());

    rclcpp::shutdown();
    return 0;
}