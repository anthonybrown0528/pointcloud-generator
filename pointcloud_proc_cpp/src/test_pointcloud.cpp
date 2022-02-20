#include <chrono>
#include <cmath>
#include <vector>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using namespace std::chrono_literals;

class TestPointCloudNode : public rclcpp::Node {
    public:
        TestPointCloudNode() : Node("test_pointcloud") {
            this->publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("test_pointcloud", 10);
            this->timer = this->create_wall_timer(500ms, std::bind(&TestPointCloudNode::timer_callback, this));

            this->sideLength = 16;
            this->points = std::vector<float>(std::pow(this->sideLength, 3) * 3, 0.0f);

            for(int i = 0; i < this->sideLength; i++) {
                for(int j = 0; j < this->sideLength; j++) {
                    for(int k = 0; k < this->sideLength; k++) {
                        points[i * this->sideLength * this->sideLength * 3 + j * this->sideLength * 3 + k * 3 + 0] = i / (sideLength - 1.0f);
                        points[i * this->sideLength * this->sideLength * 3 + j * this->sideLength * 3 + k * 3 + 1] = j / (sideLength - 1.0f);
                        points[i * this->sideLength * this->sideLength * 3 + j * this->sideLength * 3 + k * 3 + 2] = k / (sideLength - 1.0f);
                    }
                }
            }
        }
    private:
        void timer_callback() {
            auto msg = sensor_msgs::msg::PointCloud2();
            auto tmp = sensor_msgs::msg::PointField();

            std::vector<uint8_t> pointsData = std::vector<uint8_t>(this->points.size() * 4);
            uint8_t* pointsBytes = reinterpret_cast<uint8_t*>(&points[0]);

            for(int i = 0; i < pointsData.size(); i++) {
                pointsData[i] = pointsBytes[i];
            }

            tmp.count = 1;
            tmp.datatype = tmp.FLOAT32;

            //Define header
            msg.header.frame_id = "odom";
            msg.header.stamp = this->get_clock()->now();

            //Define fields
            tmp.name = "x";
            tmp.offset = 0;

            msg.fields.push_back(tmp);

            tmp.name = "y";
            tmp.offset += 4;

            msg.fields.push_back(tmp);

            tmp.name = "z";
            tmp.offset += 4;

            msg.fields.push_back(tmp);

            msg.is_bigendian = true;
            msg.is_dense = false;

            msg.height = 1;
            msg.width = std::pow(this->sideLength, 3);

            msg.point_step = 12;
            msg.row_step = msg.point_step * msg.width;

            msg.data = pointsData;

            publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
        }

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;

        uint8_t sideLength;
        std::vector<float> points;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPointCloudNode>());

    rclcpp::shutdown();
    return 0;
}