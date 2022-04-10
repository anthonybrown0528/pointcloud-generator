#include <cmath>
#include <chrono>
#include <chrono>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "pointcloud_interfaces/srv/camera_params.hpp"

/**
 * Structure to hold point position and define valid operations
 */
struct PointXYZ {
    float x;
    float y;
    float z;

    PointXYZ() {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }

    PointXYZ(float x, float y, float z) {
      this->x = x;
      this->y = y;
      this->z = z;
    }

    // Adds 4 bytes of padding to turn the size of the structure
    // into a power of 2
    float padding;

    /**
     * Defines the addition operator for PointXYZ
     */
    PointXYZ operator+(PointXYZ b) {
      PointXYZ sum;

      sum.x = this->x + b.x;
      sum.y = this->y + b.y;
      sum.z = this->z + b.z;

      return sum;
    }

    /**
     * Defines the addition operator for PointXYZ
     */
    PointXYZ operator+(std::array<float, 3> b) {
      PointXYZ sum;

      sum.x = this->x + b[0];
      sum.y = this->y + b[1];
      sum.z = this->z + b[2];

      return sum;
    }

    /**
     * Defines the subtraction operator for PointXYZ
     */
    PointXYZ operator-(PointXYZ b) {
      PointXYZ difference;

      difference.x = this->x - b.x;
      difference.y = this->y - b.y;
      difference.z = this->z - b.z;

      return difference;
    }

    /**
     * Defines the subtraction operator for PointXYZ
     */
    PointXYZ operator-(std::array<float, 3> b) {
      PointXYZ difference;

      difference.x = this->x - b[0];
      difference.y = this->y - b[1];
      difference.z = this->z - b[2];

      return difference;
    }

    bool operator==(PointXYZ b) {
      if(abs(this->x - b.x) < 0.001) {
        if(abs(this->y - b.y) < 0.001) {
          if(abs(this->z - b.z) < 0.001) {
            return true;
          }
        }
      }

      return false;
    }

    bool operator==(std::array<float, 3> b) {
      if(abs(this->x - b[0]) < 0.001) {
        if(abs(this->y - b[1]) < 0.001) {
          if(abs(this->z - b[2]) < 0.001) {
            return true;
          }
        }
      }

      return false;
    }
};

/**
 * Subscribes to depth image and camera state topic to generate
 * and publish a point cloud in the form of a PointCloud2 ROS message
 */
class GenPointCloudNode : public rclcpp::Node {
  public:

    // Creates publishers and subscribers
    // Defines structure of ROS PointCloud2 msg
    GenPointCloudNode(float hfov, float vfov, float near, float far);

  private:
    
    // Defines callbacks for subscriptions
    void subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void subscriber_camera_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void remove_clip_points();

    // Generates point cloud
    void gen_pointcloud(const cv_bridge::CvImagePtr image_ptr);

    // Updates the camera's transformation in world space
    void updatePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Updates values used for point cloud generation if necessary
    void updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg);

    // Performs hamilton product
    geometry_msgs::msg::Quaternion calc_hamilton_product(geometry_msgs::msg::Quaternion a, geometry_msgs::msg::Quaternion b);

    // Rotates point using a left hamilton product of camera rotation and right hamilton product of conjugate rotation
    void rotate_point(geometry_msgs::msg::Quaternion quaternion, PointXYZ &point, geometry_msgs::msg::Quaternion inverseQuaternion);

    // Transforms points from camera space to world space
    void convertToWorldFramePoint(unsigned int index);

    // ROS Subscribers 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subPoseStamped;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    // Published messages
    sensor_msgs::msg::PointCloud2 pointcloudMsg;

    // Subscribed messages
    geometry_msgs::msg::PoseStamped camPose;

    // Pixel size of depth image
    unsigned int imageWidth;
    unsigned int imageHeight;

    // Clipping planes of the image
    float nearPlane;
    float farPlane;

    // Used to convert from degrees to radians
    const float PI = 3.141592f;
    const float toRadians = PI / 180.0f;

    // Stores the FOV of simulated camera
    float hFov;
    float vFov;

    // Number of axes for a single point e.g. XYZ -> 3
    const unsigned int AXIS_COUNT;

    const float UCHAR_TO_FLOAT_SCALE;

    // Stores the calculated conjugate quaternion for the camera's orientation
    geometry_msgs::msg::Quaternion camPoseRotationConjugate;

    // Stores the calculated UV coordinates of the pixels of the depth image
    std::vector<float> u;
    std::vector<float> v;

    // Stores the unstructured point cloud
    std::vector<PointXYZ> flatData;
};