#include <cmath>
#include <chrono>
#include <chrono>
#include <vector>

#include <glm/vec4.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

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

    /*********** MEMBER FUNCTIONS ***********/
    
    // Callbacks for subscriptions
    void imageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void poseSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Methods for removing clipped points
    int countClipPoints(const cv_bridge::CvImagePtr imagePtr);
    void removeClipPoints(int count);

    // Generates point cloud
    void genPointCloud(const cv_bridge::CvImagePtr imagePtr);

    // Updates values used for point cloud generation if necessary
    void updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg);

    // Transforms points from camera space to world space
    void convertToWorldFramePoint(unsigned int index);

    /*********** MEMBER VARIABLES ***********/

    // ROS Subscribers 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImage;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subPoseStamped;

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

    // Topic Messages
    sensor_msgs::msg::PointCloud2 pointCloudMsg;

    // Camera translation and orientation
    glm::vec4 cameraPosition;
    glm::quat cameraOrientation;

    // Pixel size of depth image
    unsigned int imageWidth;
    unsigned int imageHeight;

    // Clipping planes of the image
    float nearPlane;
    float farPlane;

    // Stores the FOV of simulated camera
    float horFov;
    float vertFov;

    // Stores the calculated UV coordinates of the pixels of the depth image
    std::vector<float> u;
    std::vector<float> v;

    // Stores the unstructured point cloud
    std::vector<glm::vec4> data;
};