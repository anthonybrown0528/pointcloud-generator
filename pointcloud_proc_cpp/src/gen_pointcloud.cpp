#include "gen_pointcloud.hpp"

using namespace std::chrono_literals;

/**
 * Construct a new Gen Point Cloud Node::Gen Point Cloud Node object,
 * creates publishers and subscribers, and defines structure of ROS PointCloud2 msg
 * 
 * @param hfov horizontal field of view of the camera
 * @param vfov vertical field of view of the camera
 * @param near near clipping plane of the image
 * @param far far clipping plane of the image
 */
GenPointCloudNode::GenPointCloudNode(float hfov, float vfov, float near, float far) : Node("gen_pointcloud"), 
                                                                                      AXIS_COUNT(3),
                                                                                      UCHAR_TO_FLOAT_SCALE(1.0f / 255) {

  //Subscribe to depth image topic and camera state topic
  subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::imageSubCallback, this, std::placeholders::_1));
  subPoseStamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_state", 2, std::bind(&GenPointCloudNode::poseSubCallback, this, std::placeholders::_1));

  //Create publisher for point cloud
  publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 2);

  //Clipping planes of simulated camera
  nearPlane = near;
  farPlane = far;

  //FOV of simulated camera
  hFov = glm::radians(hfov);
  vFov = glm::radians(vfov);

  pointCloudMsg.is_bigendian = false;
  pointCloudMsg.is_dense = false;

  sensor_msgs::msg::PointField newField;

  //Each field represents a 32-bit floating-point value
  newField.datatype = newField.FLOAT32;
  newField.count = 1;

  const std::string fieldNames[] = {"x", "y", "z"};

  for(unsigned int i = 0; i < AXIS_COUNT; i++) {
    newField.name = fieldNames[i];
    newField.offset = i * sizeof(float);

    pointCloudMsg.fields.push_back(newField);
  }

  pointCloudMsg.point_step = sizeof(glm::vec4);
}

/**
 * Handles subscription to camera state topic and 
 * updates camera translation and orientation at each call
 * 
 * @param msg pointer to PoseStamped message
 */
void GenPointCloudNode::poseSubCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  cameraPosition.x = msg->pose.position.x;
  cameraPosition.y = msg->pose.position.y;
  cameraPosition.z = msg->pose.position.z;

  cameraOrientation.x = msg->pose.orientation.x;
  cameraOrientation.y = msg->pose.orientation.y;
  cameraOrientation.z = msg->pose.orientation.z;
  cameraOrientation.w = msg->pose.orientation.w;
}

/**
 * Handles subscription to depth image topic and 
 * generates an unstructured vector containing points in camera space
 * 
 * @param msg pointer to Image message
 */
void GenPointCloudNode::imageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  updateFromMsg(msg);

  //Resets the contents of the vector
  data = std::vector<glm::vec4>(imageWidth * imageHeight);

  //Converts ROS Image msg to cv::Mat and returns its pointer
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  //Converts image from 8-bit unsigned to 32-bit floating-point values
  image_ptr->image.convertTo(image_ptr->image, CV_32F, UCHAR_TO_FLOAT_SCALE);

  //Computes XYZ coordinates of each point and stores it in the flatData variable
  genPointCloud(image_ptr);
  removeClipPoints();

  //Converts from 32-bit floating-point to 8-bit unsigned integer
  uint8_t* bytes = reinterpret_cast<uint8_t*>(&(data[0]));
  std::vector<uint8_t> pointCloudData(bytes, bytes + data.size() * sizeof(glm::vec4));

  pointCloudMsg.height = 1;
  pointCloudMsg.width = data.size();

  pointCloudMsg.data = pointCloudData;
  pointCloudMsg.row_step = pointCloudData.size();

  auto time_end = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "Time Performance: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));

  RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
  publisher->publish(pointCloudMsg);
}

/**
 * Updates parameters of the received image such as pixel size of image,
 * frame id of camera, and uv coordinates (if size changes)
 * 
 * @param msg pointer to Image message
 */
void GenPointCloudNode::updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
  imageWidth = msg->width;
  imageHeight = msg->height;

  pointCloudMsg.height = imageHeight;
  pointCloudMsg.width = imageWidth;

  pointCloudMsg.header.frame_id = msg->header.frame_id;
  pointCloudMsg.header.stamp = this->get_clock()->now();

  //Define UV coordinates that map to depth image
  if(u.size() != this->imageWidth || v.size() != imageHeight) {
    u = std::vector<float>(imageWidth);
    v = std::vector<float>(imageHeight);

    for(unsigned int i = 0; i < u.size(); i++) {
    u[i] = i / (float)(imageWidth - 1);
    u[i] = u[i] - 0.5f;
    u[i] *= 2.0f;
    }

    for(unsigned int i = 0; i < v.size(); i++) {
      v[i] = i / (float)(imageHeight - 1);
      v[i] = 0.5f - v[i];
      v[i] *= 2.0f;
    }
  }
}

/**
 * Generates point cloud based on depth image
 * 
 * @param image_ptr pointer to depth image in the form of 32-bit floating-point values in range 0.0 - 1.0
 */
void GenPointCloudNode::genPointCloud(const cv_bridge::CvImagePtr image_ptr) {
  const float TAN_HALF_H_FOV = tan(hFov / 2.0f);
  const float TAN_HALF_V_FOV = tan(vFov / 2.0f);

  for(unsigned int i = 0; i < imageHeight; i++) {

    const float Z_TRANSFORM = v[i] * TAN_HALF_V_FOV;
    const float LOOP_OFFSET = i * imageWidth;

    for(unsigned int j = 0; j < imageWidth; j++) {

      //Scales depth image from 0.0 - 1.0 to nearPlane - farPlane
      image_ptr->image.at<float>(i, j) = nearPlane / (image_ptr->image.at<float>(i, j) * nearPlane / farPlane - image_ptr->image.at<float>(i, j) + 1);
    
      //Skips calculations for values at the clipping planes
      if(abs(image_ptr->image.at<float>(i, j) - nearPlane) < 0.001 || abs(image_ptr->image.at<float>(i, j) - farPlane) < 0.001) {
        continue;
      }

      data[LOOP_OFFSET + j].y = image_ptr->image.at<float>(i, j) * u[j] * TAN_HALF_H_FOV;
      data[LOOP_OFFSET + j].z = image_ptr->image.at<float>(i, j) * Z_TRANSFORM;
      data[LOOP_OFFSET + j].x = image_ptr->image.at<float>(i, j);

      // convertToWorldFramePoint(LOOP_OFFSET + j);
    }
  }
}

void GenPointCloudNode::removeClipPoints() {
  std::vector<glm::vec4> filteredPoints;
  glm::vec4 nullPoint(0.0f, 0.0f, 0.0f, 0.0f);

  for(unsigned int i = 0; i < data.size(); i++) {
    if(data[i] == nullPoint) {
      continue;
    }
    
    filteredPoints.push_back(data[i]);
  }

  data = filteredPoints;
}

/**
 * Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void GenPointCloudNode::convertToWorldFramePoint(unsigned int index) {
    data[index] = cameraOrientation * data[index];
    data[index] += cameraPosition;
}