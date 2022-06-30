#include "gen_pointcloud.hpp"

using namespace std::chrono_literals;

/**
 * @brief Construct a new Gen Point Cloud Node::Gen Point Cloud Node object,
 *        creates publishers and subscribers, and defines structure of ROS PointCloud2 msg
 * 
 * @param horFov horizontal field of view of the camera
 * @param vertFov vertical field of view of the camera
 * @param nearPlane near clipping plane of the image
 * @param farPlane far clipping plane of the image
 */
GenPointCloudNode::GenPointCloudNode(float horFov, float vertFov, float nearPlane, float farPlane) : Node("gen_pointcloud")  {

  const int AXIS_COUNT = 3;
  const std::string fieldNames[] = {"x", "y", "z"};

  sensor_msgs::msg::PointField newField;

  //Clipping planes of simulated camera
  this->nearPlane = nearPlane;
  this->farPlane = farPlane;

  //FOV of simulated camera
  this->horFov = glm::radians(horFov);
  this->vertFov = glm::radians(vertFov);

  //Create publisher for point cloud
  publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 2);

  //Subscribe to depth image topic and camera state topic
  subImage = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::imageSubCallback, this, std::placeholders::_1));
  subPoseStamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_state", 2, std::bind(&GenPointCloudNode::poseSubCallback, this, std::placeholders::_1));

  pointCloudMsg.is_bigendian = false;
  pointCloudMsg.is_dense = false;

  //Each field represents a 32-bit floating-point value
  newField.datatype = newField.FLOAT32;
  newField.count = 1;

  // Specifies the structure of the PointCloud2 message
  for(unsigned int i = 0; i < AXIS_COUNT; i++) {
    newField.name = fieldNames[i];
    newField.offset = i * sizeof(float);

    pointCloudMsg.fields.push_back(newField);
  }

  // Number of bytes that correspond to a single point
  pointCloudMsg.point_step = sizeof(glm::vec4);
}

/**
 * @brief Handles subscription to camera state topic and 
 *        updates camera translation and orientation at each call
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
 * @brief Handles subscription to depth image topic and 
 *        generates an unstructured vector containing points in camera space
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

  int count = countClipPoints(image_ptr);

  //Computes XYZ coordinates of each point and stores it in the flatData variable
  genPointCloud(image_ptr);
  removeClipPoints(count);

  //Converts from 32-bit floating-point to 8-bit unsigned integer
  uint8_t* bytes = reinterpret_cast<uint8_t*>(&(data[0]));
  std::vector<uint8_t> pointCloudData(bytes, bytes + data.size() * sizeof(glm::vec4));

  pointCloudMsg.height = 1;
  pointCloudMsg.width = data.size();

  pointCloudMsg.data = pointCloudData;
  pointCloudMsg.row_step = pointCloudData.size();

  auto time_end = std::chrono::steady_clock::now();
  // RCLCPP_INFO(this->get_logger(), "Time Performance: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));

  publisher->publish(pointCloudMsg);
}

/**
 * @brief Updates parameters of the received image such as pixel size of image,
 *        frame id of camera, and uv coordinates (if size changes)
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
 * @brief Generates point cloud based on depth image
 * 
 * @param image_ptr pointer to depth image in the form of 32-bit floating-point values in range 0.0 - 1.0
 */
void GenPointCloudNode::genPointCloud(const cv_bridge::CvImagePtr image_ptr) {
  const float TAN_HALF_H_FOV = tan(horFov / 2.0f);
  const float TAN_HALF_V_FOV = tan(vertFov / 2.0f);

  const float DEPTH_RANGE = farPlane - nearPlane;

  float scaled_depth;
  float current_z_transform;
  int current_loop_offset;
  int current_index;
  const int SCALE = 255;

  for(unsigned int i = 0; i < imageHeight; i++) {

    current_z_transform = v[i] * TAN_HALF_V_FOV;
    current_loop_offset = i * imageWidth;

    for(unsigned int j = 0; j < imageWidth; j++) {

      //Skips calculations for values at the clipping planes
      if(image_ptr->image.at<uchar>(i, j) == SCALE || image_ptr->image.at<uchar>(i, j) == 0) {
        continue;
      }

      current_index = current_loop_offset + j;

      //Scales depth image from 0 - 255 to nearPlane - farPlane
      scaled_depth = farPlane / (farPlane - DEPTH_RANGE / SCALE * image_ptr->image.at<uchar>(i, j));

      data[current_index].y = scaled_depth * u[j] * TAN_HALF_H_FOV;
      data[current_index].z = scaled_depth * current_z_transform;
      data[current_index].x = scaled_depth;

      // convertToWorldFramePoint(current_index);
    }
  }
}

/**
 * @brief Counts the number of points that correspond to points 
 *        farther than the far plane or nearer than the near plane
 * 
 * @param imagePtr pointer to a CvImage containing a depth image
 * @return number of points matching the given description
 */
int GenPointCloudNode::countClipPoints(const cv_bridge::CvImagePtr imagePtr) {
  const int SCALE = 255;
  int count = 0;

  for(unsigned int i = 0; i < imageHeight; i++) {
    for(unsigned int j = 0; j < imageWidth; j++) {
      if(imagePtr->image.at<uchar>(i, j) == SCALE || imagePtr->image.at<uchar>(i, j) == 0) {
        count++;
      }
    }
  }

  return count;
}

/**
 * @brief Removes the points outside the clip space of simulated camera
 * 
 * @param count number of points outside the clip space
 */
void GenPointCloudNode::removeClipPoints(int count) {
  std::vector<glm::vec4> filteredPoints(data.size() - count);
  glm::vec4 nullPoint(0.0f, 0.0f, 0.0f, 0.0f);

  int next = 0;
  for(unsigned int i = 0; i < data.size(); i++) {
    if(data[i] == nullPoint) {
      continue;
    }
    
    filteredPoints[next++] = data[i];
  }

  data = filteredPoints;
}

/**
 * @brief Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void GenPointCloudNode::convertToWorldFramePoint(unsigned int index) {
    data[index] = cameraOrientation * data[index];
    data[index] += cameraPosition;
}