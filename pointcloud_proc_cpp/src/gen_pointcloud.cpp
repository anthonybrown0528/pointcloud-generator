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
  subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
  subPoseStamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera_state", 2, std::bind(&GenPointCloudNode::subscriber_camera_state_callback, this, std::placeholders::_1));

  //Create publisher for point cloud
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

  //Each field represents a 32-bit floating-point value
  newField.datatype = newField.FLOAT32;
  newField.count = 1;

  std::string fieldNames[AXIS_COUNT] = {"x", "y", "z"};

  for(unsigned int i = 0; i < AXIS_COUNT; i++) {
    newField.name = fieldNames[i];
    newField.offset = i * sizeof(float);

    pointcloudMsg.fields.push_back(newField);
  }

  pointcloudMsg.point_step = (pointcloudMsg.fields.size() + 1) * sizeof(float);
}

/**
 * Handles subscription to camera state topic and 
 * updates camera translation and orientation at each call
 * 
 * @param msg pointer to PoseStamped message
 */
void GenPointCloudNode::subscriber_camera_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  updatePoseStamped(msg);

  camPoseRotationConjugate.x = -this->camPose.pose.orientation.x;
  camPoseRotationConjugate.y = -this->camPose.pose.orientation.y;
  camPoseRotationConjugate.z = -this->camPose.pose.orientation.z;
  camPoseRotationConjugate.w =  this->camPose.pose.orientation.w;
}

/**
 * Handles subscription to depth image topic and 
 * generates an unstructured vector containing points in camera space
 * 
 * @param msg pointer to Image message
 */
void GenPointCloudNode::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  auto time_start = std::chrono::steady_clock::now();

  updateFromMsg(msg);

  //Resets the contents of the vector
  flatData = std::vector<glm::vec4>(imageWidth * imageHeight);

  //Converts ROS Image msg to cv::Mat and returns its pointer
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  //Converts image from 8-bit unsigned to 32-bit floating-point values
  image_ptr->image.convertTo(image_ptr->image, CV_32F, UCHAR_TO_FLOAT_SCALE);

  //Computes XYZ coordinates of each point and stores it in the flatData variable
  gen_pointcloud(image_ptr);
  remove_clip_points();

  //Converts from 32-bit floating-point to 8-bit unsigned integer
  uint8_t* data = reinterpret_cast<uint8_t*>(&(flatData[0]));
  std::vector<uint8_t> dataVector(data, data + flatData.size() * sizeof(glm::vec4));

  pointcloudMsg.height = 1;
  pointcloudMsg.width = flatData.size();

  pointcloudMsg.data = dataVector;
  pointcloudMsg.row_step = dataVector.size();

  auto time_end = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count() / 1000.0f));

  RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
  publisher->publish(pointcloudMsg);
}

/**
 * Updates camera position and orientation
 * 
 * @param msg pointer to PoseStamped message
 */
void GenPointCloudNode::updatePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

  //Stores rotation and translation transformations of simulated camera
  this->camPose = *msg;
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

  pointcloudMsg.height = imageHeight;
  pointcloudMsg.width = imageWidth;

  pointcloudMsg.header.frame_id = msg->header.frame_id;
  pointcloudMsg.header.stamp = this->get_clock()->now();

  //Define UV coordinates that map to depth image
  if(this->u.size() != this->imageWidth || this->v.size() != this->imageHeight) {
    this->u = std::vector<float>(this->imageWidth);
    this->v = std::vector<float>(this->imageHeight);

    for(unsigned int i = 0; i < this->u.size(); i++) {
    this->u[i] = i / (float)(this->imageWidth - 1);
    this->u[i] = u[i] - 0.5f;
    this->u[i] *= 2.0f;
    }

    for(unsigned int i = 0; i < this->v.size(); i++) {
      this->v[i] = i / (float)(this->imageHeight - 1);
      this->v[i] = 0.5f - v[i];
      this->v[i] *= 2.0f;
    }
  }
}

/**
 * Generates point cloud based on depth image
 * 
 * @param image_ptr pointer to depth image in the form of 32-bit floating-point values in range 0.0 - 1.0
 */
void GenPointCloudNode::gen_pointcloud(const cv_bridge::CvImagePtr image_ptr) {
  const float TAN_HALF_H_FOV = tan(this->hFov / 2.0f);
  const float TAN_HALF_V_FOV = tan(this->vFov / 2.0f);

  for(unsigned int i = 0; i < this->imageHeight; i++) {

    const float Z_TRANSFORM = this->v[i] * TAN_HALF_V_FOV;
    const float LOOP_OFFSET = i * this->imageWidth;

    for(unsigned int j = 0; j < this->imageWidth; j++) {

      //Scales depth image from 0.0 - 1.0 to nearPlane - farPlane
      image_ptr->image.at<float>(i, j) = this->nearPlane / (image_ptr->image.at<float>(i, j) * this->nearPlane / this->farPlane - image_ptr->image.at<float>(i, j) + 1);
    
      //Skips calculations for values at the clipping planes
      if(abs(image_ptr->image.at<float>(i, j) - this->nearPlane) < 0.001 || abs(image_ptr->image.at<float>(i, j) - this->farPlane) < 0.001) {
        continue;
      }

      this->flatData[LOOP_OFFSET + j].y = image_ptr->image.at<float>(i, j) * this->u[j] * TAN_HALF_H_FOV;
      this->flatData[LOOP_OFFSET + j].z = image_ptr->image.at<float>(i, j) * Z_TRANSFORM;
      this->flatData[LOOP_OFFSET + j].x = image_ptr->image.at<float>(i, j);

      // convertToWorldFramePoint(LOOP_OFFSET + j);
    }
  }
}

/**
 * Performs hamilton product calculations
 * 
 * @param a left quaternion
 * @param b right quaternion
 * @return geometry_msgs::msg::Quaternion hamilton product of a and b
 */
geometry_msgs::msg::Quaternion GenPointCloudNode::calc_hamilton_product(geometry_msgs::msg::Quaternion a, geometry_msgs::msg::Quaternion b) {
  geometry_msgs::msg::Quaternion newQuaternion;

  newQuaternion.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  newQuaternion.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  newQuaternion.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  newQuaternion.w = a.w * b.w - a.x * b.x - a.y * b.y + a.z * b.z;

  return newQuaternion;
}

void GenPointCloudNode::remove_clip_points() {
  std::vector<glm::vec4> filteredPoints;
  glm::vec4 nullPoint(0.0f, 0.0f, 0.0f, 0.0f);

  for(unsigned int i = 0; i < flatData.size(); i++) {
    if(flatData[i] == nullPoint) {
      continue;
    }

    filteredPoints.push_back(flatData[i]);
  }

  flatData = filteredPoints;
}

/**
 * Rotates a point using the left hamilton product of camera orientation and the right hamilton product of the orientation's conjugate
 * 
 * @param quaternion orientation of camera
 * @param point point to be rotated
 * @param conjugateQuaternion conjugate of the orientation of the camera
 */
void GenPointCloudNode::rotate_point(geometry_msgs::msg::Quaternion quaternion, glm::vec4 &point, geometry_msgs::msg::Quaternion conjugateQuaternion) {

  //Converts point to pure quaternion to perform hamilton product
  geometry_msgs::msg::Quaternion pointQuaternion;

  pointQuaternion.x = point.x;
  pointQuaternion.y = point.y;
  pointQuaternion.z = point.z;
  pointQuaternion.w = 0.0f;

  geometry_msgs::msg::Quaternion partialRotationCalculation = calc_hamilton_product(quaternion, pointQuaternion);
  geometry_msgs::msg::Quaternion fullRotationCalculation = calc_hamilton_product(partialRotationCalculation, conjugateQuaternion);

  //Convert pure quaternion to 3-D vector
  point.x = fullRotationCalculation.x;
  point.y = fullRotationCalculation.y;
  point.z = fullRotationCalculation.z;
}

/**
 * Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void GenPointCloudNode::convertToWorldFramePoint(unsigned int index) {

    //Perform rotation transformation
    rotate_point(this->camPose.pose.orientation, this->flatData[index], camPoseRotationConjugate);

    //Perform translation transformation
    this->flatData[index].x = this->flatData[index].x + this->camPose.pose.position.x;
    this->flatData[index].y = this->flatData[index].y + this->camPose.pose.position.y;
    this->flatData[index].z = this->flatData[index].z + this->camPose.pose.position.z;
}