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
GenPointCloudNode::GenPointCloudNode(float hfov, float vfov, float near, float far) : Node("gen_pointcloud"), AXIS_COUNT(3) {

  //Subscribe to depth image topic and camera state topic
  subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
  subCameraState = this->create_subscription<pointcloud_interfaces::msg::CameraState>("camera_state", 2, std::bind(&GenPointCloudNode::subscriber_camera_state_callback, this, std::placeholders::_1));

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
 * @param msg pointer to CameraState message
 */
void GenPointCloudNode::subscriber_camera_state_callback(const pointcloud_interfaces::msg::CameraState::SharedPtr msg) {
  updateCameraState(msg);

  camStateRotationConjugate[0] = -this->camState.rotation[0];
  camStateRotationConjugate[1] = -this->camState.rotation[1];
  camStateRotationConjugate[2] = -this->camState.rotation[2];
  camStateRotationConjugate[3] =  this->camState.rotation[3];
}

/**
 * Handles subscription to depth image topic and 
 * generates an unstructured vector containing points in camera space
 * 
 * @param msg pointer to Image message
 */
void GenPointCloudNode::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  updateFromMsg(msg);

  //Resets the contents of the vector
  flatData = std::vector<PointXYZ>(imageWidth * imageHeight);

  //Converts ROS Image msg to cv::Mat and returns its pointer
  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  //Converts image from 8-bit unsigned to 32-bit floating-point values
  image_ptr->image.convertTo(image_ptr->image, CV_32F, 1.0f / 255);

  //Computes XYZ coordinates of each point and stores it in the flatData variable
  gen_pointcloud(image_ptr);

  //Converts from 32-bit floating-point to 8-bit unsigned integer
  uint8_t* data = reinterpret_cast<uint8_t*>(&(flatData[0]));
  std::vector<uint8_t> dataVector(data, data + flatData.size() * sizeof(PointXYZ));

  pointcloudMsg.data = dataVector;
  pointcloudMsg.row_step = dataVector.size();

  RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
  publisher->publish(pointcloudMsg);
}

/**
 * Updates camera position and orientation
 * 
 * @param msg pointer to CameraState message
 */
void GenPointCloudNode::updateCameraState(const pointcloud_interfaces::msg::CameraState::SharedPtr msg) {

  //Stores rotation and translation transformations of simulated camera
  this->camState = *msg;
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
  //Scales depth image from 0.0 - 1.0 to nearPlane - farPlane
  for(unsigned int i = 0; i < this->imageHeight; i++) {
    for(unsigned int j = 0; j < this->imageWidth; j++) {
      image_ptr->image.at<float>(i, j) = this->nearPlane / (image_ptr->image.at<float>(i, j) * this->nearPlane / this->farPlane - image_ptr->image.at<float>(i, j) + 1);
    }
  }

  for(unsigned int i = 0; i < this->imageHeight; i++) {
    for(unsigned int j = 0; j < this->imageWidth; j++) {

      //Skips calculations for values at the clipping planes
      if(abs(image_ptr->image.at<float>(i, j) - this->nearPlane) < 0.001 || abs(image_ptr->image.at<float>(i, j) - this->farPlane) < 0.001) {
        continue;
      }

      this->flatData[i * this->imageWidth + j].y = image_ptr->image.at<float>(i, j) * this->u[j] * tan(this->hFov / 2.0f);
      this->flatData[i * this->imageWidth + j].z = image_ptr->image.at<float>(i, j) * this->v[i] * tan(this->vFov / 2.0f);
      this->flatData[i * this->imageWidth + j].x = image_ptr->image.at<float>(i, j);
    }
  }
}

/**
 * Performs hamilton product calculations
 * 
 * @param a left quaternion
 * @param b right quaternion
 * @return std::array<float, 4> hamilton product of a and b
 */
std::array<float, 4> GenPointCloudNode::calc_hamilton_product(std::array<float, 4> a, std::array<float, 4> b) {
  std::array<float, 4> newQuaternion;

  newQuaternion[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  newQuaternion[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  newQuaternion[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  newQuaternion[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] + a[2] * b[2];

  return newQuaternion;
}

/**
 * Rotates a point using the left hamilton product of camera orientation and the right hamilton product of the orientation's conjugate
 * 
 * @param quaternion orientation of camera
 * @param point point to be rotated
 * @param conjugateQuaternion conjugate of the orientation of the camera
 */
void GenPointCloudNode::rotate_point(std::array<float, 4> quaternion, PointXYZ &point, std::array<float, 4> conjugateQuaternion) {

  //Converts point to pure quaternion to perform hamilton product
  std::array<float, 4> pointQuaternion;

  pointQuaternion[0] = point.x;
  pointQuaternion[1] = point.y;
  pointQuaternion[2] = point.z;
  pointQuaternion[3] = 0.0f;

  std::array<float, 4> partialRotationCalculation = calc_hamilton_product(quaternion, pointQuaternion);
  std::array<float, 4> fullRotationCalculation = calc_hamilton_product(partialRotationCalculation, conjugateQuaternion);

  //Convert pure quaternion to 3-D vector
  point.x = fullRotationCalculation[0];
  point.y = fullRotationCalculation[1];
  point.z = fullRotationCalculation[2];
}

/**
 * Converts point from camera space to world space using camera orientation and translation
 * 
 * @param index index of the point in the flatData vector
 */
void GenPointCloudNode::convertToWorldFramePoint(unsigned int index) {

    //Perform rotation transformation
    rotate_point(this->camState.rotation, this->flatData[index], camStateRotationConjugate);

    //Perform translation transformation
    this->flatData[index] = this->flatData[index] + this->camState.translation;
}