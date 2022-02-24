#include "gen_pointcloud.hpp"

using namespace std::chrono_literals;

GenPointCloudNode::GenPointCloudNode(float hfov, float vfov, float near, float far) : Node("gen_pointcloud"), AXIS_COUNT(3), SCALE_FACTOR(1.0f) {
  subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
  subCameraState = this->create_subscription<pointcloud_interfaces::msg::CameraState>("camera_state", 2, std::bind(&GenPointCloudNode::subscriber_camera_state_callback, this, std::placeholders::_1));

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

void GenPointCloudNode::subscriber_camera_state_callback(const pointcloud_interfaces::msg::CameraState::SharedPtr msg) {
  updateCameraState(msg);
}

void GenPointCloudNode::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  updateFromMsg(msg);

  flatData = std::vector<PointXYZ>(imageWidth * imageHeight);

  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);

  if(abs(this->SCALE_FACTOR - 1.0f) < 0.001f) {
    cv::Size origSize = image_ptr->image.size();
    origSize.width *= this->SCALE_FACTOR;
    origSize.height *= this->SCALE_FACTOR;

    cv::resize(image_ptr->image, image_ptr->image, origSize, cv::INTER_CUBIC);
  }


  image_ptr->image.convertTo(image_ptr->image, CV_32F, 1.0f / 255);

  gen_pointcloud(image_ptr);

  uint8_t* data = reinterpret_cast<uint8_t*>(&(flatData[0]));
  std::vector<uint8_t> dataVector(data, data + flatData.size() * sizeof(PointXYZ));

  pointcloudMsg.data = dataVector;
  pointcloudMsg.row_step = dataVector.size();

  RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
  publisher->publish(pointcloudMsg);
}

void GenPointCloudNode::updateCameraState(const pointcloud_interfaces::msg::CameraState::SharedPtr msg) {
  this->camState = *msg;
}

void GenPointCloudNode::updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
  this->imageWidth = msg->width * this->SCALE_FACTOR;
  this->imageHeight = msg->height * this->SCALE_FACTOR;

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

void GenPointCloudNode::gen_pointcloud(const cv_bridge::CvImagePtr image_ptr) {
  for(unsigned int i = 0; i < this->imageHeight; i++) {
    for(unsigned int j = 0; j < this->imageWidth; j++) {
      image_ptr->image.at<float>(i, j) = this->nearPlane / (image_ptr->image.at<float>(i, j) * this->nearPlane / this->farPlane - image_ptr->image.at<float>(i, j) + 1);
    }
  }

  for(unsigned int i = 0; i < this->imageHeight; i++) {
    for(unsigned int j = 0; j < this->imageWidth; j++) {
      if(abs(image_ptr->image.at<float>(i, j) - this->nearPlane) < 0.001 || abs(image_ptr->image.at<float>(i, j) - this->farPlane) < 0.001) {
        continue;
      }

      this->flatData[i * this->imageWidth + j].y = image_ptr->image.at<float>(i, j) * this->u[j] * tan(this->hFov / 2.0f);
      this->flatData[i * this->imageWidth + j].z = image_ptr->image.at<float>(i, j) * this->v[i] * tan(this->vFov / 2.0f);
      this->flatData[i * this->imageWidth + j].x = image_ptr->image.at<float>(i, j);

      //this->flatData[i * this->imageWidth + j] = this->flatData[i * this->imageWidth + j] - this->camState.translation;

      std::array<float, 4> conjugateRotation;

      conjugateRotation[0] = -this->camState.rotation[0];
      conjugateRotation[1] = -this->camState.rotation[1];
      conjugateRotation[2] = -this->camState.rotation[2];
      conjugateRotation[3] =  this->camState.rotation[3];

      rotate_point(conjugateRotation, this->flatData[i * this->imageWidth + j], this->camState.rotation);
    }
  }
}

std::array<float, 4> GenPointCloudNode::calc_hamilton_product(std::array<float, 4> a, std::array<float, 4> b) {
  std::array<float, 4> newQuaternion;

  newQuaternion[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  newQuaternion[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  newQuaternion[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  newQuaternion[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] + a[2] * b[2];

  return newQuaternion;
}

void GenPointCloudNode::rotate_point(std::array<float, 4> quaternion, PointXYZ &point, std::array<float, 4> conjugateQuaternion) {
  std::array<float, 4> pointQuaternion;

  pointQuaternion[0] = point.x;
  pointQuaternion[1] = point.y;
  pointQuaternion[2] = point.z;
  pointQuaternion[3] = 0.0f;

  std::array<float, 4> partialRotationCalculation = calc_hamilton_product(quaternion, pointQuaternion);
  std::array<float, 4> fullRotationCalculation = calc_hamilton_product(partialRotationCalculation, conjugateQuaternion);

  point.x = fullRotationCalculation[0];
  point.y = fullRotationCalculation[1];
  point.z = fullRotationCalculation[2];
}