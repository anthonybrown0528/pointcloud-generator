#include "gen_pointcloud.hpp"

using namespace std::chrono_literals;

GenPointCloudNode::GenPointCloudNode(float hfov, float vfov, float near, float far) : Node("gen_pointcloud"), AXIS_COUNT(3) {
  subscriber = this->create_subscription<sensor_msgs::msg::Image>("image", 2, std::bind(&GenPointCloudNode::subscriber_callback, this, std::placeholders::_1));
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

void GenPointCloudNode::subscriber_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  updateFromMsg(msg);

  flatData = std::vector<PointXYZ>(imageWidth * imageHeight);

  const cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg);
  image_ptr->image.convertTo(image_ptr->image, CV_32F, 1.0f / 255);

  gen_pointcloud(image_ptr);

  uint8_t* data = reinterpret_cast<uint8_t*>(&(flatData[0]));
  std::vector<uint8_t> dataVector(data, data + flatData.size() * sizeof(PointXYZ));

  pointcloudMsg.data = dataVector;
  pointcloudMsg.row_step = dataVector.size();

  RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
  publisher->publish(pointcloudMsg);
}

void GenPointCloudNode::updateFromMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
  this->imageWidth = msg->width;
  this->imageHeight = msg->height;

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
    this->u[i] = 0.5 - u[i];
    this->u[i] *= -1;
    }

    for(unsigned int i = 0; i < this->v.size(); i++) {
      this->v[i] = i / (float)(this->imageHeight - 1);
      this->v[i] = 0.5 - v[i];
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
    }
  }
}