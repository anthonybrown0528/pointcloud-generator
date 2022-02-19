import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge
from pointcloud_interfaces.msg import ImageCamInfo

import numpy as np
import math

class GenPointCloudNode(Node):
    def __init__(self):
        super().__init__('gen_point_cloud_node')

        # Defines the structure of the pointcloud msg data
        self.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        self.num_of_fields = len(self.fields)
        
        # Contains XYZ coordinates of points
        self.points = np.zeros((0, 3)).astype(np.float32)

        # Depth image
        self.image_depth = None

        # Horizontal and vertical FOV of camera
        self.hfov = None
        self.vfov = None

        self.u = np.array([])
        self.v = np.array([])

        self.near_plane = None
        self.far_plane = None

        # Number of bytes describing a single point
        point_step = 4 * self.num_of_fields

        # PointCloud2 msg used for publishing
        self.pointcloud = PointCloud2(height=1,
                                      width=self.points.shape[0],
                                      fields=self.fields,
                                      is_bigendian=False,
                                      is_dense=False,
                                      point_step=point_step,
                                      row_step=self.points.shape[0]*point_step,
                                      data=self.points.tobytes())

        self.pointcloud.header.frame_id = 'odom'

        # Used to converts ROS2 Image msg to numpy array
        self.bridge = CvBridge()

        # Subscribes to topic to retrieve depth image and hfov/vfov
        self.sub = self.create_subscription(ImageCamInfo, 'image_cam_info', self.sub_callback, 5)

        # Publishes generated pointcloud
        self.pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

    def sub_callback(self, msg):
        self.hfov = msg.hfov
        self.vfov = msg.vfov

        self.near_plane = msg.near_plane
        self.far_plane = msg.far_plane

        # Converts ROS2 Image msg to numpy array
        self.image_depth = self.bridge.imgmsg_to_cv2(msg.image)
        if msg.image.encoding == '8UC1':
            self.image_depth = self.image_depth.astype(np.float32) / 255.0
            self.image_depth = self.image_depth.astype(np.float32)

        # Scale image to size in meters
        #image = image * (far_plane - near_plane) + near_plane
        self.image_depth = self.near_plane / (self.image_depth * self.near_plane / self.far_plane - self.image_depth + 1)

        self.points = self.gen_pointcloud()
        self.remove_boundary_values()

        self.pointcloud.width = self.points.shape[0]
        self.row_step = self.num_of_fields * self.points.shape[0]
        self.pointcloud.data = self.points.tobytes()

        self.pointcloud.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('Publishing pointcloud')
        self.pub.publish(self.pointcloud)

    def remove_boundary_values(self):
        removing_indices = []
        for i in range(self.points.shape[0]):
            if math.isclose(self.points[i][2], self.far_plane, rel_tol=0.01) or math.isclose(self.points[i][2], self.near_plane, rel_tol=0.01):
                removing_indices.append(i)

        self.points = np.delete(self.points, removing_indices, axis=0)

    def gen_pointcloud(self):

        if len(self.u) == 0 or len(self.v) == 0:
            # Used to map pixel of depth image to corresponding uv coordinates
            self.u = 0.5 - np.arange(self.image_depth.shape[1]).reshape((1, self.image_depth.shape[1])) / (self.image_depth.shape[1] - 1)
            self.u *= -1.0

            self.v = 0.5 - np.arange(self.image_depth.shape[0]).reshape((self.image_depth.shape[0], 1)) / (self.image_depth.shape[0] - 1)
            self.v *= -1.0

        x = self.image_depth * self.u * np.tan(np.deg2rad(self.hfov / 2.0))   # x-coordinate
        y = self.image_depth * self.v * np.tan(np.deg2rad(self.vfov / 2.0))   # y-coordinate
        z = self.image_depth                                                  # z-coordinate

        # Flatten coordinates into columns
        x = np.reshape(x, (x.shape[0] * x.shape[1], 1))
        y = np.reshape(y, (y.shape[0] * y.shape[1], 1))
        z = np.reshape(z, (z.shape[0] * z.shape[1], 1))

        # Combines XYZ coordinates and returns with datatype FLOAT32
        xyz = np.hstack((x, y, z))
        return xyz.astype(np.float32)


def main():
    rclpy.init()
    node = GenPointCloudNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()