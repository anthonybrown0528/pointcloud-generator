import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from cv_bridge import CvBridge
from pointcloud_interfaces.msg import ImageCamInfo

import numpy as np

class GenPointCloudNode(Node):
    def __init__(self):
        super().__init__('gen_point_cloud_node')

        # Defines the structure of the pointcloud msg data
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        
        # Contains XYZ coordinates of points
        points = np.zeros((0, 3)).astype(np.float32)

        # Depth image
        self.image_depth = None

        # Horizontal and vertical FOV of camera
        self.hfov = None
        self.vfov = None

        # Number of bytes describing a single point
        point_step = 4 * len(fields)

        # PointCloud2 msg used for publishing
        self.pointcloud = PointCloud2(height=1,
                                      width=points.shape[0],
                                      fields=fields,
                                      is_bigendian=False,
                                      is_dense=False,
                                      point_step=point_step,
                                      row_step=points.shape[0]*point_step,
                                      data=points.tobytes())

        self.pointcloud.header.frame_id = 'odom'

        # Used to converts ROS2 Image msg to numpy array
        self.bridge = CvBridge()

        # Subscribes to topic to retrieve depth image and hfov/vfov
        self.sub = self.create_subscription(ImageCamInfo, 'image_cam_info', self.sub_callback, 10)

        # Publishes generated pointcloud
        self.pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

    def sub_callback(self, msg):
        self.hfov = msg.hfov
        self.vfov = msg.vfov

        # Converts ROS2 Image msg to numpy array
        self.image_depth = self.bridge.imgmsg_to_cv2(msg.image)

        points = self.gen_pointcloud(self.image_depth, self.hfov, self.vfov, msg.near_plane, msg.far_plane)

        self.pointcloud.width = points.shape[0]
        self.row_step = 12 * points.shape[0]
        self.pointcloud.data = points.tobytes()

        self.pointcloud.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('Publishing pointcloud')
        self.pub.publish(self.pointcloud)

    def gen_pointcloud(self, image, hfov, vfov, near_plane, far_plane):
        # Used to map pixel of depth image to corresponding uv coordinates
        u = np.zeros((1, image.shape[1]))
        v = np.zeros((image.shape[0], 1))

        for i in range(u.shape[1]):
            u[0][i] = i / (u.shape[1] - 1)
        for i in range(v.shape[0]):
            v[i][0] = i / (v.shape[0] - 1)

        # TODO: Scale image to size in meters
        image = image * (far_plane - near_plane) + near_plane

        x = image * (1.0 - 2.0 * u) * np.tan(hfov*np.pi/180)   # x-coordinate
        y = image * (1.0 - 2.0 * v) * np.tan(vfov*np.pi/180)   # y-coordinate
        z = image                                              # z-coordinate

        # Combines XYZ coordinates and returns with datatype FLOAT32
        xyz_map = np.hstack((np.reshape(x, (x.shape[0] * x.shape[1], 1)), np.reshape(y, (y.shape[0] * y.shape[1], 1)), np.reshape(z, (z.shape[0] * z.shape[1], 1))))
        return xyz_map.astype(np.float32)


def main():
    rclpy.init()
    node = GenPointCloudNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()