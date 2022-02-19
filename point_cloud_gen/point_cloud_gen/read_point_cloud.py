import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import numpy as np

class ReadPointCloudNode(Node):
    def __init__(self):
        super().__init__('read_point_cloud_node')

        # Subscribes to topic to retrieve depth image and hfov/vfov
        self.sub = self.create_subscription(PointCloud2, 'camera/point_cloud', self.sub_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'camera/point_reliable', 10)

        self.pub_msg = None

    def sub_callback(self, msg):
        self.get_logger().info('Receiving message')

        float_data = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.width * msg.height, 4))
        self.get_logger().info(str(float_data.shape))

        for i in range(msg.height):
            self.get_logger().info(str(float_data[i * msg.width]))


        """ new_data = msg.data

        msg.width = msg.width * msg.height
        msg.height = 1

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width

        if self.pub_msg == None:
            for i in range(int(len(new_data) / 16)):

                new_data[i*12:i*12+12] = msg.data[i*16:i*16+12]
            
            self.pub_msg = new_data[:msg.width*msg.point_step]
        
        self.get_logger().info(str(len(self.pub_msg)))
        msg.data = self.pub_msg

        np_arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.width, 3))
        self.get_logger().info(str(np_arr.shape))
        self.get_logger().info("List of points")

        for i in range(np_arr.shape[0]):
            self.get_logger().info(str(np_arr[np_arr.shape[0] - 1 - i])) """

        self.pub.publish(msg)
        self.get_logger().info('Publishing message')


def main():
    rclpy.init()
    node = ReadPointCloudNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()