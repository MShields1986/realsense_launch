#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


class CameraInfoOverrideNode(Node):
    def __init__(self):
        super().__init__('camera_info_override')

        self.declare_parameter('calibration_file', '')
        cal_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        if not cal_file:
            self.get_logger().error('calibration_file parameter is empty')
            return

        with open(cal_file, 'r') as f:
            cal = yaml.safe_load(f)

        self._K = [float(x) for x in cal['camera_matrix']['data']]
        self._D = [float(x) for x in cal['distortion_coefficients']['data']]
        self._P = [float(x) for x in cal['projection_matrix']['data']]
        self._distortion_model = cal.get('distortion_model', 'plumb_bob')

        self._pub = self.create_publisher(CameraInfo, 'camera_info_out', 10)
        self._sub = self.create_subscription(CameraInfo, 'camera_info_in', self._callback, 10)

        self.get_logger().info(f'Loaded calibration from {cal_file}')

    def _callback(self, msg: CameraInfo):
        msg.k = self._K
        msg.d = self._D
        msg.p = self._P
        msg.distortion_model = self._distortion_model
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CameraInfoOverrideNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
