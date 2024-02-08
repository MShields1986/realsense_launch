#!/usr/bin/env python3

# Python
import signal
import numpy as np
# Short-handing maths functions
sqrt = np.sqrt
sin = np.sin
cos = np.cos
tan = np.tan
asin = np.arcsin
acos = np.arccos
atan = np.arctan
atan2 = np.arctan2
pi = np.pi
to_rads = pi/180
to_degs = 180/pi

# ROS
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler

from pcl_cropbox_setter.srv import SetCropBoxPose, SetCropBoxPoseResponse


class CropBoxSetter:
    def __init__(self):
        self.kill_now = False

        # Termination callbacks
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        rospy.init_node('cropbox_setter', log_level=rospy.INFO)
        rospy.loginfo(f'Cropbox Setter Node Starting...')

        self.ns = rospy.get_namespace()

        self.rate = rospy.Rate(30)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.service = rospy.Service('set_crop_box_pose', SetCropBoxPose, self.service_callback)
        self.servicable_value = False

        # TODO: Set initial value from config file
        self.origin = self.populate_tf(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
            )

        self.x = self.y = self.z = 0

        while not rospy.has_param(f'{self.ns}crop_box/min_x'):
            rospy.loginfo_throttle(5, f'Holding for cropbox size parameters on {self.ns}crop_box')

        if rospy.has_param(f'{self.ns}crop_box/min_x'):
            self.x = round(rospy.get_param(f'{self.ns}crop_box/max_x') - rospy.get_param(f'{self.ns}crop_box/min_x'), 3)
            self.y = round(rospy.get_param(f'{self.ns}crop_box/max_y') - rospy.get_param(f'{self.ns}crop_box/min_y'), 3)
            self.z = round(rospy.get_param(f'{self.ns}crop_box/max_z') - rospy.get_param(f'{self.ns}crop_box/min_z'), 3)

            rospy.loginfo(f'Cropbox geometry...')
            rospy.loginfo(f'x: {self.x}')
            rospy.loginfo(f'y: {self.y}')
            rospy.loginfo(f'z: {self.z}')

        self.marker_pub = rospy.Publisher(f'{self.ns}cropbox_marker', Marker, queue_size = 2, latch=True)
        self.populate_marker(self.x, self.y, self.z)


    def exit_gracefully(self, *args):
        self.kill_now = True


    def populate_tf(self, x, y, z, yaw, pitch, roll):
        tf = TransformStamped()

        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = f'{self.ns[:-1]}_link' #'base_link' #'map'
        tf.child_frame_id = 'pcl_crop_box'

        tf.transform.translation.x = round(x, 3)
        tf.transform.translation.y = round(y, 3)
        tf.transform.translation.z = round(z, 3)

        q = quaternion_from_euler(
            round(roll * to_rads, 3),
            round(pitch * to_rads, 3),
            round(yaw * to_rads, 3))

        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        return tf


    def populate_marker(self, x, y, z):
        self.marker = Marker()
        #self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = 'pcl_crop_box'
        self.marker.type = 1 # Cube
        self.marker.id = 0

        self.marker.scale.x = x
        self.marker.scale.y = y
        self.marker.scale.z = z

        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.3

        self.marker.pose.position.x = x/2
        self.marker.pose.position.y = y/2
        self.marker.pose.position.z = z/2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0


    def tf_publish(self, origin):
        self.tf_broadcaster.sendTransform(origin)


    def service_callback(self, req):
        """
        Service to interactively set the frame of the crop box
        """
        rospy.loginfo(f'Request for change to cropbox location received')
        self.origin = self.populate_tf(
            req.x,
            req.y,
            req.z,
            req.yaw,
            req.pitch,
            req.roll
            )

        return SetCropBoxPoseResponse(True)


    def run(self):
        while not rospy.is_shutdown() or not self.kill_now:
            rospy.loginfo_throttle(60, f'Cropbox Setter Node Up Awaiting Location')
            self.origin.header.stamp = rospy.Time.now()
            self.tf_publish(self.origin)

            self.marker_pub.publish(self.marker)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        app = CropBoxSetter()
        app.run()
    except rospy.ROSInterruptException:
        pass
