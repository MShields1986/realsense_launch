#!/usr/bin/env python3
"""
compute_urdf_extrinsic.py

Reads the TF difference between a nominal and calibrated camera optical frame
and prints the corrected URDF <origin xyz="..." rpy="..."/> for the joint that
places the camera onto its parent link.

Math
----
The URDF joint defines T(parent → camera_base). The d435 description then has
a fixed chain T(camera_base → nominal_optical). After extrinsic calibration we
know where the optical frame truly is (calibrated_optical) and want:

    T_new(parent → camera_base) * T(camera_base → nominal_optical)
        = T(parent → calibrated_optical)

So:
    T_new = T(parent → calibrated_optical) * inv(T(camera_base → nominal_optical))

Both transforms are read directly from the live TF tree.

Parameters
----------
nominal_frame      - TF frame of the camera optical frame as placed by the URDF
                     default: rs_camera_1_color_optical_frame
calibrated_frame   - TF frame from the extrinsic calibration result
                     default: rs_camera_1_color_optical_frame_calibrated
joint_parent_frame - Parent link frame of the URDF joint to correct
                     default: ut_probe_realsense_link
joint_child_frame  - Child (camera base) link frame of that URDF joint
                     default: rs_camera_1_bottom_screw_frame
tf_timeout         - Seconds to wait for each TF lookup
                     default: 5.0

Usage
-----
ros2 run realsense_launch compute_urdf_extrinsic --ros-args \\
    -p nominal_frame:=rs_camera_1_color_optical_frame \\
    -p calibrated_frame:=rs_camera_1_color_optical_frame_calibrated \\
    -p joint_parent_frame:=ut_probe_realsense_link \\
    -p joint_child_frame:=rs_camera_1_bottom_screw_frame
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Pure-numpy rotation helpers (no scipy / tf_transformations dependency)
# ---------------------------------------------------------------------------

def _quat_to_rot(q):
    """quaternion [x, y, z, w] → 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _rot_to_rpy(R):
    """3x3 rotation matrix → [roll, pitch, yaw] in XYZ extrinsic convention."""
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    cos_p = np.cos(pitch)
    if abs(cos_p) < 1e-6:
        # Gimbal lock
        roll = np.arctan2(-R[1, 2] if pitch > 0 else R[1, 2], R[1, 1])
        yaw = 0.0
    else:
        roll = np.arctan2(R[2, 1] / cos_p, R[2, 2] / cos_p)
        yaw  = np.arctan2(R[1, 0] / cos_p, R[0, 0] / cos_p)
    return np.array([roll, pitch, yaw])


def _tf_to_matrix(transform_stamped):
    """geometry_msgs TransformStamped → 4x4 homogeneous matrix."""
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation
    T = np.eye(4)
    T[:3, :3] = _quat_to_rot([r.x, r.y, r.z, r.w])
    T[:3,  3] = [t.x, t.y, t.z]
    return T


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class ComputeUrdfExtrinsic(Node):

    def __init__(self):
        super().__init__('compute_urdf_extrinsic')

        self.declare_parameter('nominal_frame',      'rs_camera_1_color_optical_frame')
        self.declare_parameter('calibrated_frame',   'rs_camera_1_color_optical_frame_calibrated')
        self.declare_parameter('joint_parent_frame', 'ut_probe_realsense_link')
        self.declare_parameter('joint_child_frame',  'rs_camera_1_bottom_screw_frame')
        self.declare_parameter('tf_timeout',         5.0)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Allow TF to fill before attempting lookups
        self._timer = self.create_timer(2.0, self._run)

    # ------------------------------------------------------------------

    def _run(self):
        self._timer.cancel()

        nominal_frame    = self.get_parameter('nominal_frame').value
        calibrated_frame = self.get_parameter('calibrated_frame').value
        parent_frame     = self.get_parameter('joint_parent_frame').value
        child_frame      = self.get_parameter('joint_child_frame').value
        timeout_s        = self.get_parameter('tf_timeout').value
        timeout          = Duration(seconds=timeout_s)
        stamp            = Time()

        log = self.get_logger()

        try:
            # T(parent → calibrated_optical)
            t_parent_to_cal = self._tf_buffer.lookup_transform(
                parent_frame, calibrated_frame, stamp, timeout)
            T_parent_cal = _tf_to_matrix(t_parent_to_cal)

            # T(camera_base → nominal_optical) — fixed chain inside d435 description
            t_child_to_opt = self._tf_buffer.lookup_transform(
                child_frame, nominal_frame, stamp, timeout)
            T_child_opt = _tf_to_matrix(t_child_to_opt)

        except Exception as exc:
            log.error(f'TF lookup failed: {exc}')
            log.error('Make sure both frames are being published and TF is running.')
            return

        # New URDF joint origin: T(parent → child)_new
        T_new = T_parent_cal @ np.linalg.inv(T_child_opt)

        xyz = T_new[:3, 3]
        rpy = _rot_to_rpy(T_new[:3, :3])

        separator = '=' * 70
        log.info(separator)
        log.info('Corrected URDF <origin> for:')
        log.info(f'  Joint parent : {parent_frame}')
        log.info(f'  Joint child  : {child_frame}')
        log.info(f'  Calibration  : {nominal_frame}  →  {calibrated_frame}')
        log.info('')
        log.info('Copy this line into your URDF / xacro:')
        log.info('')
        log.info(
            f'  <origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}"'
            f' rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>'
        )
        log.info('')
        log.info('Individual values:')
        log.info(f'  x={xyz[0]:.6f}  y={xyz[1]:.6f}  z={xyz[2]:.6f}')
        log.info(f'  roll={rpy[0]:.6f}  pitch={rpy[1]:.6f}  yaw={rpy[2]:.6f}')
        log.info(separator)

        rclpy.shutdown()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ComputeUrdfExtrinsic())


if __name__ == '__main__':
    main()
