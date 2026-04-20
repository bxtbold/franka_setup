"""ROS adapter for Franka control and state access."""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, List

try:
    import rospy
    from franka_msgs.msg import FrankaState
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    import tf2_ros
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "ROS Python dependencies are unavailable. Run inside ROS Noetic environment."
    ) from exc


class RosAdapter:
    def __init__(self, node_name: str = "franka_bridge", default_frame: str = "panda_link0") -> None:
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True, disable_signals=True)

        self._lock = threading.Lock()
        self._default_frame = default_frame
        self._state: Dict[str, Any] = {
            "timestamp": time.time(),
            "joint_positions": [],
            "joint_velocities": [],
            "ee_pose": None,
            "ee_pose_target": None,
            "gripper": {"position": None, "is_open": None},
        }

        self._pose_pub = rospy.Publisher(
            "/cartesian_impedance_controller/equilibrium_pose", PoseStamped, queue_size=10
        )
        self._target_pose_pub = rospy.Publisher(
            "/franka_bridge/target_pose", PoseStamped, queue_size=10, latch=True
        )
        self._joint_pub = rospy.Publisher(
            "/joint_position_controller/command", Float64MultiArray, queue_size=10
        )
        self._target_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._target_child_frame = "franka_target"
        self._joint_sub = rospy.Subscriber(
            "/franka_state_controller/joint_states", JointState, self._joint_state_cb, queue_size=10
        )
        self._franka_state_sub = rospy.Subscriber(
            "/franka_state_controller/franka_states", FrankaState, self._franka_state_cb, queue_size=10
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["joint_positions"] = list(msg.position)
            self._state["joint_velocities"] = list(msg.velocity)

    @staticmethod
    def _quat_from_rotation_matrix(m: List[float]) -> List[float]:
        # m is row-major [r00 r01 r02 r10 r11 ... r22]
        r00, r01, r02 = m[0], m[1], m[2]
        r10, r11, r12 = m[3], m[4], m[5]
        r20, r21, r22 = m[6], m[7], m[8]
        trace = r00 + r11 + r22
        if trace > 0.0:
            s = (trace + 1.0) ** 0.5 * 2.0
            qw = 0.25 * s
            qx = (r21 - r12) / s
            qy = (r02 - r20) / s
            qz = (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = (1.0 + r00 - r11 - r22) ** 0.5 * 2.0
            qw = (r21 - r12) / s
            qx = 0.25 * s
            qy = (r01 + r10) / s
            qz = (r02 + r20) / s
        elif r11 > r22:
            s = (1.0 + r11 - r00 - r22) ** 0.5 * 2.0
            qw = (r02 - r20) / s
            qx = (r01 + r10) / s
            qy = 0.25 * s
            qz = (r12 + r21) / s
        else:
            s = (1.0 + r22 - r00 - r11) ** 0.5 * 2.0
            qw = (r10 - r01) / s
            qx = (r02 + r20) / s
            qy = (r12 + r21) / s
            qz = 0.25 * s
        return [qx, qy, qz, qw]

    def _franka_state_cb(self, msg: FrankaState) -> None:
        # O_T_EE is homogeneous transform flattened in column-major order.
        t = list(msg.O_T_EE)
        if len(t) != 16:
            return
        rot_row_major = [
            t[0], t[4], t[8],
            t[1], t[5], t[9],
            t[2], t[6], t[10],
        ]
        pose = {
            "position": [float(t[12]), float(t[13]), float(t[14])],
            "orientation": self._quat_from_rotation_matrix(rot_row_major),
            "frame_id": self._default_frame,
        }
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["ee_pose"] = pose

    def move_pose(
        self,
        position: List[float],
        orientation: List[float],
        frame_id: str | None = None,
    ) -> None:
        if len(position) != 3:
            raise ValueError("'position' must contain [x, y, z].")
        if len(orientation) != 4:
            raise ValueError("'orientation' must contain quaternion [x, y, z, w].")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        target_frame = frame_id or self._default_frame
        pose.header.frame_id = target_frame

        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])

        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])

        self._pose_pub.publish(pose)
        self._target_pose_pub.publish(pose)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = pose.header.stamp
        tf_msg.header.frame_id = (
            self._default_frame if target_frame == "0" else target_frame
        )
        tf_msg.child_frame_id = self._target_child_frame
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation.x = pose.pose.orientation.x
        tf_msg.transform.rotation.y = pose.pose.orientation.y
        tf_msg.transform.rotation.z = pose.pose.orientation.z
        tf_msg.transform.rotation.w = pose.pose.orientation.w
        self._target_tf_broadcaster.sendTransform(tf_msg)

        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["ee_pose_target"] = {
                "position": list(position),
                "orientation": list(orientation),
                "frame_id": target_frame,
            }

    def joint_reset(self, joints: List[float]) -> None:
        if len(joints) != 7:
            raise ValueError("'joints' must contain 7 joint values.")
        msg = Float64MultiArray(data=[float(v) for v in joints])
        self._joint_pub.publish(msg)

    def open_gripper(self) -> Dict[str, Any]:
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["gripper"]["is_open"] = True
        return self.get_gripper_state()

    def close_gripper(self) -> Dict[str, Any]:
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["gripper"]["is_open"] = False
        return self.get_gripper_state()

    def move_gripper(self, position: float) -> Dict[str, Any]:
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["gripper"]["position"] = float(position)
        return self.get_gripper_state()

    def get_gripper_state(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state["gripper"])

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            # Copy nested structures to avoid external mutation.
            return {
                "timestamp": float(self._state["timestamp"]),
                "joint_positions": list(self._state["joint_positions"]),
                "joint_velocities": list(self._state["joint_velocities"]),
                "ee_pose": (
                    dict(self._state["ee_pose"])
                    if isinstance(self._state["ee_pose"], dict)
                    else self._state["ee_pose"]
                ),
                "ee_pose_target": (
                    dict(self._state["ee_pose_target"])
                    if isinstance(self._state["ee_pose_target"], dict)
                    else self._state["ee_pose_target"]
                ),
                "gripper": dict(self._state["gripper"]),
            }
