# Copyright 2026 Taha Zahid
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bridge named Gazebo Pose_V messages to ROS 2 TFMessage."""

from collections import Counter
import re
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V  # type: ignore[import-not-found]
from gz.transport13 import Node as GzNode  # type: ignore[import-not-found]
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

_CHILD_LINK_SUFFIX_RE = re.compile(
    r'(?:^|_|/)(?:'
    r'base_link|base_footprint|trunk|imu_link|lidar_link|follow_camera_link'
    r'|link0[0-6]|gripperStator|gripperMover|gripper_tcp'
    r'|(?:FR|FL|RR|RL)_(?:hip|hip_rotor|thigh|thigh_rotor|calf|calf_rotor|foot)'
    r')$'
)


def _is_model_pose_name(name: str, count: int) -> bool:
    """Return True if a Gazebo Pose_V entry represents a uniquely named top-level model."""
    if not name or count != 1:
        return False
    if '::' in name:
        return False
    if name.endswith(('_visual', '_collision')):
        return False
    if '_visual_' in name or '_collision_' in name:
        return False
    if _CHILD_LINK_SUFFIX_RE.search(name):
        return False
    return True


def _pose_tuple(pose: Any) -> Tuple[float, float, float, float, float, float, float]:
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


class GzPoseBridgeNode(Node):
    """Republish Gazebo /world/<world>/pose/info with populated child_frame_id."""

    def __init__(self) -> None:
        super().__init__('gz_pose_bridge')
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('rate_hz', 5.0)

        self._world_name = str(self.get_parameter('world_name').value)
        rate_hz = max(0.5, float(self.get_parameter('rate_hz').value))
        topic = f'/world/{self._world_name}/pose/info'

        self._latest_msg: Optional[Any] = None
        self._last_poses: Dict[str, Tuple[float, ...]] = {}
        self._tick_count: int = 0

        self._pub = self.create_publisher(TFMessage, topic, 10)
        self._gz_node = GzNode()
        ok = self._gz_node.subscribe(Pose_V, topic, self._on_gz_pose)
        if not ok:
            self.get_logger().error(f'Failed to subscribe to Gazebo topic: {topic}')
        else:
            self.get_logger().info(f'Bridging Gazebo Pose_V -> ROS 2 TFMessage on {topic}')

        self._timer = self.create_timer(1.0 / rate_hz, self._publish_latest)

    def _on_gz_pose(self, msg: Any) -> None:
        self._latest_msg = msg

    def _publish_latest(self) -> None:
        msg = self._latest_msg
        if msg is None:
            return

        counts = Counter(pose.name for pose in msg.pose)

        full_sync = (self._tick_count % 5 == 0)
        self._tick_count += 1

        tf_msg = TFMessage()
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nsec)

        for pose in msg.pose:
            if not _is_model_pose_name(pose.name, counts[pose.name]):
                continue
            cur = _pose_tuple(pose)
            prev = self._last_poses.get(pose.name)
            if not full_sync and prev is not None:
                if max(abs(a - b) for a, b in zip(cur, prev)) < 1e-4:
                    continue
            self._last_poses[pose.name] = cur

            tf = TransformStamped()
            tf.header.stamp.sec = sec
            tf.header.stamp.nanosec = nsec
            tf.header.frame_id = self._world_name
            tf.child_frame_id = pose.name
            tf.transform.translation.x = cur[0]
            tf.transform.translation.y = cur[1]
            tf.transform.translation.z = cur[2]
            tf.transform.rotation.x = cur[3]
            tf.transform.rotation.y = cur[4]
            tf.transform.rotation.z = cur[5]
            tf.transform.rotation.w = cur[6]
            tf_msg.transforms.append(tf)

        if tf_msg.transforms:
            self._pub.publish(tf_msg)


def main(args: Optional[list[str]] = None) -> None:
    """Entry point for gz_pose_bridge."""
    rclpy.init(args=args)
    node = GzPoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
