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

"""Record a Gazebo Sim camera topic directly to an MP4 video file."""

from datetime import datetime
import os
import threading
from typing import Any, Optional

import cv2  # type: ignore[import-not-found]
from gz.msgs10.image_pb2 import Image  # type: ignore[import-not-found]
from gz.transport13 import Node as GzNode  # type: ignore[import-not-found]
import numpy as np
import rclpy
from rclpy.node import Node


def _default_recordings_dir() -> str:
    """Resolve recordings directory relative to workspace src/ when run from source or symlink-install."""
    cur = os.path.dirname(os.path.realpath(__file__))
    while cur and cur != os.path.dirname(cur):
        if os.path.basename(cur) == 'src':
            return os.path.join(cur, 'recordings')
        cur = os.path.dirname(cur)
    return os.path.abspath('recordings')


def _resolve_output_path(raw_path: str) -> str:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    clean = raw_path.strip()
    if not clean:
        resolved = os.path.join(_default_recordings_dir(), f'sim_{stamp}.mp4')
    elif not os.path.isabs(clean):
        resolved = os.path.join(_default_recordings_dir(), clean)
    else:
        resolved = clean

    if os.path.isdir(resolved) or resolved.endswith(os.sep):
        resolved = os.path.join(resolved, f'sim_{stamp}.mp4')
    elif not resolved.lower().endswith(('.mp4', '.avi', '.mkv')):
        resolved = f'{resolved}.mp4'

    return resolved


class GzVideoRecorderNode(Node):
    """Subscribe to a Gazebo Image topic and encode frames into an MP4 file."""

    def __init__(self) -> None:
        """Initialize Gazebo image subscriber and lazy OpenCV video writer."""
        super().__init__('gz_video_recorder')
        self.declare_parameter('topic', '/video_camera')
        self.declare_parameter('output_path', '')
        self.declare_parameter('fps', 15.0)

        topic = str(self.get_parameter('topic').value).strip() or '/video_camera'
        raw_out = str(self.get_parameter('output_path').value)
        self._fps = max(1.0, float(self.get_parameter('fps').value))
        self._output_path = _resolve_output_path(raw_out)

        self._lock = threading.Lock()
        self._writer: Optional[Any] = None
        self._frame_count: int = 0
        self._closed: bool = False
        self._warned_encoding: bool = False

        self._gz_node = GzNode()
        ok = self._gz_node.subscribe(Image, topic, self._on_gz_image)
        if not ok:
            self.get_logger().error(f'Failed to subscribe to Gazebo image topic: {topic}')
        else:
            self.get_logger().info(
                f'Recording {topic} -> {self._output_path} ({self._fps:.1f} fps)'
            )

    def _on_gz_image(self, msg: Any) -> None:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_rgb = width * height * 3
        if raw.size == expected_rgb:
            frame = cv2.cvtColor(raw.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        elif raw.size == width * height * 4:
            frame = cv2.cvtColor(raw.reshape((height, width, 4)), cv2.COLOR_RGBA2BGR)
        else:
            if not self._warned_encoding:
                self._warned_encoding = True
                self.get_logger().warning(
                    f'Unsupported image buffer size {raw.size} for {width}x{height}; '
                    'expected 3-channel RGB or 4-channel RGBA uint8 frames.'
                )
            return

        with self._lock:
            if self._closed:
                return
            if self._writer is None:
                parent = os.path.dirname(os.path.abspath(self._output_path))
                if parent:
                    os.makedirs(parent, exist_ok=True)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self._writer = cv2.VideoWriter(
                    self._output_path, fourcc, self._fps, (width, height)
                )
                if not self._writer.isOpened():
                    self.get_logger().error(
                        f'Failed to open VideoWriter for {self._output_path}'
                    )
                    self._closed = True
                    return
                self.get_logger().info(
                    f'Writing {width}x{height} video frames to {self._output_path}'
                )
            self._writer.write(frame)
            self._frame_count += 1
            if self._frame_count % 150 == 0:
                self.get_logger().info(
                    f'Recorded {self._frame_count} frames -> {self._output_path}'
                )

    def close_writer(self) -> None:
        """Finalize and release the MP4 VideoWriter."""
        with self._lock:
            self._closed = True
            if self._writer is not None:
                self._writer.release()
                self._writer = None
        self.get_logger().info(
            f'Saved {self._frame_count} frames to {self._output_path}'
        )


def main(args: Optional[list[str]] = None) -> None:
    """Entry point for gz_video_recorder."""
    rclpy.init(args=args)
    node = GzVideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_writer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
