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

"""Unit tests for gz_video_recorder path resolution and MP4 writing."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from warehouse_sim.gz_video_recorder import _resolve_output_path, GzVideoRecorderNode


def test_resolve_output_path_appends_mp4(tmp_path: Path) -> None:
    """Verify explicit paths resolve cleanly and auto-append .mp4."""
    out = _resolve_output_path(str(tmp_path / 'demo_clip'))
    assert out.endswith('demo_clip.mp4')
    assert Path(out).parent.is_dir()


def test_video_recorder_writes_nonempty_mp4(tmp_path: Path) -> None:
    """Verify synthetic RGB frames encode into a valid MP4 file."""
    out_file = tmp_path / 'test_clip.mp4'
    rclpy.init()
    node = GzVideoRecorderNode()
    try:
        node._output_path = str(out_file)
        rgb = np.full((120, 160, 3), 180, dtype=np.uint8).tobytes()
        for _ in range(5):
            node._on_gz_image(SimpleNamespace(width=160, height=120, data=rgb))
        node.close_writer()
        assert node._frame_count == 5
        assert out_file.is_file()
        assert out_file.stat().st_size > 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
