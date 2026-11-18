# Copyright 2026 TEMPO Contributors
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

"""Reusable Python wrapper for MoveIt 2 Planning Scene attach/detach services."""

import threading
from typing import List, Optional, Sequence

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
    PlanningSceneWorld,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


DEFAULT_GRIPPER_TOUCH_LINKS: List[str] = [
    'gripper_tcp',
    'gripperStator',
    'gripperMover',
    'link06',
]


class PlanningSceneManager:
    """Manage MoveIt 2 Planning Scene collision objects via service calls."""

    def __init__(
        self,
        node: Node,
        service_timeout_sec: float = 3.0,
        spin_locally: bool = False,
    ) -> None:
        """Initialise MoveIt planning scene service clients."""
        self.node = node
        self.logger = node.get_logger()
        self._tracked_objects: set = set()
        self._tracked_attached: set = set()
        self.spin_locally = spin_locally
        self.service_timeout_sec = service_timeout_sec

        self._apply_scene_client = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        self._get_scene_client = self.node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )

    def _wait_future(self, future) -> None:
        if self.spin_locally:
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=self.service_timeout_sec
            )
            return
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        done_event.wait(timeout=self.service_timeout_sec)

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        """Wait for the /apply_planning_scene service to become available."""
        timeout = timeout_sec if timeout_sec is not None else self.service_timeout_sec
        return self._apply_scene_client.wait_for_service(timeout_sec=timeout)

    def apply_scene_diff(self, scene: PlanningScene) -> bool:
        """Send a PlanningScene diff request and return True if accepted."""
        if not self._apply_scene_client.service_is_ready():
            if not self.wait_for_service(timeout_sec=1.5):
                self.logger.warn("Service '/apply_planning_scene' not ready.")
                return False

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self._apply_scene_client.call_async(request)
        self._wait_future(future)

        if not future.done():
            self.logger.error("Call to '/apply_planning_scene' timed out.")
            return False

        response = future.result()
        return response is not None and bool(response.success)

    def add_box(
        self,
        name: str,
        dimensions: Sequence[float],
        position: Sequence[float],
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = 'base_link',
    ) -> bool:
        """Add a solid box collision object to the MoveIt planning scene."""
        if len(dimensions) != 3 or len(position) != 3 or len(orientation) != 4:
            raise ValueError(
                'Invalid parameters: dimensions(3), position(3), orientation(4) required.'
            )

        collision_obj = CollisionObject()
        collision_obj.header.frame_id = frame_id
        collision_obj.id = name
        collision_obj.operation = CollisionObject.ADD

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [float(d) for d in dimensions]

        box_pose = Pose()
        box_pose.position = Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        )
        box_pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )

        collision_obj.primitives = [box_primitive]
        collision_obj.primitive_poses = [box_pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        scene.world.collision_objects = [collision_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_objects.add(name)
        return success

    def remove_object(self, name: str) -> bool:
        """Remove a collision object from the world planning scene."""
        collision_obj = CollisionObject()
        collision_obj.id = name
        collision_obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        scene.world.collision_objects = [collision_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_objects.discard(name)
        return success

    def attach_object(
        self,
        name: str,
        link_name: str = 'gripper_tcp',
        touch_links: Optional[Sequence[str]] = None,
    ) -> bool:
        """Attach an existing world collision object to a robot gripper link."""
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = link_name
        attached_obj.object.id = name
        attached_obj.object.operation = CollisionObject.ADD

        resolved_links = (
            list(touch_links)
            if touch_links is not None
            else list(DEFAULT_GRIPPER_TOUCH_LINKS)
        )
        if link_name not in resolved_links:
            resolved_links.append(link_name)
        attached_obj.touch_links = resolved_links

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_attached.add(name)
        return success

    def detach_object(self, name: str) -> bool:
        """Detach an attached object from the robot link back to the world."""
        attached_obj = AttachedCollisionObject()
        attached_obj.object.id = name
        attached_obj.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_attached.discard(name)
        return success

    def get_planning_scene(
        self,
        components: int = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        ),
    ) -> Optional[PlanningScene]:
        """Query MoveIt '/get_planning_scene' for requested scene components."""
        if not self._get_scene_client.service_is_ready():
            if not self._get_scene_client.wait_for_service(
                timeout_sec=self.service_timeout_sec
            ):
                return None

        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(components=components)

        future = self._get_scene_client.call_async(request)
        self._wait_future(future)

        if not future.done() or future.result() is None:
            return None
        return future.result().scene

    def set_allowed_collisions(
        self,
        target_name: str,
        other_names: Sequence[str],
        allowed: bool = True,
    ) -> bool:
        """Update MoveIt Allowed Collision Matrix for target_name vs other_names."""
        current_scene = self.get_planning_scene(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        if current_scene is None:
            return False

        acm: AllowedCollisionMatrix = current_scene.allowed_collision_matrix
        names: List[str] = list(acm.entry_names)

        def _ensure_entry(entity_name: str) -> int:
            if entity_name in names:
                return names.index(entity_name)
            for row in acm.entry_values:
                row.enabled.append(False)
            new_idx = len(names)
            names.append(entity_name)
            new_row = AllowedCollisionEntry()
            new_row.enabled = [False] * len(names)
            new_row.enabled[new_idx] = True
            acm.entry_values.append(new_row)
            return new_idx

        target_idx = _ensure_entry(target_name)
        for other in other_names:
            other_idx = _ensure_entry(other)
            acm.entry_values[target_idx].enabled[other_idx] = bool(allowed)
            acm.entry_values[other_idx].enabled[target_idx] = bool(allowed)

        acm.entry_names = names

        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.allowed_collision_matrix = acm
        return self.apply_scene_diff(scene_diff)

    def allow_gripper_collisions(
        self,
        object_name: str,
        allowed: bool = True,
        extra_links: Optional[Sequence[str]] = None,
    ) -> bool:
        """Allow or forbid gripper link contact with object_name in MoveIt ACM."""
        links = list(DEFAULT_GRIPPER_TOUCH_LINKS)
        if extra_links:
            for item in extra_links:
                if item not in links:
                    links.append(item)
        return self.set_allowed_collisions(object_name, links, allowed=allowed)
