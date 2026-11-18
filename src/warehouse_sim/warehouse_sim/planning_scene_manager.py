#!/usr/bin/env python3
"""
PlanningSceneManager: Reusable Python wrapper for MoveIt 2 Planning Scene services.

Provides synchronous methods to add, remove, attach, and detach collision
geometries via MoveIt's `/apply_planning_scene` service.
"""

import threading
from typing import List, Optional, Sequence, Union
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
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


# Default gripper and end-effector links allowed to contact grasped objects
DEFAULT_GRIPPER_TOUCH_LINKS: List[str] = [
    "grasp_tcp",
    "robotiq_85_base_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_right_knuckle_link",
]


class PlanningSceneManager:
    """Manages MoveIt 2 Planning Scene collision objects synchronously via service calls."""

    def __init__(
        self,
        node: Node,
        service_timeout_sec: float = 10.0,
        spin_locally: bool = True,
    ):
        self.node = node
        self.logger = node.get_logger()
        self._tracked_objects: set = set()
        self._tracked_attached: set = set()
        self.spin_locally = spin_locally

        self._apply_scene_client = self.node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._get_scene_client = self.node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

        self.service_timeout_sec = service_timeout_sec

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
        self.logger.info(
            f"Waiting for '/apply_planning_scene' service (timeout: {timeout}s)..."
        )
        ready = self._apply_scene_client.wait_for_service(timeout_sec=timeout)
        if ready:
            self.logger.info("'/apply_planning_scene' service is ready.")
        else:
            self.logger.error("Timed out waiting for '/apply_planning_scene' service.")
        return ready

    def apply_scene_diff(self, scene: PlanningScene) -> bool:
        """Send a PlanningScene diff request synchronously and return True if accepted."""
        if not self._apply_scene_client.service_is_ready():
            if not self.wait_for_service(timeout_sec=2.0):
                return False

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self._apply_scene_client.call_async(request)
        self._wait_future(future)

        if not future.done():
            self.logger.error("Call to '/apply_planning_scene' timed out.")
            return False

        response = future.result()
        if response is not None and response.success:
            return True
        else:
            self.logger.error("'/apply_planning_scene' returned failure.")
            return False

    def add_box(
        self,
        name: str,
        dimensions: Sequence[float],
        position: Sequence[float],
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = "world",
    ) -> bool:
        """
        Add a solid box obstacle to the MoveIt planning scene.

        :param name: Unique identifier for the collision object.
        :param dimensions: (size_x, size_y, size_z) in meters.
        :param position: (x, y, z) coordinates.
        :param orientation: (x, y, z, w) quaternion.
        :param frame_id: Reference frame for the pose (default: 'world').
        :return: True if successfully applied to the planning scene.
        """
        if len(dimensions) != 3 or len(position) != 3 or len(orientation) != 4:
            raise ValueError(
                "Invalid parameters: dimensions must have 3 elements, position 3, orientation 4."
            )

        collision_obj = CollisionObject()
        collision_obj.header.frame_id = frame_id
        collision_obj.id = name
        collision_obj.operation = CollisionObject.ADD

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [
            float(dimensions[0]),
            float(dimensions[1]),
            float(dimensions[2]),
        ]

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
            self.logger.info(
                f"Added box collision object '{name}' at {position} (size: {dimensions}) in '{frame_id}'."
            )
        else:
            self.logger.error(f"Failed to add box collision object '{name}'.")
        return success

    def add_cylinder(
        self,
        name: str,
        height: float,
        radius: float,
        position: Sequence[float],
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = "world",
    ) -> bool:
        """
        Add a solid cylinder obstacle to the MoveIt planning scene.

        :param name: Unique identifier for the collision object.
        :param height: Length along the cylinder's local z-axis in meters.
        :param radius: Radius of the cylinder in meters.
        :param position: (x, y, z) coordinates.
        :param orientation: (x, y, z, w) quaternion.
        :param frame_id: Reference frame for the pose.
        :return: True if successfully applied.
        """
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = frame_id
        collision_obj.id = name
        collision_obj.operation = CollisionObject.ADD

        cyl_primitive = SolidPrimitive()
        cyl_primitive.type = SolidPrimitive.CYLINDER
        cyl_primitive.dimensions = [float(height), float(radius)]

        cyl_pose = Pose()
        cyl_pose.position = Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        )
        cyl_pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )

        collision_obj.primitives = [cyl_primitive]
        collision_obj.primitive_poses = [cyl_pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld()
        scene.world.collision_objects = [collision_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_objects.add(name)
            self.logger.info(
                f"Added cylinder collision object '{name}' (h={height}, r={radius}) at {position}."
            )
        else:
            self.logger.error(f"Failed to add cylinder collision object '{name}'.")
        return success

    def remove_object(self, name: str) -> bool:
        """
        Remove a collision object from the world planning scene.

        :param name: Identifier of the object to remove.
        :return: True if removal was acknowledged.
        """
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
            self.logger.info(f"Removed collision object '{name}' from planning scene.")
        else:
            self.logger.error(f"Failed to remove collision object '{name}'.")
        return success

    def attach_object(
        self,
        name: str,
        link_name: str = "grasp_tcp",
        touch_links: Optional[List[str]] = None,
    ) -> bool:
        """
        Attach an existing world collision object to a robot link.

        This moves the object from the world into the robot's collision model so
        that trajectory planning accounts for the held payload while moving.

        :param name: Identifier of the object to attach.
        :param link_name: Robot link to bind the object to (default: 'grasp_tcp').
        :param touch_links: Links permitted to contact the object without triggering collision errors.
        :return: True if successfully attached.
        """
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = link_name
        attached_obj.object.id = name
        attached_obj.object.operation = CollisionObject.ADD

        # Default allowed touch links (gripper fingers, knuckles, base, and tool frames)
        if touch_links is None:
            touch_links = list(DEFAULT_GRIPPER_TOUCH_LINKS)
            if link_name not in touch_links:
                touch_links.append(link_name)
        attached_obj.touch_links = touch_links

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state = RobotState()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_obj]

        success = self.apply_scene_diff(scene)
        if success:
            self._tracked_attached.add(name)
            self.logger.info(
                f"Attached object '{name}' to robot link '{link_name}' (touch links: {touch_links})."
            )
        else:
            self.logger.error(f"Failed to attach object '{name}' to '{link_name}'.")
        return success

    def detach_object(self, name: str) -> bool:
        """
        Detach an attached object from the robot link and return it to the world.

        :param name: Identifier of the object to detach.
        :return: True if successfully detached.
        """
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
            self.logger.info(f"Detached object '{name}' from robot.")
        else:
            self.logger.error(f"Failed to detach object '{name}'.")
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
        """
        Query MoveIt's '/get_planning_scene' service for requested scene components.

        :param components: Bitmask of PlanningSceneComponents flags.
        :return: PlanningScene message if successful, else None.
        """
        if not self._get_scene_client.service_is_ready():
            if not self._get_scene_client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.logger.error("Timed out waiting for '/get_planning_scene' service.")
                return None

        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(components=components)

        future = self._get_scene_client.call_async(request)
        self._wait_future(future)

        if not future.done() or future.result() is None:
            self.logger.error("Call to '/get_planning_scene' failed or timed out.")
            return None

        return future.result().scene

    def set_allowed_collisions(
        self,
        target_name: str,
        other_names: Sequence[str],
        allowed: bool = True,
    ) -> bool:
        """
        Update MoveIt's Allowed Collision Matrix (ACM) to permit or forbid contact
        between `target_name` (e.g., 'pickable_cube') and `other_names` (e.g., gripper links).

        Preserves all existing SRDF and scene ACM entries by querying '/get_planning_scene' first.

        :param target_name: Collision object or link name.
        :param other_names: Sequence of link or object names to pair with `target_name`.
        :param allowed: True to allow collisions (ignore contact), False to enforce collision check.
        :return: True if the updated ACM was successfully applied.
        """
        current_scene = self.get_planning_scene(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        if current_scene is None:
            self.logger.error("Cannot update ACM: failed to retrieve current AllowedCollisionMatrix.")
            return False

        acm: AllowedCollisionMatrix = current_scene.allowed_collision_matrix
        names: List[str] = list(acm.entry_names)

        # Helper to append a new entity into the square N x N matrix if not yet present
        def _ensure_entry(entity_name: str) -> int:
            if entity_name in names:
                return names.index(entity_name)
            # Expand existing rows by 1 column (default False)
            for row in acm.entry_values:
                row.enabled.append(False)
            # Append new row of length N+1 (False off-diagonal, True on self-diagonal)
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

        success = self.apply_scene_diff(scene_diff)
        if success:
            state_str = "ALLOWED" if allowed else "FORBIDDEN"
            self.logger.info(
                f"Updated ACM: collisions between '{target_name}' and {list(other_names)} set to {state_str}."
            )
        else:
            self.logger.error(f"Failed to update ACM for '{target_name}'.")
        return success

    def allow_gripper_collisions(
        self,
        object_name: str,
        allowed: bool = True,
        extra_links: Optional[Sequence[str]] = None,
    ) -> bool:
        """
        Configure the Allowed Collision Matrix so the gripper links (and optional support surfaces)
        can contact `object_name` without triggering collision aborts during approach/grasp/lift.

        :param object_name: Target object ID (e.g., 'pickable_cube').
        :param allowed: True to allow contact, False to re-enable collision checking.
        :param extra_links: Additional links or world object IDs (e.g., ['shelf_table']).
        :return: True if ACM update succeeded.
        """
        links = list(DEFAULT_GRIPPER_TOUCH_LINKS)
        if extra_links:
            for item in extra_links:
                if item not in links:
                    links.append(item)
        return self.set_allowed_collisions(object_name, links, allowed=allowed)

    def verify_planning_scene(
        self,
        expected_objects: Sequence[str],
        acm_target: Optional[str] = None,
        expected_allowed_links: Optional[Sequence[str]] = None,
    ) -> bool:
        """
        Verify that the active MoveIt planning scene contains all expected world collision
        objects (with positive geometry bounds) and expected Allowed Collision Matrix rules.

        :param expected_objects: Object IDs that must exist in PlanningSceneWorld.
        :param acm_target: Optional object ID to check in the AllowedCollisionMatrix.
        :param expected_allowed_links: Links/objects that must have allowed collision with `acm_target`.
        :return: True if all checks pass.
        """
        scene = self.get_planning_scene(
            components=(
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
        )
        if scene is None:
            self.logger.error("Scene verification failed: '/get_planning_scene' returned None.")
            return False

        world_objs = {obj.id: obj for obj in scene.world.collision_objects}
        all_passed = True

        self.logger.info("=== MoveIt Planning Scene Verification ===")
        for obj_id in expected_objects:
            if obj_id not in world_objs:
                self.logger.error(f"  [FAIL] Missing world collision object: '{obj_id}'")
                all_passed = False
                continue

            obj = world_objs[obj_id]
            if not obj.primitives or any(d <= 0.0 for d in obj.primitives[0].dimensions):
                self.logger.error(f"  [FAIL] Object '{obj_id}' has invalid/empty primitive dimensions.")
                all_passed = False
                continue

            dims = [round(d, 4) for d in obj.primitives[0].dimensions]
            pos = obj.primitive_poses[0].position
            pos_list = [round(pos.x, 4), round(pos.y, 4), round(pos.z, 4)]
            self.logger.info(
                f"  [OK] World Object '{obj_id}': dims={dims}, pos={pos_list}, frame='{obj.header.frame_id}'"
            )

        if acm_target and expected_allowed_links:
            acm = scene.allowed_collision_matrix
            names = list(acm.entry_names)
            if acm_target not in names:
                self.logger.error(
                    f"  [FAIL] ACM target '{acm_target}' not found in AllowedCollisionMatrix."
                )
                all_passed = False
            else:
                t_idx = names.index(acm_target)
                missing_or_disallowed = []
                for link in expected_allowed_links:
                    if link not in names:
                        missing_or_disallowed.append(f"{link} (not in ACM)")
                    else:
                        l_idx = names.index(link)
                        if not acm.entry_values[t_idx].enabled[l_idx]:
                            missing_or_disallowed.append(f"{link} (enabled=False)")

                if missing_or_disallowed:
                    self.logger.error(
                        f"  [FAIL] ACM for '{acm_target}' missing allowed entries: {missing_or_disallowed}"
                    )
                    all_passed = False
                else:
                    self.logger.info(
                        f"  [OK] ACM verified for '{acm_target}': allowed contact with {len(expected_allowed_links)} links/surfaces."
                    )

        if all_passed:
            self.logger.info("=== Planning Scene Verification PASSED ===")
        else:
            self.logger.error("=== Planning Scene Verification FAILED ===")

        return all_passed

    def clear_all(self) -> bool:
        """Remove all tracked world and attached collision objects."""
        success = True
        for att in list(self._tracked_attached):
            if not self.detach_object(att):
                success = False

        for name in list(self._tracked_objects):
            if not self.remove_object(name):
                success = False

        self.logger.info("Cleared tracked planning scene collision objects.")
        return success

    @property
    def tracked_objects(self) -> List[str]:
        """Return list of world object IDs currently tracked by this manager."""
        return sorted(list(self._tracked_objects))

    @property
    def tracked_attached(self) -> List[str]:
        """Return list of attached object IDs currently tracked by this manager."""
        return sorted(list(self._tracked_attached))


def main():
    """Standalone CLI verification for PlanningSceneManager."""
    rclpy.init()
    node = Node("test_planning_scene_manager")

    manager = PlanningSceneManager(node)
    node.get_logger().info("Testing PlanningSceneManager against active MoveIt instance...")

    if not manager.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Service /apply_planning_scene not available. Ensure move_group is running."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    # Test 1: Add a test box
    box_ok = manager.add_box(
        name="cli_verification_box",
        dimensions=(0.1, 0.1, 0.1),
        position=(0.5, 0.2, 0.45),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )

    # Test 2: Remove the test box
    rm_ok = manager.remove_object("cli_verification_box")

    if box_ok and rm_ok:
        node.get_logger().info("SUCCESS: PlanningSceneManager verified successfully!")
    else:
        node.get_logger().error("FAILURE: One or more planning scene operations failed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
