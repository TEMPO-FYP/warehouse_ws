#!/usr/bin/env python3
"""
load_test_shelf_scene.py

Loads collision geometries corresponding to `test_shelf_world.sdf` into MoveIt 2's
active Planning Scene via PlanningSceneManager, configures the Allowed Collision
Matrix (ACM) for grasping, and verifies the populated scene:
1. `ground_plane` (z <= 0.0m boundary floor)
2. `shelf_table` (0.5m x 0.8m x 0.4m static table, surface at z=0.4m)
3. `pickable_cube` (0.05m x 0.05m x 0.05m target cube at x=0.50, y=0.0, z=0.425)
4. `place_surface` (0.3m x 0.3m x 0.4m designated side drop surface at x=0.0, y=0.5, z=0.2)
5. Allowed Collision Matrix (ACM): permits contact between `pickable_cube`, gripper touch links,
   and support surfaces (`shelf_table`, `place_surface`).
"""

import sys
import rclpy
from rclpy.node import Node
from warehouse_sim.planning_scene_manager import (
    DEFAULT_GRIPPER_TOUCH_LINKS,
    PlanningSceneManager,
)

EXPECTED_SCENE_OBJECTS = [
    "ground_plane",
    "shelf_table",
    "pickable_cube",
    "place_surface",
]

EXPECTED_ACM_ALLOWED_LINKS = list(DEFAULT_GRIPPER_TOUCH_LINKS) + [
    "shelf_table",
    "place_surface",
]


def verify_test_shelf_scene(manager: PlanningSceneManager) -> bool:
    """
    Query MoveIt's `/get_planning_scene` service to validate that all 4 test world
    collision objects and the Allowed Collision Matrix (ACM) rules for `pickable_cube`
    are active and properly configured.
    """
    return manager.verify_planning_scene(
        expected_objects=EXPECTED_SCENE_OBJECTS,
        acm_target="pickable_cube",
        expected_allowed_links=EXPECTED_ACM_ALLOWED_LINKS,
    )


def load_test_shelf_scene(
    manager: PlanningSceneManager,
    cube_pos: tuple = (0.50, 0.00, 0.425),
    place_pos: tuple = (0.00, 0.50, 0.20),
    frame_id: str = "world",
    configure_acm: bool = True,
    verify: bool = True,
) -> bool:
    """
    Publish all collision objects for the test shelf environment to the planning scene,
    configure the Allowed Collision Matrix (Subtask 1.5), and verify the scene (Subtask 1.6).

    :param manager: Initialized PlanningSceneManager instance.
    :param cube_pos: (x, y, z) position of the pickable cube center.
    :param place_pos: (x, y, z) position of the place platform center.
    :param frame_id: Reference frame for coordinates (default: 'world').
    :param configure_acm: Whether to update ACM for `pickable_cube` and gripper touch links.
    :param verify: Whether to verify scene objects and ACM via `/get_planning_scene`.
    :return: True if all collision objects, ACM rules, and verification checks succeeded.
    """
    logger = manager.logger
    logger.info("Populating test shelf environment collision geometries into MoveIt...")

    # 1. Ground Plane (large flat floor to prevent trajectories passing beneath robot base)
    ground_ok = manager.add_box(
        name="ground_plane",
        dimensions=(10.0, 10.0, 0.10),
        position=(0.0, 0.0, -0.05),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
    )

    # 2. Shelf Table (static table from test_shelf_world.sdf, size: [0.5, 0.8, 0.4] at [0.55, 0.0, 0.2])
    shelf_ok = manager.add_box(
        name="shelf_table",
        dimensions=(0.50, 0.80, 0.40),
        position=(0.55, 0.00, 0.20),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
    )

    # 3. Pickable Cube (5cm cube sitting on shelf surface at z = 0.40m + 0.025m = 0.425m)
    cube_ok = manager.add_box(
        name="pickable_cube",
        dimensions=(0.05, 0.05, 0.05),
        position=cube_pos,
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
    )

    # 4. Place Surface (designated destination table/platform for release)
    place_ok = manager.add_box(
        name="place_surface",
        dimensions=(0.30, 0.30, 0.40),
        position=place_pos,
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
    )

    objects_ok = ground_ok and shelf_ok and cube_ok and place_ok
    if not objects_ok:
        logger.error("Failed to load one or more collision objects.")
        return False

    logger.info("Successfully loaded all 4 collision objects into the MoveIt planning scene:")
    logger.info("  - ground_plane   : size [10.0, 10.0, 0.10] at [0.0, 0.0, -0.05]")
    logger.info("  - shelf_table    : size [0.50, 0.80, 0.40] at [0.55, 0.0, 0.20] (top surface z=0.40m)")
    logger.info(f"  - pickable_cube  : size [0.05, 0.05, 0.05] at {list(cube_pos)}")
    logger.info(f"  - place_surface  : size [0.30, 0.30, 0.40] at {list(place_pos)} (top surface z=0.40m)")

    # Subtask 1.5: Configure Allowed Collision Matrix (ACM) for pickable_cube
    acm_ok = True
    if configure_acm:
        acm_ok = manager.allow_gripper_collisions(
            object_name="pickable_cube",
            allowed=True,
            extra_links=["shelf_table", "place_surface"],
        )

    # Subtask 1.6: Verify Planning Scene & ACM state via /get_planning_scene
    verify_ok = True
    if verify:
        verify_ok = verify_test_shelf_scene(manager)

    return objects_ok and acm_ok and verify_ok


class LoadTestShelfSceneNode(Node):
    """ROS 2 Node that initializes, applies, and verifies the test shelf collision scene."""

    def __init__(self):
        super().__init__("load_test_shelf_scene")

        # Declare parameters for easy reconfiguration
        self.declare_parameter("cube_x", 0.50)
        self.declare_parameter("cube_y", 0.00)
        self.declare_parameter("cube_z", 0.425)

        self.declare_parameter("place_x", 0.00)
        self.declare_parameter("place_y", 0.50)
        self.declare_parameter("place_z", 0.20)

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("configure_acm", True)
        self.declare_parameter("verify", True)

        cube_pos = (
            float(self.get_parameter("cube_x").value),
            float(self.get_parameter("cube_y").value),
            float(self.get_parameter("cube_z").value),
        )
        place_pos = (
            float(self.get_parameter("place_x").value),
            float(self.get_parameter("place_y").value),
            float(self.get_parameter("place_z").value),
        )
        frame_id = str(self.get_parameter("frame_id").value)
        configure_acm = bool(self.get_parameter("configure_acm").value)
        verify = bool(self.get_parameter("verify").value)

        self.manager = PlanningSceneManager(self)
        if not self.manager.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "MoveIt '/apply_planning_scene' service unavailable. Ensure move_group is active."
            )
            self.success = False
            return

        self.success = load_test_shelf_scene(
            self.manager,
            cube_pos=cube_pos,
            place_pos=place_pos,
            frame_id=frame_id,
            configure_acm=configure_acm,
            verify=verify,
        )


def verify_main():
    """Standalone CLI entry point to verify the active MoveIt planning scene and ACM."""
    rclpy.init()
    node = Node("verify_test_shelf_scene")
    manager = PlanningSceneManager(node)
    ok = verify_test_shelf_scene(manager)
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


def main():
    rclpy.init()
    node = LoadTestShelfSceneNode()
    ok = getattr(node, "success", False)
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
