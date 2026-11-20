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

"""TEMPO shared multi-robot controller node using Nav2, native MoveIt 2, and TF2."""

from dataclasses import dataclass, field
import json
import math
import os
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from gz.msgs10.empty_pb2 import Empty as GzEmpty  # type: ignore[import-not-found]
from gz.msgs10.stringmsg_pb2 import StringMsg as GzStringMsg  # type: ignore[import-not-found]
from gz.transport13 import Node as GzNode  # type: ignore[import-not-found]
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_controllers.geometry_utils import (
    quaternion_to_yaw,
    rpy_to_quat_tuple,
    transform_base_to_world,
    transform_world_to_base,
    yaw_to_quaternion,
)
from robot_controllers.map_standoff import OccupancyMap
from robot_controllers.moveit_client import MoveItRobotClient
from robot_controllers.planning_scene_manager import PlanningSceneManager
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
import yaml


PoseEntry = Tuple[float, float, float, Tuple[float, float, float, float]]


@dataclass
class RobotProfile:
    """Resolved per-instance configuration loaded from `controller_config.yaml`."""

    instance_id: str
    profile_name: str
    namespace: str
    gazebo_model_name: str
    base_frame: str
    tcp_frame: str
    standoff_distance: float
    manipulation_standoff: float
    nav_timeout_sec: float
    robot_clearance_radius: float
    fine_align_pulses: bool
    min_pulse_duration_sec: float
    move_group_name: str
    gripper_mode: str
    gripper_group_name: str
    gripper_action_name: Optional[str]
    gripper_open_position: float
    gripper_closed_position: float
    gripper_max_effort: float
    gripper_open_tolerance: float
    gripper_closed_tolerance: float
    arm_joint_tolerance: float
    moveit_timeout_sec: float
    min_reach_m: float
    max_reach_m: float
    place_drop_standoff: float
    place_clear_z_offset: float
    use_kinematic_attach: bool
    attach_max_distance_m: float
    default_object_dims: Tuple[float, float, float]
    grasp_roll_rad: float
    grasp_pitch_rad: float
    grasp_lateral_offset_m: float
    srdf_states: Dict[str, str] = field(default_factory=dict)
    touch_links: List[str] = field(default_factory=list)


def _build_profile(instance_id: str, profile_name: str, raw: Dict[str, Any]) -> RobotProfile:
    nav = raw.get('navigation', {})
    manip = raw.get('manipulation', {})
    raw_dims = manip.get('default_object_dims', [0.06, 0.06, 0.06])
    return RobotProfile(
        instance_id=instance_id,
        profile_name=profile_name,
        namespace=str(raw.get('namespace', '')).strip().rstrip('/'),
        gazebo_model_name=str(raw.get('gazebo_model_name', instance_id)),
        base_frame=str(raw.get('base_frame', 'base_link')),
        tcp_frame=str(raw.get('tcp_frame', 'gripper_tcp')),
        standoff_distance=float(nav.get('standoff_distance', 0.90)),
        manipulation_standoff=float(nav.get('manipulation_standoff', 0.73)),
        nav_timeout_sec=float(nav.get('nav_timeout_sec', 150.0)),
        robot_clearance_radius=float(nav.get('robot_clearance_radius', 0.36)),
        fine_align_pulses=bool(nav.get('fine_align_pulses', False)),
        min_pulse_duration_sec=float(nav.get('min_pulse_duration_sec', 0.55)),
        move_group_name=str(manip.get('move_group_name', 'z1_arm')),
        gripper_mode=str(manip.get('gripper_mode', 'move_group')),
        gripper_group_name=str(manip.get('gripper_group_name', 'z1_gripper')),
        gripper_action_name=manip.get('gripper_action_name'),
        gripper_open_position=float(manip.get('gripper_open_position', -1.50)),
        gripper_closed_position=float(manip.get('gripper_closed_position', 0.0)),
        gripper_max_effort=float(manip.get('gripper_max_effort', 50.0)),
        gripper_open_tolerance=float(manip.get('gripper_open_tolerance', 0.08)),
        gripper_closed_tolerance=float(manip.get('gripper_closed_tolerance', 0.02)),
        arm_joint_tolerance=float(manip.get('arm_joint_tolerance', 0.005)),
        moveit_timeout_sec=float(manip.get('moveit_timeout_sec', 45.0)),
        min_reach_m=float(manip.get('min_reach_m', 0.40)),
        max_reach_m=float(manip.get('max_reach_m', 0.85)),
        place_drop_standoff=float(manip.get('place_drop_standoff', 0.72)),
        place_clear_z_offset=float(manip.get('place_clear_z_offset', 0.055)),
        use_kinematic_attach=bool(manip.get('use_kinematic_attach', False)),
        attach_max_distance_m=float(manip.get('attach_max_distance_m', 0.18)),
        default_object_dims=(float(raw_dims[0]), float(raw_dims[1]), float(raw_dims[2])),
        grasp_roll_rad=float(
            manip.get('grasp_roll_rad', math.pi / 2.0 if profile_name == 'b1z1' else 0.0)
        ),
        grasp_pitch_rad=float(
            manip.get('grasp_pitch_rad', 0.26 if profile_name == 'b1z1' else 0.0)
        ),
        grasp_lateral_offset_m=float(
            manip.get('grasp_lateral_offset_m', 0.028 if profile_name == 'b1z1' else 0.0)
        ),
        srdf_states=dict(manip.get('srdf_states', {})),
        touch_links=list(manip.get('touch_links', [])),
    )


def _load_config() -> Dict[str, Any]:
    pkg_dir = get_package_share_directory('robot_controllers')
    config_path = os.path.join(pkg_dir, 'config', 'controller_config.yaml')
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


class MobileRobotController(Node):
    """Execute TEMPO mobile_robot subtasks across configured robot profiles via Nav2 & MoveIt 2."""

    def __init__(self) -> None:
        """Initialise per-robot clients, TF2 listener, pose bridge, and occupancy map."""
        super().__init__(
            'mobile_robot_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        config = _load_config()
        world_cfg = config.get('world', {})
        self._world_name: str = str(world_cfg.get('name', 'default'))
        self._fixture_surface_z: Dict[str, float] = {
            k: float(v) for k, v in world_cfg.get('fixture_surface_z', {}).items()
        }
        self._fixture_dims: Dict[str, Tuple[float, float, float]] = {
            k: (float(v[0]), float(v[1]), float(v[2]))
            for k, v in world_cfg.get('fixture_dims', {}).items()
        }
        self._occ_map = self._load_occupancy_map(
            str(world_cfg.get('map_package', 'warehouse_sim')),
            str(world_cfg.get('map_yaml_relpath', 'maps/warehouse_map.yaml')),
        )

        raw_profiles = config.get('robot_profiles', {})
        raw_instances = config.get('robot_instances', {})
        self._profiles: Dict[str, RobotProfile] = {}
        self._managed_prefixes = set(raw_profiles.keys())
        for instance_id, profile_name in raw_instances.items():
            if profile_name not in raw_profiles:
                raise KeyError(
                    f"Robot instance '{instance_id}' references unknown profile '{profile_name}'."
                )
            self._profiles[instance_id] = _build_profile(
                instance_id, profile_name, raw_profiles[profile_name]
            )
            self._managed_prefixes.add(instance_id.rsplit('_', 1)[0])

        self._cb_group = ReentrantCallbackGroup()
        self._pose_lock = threading.Lock()
        self._pose_registry: Dict[str, PoseEntry] = {}
        self._js_lock = threading.Lock()
        self._joint_positions: Dict[str, Dict[str, float]] = {
            inst: {} for inst in self._profiles
        }
        self._attach_lock = threading.Lock()
        self._attached_objects: Dict[str, Optional[str]] = {
            inst: None for inst in self._profiles
        }
        self._stop_event = threading.Event()
        self._robot_queues: Dict[str, queue.Queue] = {
            inst: queue.Queue() for inst in self._profiles
        }
        self._worker_threads: Dict[str, threading.Thread] = {}

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        any_attach = any(p.use_kinematic_attach for p in self._profiles.values())
        self._gz_node: Optional[GzNode] = GzNode() if any_attach else None

        self.create_subscription(
            String, '/mission/subtask', self._subtask_callback, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            TFMessage,
            f'/world/{self._world_name}/pose/info',
            self._pose_callback,
            10,
            callback_group=self._cb_group,
        )
        self._feedback_pub = self.create_publisher(String, '/mission/feedback', 10)

        self._nav_clients: Dict[str, ActionClient] = {}
        self._moveit_clients: Dict[str, MoveItRobotClient] = {}
        self._scene_managers: Dict[str, PlanningSceneManager] = {}
        self._cmd_vel_pubs: Dict[str, Any] = {}
        self._gz_attach_pubs: Dict[str, Any] = {}
        self._gz_detach_pubs: Dict[str, Any] = {}

        for inst, prof in self._profiles.items():
            ns_prefix = f'/{prof.namespace.strip("/")}' if prof.namespace.strip('/') else ''
            nav_topic = f'{ns_prefix}/navigate_to_pose'
            js_topic = f'{ns_prefix}/joint_states'
            cmd_topic = f'{ns_prefix}/cmd_vel'

            self._nav_clients[inst] = ActionClient(
                self, NavigateToPose, nav_topic, callback_group=self._cb_group
            )
            self._moveit_clients[inst] = MoveItRobotClient(
                self,
                namespace=ns_prefix,
                wait_future_fn=self._wait_future,
                callback_group=self._cb_group,
                moveit_timeout_sec=prof.moveit_timeout_sec,
                gripper_action_name=(
                    prof.gripper_action_name if prof.gripper_mode == 'gripper_command' else None
                ),
            )
            self._scene_managers[inst] = PlanningSceneManager(
                self,
                spin_locally=False,
                namespace=ns_prefix,
                touch_links=prof.touch_links or None,
            )
            self._cmd_vel_pubs[inst] = self.create_publisher(Twist, cmd_topic, 10)
            if prof.use_kinematic_attach and self._gz_node is not None:
                self._gz_attach_pubs[inst] = self._gz_node.advertise(
                    f'{ns_prefix}/attach', GzStringMsg
                )
                self._gz_detach_pubs[inst] = self._gz_node.advertise(
                    f'{ns_prefix}/detach', GzEmpty
                )
            self.create_subscription(
                JointState,
                js_topic,
                lambda msg, robot_id=inst: self._joint_state_callback(robot_id, msg),
                1,
                callback_group=self._cb_group,
            )
            self.get_logger().info(
                f'[init] {inst} (profile={prof.profile_name}): nav={nav_topic}, '
                f'moveit={ns_prefix}/move_action, tcp={prof.tcp_frame}'
            )
            worker = threading.Thread(
                target=self._robot_worker_loop,
                args=(inst,),
                name=f'{inst}_worker',
                daemon=True,
            )
            worker.start()
            self._worker_threads[inst] = worker

    def destroy_node(self) -> bool:
        """Stop per-robot worker threads cleanly before destroying the ROS node."""
        self._stop_event.set()
        for q in self._robot_queues.values():
            q.put(None)
        for worker in self._worker_threads.values():
            worker.join(timeout=1.0)
        return super().destroy_node()

    def _gz_attach(self, robot: str, object_name: str) -> bool:
        """Publish to Gazebo DynamicAttachSystem to weld `object_name` rigidly to the gripper."""
        pub = self._gz_attach_pubs.get(robot)
        if pub is None:
            return False
        msg = GzStringMsg()
        msg.data = object_name
        ok = bool(pub.publish(msg))
        time.sleep(0.05)
        ok = bool(pub.publish(msg)) or ok
        self._sim_sleep(0.15)
        return ok

    def _gz_detach(self, robot: str) -> bool:
        """Publish to Gazebo DynamicAttachSystem to remove the rigid grasp joint."""
        pub = self._gz_detach_pubs.get(robot)
        if pub is None:
            return False
        msg = GzEmpty()
        ok = bool(pub.publish(msg))
        time.sleep(0.05)
        ok = bool(pub.publish(msg)) or ok
        self._sim_sleep(0.25)
        return ok

    def _is_large_fixture(self, entity_name: str) -> bool:
        return any(
            entity_name.startswith(p)
            for p in ('Packing_Station', 'Picking_Shelves', 'Pallet_Standard')
        )

    def _lookup_frame_in_map(self, frame_id: str) -> Optional[PoseEntry]:
        """Look up `frame_id` in world `map` coordinates via TF2."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
        except Exception:
            return None
        tr = tf.transform.translation
        rot = tf.transform.rotation
        return (float(tr.x), float(tr.y), float(tr.z), (rot.x, rot.y, rot.z, rot.w))

    def _get_tcp_world_pose(
        self,
        robot: str,
        offset_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Optional[PoseEntry]:
        """Return world `(x, y, z, quat)` of `robot`'s `tcp_frame` + local `offset_xyz`."""
        prof = self._profiles[robot]
        tcp_pose = self._lookup_frame_in_map(prof.tcp_frame)
        if tcp_pose is None:
            return None
        px, py, pz, quat = tcp_pose
        wx, wy, wz = transform_base_to_world(
            offset_xyz[0], offset_xyz[1], offset_xyz[2], (px, py, pz), quat
        )
        return (wx, wy, wz, quat)

    def _confirm_object_z(
        self,
        robot: str,
        object_name: str,
        min_z: float,
        timeout_sec: float = 3.0,
    ) -> Optional[float]:
        """Wait until the bridged Gazebo pose of `object_name` reaches `min_z`."""
        deadline = time.monotonic() + timeout_sec
        z_val: Optional[float] = None
        while True:
            pose = self._resolve_target_pose(object_name)
            z_val = pose[2] if pose is not None else None
            if z_val is not None and z_val >= min_z:
                return z_val
            if time.monotonic() >= deadline:
                return z_val
            time.sleep(0.2)

    def _load_occupancy_map(self, pkg_name: str, rel_path: str) -> Optional[OccupancyMap]:
        try:
            map_yaml = os.path.join(get_package_share_directory(pkg_name), rel_path)
            if os.path.isfile(map_yaml):
                self.get_logger().info(f'[init] Loaded standoff map from {map_yaml}')
                return OccupancyMap(map_yaml)
        except Exception as exc:
            self.get_logger().warn(f'[init] Could not load static standoff map: {exc}')
        return None

    def _pose_callback(self, msg: TFMessage) -> None:
        with self._pose_lock:
            for transform in msg.transforms:
                name = transform.child_frame_id
                if not name:
                    continue
                tr = transform.transform.translation
                rot = transform.transform.rotation
                self._pose_registry[name] = (tr.x, tr.y, tr.z, (rot.x, rot.y, rot.z, rot.w))

    def _joint_state_callback(self, robot: str, msg: JointState) -> None:
        with self._js_lock:
            table = self._joint_positions[robot]
            for name, pos in zip(msg.name, msg.position):
                table[name] = float(pos)

    def _get_joint_positions(self, robot: str) -> Dict[str, float]:
        with self._js_lock:
            return dict(self._joint_positions.get(robot, {}))

    def _sim_sleep(self, sim_sec: float) -> None:
        if sim_sec <= 0.0:
            return
        start_ns = self.get_clock().now().nanoseconds
        if start_ns <= 0:
            time.sleep(sim_sec)
            return
        target_ns = start_ns + int(sim_sec * 1e9)
        wall_deadline = time.monotonic() + max(15.0, sim_sec * 18.0)
        while self.get_clock().now().nanoseconds < target_ns and time.monotonic() < wall_deadline:
            time.sleep(0.05)

    def _get_robot_pose(self, robot: str) -> Optional[PoseEntry]:
        prof = self._profiles[robot]
        with self._pose_lock:
            return (
                self._pose_registry.get(prof.gazebo_model_name)
                or self._pose_registry.get(f'{prof.gazebo_model_name}::base_link')
            )

    def _resolve_target_pose(self, entity_name: str) -> Optional[PoseEntry]:
        """Resolve a physical Gazebo entity pose from `/world/<world>/pose/info`."""
        deadline = time.monotonic() + 1.5
        while True:
            with self._pose_lock:
                entry = self._pose_registry.get(entity_name)
            if entry is not None:
                return entry
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)

        with self._pose_lock:
            sample = sorted(self._pose_registry.keys())[:10]
        self.get_logger().error(
            f"[pose] '{entity_name}' not found in /world/{self._world_name}/pose/info. "
            f'Sample entities: {sample}'
        )
        return None

    def _wait_future(self, future: Any, timeout_sec: float) -> bool:
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        start_ns = self.get_clock().now().nanoseconds
        if start_ns <= 0:
            return done.wait(timeout=timeout_sec)
        target_ns = start_ns + int(timeout_sec * 1e9)
        wall_deadline = time.monotonic() + max(30.0, timeout_sec * 10.0)
        while time.monotonic() < wall_deadline:
            if done.wait(timeout=0.05):
                return True
            if self.get_clock().now().nanoseconds >= target_ns:
                break
        return done.is_set()

    def _subtask_callback(self, msg: String) -> None:
        try:
            subtask = json.loads(msg.data.strip())
        except Exception as exc:
            self.get_logger().error(f'[subtask] Unhandled exception: {exc}')
            return

        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        robot = subtask.get('robot', '')
        skills = subtask.get('required_skills', [])

        if robot not in self._profiles and not any(
            robot.startswith(prefix) for prefix in self._managed_prefixes
        ):
            return

        if robot not in self._profiles:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='config_error',
                detail=f'Unknown robot instance: {robot}',
            )
            return

        if not (skills[0] if skills else None):
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='config_error',
                detail='No required_skills specified.',
            )
            return

        self._robot_queues[robot].put(subtask)

    def _robot_worker_loop(self, robot: str) -> None:
        q = self._robot_queues[robot]
        while not self._stop_event.is_set():
            try:
                subtask = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if subtask is None:
                q.task_done()
                break
            try:
                self._handle_subtask(subtask)
            except Exception as exc:
                self.get_logger().error(f'[{robot}] Worker unhandled exception: {exc}')
            finally:
                q.task_done()

    def _handle_subtask(self, subtask: Dict[str, Any]) -> None:
        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        robot = subtask.get('robot', '')
        skills = subtask.get('required_skills', [])
        args = subtask.get('args', {})

        if robot not in self._profiles and not any(
            robot.startswith(prefix) for prefix in self._managed_prefixes
        ):
            return

        if robot not in self._profiles:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='config_error',
                detail=f'Unknown robot instance: {robot}',
            )
            return

        skill = skills[0] if skills else None
        if not skill:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='config_error',
                detail='No required_skills specified.',
            )
            return

        if skill == 'navigate_to':
            self._execute_navigate_to(plan_id, task_id, robot, args)
        elif skill == 'pick':
            self._execute_pick(plan_id, task_id, robot, args)
        elif skill == 'place':
            self._execute_place(plan_id, task_id, robot, args)
        else:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='config_error',
                detail=f'Unknown skill: {skill}',
            )

    def _compute_nav_goal(
        self,
        robot: str,
        tx: float,
        ty: float,
        target_yaw: float,
        preferred_dist: float,
        max_standoff_dist: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        prof = self._profiles[robot]
        robot_pose = self._get_robot_pose(robot)
        rx = robot_pose[0] if robot_pose is not None else None
        ry = robot_pose[1] if robot_pose is not None else None

        if self._occ_map is not None:
            return self._occ_map.find_standoff_pose(
                tx, ty, target_yaw, rx, ry, preferred_dist,
                prof.robot_clearance_radius, max_standoff_dist=max_standoff_dist,
            )

        approach = (
            math.atan2(ry - ty, rx - tx)
            if rx is not None and ry is not None
            else (target_yaw + math.pi)
        )
        gx = tx + preferred_dist * math.cos(approach)
        gy = ty + preferred_dist * math.sin(approach)
        return gx, gy, math.atan2(ty - gy, tx - gx)

    def _send_nav_goal(
        self,
        robot: str,
        gx: float,
        gy: float,
        gyaw: float,
        task_id: str,
    ) -> Tuple[bool, str]:
        prof = self._profiles[robot]
        nav_client = self._nav_clients[robot]
        if not nav_client.wait_for_server(timeout_sec=5.0):
            return False, 'Nav2 action server not available.'

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quaternion(gyaw)

        send_future = nav_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=10.0):
            return False, f'Timed out sending Nav2 goal for {task_id}.'

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, f'Nav2 rejected goal ({gx:.2f}, {gy:.2f}) for {task_id}.'

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=prof.nav_timeout_sec):
            goal_handle.cancel_goal_async()
            return False, f'Nav2 timed out after {prof.nav_timeout_sec}s for {task_id}.'

        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = wrapped.status if wrapped is not None else -1
            return False, f'Nav2 failed with status {status}.'
        return True, ''

    def _execute_navigate_to(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        location = args.get('location')
        if not location:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='navigate_to', detail="Missing 'location' in args.",
            )
            return

        pose = self._resolve_target_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot,
                status='failed', stage='navigate_to',
                detail=f"Entity '{location}' not found in /world/{self._world_name}/pose/info.",
            )
            return

        prof = self._profiles[robot]
        tx, ty, _, quat = pose
        target_yaw = quaternion_to_yaw(*quat)
        is_fixture = self._is_large_fixture(location)
        pref_dist = prof.standoff_distance if is_fixture else prof.manipulation_standoff
        gx, gy, gyaw = self._compute_nav_goal(
            robot, tx, ty, target_yaw, pref_dist,
            max_standoff_dist=None if is_fixture else prof.max_reach_m,
        )

        self.get_logger().info(
            f"[navigate_to] {task_id} -> '{location}' at ({tx:.2f}, {ty:.2f}) "
            f'goal=({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})'
        )
        self._publish_feedback(
            plan_id, task_id, robot,
            status='in_progress', stage='navigating', detail=f"Navigating to '{location}'.",
        )

        ok, err = self._send_nav_goal(robot, gx, gy, gyaw, task_id)
        if not ok:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='navigate_to', detail=err
            )
            return

        self._publish_feedback(
            plan_id, task_id, robot,
            status='success', stage='complete', detail=f"Reached '{location}'.",
        )

    def _drive_pulse(
        self, robot: str, vx: float, wz: float, sim_dur: float, vy: float = 0.0
    ) -> None:
        pub = self._cmd_vel_pubs[robot]
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        start_ns = self.get_clock().now().nanoseconds
        target_ns = start_ns + int(sim_dur * 1e9) if start_ns > 0 else 0
        wall_deadline = time.monotonic() + max(10.0, sim_dur * 18.0)
        while time.monotonic() < wall_deadline:
            pub.publish(cmd)
            if target_ns > 0:
                if self.get_clock().now().nanoseconds >= target_ns:
                    break
            elif time.monotonic() >= (wall_deadline - max(10.0, sim_dur * 18.0) + sim_dur):
                break
            time.sleep(0.03)
        pub.publish(Twist())
        self._sim_sleep(0.9)

    def _ensure_aligned(
        self,
        robot: str,
        tx: float,
        ty: float,
        task_id: str,
        max_dist: float,
        preferred_standoff: float,
        tight: bool = True,
    ) -> Tuple[bool, str]:
        """Reposition `robot` so `(tx, ty)` sits inside its reach and alignment window."""
        prof = self._profiles[robot]
        robot_pose = self._get_robot_pose(robot)
        if robot_pose is None:
            return False, f'Robot pose unavailable in /world/{self._world_name}/pose/info.'

        rx, ry, rz, rquat = robot_pose
        bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
        horiz = math.hypot(bx, by)

        if horiz > 1.25 or horiz < prof.min_reach_m or bx < 0.25:
            target_yaw = math.atan2(ty - ry, tx - rx)
            gx, gy, gyaw = self._compute_nav_goal(
                robot, tx, ty, target_yaw, preferred_standoff, max_standoff_dist=max_dist
            )
            self.get_logger().info(
                f'[{task_id}] Target in base_link ({bx:.2f}, {by:.2f}) far; '
                f'navigating to ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})'
            )
            ok, err = self._send_nav_goal(robot, gx, gy, gyaw, f'{task_id}_align')
            if not ok:
                return False, f'Pre-manipulation alignment failed: {err}'
            self._sim_sleep(0.5)

        if prof.fine_align_pulses:
            dist_tol = 0.085 if tight else 0.12
            lat_tol = 0.055 if tight else 0.16
            yaw_tol = 0.070 if tight else 0.14
            for step in range(12):
                robot_pose = self._get_robot_pose(robot)
                if robot_pose is None:
                    return False, 'Robot pose lost during fine alignment.'
                rx, ry, rz, rquat = robot_pose
                bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
                ryaw = quaternion_to_yaw(*rquat)
                target_yaw = math.atan2(ty - ry, tx - rx)
                yaw_err = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))
                ex = bx - preferred_standoff
                if abs(ex) <= dist_tol and abs(by) <= lat_tol and abs(yaw_err) <= yaw_tol:
                    break
                dur = max(
                    prof.min_pulse_duration_sec,
                    min(1.35, max(abs(ex) / 0.18, abs(yaw_err) / 0.28)),
                )
                vx = 0.0
                if ex > dist_tol or (ex > 0.035 and abs(yaw_err) > yaw_tol):
                    vx = min(0.22, max(0.12, ex / dur))
                elif ex < -dist_tol:
                    vx = -min(0.12, max(0.08, -ex / dur))
                vy = max(-0.08, min(0.08, by / dur)) if abs(by) > lat_tol else 0.0
                wz = max(-0.28, min(0.28, yaw_err / dur)) if abs(yaw_err) > yaw_tol else 0.0
                self._drive_pulse(robot, vx, wz, dur, vy=vy)
                self.get_logger().info(
                    f'[{task_id}] align pulse {step}: '
                    f'base=({bx:.3f}, {by:.3f}) yaw_err={yaw_err:.3f}'
                )
            self._cmd_vel_pubs[robot].publish(Twist())
            self._sim_sleep(1.0)

        robot_pose = self._get_robot_pose(robot)
        if robot_pose is None:
            return False, 'Robot pose lost after alignment.'
        rx, ry, rz, rquat = robot_pose
        bx, by, _ = transform_world_to_base(tx, ty, rz, (rx, ry, rz), rquat)
        horiz = math.hypot(bx, by)
        max_lat = 0.10 if tight else 0.22
        max_horiz = max_dist + 0.06 if tight else max_dist + 0.15
        if bx <= 0.25 or horiz < prof.min_reach_m or horiz > max_horiz or abs(by) > max_lat:
            return False, (
                f'Alignment outside manipulation window: '
                f'base=({bx:.3f}, {by:.3f}), horiz={horiz:.3f}'
            )
        return True, ''

    def _to_robot_base(
        self, robot: str, wx: float, wy: float, wz: float
    ) -> Optional[Tuple[float, float, float]]:
        robot_pose = self._get_robot_pose(robot)
        if robot_pose is None:
            return None
        rx, ry, rz, rquat = robot_pose
        return transform_world_to_base(wx, wy, wz, (rx, ry, rz), rquat)

    def _infer_surface_z(self, location_name: str, default_z: float) -> float:
        for prefix, surf_z in self._fixture_surface_z.items():
            if location_name.startswith(prefix):
                return surf_z
        return default_z + 0.12

    def _add_nearby_fixture_obstacle(
        self,
        robot: str,
        target_x: float,
        target_y: float,
        preferred_fixture: Optional[str] = None,
    ) -> Optional[str]:
        """Register the closest known support fixture box in MoveIt's PlanningScene."""
        best_name: Optional[str] = None
        best_dims: Optional[Tuple[float, float, float]] = None
        best_pose: Optional[PoseEntry] = None
        best_dist = 1.5

        with self._pose_lock:
            snapshot = list(self._pose_registry.items())

        for entity_name, pose_entry in snapshot:
            if preferred_fixture and entity_name != preferred_fixture and best_name is not None:
                continue
            dims = next(
                (d for prefix, d in self._fixture_dims.items() if entity_name.startswith(prefix)),
                None,
            )
            if dims is None:
                continue
            dist = math.hypot(pose_entry[0] - target_x, pose_entry[1] - target_y)
            if entity_name == preferred_fixture or dist < best_dist:
                best_name = entity_name
                best_dims = dims
                best_pose = pose_entry
                best_dist = dist if entity_name != preferred_fixture else -1.0

        if best_name is None or best_dims is None or best_pose is None:
            return None

        scene_id = f'fixture_{best_name}'
        center_z = 0.5 * best_dims[2]
        if self._scene_managers[robot].add_box(
            scene_id,
            best_dims,
            (best_pose[0], best_pose[1], center_z),
            orientation=best_pose[3],
            frame_id='map',
        ):
            return scene_id
        return None

    def _open_gripper(self, robot: str, label: str, pause_sim: float = 0.3) -> bool:
        prof = self._profiles[robot]
        moveit = self._moveit_clients[robot]
        if prof.gripper_mode == 'gripper_command':
            ok = moveit.move_gripper_command(
                prof.gripper_open_position, prof.gripper_max_effort, label
            )
        else:
            state_name = prof.srdf_states.get('gripper_open', 'open')
            ok = moveit.move_to_named_state(
                prof.gripper_group_name, state_name, label,
                tol=prof.gripper_open_tolerance, vel_scale=0.5,
            )
        if ok:
            self._sim_sleep(pause_sim)
        return ok

    def _close_gripper(self, robot: str, label: str, for_attach: bool = False) -> bool:
        prof = self._profiles[robot]
        moveit = self._moveit_clients[robot]
        if prof.gripper_mode == 'gripper_command':
            ok = moveit.move_gripper_command(
                prof.gripper_closed_position, prof.gripper_max_effort, label
            )
        else:
            key = 'gripper_attach' if for_attach else 'gripper_close'
            default_state = 'attach' if for_attach else 'close'
            state_name = prof.srdf_states.get(key, default_state)
            tol = prof.gripper_open_tolerance if for_attach else prof.gripper_closed_tolerance
            ok = moveit.move_to_named_state(
                prof.gripper_group_name, state_name, label, tol=tol, vel_scale=0.4
            )
        if ok:
            self._sim_sleep(1.0 if for_attach else 2.0)
        return ok

    def _move_arm_state(
        self,
        robot: str,
        state_key: str,
        label: str,
        vel_scale: float = 0.4,
        pause_sim: float = 0.3,
    ) -> bool:
        prof = self._profiles[robot]
        state_name = prof.srdf_states.get(state_key)
        if not state_name:
            return True
        ok = self._moveit_clients[robot].move_to_named_state(
            prof.move_group_name,
            state_name,
            label,
            tol=prof.arm_joint_tolerance,
            vel_scale=vel_scale,
        )
        if ok:
            self._sim_sleep(pause_sim)
        return ok

    def _make_base_pose_stamped(
        self,
        prof: RobotProfile,
        bx: float,
        by: float,
        bz: float,
        roll: float = 0.0,
        pitch: float = 0.0,
    ) -> PoseStamped:
        aim_yaw = math.atan2(by, max(bx - 0.15, 0.05))
        qx, qy, qz, qw = rpy_to_quat_tuple(roll, pitch, aim_yaw)
        ps = PoseStamped()
        ps.header.frame_id = prof.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(bx)
        ps.pose.position.y = float(by)
        ps.pose.position.z = float(bz)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _move_arm_to_pose(
        self,
        robot: str,
        pose_stamped: PoseStamped,
        label: str,
        vel_scale: float = 0.35,
        pause_sim: float = 0.3,
        cartesian: bool = False,
    ) -> bool:
        prof = self._profiles[robot]
        moveit = self._moveit_clients[robot]
        cur_joints = self._get_joint_positions(robot)

        if cartesian:
            if moveit.execute_cartesian_path(
                prof.move_group_name, prof.tcp_frame, [pose_stamped.pose],
                prof.base_frame, label, cur_joints, vel_scale=vel_scale,
            ):
                self._sim_sleep(pause_sim)
                return True

        pre_fold_name = prof.srdf_states.get('pre_fold')
        seed = (
            moveit.get_group_state(prof.move_group_name, pre_fold_name)
            if pre_fold_name
            else None
        )
        if moveit.move_to_pose(
            prof.move_group_name, prof.tcp_frame, pose_stamped, label,
            cur_joints, seed_overrides=seed, vel_scale=vel_scale,
            arm_tol=prof.arm_joint_tolerance,
        ):
            self._sim_sleep(pause_sim)
            return True
        return False

    def _clear_attachment(self, robot: str, object_name: str) -> None:
        prof = self._profiles[robot]
        with self._attach_lock:
            if self._attached_objects.get(robot) == object_name:
                self._attached_objects[robot] = None
        if prof.use_kinematic_attach:
            self._gz_detach(robot)
            self._scene_managers[robot].detach_object(object_name, link_name=prof.tcp_frame)
            self._scene_managers[robot].remove_object(object_name)

    def _attach_kinematic_object(self, robot: str, object_name: str) -> Optional[str]:
        prof = self._profiles[robot]
        tcp_world = self._get_tcp_world_pose(robot)
        obj_pose = self._resolve_target_pose(object_name)
        if tcp_world is None or obj_pose is None:
            return f"Unable to verify attach proximity for '{object_name}'."
        dist = math.dist(tcp_world[:3], obj_pose[:3])
        if dist > prof.attach_max_distance_m:
            return (
                f"Gripper too far from '{object_name}' to attach "
                f'({dist:.3f}m > {prof.attach_max_distance_m:.3f}m).'
            )

        base_now = self._to_robot_base(robot, *obj_pose[:3])
        psm = self._scene_managers[robot]
        if base_now is None:
            self.get_logger().warn(
                f"[attach] Robot base pose unavailable; '{object_name}' not added to MoveIt scene."
            )
        elif not psm.add_box(
            object_name, prof.default_object_dims, base_now, frame_id=prof.base_frame
        ):
            self.get_logger().warn(f"[attach] MoveIt add_box failed for '{object_name}'.")
        elif not psm.attach_object(object_name, link_name=prof.tcp_frame):
            self.get_logger().warn(f"[attach] MoveIt attach_object failed for '{object_name}'.")

        if not self._gz_attach(robot, object_name):
            return f"Failed to publish Gazebo rigid attach for '{object_name}'."

        with self._attach_lock:
            self._attached_objects[robot] = object_name
        return None

    def _execute_pick(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        object_name = args.get('object')
        if not object_name:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick',
                detail="Missing 'object' in args.",
            )
            return

        pose = self._resolve_target_pose(object_name)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick',
                detail=f"Object '{object_name}' not found in /world/{self._world_name}/pose/info.",
            )
            return

        prof = self._profiles[robot]
        wx, wy, _, _ = pose
        self._publish_feedback(
            plan_id, task_id, robot, status='in_progress', stage='pick_in_progress',
            detail=f"Picking '{object_name}'.",
        )

        ok, err = self._ensure_aligned(
            robot, wx, wy, task_id, prof.max_reach_m, prof.manipulation_standoff, tight=True
        )
        if not ok:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick', detail=err
            )
            return

        pose = self._resolve_target_pose(object_name)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick',
                detail=f"Lost pose of '{object_name}'.",
            )
            return
        wx, wy, pre_pick_z, _ = pose
        base_pt = self._to_robot_base(robot, wx, wy, pre_pick_z)
        if base_pt is None:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='pick',
                detail='Robot base pose unavailable.',
            )
            return

        fixture_scene_id = self._add_nearby_fixture_obstacle(robot, wx, wy)
        try:
            if not (
                self._open_gripper(robot, f'{task_id}_open')
                and self._move_arm_state(robot, 'pre_fold', f'{task_id}_pre_fold', vel_scale=0.5)
            ):
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=f"MoveIt 2 failed during pick preparation for '{object_name}'.",
                )
                return

            # Re-sample object pose after B1 settles and arm is in pre_fold.
            pose_now = self._resolve_target_pose(object_name)
            base_pt_now = (
                self._to_robot_base(robot, pose_now[0], pose_now[1], pose_now[2])
                if pose_now is not None
                else None
            )
            if base_pt_now is not None:
                bx, by, bz = base_pt_now
                pre_pick_z = pose_now[2]

            aim_yaw = math.atan2(by, max(bx - 0.15, 0.05))
            lat_dx = -prof.grasp_lateral_offset_m * math.sin(aim_yaw)
            lat_dy = prof.grasp_lateral_offset_m * math.cos(aim_yaw)
            pre_grasp_pose = self._make_base_pose_stamped(
                prof,
                bx - 0.025 + lat_dx,
                by + lat_dy,
                bz + 0.15,
                prof.grasp_roll_rad,
                prof.grasp_pitch_rad,
            )

            if not self._move_arm_to_pose(
                robot, pre_grasp_pose, f'{task_id}_pre_grasp', vel_scale=0.4
            ):
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=f"MoveIt 2 failed during pre-grasp for '{object_name}'.",
                )
                return

            # Re-sample again after pre_grasp extends the arm (COM pitch shift settled).
            pose_now = self._resolve_target_pose(object_name)
            base_pt_now = (
                self._to_robot_base(robot, pose_now[0], pose_now[1], pose_now[2])
                if pose_now is not None
                else None
            )
            if base_pt_now is not None:
                bx, by, bz = base_pt_now
                pre_pick_z = pose_now[2]
                aim_yaw = math.atan2(by, max(bx - 0.15, 0.05))
                lat_dx = -prof.grasp_lateral_offset_m * math.sin(aim_yaw)
                lat_dy = prof.grasp_lateral_offset_m * math.cos(aim_yaw)

            grasp_pose = self._make_base_pose_stamped(
                prof,
                bx - 0.025 + lat_dx,
                by + lat_dy,
                bz + 0.022,
                prof.grasp_roll_rad,
                prof.grasp_pitch_rad,
            )
            lift_pose = self._make_base_pose_stamped(
                prof,
                bx - 0.045 + lat_dx,
                by + lat_dy,
                bz + 0.22,
                prof.grasp_roll_rad,
                prof.grasp_pitch_rad,
            )

            if not self._move_arm_to_pose(
                robot,
                grasp_pose,
                f'{task_id}_grasp',
                vel_scale=0.25,
                pause_sim=0.4,
                cartesian=True,
            ):
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=f"MoveIt 2 failed during pick approach for '{object_name}'.",
                )
                return

            if prof.use_kinematic_attach:
                attach_err = self._attach_kinematic_object(robot, object_name)
                if attach_err is not None:
                    self._publish_feedback(
                        plan_id, task_id, robot, status='failed', stage='pick', detail=attach_err
                    )
                    return

            if not (
                self._close_gripper(
                    robot, f'{task_id}_close', for_attach=prof.use_kinematic_attach
                )
                and self._move_arm_to_pose(
                    robot,
                    lift_pose,
                    f'{task_id}_lift',
                    vel_scale=0.25,
                    pause_sim=0.5,
                    cartesian=True,
                )
            ):
                self._clear_attachment(robot, object_name)
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=f"MoveIt 2 failed during lift for '{object_name}'.",
                )
                return

            lift_z = self._confirm_object_z(robot, object_name, pre_pick_z + 0.08) or pre_pick_z
            if lift_z < pre_pick_z + 0.08:
                self._clear_attachment(robot, object_name)
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=(
                        f"Grasp verification failed for '{object_name}': "
                        f'z={lift_z:.3f}m vs {pre_pick_z:.3f}m'
                    ),
                )
                return

            if not (
                self._move_arm_state(
                    robot, 'carry_retract', f'{task_id}_carry_retract', vel_scale=0.25
                )
                and self._move_arm_state(
                    robot, 'carry', f'{task_id}_carry_cradle', vel_scale=0.15, pause_sim=0.5
                )
            ):
                self._clear_attachment(robot, object_name)
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=f"MoveIt 2 failed during carry tuck for '{object_name}'.",
                )
                return

            carry_z = self._confirm_object_z(robot, object_name, pre_pick_z + 0.08) or pre_pick_z
            if carry_z < pre_pick_z + 0.08:
                self._clear_attachment(robot, object_name)
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='pick',
                    detail=(
                        f"Carry verification failed for '{object_name}': "
                        f'dropped to z={carry_z:.3f}m'
                    ),
                )
                return
        finally:
            if fixture_scene_id:
                self._scene_managers[robot].remove_object(fixture_scene_id)

        self._publish_feedback(
            plan_id, task_id, robot, status='success', stage='complete',
            detail=f"Picked '{object_name}'.",
        )

    def _execute_place(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        args: Dict[str, Any],
    ) -> None:
        location = args.get('location')
        if not location:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='place',
                detail="Missing 'location' in args.",
            )
            return

        pose = self._resolve_target_pose(location)
        if pose is None:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='place',
                detail=f"Location '{location}' not found in /world/{self._world_name}/pose/info.",
            )
            return

        prof = self._profiles[robot]
        with self._attach_lock:
            held_object = self._attached_objects.get(robot)
        object_name = args.get('object') or held_object or 'object'

        lx, ly, lz, _ = pose
        self._publish_feedback(
            plan_id, task_id, robot, status='in_progress', stage='place_in_progress',
            detail=f"Placing '{object_name}' at '{location}'.",
        )

        large_fixture = self._is_large_fixture(location)
        max_dist = 1.02 if large_fixture else prof.max_reach_m
        pref_dist = prof.standoff_distance if large_fixture else prof.manipulation_standoff
        ok, err = self._ensure_aligned(
            robot, lx, ly, task_id, max_dist, pref_dist, tight=not large_fixture
        )
        if not ok:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='place', detail=err
            )
            return

        robot_pose = self._get_robot_pose(robot)
        if robot_pose is None:
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='place',
                detail='Robot base pose unavailable.',
            )
            return

        rx, ry, rz, rquat = robot_pose
        bx_raw, by_raw, _ = transform_world_to_base(lx, ly, lz, (rx, ry, rz), rquat)
        dist = math.hypot(bx_raw, by_raw)
        if dist > prof.place_drop_standoff and dist > 1e-4:
            scale = prof.place_drop_standoff / dist
            bx, by = bx_raw * scale, by_raw * scale
        else:
            bx, by = max(0.55, bx_raw - 0.03), by_raw

        surface_z = self._infer_surface_z(location, lz)
        drop_world_z = surface_z + prof.place_clear_z_offset
        bz = drop_world_z - rz

        aim_yaw = math.atan2(by, max(bx - 0.15, 0.05))
        lat_dx = -prof.grasp_lateral_offset_m * math.sin(aim_yaw)
        lat_dy = prof.grasp_lateral_offset_m * math.cos(aim_yaw)

        pre_place_pose = self._make_base_pose_stamped(
            prof, bx + lat_dx, by + lat_dy, bz + 0.16, prof.grasp_roll_rad, prof.grasp_pitch_rad
        )
        place_pose = self._make_base_pose_stamped(
            prof, bx + lat_dx, by + lat_dy, bz + 0.022, prof.grasp_roll_rad, prof.grasp_pitch_rad
        )
        retreat_pose = self._make_base_pose_stamped(
            prof, bx + lat_dx, by + lat_dy, bz + 0.18, prof.grasp_roll_rad, prof.grasp_pitch_rad
        )

        fixture_scene_id = self._add_nearby_fixture_obstacle(
            robot, lx, ly, preferred_fixture=location
        )
        try:
            descend_ok = (
                self._move_arm_to_pose(
                    robot, pre_place_pose, f'{task_id}_pre_place', vel_scale=0.30
                )
                and self._move_arm_to_pose(
                    robot, place_pose, f'{task_id}_place_descend',
                    vel_scale=0.25, pause_sim=0.3, cartesian=True,
                )
            )
            if not descend_ok:
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='place',
                    detail=f"MoveIt 2 failed during place approach at '{location}'.",
                )
                return

            if not self._open_gripper(robot, f'{task_id}_release', pause_sim=0.3):
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='place',
                    detail=f"MoveIt 2 failed to open gripper during place at '{location}'.",
                )
                return

            if prof.use_kinematic_attach and object_name != 'object':
                with self._attach_lock:
                    if self._attached_objects.get(robot) == object_name:
                        self._attached_objects[robot] = None
                self._gz_detach(robot)
                self._scene_managers[robot].detach_object(object_name, link_name=prof.tcp_frame)
                self._scene_managers[robot].remove_object(object_name)
                self._sim_sleep(0.35)

            if not (
                self._move_arm_to_pose(
                    robot, retreat_pose, f'{task_id}_retreat', vel_scale=0.35, cartesian=True
                )
                and self._move_arm_state(
                    robot, 'home', f'{task_id}_stow', vel_scale=0.4, pause_sim=0.5
                )
            ):
                self._publish_feedback(
                    plan_id, task_id, robot, status='failed', stage='place',
                    detail=f"MoveIt 2 failed during release/retreat at '{location}'.",
                )
                return
        finally:
            if fixture_scene_id:
                self._scene_managers[robot].remove_object(fixture_scene_id)

        if object_name != 'object':
            with self._pose_lock:
                obj_pose = self._pose_registry.get(object_name)
            if obj_pose is not None:
                ox, oy, oz, _ = obj_pose
                horiz_err = math.hypot(ox - lx, oy - ly)
                self.get_logger().info(
                    f'[{task_id}] Post-place verify ({object_name}): '
                    f'obj=({ox:.3f}, {oy:.3f}, {oz:.3f}) '
                    f'vs target=({lx:.3f}, {ly:.3f}, surf_z={surface_z:.3f}) '
                    f'horiz_err={horiz_err:.3f}'
                )
                if horiz_err > 1.25 or oz < surface_z - 0.05:
                    self._publish_feedback(
                        plan_id, task_id, robot, status='failed', stage='place',
                        detail=(
                            f"Place verification failed for '{object_name}' at '{location}': "
                            f'obj=({ox:.3f}, {oy:.3f}, {oz:.3f})'
                        ),
                    )
                    return

        self._publish_feedback(
            plan_id, task_id, robot, status='success', stage='complete',
            detail=f"Placed '{object_name}' at '{location}'.",
        )

    def _publish_feedback(
        self,
        plan_id: str,
        task_id: str,
        robot: str,
        status: str,
        stage: str,
        detail: str = '',
    ) -> None:
        payload = json.dumps({
            'plan_id': plan_id,
            'task_id': task_id,
            'robot': robot,
            'status': status,
            'stage': stage,
            'detail': detail,
        })
        msg = String()
        msg.data = payload
        self._feedback_pub.publish(msg)
        self.get_logger().info(
            f'[feedback] task={task_id} status={status} stage={stage} detail={detail}'
        )


def main(args: Any = None) -> None:
    """Run the TEMPO multi-robot controller node."""
    rclpy.init(args=args)
    node = MobileRobotController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
