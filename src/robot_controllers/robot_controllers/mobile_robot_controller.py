#!/usr/bin/env python3
"""
Mobile robot controller node for TEMPO.

ROS 2 controller node for the TEMPO mobile manipulator
(Clearpath Husky A200 + UR5e + Robotiq 2F-85).

Subscribes to /mission/subtask (std_msgs/String, JSON), checks whether the
assigned robot matches one of this controller's instances, and executes the
skill through the robot's ROS 2 interfaces:

  navigate_to -> Nav2 NavigateToPose (to a standoff pose in front of the target)
  pick        -> open gripper, plan to a pre-grasp pose, move straight in,
                 secure the object (snap it into the gripper if the arm is
                 off, then attach it), close gripper, lift and retreat,
                 back the base off, move to the carry pose; the hold is
                 checked and re-established if needed
  place       -> plan to a pre-place pose, move straight down, detach the
                 object and set it exactly on the release point, open
                 gripper, retreat, stow the arm; the result is checked and
                 corrected if needed

Holding does not rely on the fingers: the object is fixed to the wrist with
a Gazebo DetachableJoint, and Gazebo's set_pose puts it exactly where it
belongs (in the gripper when picking, on the release point when placing), so
an imprecise arm motion does not make a pick or place fail.

The gripper approaches horizontally (fingers closing horizontally); straight
moves use MoveIt's Cartesian path service so the gripper cannot sweep through
the object on the way in.

Entity poses (objects, locations and the robot itself) come from the Gazebo
world pose topic /world/<world>/pose/info, read directly over gz-transport:
ros_gz_bridge's Pose_V -> TFMessage conversion drops the entity names. The
maps used by Nav2 are in Gazebo world coordinates, so world poses are valid
map poses.

Grasping: the Robotiq 2F-85 opens to 85 mm, while the warehouse boxes are
200-300 mm wide, and friction grasps are unreliable in Gazebo anyway. The
held object is therefore also attached to the wrist with a Gazebo
DetachableJoint system, added to the robot model at pick time through the
world's /entity/system/add service, and released at place time through the
system's detach topic (manipulation.attach in the config).

Action timeouts run on the node clock (simulation time with use_sim_time), so
a simulation running slower than real time does not cause early timeouts.

Topics / Actions
----------------
Subscribed:
  /mission/subtask                              (std_msgs/String)
  /world/<world>/pose/info                      (gz.msgs.Pose_V, gz-transport)

Published:
  /mission/feedback                             (std_msgs/String)

Action Clients:
  /<namespace>/navigate_to_pose                 (nav2_msgs/NavigateToPose)
  /<namespace>/move_action                      (moveit_msgs/MoveGroup)
  /<namespace>/execute_trajectory               (moveit_msgs/ExecuteTrajectory)
  /<namespace>/<gripper_action>                 (control_msgs/GripperCommand)

Service Clients:
  /<namespace>/compute_cartesian_path           (moveit_msgs/GetCartesianPath)

  /<namespace>/compute_fk                       (moveit_msgs/GetPositionFK)

Gazebo services / topics (only when manipulation.attach.enabled):
  /world/<world>/entity/system/add              (gz.msgs.EntityPlugin_V)
  /world/<world>/set_pose                       (gz.msgs.Pose)
  /<namespace>/attach/<object>/{attach,detach}  (gz.msgs.Empty)
  /model/<object>/detachable_joint/state        (gz.msgs.StringMsg)
"""

import json
import math
import os
import re
import threading
from typing import Any, Dict, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TwistStamped, Vector3
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionFK,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import yaml


ROBOT_TYPE = 'mobile_robot'

Vec3 = Tuple[float, float, float]
EntityPose = Tuple[float, float, float, float]  # x, y, z, yaw


# Config loader

def _load_config() -> Dict[str, Any]:
    """Load controller_config.yaml from the package share directory."""
    share_dir = get_package_share_directory('robot_controllers')
    config_path = os.path.join(share_dir, 'config', 'controller_config.yaml')
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _lookup_by_prefix(table: Dict[str, Any], name: str, default: Any) -> Any:
    """
    Return the value of the longest key in `table` that prefixes `name`.

    Gazebo copies of a model are named '<model>', '<model>.001', '<model>_000_1'
    and so on, so per-model settings are keyed by name prefix.
    """
    best = None
    for key in table or {}:
        if name.startswith(key) and (best is None or len(key) > len(best)):
            best = key
    return table[best] if best is not None else default


def _model_prefix(name: str) -> str:
    """Strip Gazebo copy suffixes: 'small_box.027_2' -> 'small_box'."""
    return re.sub(r'([._]\d+)+$', '', name)


# Pose helpers

def _apply_standoff(
    x: float,
    y: float,
    yaw: float,
    standoff: float,
    robot_xy: Optional[Tuple[float, float]] = None,
    axis: str = 'local_y',
    blockers: Sequence[Tuple[float, float]] = (),
    clearance: float = 0.0,
) -> Tuple[float, float, float]:
    """
    Compute an approach pose `standoff` metres from the target.

    axis:
      local_y / local_x: the offset is along the target's local Y (or X) axis,
          on whichever side is closer to the robot (+ side if the robot pose is
          unknown). Picking_Shelves are open along their local +/-Y axis.
          A side is skipped if one of `blockers` (x, y) lies within
          `clearance` of it, e.g. the back-to-back twin of a shelf.
      radial: along the line from the target towards the robot (for small
          targets that can be approached from any side).

    Returns (goal_x, goal_y, heading), where heading faces the target.
    """
    if axis == 'radial' and robot_xy is not None:
        dx, dy = robot_xy[0] - x, robot_xy[1] - y
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            goal_x = x + standoff * dx / dist
            goal_y = y + standoff * dy / dist
            return goal_x, goal_y, math.atan2(y - goal_y, x - goal_x)
        axis = 'local_y'

    if axis == 'local_x':
        ux, uy = math.cos(yaw), math.sin(yaw)
    else:
        ux, uy = -math.sin(yaw), math.cos(yaw)  # target's local +Y in world
    candidates = [
        (x + standoff * ux, y + standoff * uy),
        (x - standoff * ux, y - standoff * uy),
    ]
    if robot_xy is not None:
        candidates.sort(
            key=lambda c: math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1])
        )
    free = [c for c in candidates
            if all(math.hypot(c[0] - bx, c[1] - by) >= clearance for bx, by in blockers)]
    if free:
        candidates = free
    goal_x, goal_y = candidates[0]
    heading = math.atan2(y - goal_y, x - goal_x)
    return goal_x, goal_y, heading


def _quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from a quaternion (valid for tilted entities too)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _world_to_robot(robot_pose: EntityPose, point: Vec3) -> Vec3:
    """Express a world point in the frame of a robot at robot_pose (x, y, z, yaw)."""
    rx, ry, rz, ryaw = robot_pose
    dx, dy, dz = point[0] - rx, point[1] - ry, point[2] - rz
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
    return cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy, dz


def _robot_to_world(robot_pose: EntityPose, point: Vec3) -> Vec3:
    """Express a point given in the robot's base_link frame in world coordinates."""
    c, s = math.cos(robot_pose[3]), math.sin(robot_pose[3])
    return (robot_pose[0] + c * point[0] - s * point[1],
            robot_pose[1] + s * point[0] + c * point[1],
            robot_pose[2] + point[2])


def _horizontal_approach(origin: Vec3, target: Vec3) -> Tuple[float, float]:
    """Return the unit XY vector from origin to target (+X if they coincide)."""
    dx, dy = target[0] - origin[0], target[1] - origin[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 1.0, 0.0
    return dx / dist, dy / dist


def _tool_orientation(ax: float, ay: float) -> Quaternion:
    """
    Orientation of the tool frame for a horizontal approach along (ax, ay).

    tool z = approach (the gripper points at the object), tool y = up, so the
    Robotiq fingers (which close along tool x) close horizontally.
    Rotation matrix columns: x = (-ay, ax, 0), y = (0, 0, 1), z = (ax, ay, 0).
    """
    m = [
        [-ay, 0.0, ax],
        [ax, 0.0, ay],
        [0.0, 1.0, 0.0],
    ]
    return _matrix_to_quaternion(m)


def _matrix_to_quaternion(m: Sequence[Sequence[float]]) -> Quaternion:
    """Convert a 3x3 rotation matrix to a geometry_msgs Quaternion."""
    q = Quaternion()
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q.w = 0.25 * s
        q.x = (m[2][1] - m[1][2]) / s
        q.y = (m[0][2] - m[2][0]) / s
        q.z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q.w = (m[2][1] - m[1][2]) / s
        q.x = 0.25 * s
        q.y = (m[0][1] + m[1][0]) / s
        q.z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q.w = (m[0][2] - m[2][0]) / s
        q.x = (m[0][1] + m[1][0]) / s
        q.y = 0.25 * s
        q.z = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q.w = (m[1][0] - m[0][1]) / s
        q.x = (m[0][2] + m[2][0]) / s
        q.y = (m[1][2] + m[2][1]) / s
        q.z = 0.25 * s
    return q


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, limit]."""
    return max(-limit, min(limit, value))


def _sanitize_topic_token(name: str) -> str:
    """Make an entity name usable as one gz-transport topic token."""
    return re.sub(r'[^A-Za-z0-9_]', '_', name)


# Mobile robot controller node

class MobileRobotController(Node):
    """
    Execute navigate_to, pick and place skills on a Husky A200 manipulator.

    Lifecycle
    ---------
    1. /mission/subtask arrives -> _subtask_callback
    2. Robot type and instance checked -> _handle_subtask
    3. Skill dispatched -> _execute_navigate_to / _execute_pick / _execute_place
       (one subtask at a time per robot instance)
    4. Feedback published -> _publish_feedback
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__('mobile_robot_controller')

        config = config if config is not None else _load_config()
        nav = config['navigation']
        manip = config['manipulation']

        self._robot_instances: Dict[str, str] = {
            k: '/' + v.strip('/') for k, v in config['robot_instances'].items()
        }

        # Navigation
        self._standoff: float = float(nav['standoff_distance'])
        self._standoffs: Dict[str, float] = nav.get('standoff_overrides') or {}
        self._approach_axes: Dict[str, str] = nav.get('approach_axes') or {}
        self._default_axis: str = nav.get('default_approach_axis', 'radial')
        self._nav_timeout: float = float(nav['nav_timeout_sec'])
        # A standoff side is blocked if another copy of the target's model is
        # closer to it than this fraction of the standoff distance
        self._blocker_clearance: float = float(nav.get('blocker_clearance', 0.75))
        fine = nav.get('fine_positioning') or {}
        self._fine_positioning: bool = bool(fine.get('enabled', False))
        self._fine_tolerance: float = float(fine.get('tolerance', 0.03))
        self._fine_yaw_tolerance: float = float(fine.get('yaw_tolerance', 0.05))
        self._fine_max_correction: float = float(fine.get('max_correction', 0.5))
        self._fine_max_linear: float = float(fine.get('max_linear_speed', 0.1))
        self._fine_max_angular: float = float(fine.get('max_angular_speed', 0.3))
        self._fine_timeout: float = float(fine.get('timeout_sec', 60.0))

        # Manipulation
        self._move_group_name: str = manip['move_group_name']
        self._end_effector_link: str = manip['end_effector_link']
        self._planning_frame: str = manip.get('planning_frame', 'base_link')
        self._moveit_timeout: float = float(manip['moveit_timeout_sec'])
        self._planning_time: float = float(manip.get('planning_time_sec', 10.0))
        self._planning_attempts: int = int(manip.get('planning_attempts', 5))
        self._velocity_scaling: float = float(manip.get('velocity_scaling', 0.5))
        self._cartesian_scaling: float = float(manip.get('cartesian_scaling', 0.2))
        self._min_cartesian_fraction: float = float(
            manip.get('min_cartesian_fraction', 0.95))
        self._max_reach: float = float(manip.get('max_reach', 0.85))
        self._ground_z: float = float(manip.get('ground_z', -0.13))
        self._retreat_distance: float = float(manip.get('retreat_distance', 0.3))
        self._back_off_distance: float = float(manip.get('back_off_distance', 0.3))
        self._carry_tcp: Vec3 = tuple(manip.get('carry_tcp', [0.5, 0.0, 1.0]))
        self._object_sizes: Dict[str, List[float]] = manip.get('object_sizes') or {}
        self._default_object_size: List[float] = list(
            manip.get('default_object_size', [0.1, 0.1, 0.1]))
        self._fixture_sizes: Dict[str, List[float]] = manip.get('fixture_sizes') or {}
        self._fixture_radius: float = float(manip.get('fixture_radius', 2.5))
        self._touch_links: List[str] = list(manip.get('touch_links') or [])
        self._allowed_pairs: List[List[str]] = [
            list(p) for p in manip.get('allowed_collision_pairs') or []]
        self._arm_base: Vec3 = tuple(manip.get('arm_base_xyz', [0.0, 0.0, 0.0]))
        self._tcp_offset: float = float(manip.get('tcp_offset', 0.0))
        self._approach_distance: float = float(manip.get('approach_distance', 0.1))
        self._lift_distance: float = float(manip.get('lift_distance', 0.05))
        self._place_clearance: float = float(manip.get('place_clearance', 0.02))
        self._position_tolerance: float = float(manip.get('position_tolerance', 0.01))
        self._orientation_tolerance: float = float(
            manip.get('orientation_tolerance', 0.1))
        self._grasp_offsets: Dict[str, float] = manip.get('grasp_offsets') or {}
        self._place_heights: Dict[str, Any] = manip.get('place_heights') or {}
        self._place_offsets: Dict[str, float] = manip.get('place_offsets') or {}
        # A place height counts as occupied if another copy of the held
        # object's model is within these distances of the release point
        self._slot_radius: float = float(manip.get('slot_radius', 0.2))
        self._slot_half_height: float = float(manip.get('slot_half_height', 0.12))
        self._arm_joints: List[str] = list(manip.get('arm_joint_names') or [])
        self._stow_positions: List[float] = [
            float(v) for v in (manip.get('stow_joint_positions') or [])
        ]
        self._gripper_open: float = float(manip['gripper_open_position'])
        self._gripper_closed: float = float(manip['gripper_closed_position'])
        self._gripper_max_effort: float = float(manip['gripper_max_effort'])
        self._gripper_max_opening: float = float(manip.get('gripper_max_opening', 0.085))
        self._gripper_clearance: float = float(manip.get('gripper_clearance', 0.004))
        self._gripper_timeout: float = float(manip['gripper_timeout_sec'])
        self._gripper_action: str = manip.get(
            'gripper_action', 'arm_0_gripper_controller/gripper_cmd')
        attach = manip.get('attach') or {}
        self._attach_enabled: bool = bool(attach.get('enabled', False))
        self._attach_parent_link: str = attach.get('parent_link', 'arm_0_wrist_3_link')
        self._attach_child_link: str = attach.get('child_link', 'link')
        self._attach_timeout_ms: int = int(attach.get('service_timeout_ms', 5000))
        # Snap the object into the gripper if it is further than this from
        # where the gripper holds it, but not if the gripper is further than
        # snap_max_distance away (then the arm did not get to the object)
        self._snap_tolerance: float = float(attach.get('snap_tolerance', 0.02))
        self._snap_max_distance: float = float(attach.get('snap_max_distance', 0.3))
        # Held / placed checks (m) and how often a hold or placement is retried
        self._held_tolerance: float = float(attach.get('held_tolerance', 0.06))
        self._place_tolerance: float = float(attach.get('place_tolerance', 0.05))
        # A placed object may settle this far below the release point
        self._place_drop_limit: float = float(attach.get('place_drop_limit', 0.25))
        self._attach_attempts: int = max(1, int(attach.get('attempts', 3)))
        # Simulation time to let Gazebo apply a teleport / (de)attachment
        self._settle_time: float = float(attach.get('settle_sec', 0.05))
        self._joint_state_timeout: float = float(attach.get('joint_state_timeout_sec', 2.0))
        # Planned arm motions are retried this many times before giving up
        self._motion_retries: int = max(0, int(manip.get('motion_retries', 2)))

        # World
        self._world_name: str = config['world']['name']
        self._pose_period: float = float(
            config['world'].get('pose_update_period_sec', 0.5))

        self._cb_group = ReentrantCallbackGroup()

        # World pose registry: Gazebo entity name -> (x, y, z, yaw), filled
        # by _pose_callback on a gz-transport thread; guarded by _pose_lock.
        self._pose_registry: Dict[str, EntityPose] = {}
        # Gazebo entity name -> entity id (same source and lock)
        self._entity_ids: Dict[str, int] = {}
        # Incremented on every pose message (to wait for fresh poses)
        self._pose_seq = 0
        self._pose_lock = threading.Lock()
        # Latest DetachableJoint state per object ('attached' / 'detached'),
        # from its output topic; missing if none has been received
        self._joint_states: Dict[str, str] = {}
        self._joint_state_watched: set = set()

        # One subtask at a time per robot instance
        self._robot_locks: Dict[str, threading.Lock] = {
            k: threading.Lock() for k in self._robot_instances
        }
        # Objects attached per robot instance: object -> detach/attach topics
        self._attached: Dict[str, Dict[str, Tuple[str, str]]] = {
            k: {} for k in self._robot_instances
        }
        # Every object ever given a DetachableJoint, per robot (reused on re-pick)
        self._attach_plugins: Dict[str, Dict[str, Tuple[str, str]]] = {
            k: {} for k in self._robot_instances
        }

        self.create_subscription(
            String, '/mission/subtask', self._subtask_callback, 10,
            callback_group=self._cb_group,
        )
        # Velocity commands for fine positioning (twist_mux's 'external' input,
        # the same one Nav2 uses)
        self._cmd_vel_pubs = {
            robot: self.create_publisher(TwistStamped, f'{namespace}/cmd_vel', 10)
            for robot, namespace in self._robot_instances.items()}

        # Gazebo world poses, straight from gz-transport so entity names are
        # kept. The warehouse publishes thousands of poses at up to ~44 Hz, so
        # gz-transport throttles delivery to one message per period.
        self._pose_topic = f'/world/{self._world_name}/pose/info'
        self._gz_node = GzNode()
        pose_opts = SubscribeOptions()
        pose_opts.msgs_per_sec = max(1, round(1.0 / self._pose_period))
        if not self._gz_node.subscribe(
            Pose_V, self._pose_topic, self._pose_callback, pose_opts
        ):
            self.get_logger().error(
                f'Could not subscribe to Gazebo topic {self._pose_topic}')
        self._gz_publishers: Dict[str, Any] = {}

        self._feedback_pub = self.create_publisher(String, '/mission/feedback', 10)

        self._nav_clients: Dict[str, ActionClient] = {}
        self._moveit_clients: Dict[str, ActionClient] = {}
        self._gripper_clients: Dict[str, ActionClient] = {}
        self._execute_clients: Dict[str, ActionClient] = {}
        self._cartesian_clients: Dict[str, Any] = {}
        self._scene_clients: Dict[str, Any] = {}
        self._get_scene_clients: Dict[str, Any] = {}
        self._fk_clients: Dict[str, Any] = {}
        # Planning-scene bookkeeping per robot
        self._scene_fixtures: Dict[str, List[str]] = {k: [] for k in self._robot_instances}
        self._scene_holding: Dict[str, bool] = {k: False for k in self._robot_instances}
        self._acm_ready: Dict[str, bool] = {k: False for k in self._robot_instances}
        # Object held by each robot: {'name', 'size', 'grasp_offset'} or None
        self._held: Dict[str, Optional[Dict[str, Any]]] = {
            k: None for k in self._robot_instances}
        self._init_action_clients()

        # The arm spawns stretched out in front of the robot (all joints at 0),
        # in the lidar's plane and beyond the Nav2 footprint, so stow it as
        # soon as MoveIt is up.
        self._stow_pending = set(self._robot_instances) if manip.get(
            'stow_on_startup', True) else set()
        self._startup_timer = None
        if self._stow_pending:
            self._startup_timer = self.create_timer(
                5.0, self._startup_stow, callback_group=self._cb_group)

        self.get_logger().info(
            'MobileRobotController ready.\n'
            '  -> Subscribing to: /mission/subtask\n'
            f'  -> Subscribing to: {self._pose_topic} (gz-transport)\n'
            '  -> Publishing to:  /mission/feedback\n'
            f'  -> Managing instances: {list(self._robot_instances.keys())}'
        )

    def _init_action_clients(self) -> None:
        """Create Nav2, MoveIt and gripper action clients for every instance."""
        for instance_id, namespace in self._robot_instances.items():
            self._nav_clients[instance_id] = ActionClient(
                self, NavigateToPose, f'{namespace}/navigate_to_pose',
                callback_group=self._cb_group)
            self._moveit_clients[instance_id] = ActionClient(
                self, MoveGroup, f'{namespace}/move_action',
                callback_group=self._cb_group)
            self._execute_clients[instance_id] = ActionClient(
                self, ExecuteTrajectory, f'{namespace}/execute_trajectory',
                callback_group=self._cb_group)
            self._cartesian_clients[instance_id] = self.create_client(
                GetCartesianPath, f'{namespace}/compute_cartesian_path',
                callback_group=self._cb_group)
            self._scene_clients[instance_id] = self.create_client(
                ApplyPlanningScene, f'{namespace}/apply_planning_scene',
                callback_group=self._cb_group)
            self._get_scene_clients[instance_id] = self.create_client(
                GetPlanningScene, f'{namespace}/get_planning_scene',
                callback_group=self._cb_group)
            self._fk_clients[instance_id] = self.create_client(
                GetPositionFK, f'{namespace}/compute_fk',
                callback_group=self._cb_group)
            self._gripper_clients[instance_id] = ActionClient(
                self, GripperCommand, f'{namespace}/{self._gripper_action}',
                callback_group=self._cb_group)
            self.get_logger().info(
                f'[init] Action clients ready for {instance_id} '
                f'(namespace={namespace})')

    def _startup_stow(self) -> None:
        """Stow each robot's arm once its MoveIt server is available."""
        for robot in sorted(self._stow_pending):
            if not self._moveit_clients[robot].server_is_ready():
                continue
            lock = self._robot_locks[robot]
            if not lock.acquire(blocking=False):
                continue
            try:
                ok, err = self._stow(robot)
            finally:
                lock.release()
            if ok:
                self.get_logger().info(f'[startup] {robot}: arm stowed.')
                self._stow_pending.discard(robot)
            else:
                self.get_logger().warn(f'[startup] {robot}: stow failed ({err}); retrying.')
        if not self._stow_pending and self._startup_timer is not None:
            # Destroyed rather than cancelled: a cancelled timer left in a
            # MultiThreadedExecutor makes it busy-loop.
            self.destroy_timer(self._startup_timer)
            self._startup_timer = None

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """
        Block until `future` completes or `timeout_sec` passes on the node clock.

        Used instead of rclpy.spin_until_future_complete, which must not be
        called from callbacks running in a MultiThreadedExecutor; the
        executor's other threads complete the future. The timeout runs on the
        node clock (simulation time with use_sim_time). Returns True if done.
        """
        if future.done():
            return True
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while not done.wait(0.1):
            if self.get_clock().now() >= deadline:
                return future.done()
        return True

    def _run_action(self, client: ActionClient, goal, timeout_sec: float,
                    label: str):
        """
        Send an action goal and wait for its result.

        Returns (result_response, error_detail). result_response is None on
        failure, with error_detail saying why.
        """
        if not client.wait_for_server(timeout_sec=10.0):
            return None, f'{label} action server not available.'
        send_goal_future = client.send_goal_async(goal)
        if not self._wait_for_future(send_goal_future, timeout_sec):
            return None, f'{label} goal was not acknowledged in time.'
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None, f'{label} goal rejected.'
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec):
            goal_handle.cancel_goal_async()
            return None, f'{label} timed out after {timeout_sec:.0f}s (sim time).'
        return result_future.result(), ''

    def destroy_node(self) -> None:
        """Stop the Gazebo pose subscription, then destroy the ROS node."""
        self._gz_node.unsubscribe(self._pose_topic)
        super().destroy_node()

    # World pose subscription

    def _pose_callback(self, msg: Pose_V) -> None:
        """
        Update the pose registry from the Gazebo world pose topic.

        Runs on a gz-transport thread. Each pose's name is the Gazebo entity
        name (e.g. 'Picking_Shelves.002', or 'a200_0000/robot' for the robot).
        Top-level models are in world coordinates; nested entities (links)
        are relative to their parent, and only top-level models are used here.
        """
        updates = {}
        ids = {}
        for pose in msg.pose:
            if not pose.name:
                continue
            p, q = pose.position, pose.orientation
            updates[pose.name] = (
                p.x, p.y, p.z, _quaternion_to_yaw(q.x, q.y, q.z, q.w))
            ids[pose.name] = pose.id
        with self._pose_lock:
            self._pose_registry.update(updates)
            self._entity_ids.update(ids)
            self._pose_seq += 1

    def _same_model_nearby(self, name: str, xy: Tuple[float, float],
                           radius: float, exclude: Sequence[str] = ()) -> List[EntityPose]:
        """Poses of other copies of `name`'s model within `radius` (XY) of xy."""
        prefix = _model_prefix(name)
        skip = {name, *exclude}
        with self._pose_lock:
            return [p for n, p in self._pose_registry.items()
                    if n not in skip and _model_prefix(n) == prefix
                    and math.hypot(p[0] - xy[0], p[1] - xy[1]) <= radius]

    def _get_pose(self, entity_name: str) -> Optional[EntityPose]:
        """Look up an entity pose (x, y, z, yaw); None if unknown."""
        with self._pose_lock:
            pose = self._pose_registry.get(entity_name)
            known = len(self._pose_registry)
        if pose is None:
            self.get_logger().error(
                f"[get_pose] Entity '{entity_name}' not found among {known} "
                f'Gazebo entities on {self._pose_topic}.')
        return pose

    def _robot_model_name(self, robot: str) -> str:
        """Return the Gazebo model name of a robot instance (e.g. a200_0000/robot)."""
        return f"{self._robot_instances[robot].strip('/')}/robot"

    def _robot_pose(self, robot: str) -> Optional[EntityPose]:
        """World pose of the robot's base_link (the Gazebo model origin)."""
        with self._pose_lock:
            return self._pose_registry.get(self._robot_model_name(robot))

    # Subtask subscription

    def _subtask_callback(self, msg: String) -> None:
        """Parse a /mission/subtask message; never let an exception escape."""
        try:
            subtask = json.loads(msg.data.strip())
            self._handle_subtask(subtask)
        except Exception as exc:  # noqa: BLE001 - keep the node alive
            self.get_logger().error(f'[subtask_callback] Unhandled exception: {exc}')

    def _handle_subtask(self, subtask: Dict[str, Any]) -> None:
        """Validate a subtask and dispatch it to the matching skill handler."""
        plan_id = subtask.get('plan_id', '')
        task_id = subtask.get('task_id', '')
        robot = subtask.get('robot', '')
        skills = subtask.get('required_skills', [])
        args = subtask.get('args', {}) or {}

        self.get_logger().info(
            f'[handle_subtask] Received subtask {task_id} '
            f'(robot={robot}, skills={skills})')

        if not robot.startswith(ROBOT_TYPE):
            self.get_logger().info(
                f"[handle_subtask] Subtask {task_id} is assigned to '{robot}' "
                f"which is not a '{ROBOT_TYPE}', ignoring.")
            return

        if robot not in self._robot_instances:
            self.get_logger().error(
                f"[handle_subtask] Robot instance '{robot}' not in config. "
                f'Known instances: {list(self._robot_instances.keys())}')
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='config_error',
                detail=f'Unknown robot instance: {robot}')
            return

        skill = skills[0] if skills else None
        handlers = {
            'navigate_to': self._execute_navigate_to,
            'pick': self._execute_pick,
            'place': self._execute_place,
        }
        if skill not in handlers:
            detail = ('No required_skills specified.' if skill is None
                      else f'Unknown skill: {skill}')
            self.get_logger().error(f'[handle_subtask] {task_id}: {detail}')
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage='config_error',
                detail=detail)
            return

        lock = self._robot_locks[robot]
        if not lock.acquire(blocking=False):
            self.get_logger().warn(
                f'[handle_subtask] {robot} is busy; {task_id} waits for the '
                'running subtask to finish.')
            lock.acquire()
        try:
            handlers[skill](plan_id, task_id, robot, args)
        except Exception as exc:  # noqa: BLE001 - report instead of hanging the plan
            self.get_logger().error(f'[{skill}] Task {task_id} raised: {exc}')
            self._publish_feedback(
                plan_id, task_id, robot, status='failed', stage=skill,
                detail=f'Exception: {exc}')
        finally:
            lock.release()

    # Skill: navigate_to

    def _execute_navigate_to(self, plan_id: str, task_id: str, robot: str,
                             args: Dict[str, Any]) -> None:
        """
        Send a NavigateToPose goal to a standoff pose in front of a location.

        Args expected:
          location (str): Gazebo entity name, e.g. "Picking_Shelves.001"
        """
        def fail(detail: str) -> None:
            self.get_logger().error(f'[navigate_to] Task {task_id}: {detail}')
            self._publish_feedback(plan_id, task_id, robot, status='failed',
                                   stage='navigate_to', detail=detail)

        location = args.get('location')
        if not location:
            fail("Missing 'location' in args.")
            return
        pose = self._get_pose(location)
        if pose is None:
            fail(f"Entity '{location}' not in pose registry.")
            return

        x, y, _, yaw = pose
        robot_pose = self._robot_pose(robot)
        robot_xy = (robot_pose[0], robot_pose[1]) if robot_pose else None
        standoff = float(_lookup_by_prefix(self._standoffs, location, self._standoff))
        axis = _lookup_by_prefix(self._approach_axes, location, self._default_axis)
        blockers = self._same_model_nearby(location, (x, y), 2.0 * standoff)
        nav_x, nav_y, nav_yaw = _apply_standoff(
            x, y, yaw, standoff, robot_xy, axis,
            blockers=[(b[0], b[1]) for b in blockers],
            clearance=self._blocker_clearance * standoff)

        self.get_logger().info(
            f"[navigate_to] Task {task_id}: navigating to '{location}' "
            f'(standoff {standoff:.2f} m, {axis}) -> ({nav_x:.2f}, {nav_y:.2f}, '
            f'yaw {nav_yaw:.2f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        # Zero stamp: Nav2 uses the latest transform instead of looking up
        # TF at the stamp, which fails once the goal is older than the buffer
        goal.pose.pose.position.x = nav_x
        goal.pose.pose.position.y = nav_y
        goal.pose.pose.orientation = _yaw_to_quaternion(nav_yaw)

        self._publish_feedback(plan_id, task_id, robot, status='in_progress',
                               stage='navigating',
                               detail=f"Navigating to '{location}'.")

        result, error = self._run_action(
            self._nav_clients[robot], goal, self._nav_timeout, 'Nav2')
        if result is None:
            fail(error)
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = (f'Navigation failed with status {result.status} '
                      f'(error code {getattr(result.result, "error_code", "n/a")})')
            # Nav2 often gives up just short of a tight standoff pose (its
            # progress checker and collision monitor near a fixture); finish
            # the approach with fine positioning if the robot is close
            robot_pose = self._robot_pose(robot)
            near = robot_pose is not None and math.hypot(
                robot_pose[0] - nav_x, robot_pose[1] - nav_y) <= self._fine_max_correction
            if not (self._fine_positioning and near
                    and self._fine_position(robot, task_id, (x, y), standoff)):
                fail(detail + '.')
                return
            self.get_logger().warn(
                f'[navigate_to] Task {task_id}: {detail}, but fine positioning '
                'reached the standoff pose.')
        elif self._fine_positioning:
            self._fine_position(robot, task_id, (x, y), standoff)
        self.get_logger().info(f'[navigate_to] Task {task_id}: navigation complete.')
        self._publish_feedback(plan_id, task_id, robot, status='success',
                               stage='complete', detail=f"Reached '{location}'.")

    def _fine_position(self, robot: str, task_id: str,
                       target: Tuple[float, float], standoff: float) -> bool:
        """
        Creep to exactly `standoff` from the target, facing it.

        Nav2 stops within its goal tolerance of where AMCL thinks the robot
        is, and AMCL is a few decimetres off at times; the arm needs better.
        Using the robot's true pose from Gazebo, this turns in place to face
        the target and drives straight forwards or backwards to the standoff
        distance (the line Nav2 already brought the robot onto). A sideways
        offset is left alone: the arm aims from its own base. Corrections
        larger than max_correction are not attempted (simulation only).
        Returns True once at the standoff pose.
        """
        pub = self._cmd_vel_pubs[robot]
        deadline = self.get_clock().now() + Duration(seconds=self._fine_timeout)
        reason = 'timed out'
        try:
            while self.get_clock().now() < deadline:
                pose = self._robot_pose(robot)
                if pose is None:
                    reason = 'no robot pose'
                    break
                dx, dy = target[0] - pose[0], target[1] - pose[1]
                yaw_err = _wrap_angle(math.atan2(dy, dx) - pose[3])
                dist_err = math.hypot(dx, dy) - standoff
                if abs(dist_err) > self._fine_max_correction:
                    reason = f'{dist_err:+.2f} m off, more than max_correction'
                    break
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = 'base_link'
                if abs(yaw_err) > self._fine_yaw_tolerance:
                    cmd.twist.angular.z = _clamp(1.0 * yaw_err, self._fine_max_angular)
                elif abs(dist_err) > self._fine_tolerance:
                    cmd.twist.linear.x = _clamp(0.6 * dist_err, self._fine_max_linear)
                    cmd.twist.angular.z = _clamp(1.0 * yaw_err, self._fine_max_angular)
                else:
                    self.get_logger().info(
                        f'[navigate_to] Task {task_id}: fine positioning done '
                        f'({dist_err:+.3f} m, {yaw_err:+.3f} rad).')
                    return True
                pub.publish(cmd)
                threading.Event().wait(0.05)
        finally:
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            pub.publish(stop)
        self.get_logger().warn(
            f'[navigate_to] Task {task_id}: fine positioning stopped ({reason}).')
        return False

    def _drive_straight(self, robot: str, distance: float) -> Tuple[bool, str]:
        """
        Drive the base straight forwards (+) or backwards (-) by `distance` m.

        Uses the true pose from Gazebo and holds the starting heading. Not
        collision-checked: only for short moves out of a standoff pose.
        """
        start = self._robot_pose(robot)
        if start is None:
            return False, 'no robot pose.'
        pub = self._cmd_vel_pubs[robot]
        deadline = self.get_clock().now() + Duration(seconds=self._fine_timeout)
        try:
            while self.get_clock().now() < deadline:
                pose = self._robot_pose(robot)
                if pose is None:
                    return False, 'lost the robot pose.'
                travelled = math.hypot(pose[0] - start[0], pose[1] - start[1])
                remaining = abs(distance) - travelled
                if remaining <= self._fine_tolerance:
                    return True, ''
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = 'base_link'
                speed = _clamp(0.6 * remaining, self._fine_max_linear)
                cmd.twist.linear.x = math.copysign(max(speed, 0.02), distance)
                cmd.twist.angular.z = _clamp(
                    1.0 * _wrap_angle(start[3] - pose[3]), self._fine_max_angular)
                pub.publish(cmd)
                threading.Event().wait(0.05)
        finally:
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            pub.publish(stop)
        return False, 'timed out.'

    # Skill: pick

    def _execute_pick(self, plan_id: str, task_id: str, robot: str,
                      args: Dict[str, Any]) -> None:
        """
        Pick an object within reach of the arm.

        The arm motions are best effort: the hold comes from a Gazebo
        DetachableJoint, not from the fingers gripping the object.

        1. Open the gripper
        2. Plan to a pre-grasp pose (gripper horizontal, short of the
           object), then move straight in, as far as feasible
        3. Secure the object: snap it into the gripper if the arm stopped
           off target, then fix it to the wrist (_secure_object)
        4. Close the gripper, lift and pull straight back
        5. Check the object moved with the gripper; if not, snap and
           re-attach it (_ensure_held)
        6. Back the base off, move to the carry pose, check the hold again

        Fails only if the object is out of reach, the arm cannot get near
        it, or the hold cannot be established after every attempt.

        Args expected:
          object (str): Gazebo entity name, e.g. "small_box_000_3"
        """
        def fail(detail: str) -> None:
            self.get_logger().error(f'[pick] Task {task_id}: {detail}')
            self._publish_feedback(plan_id, task_id, robot, status='failed',
                                   stage='pick', detail=detail)

        object_name = args.get('object')
        if not object_name:
            fail("Missing 'object' in args.")
            return
        obj = self._get_pose(object_name)
        robot_pose = self._robot_pose(robot)
        if obj is None:
            fail(f"Object '{object_name}' not in pose registry.")
            return
        if robot_pose is None:
            fail(f"Robot model '{self._robot_model_name(robot)}' not in pose registry.")
            return

        target = _world_to_robot(robot_pose, obj[:3])
        approach = _horizontal_approach(self._arm_base, target)
        grasp_offset = float(_lookup_by_prefix(self._grasp_offsets, object_name, 0.0))
        grasp_tcp = self._along(target, approach, -grasp_offset)
        pre_tcp = self._along(grasp_tcp, approach, -self._approach_distance)
        lift_tcp = (grasp_tcp[0], grasp_tcp[1], grasp_tcp[2] + self._lift_distance)
        retreat_tcp = self._along(lift_tcp, approach, -self._retreat_distance)

        self.get_logger().info(
            f"[pick] Task {task_id}: picking '{object_name}' at "
            f'({obj[0]:.2f}, {obj[1]:.2f}, {obj[2]:.2f}) world = '
            f'({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) in '
            f'{self._planning_frame}')
        self._publish_feedback(plan_id, task_id, robot, status='in_progress',
                               stage='pick_in_progress',
                               detail=f"Picking '{object_name}'.")

        if self._attached[robot] or self._held[robot]:
            holding = list(self._attached[robot]) or [self._held[robot]['name']]
            fail(f'Gripper is not empty (holding {holding}).')
            return
        err = self._reach_error(pre_tcp, approach) or self._reach_error(grasp_tcp, approach)
        if err:
            fail(f"'{object_name}' is out of the arm's reach: {err}")
            return
        ok, err = self._send_gripper_goal(robot, self._gripper_open)
        if not ok:
            self.get_logger().warn(f'[pick] Could not open the gripper ({err}); continuing.')
        # Free motion to the pre-grasp pose, then straight in: a planned
        # (joint-space) path could sweep the gripper through the object.
        ok, err = self._move_tcp_retry(robot, pre_tcp, approach)
        if not ok:
            fail(f"MoveIt could not reach the pre-grasp pose for '{object_name}': {err}")
            return
        self._move_cartesian_best_effort(robot, [grasp_tcp], approach, 'approach')

        ok, err = self._secure_object(robot, object_name, grasp_offset, resting=True)
        if not ok:
            fail(f"Could not get hold of '{object_name}': {err}")
            return
        size = list(_lookup_by_prefix(
            self._object_sizes, object_name, self._default_object_size))
        self._held[robot] = {'name': object_name, 'size': size,
                             'grasp_offset': grasp_offset}
        # Close onto the object's width, not fully: squeezing an object that
        # is already attached makes the fingers fight the joint
        ok, err = self._send_gripper_goal(robot, self._gripper_position_for(size[0]))
        if not ok:
            self.get_logger().warn(f'[pick] Could not close the gripper ({err}); '
                                   'the object is held by the attachment.')

        self._move_cartesian_best_effort(robot, [lift_tcp], approach, 'lift')
        ok, err = self._ensure_held(robot, object_name, grasp_offset)
        if not ok:
            self._release(robot, object_name)
            fail(f"Could not keep hold of '{object_name}': {err}")
            return
        # Pull the object straight out (clear of shelf tiers) before any
        # planned motion
        self._move_cartesian_best_effort(robot, [retreat_tcp], approach, 'retreat')
        # The arm alone cannot pull a deep object clear of a shelf without
        # folding into itself, so the base backs off before the arm moves on
        if self._back_off_distance > 0.0:
            ok, err = self._drive_straight(robot, -self._back_off_distance)
            if not ok:
                self.get_logger().warn(f'[pick] Could not back away ({err}); continuing.')
        # Carry high over the robot: the stow pose would hold the object low
        # and in front, where tables and shelves are
        ok, err = self._move_tcp_retry(robot, self._carry_tcp, (1.0, 0.0))
        if not ok:
            self.get_logger().warn(
                f'[pick] Could not move to the carry pose ({err}); keeping the arm here.')
        ok, err = self._ensure_held(robot, object_name, grasp_offset)
        if not ok:
            self._release(robot, object_name)
            fail(f"Could not keep hold of '{object_name}': {err}")
            return

        self.get_logger().info(f'[pick] Task {task_id}: pick complete.')
        self._publish_feedback(plan_id, task_id, robot, status='success',
                               stage='complete', detail=f"Picked '{object_name}'.")

    # Skill: place

    def _execute_place(self, plan_id: str, task_id: str, robot: str,
                       args: Dict[str, Any]) -> None:
        """
        Place the held object at a location within reach of the arm.

        The release point is the location's origin raised by its place height
        (manipulation.place_heights, by name prefix; e.g. a shelf tier).

        1. Plan to a pre-place pose, above and short of the release point,
           and move straight down (best effort)
        2. Detach the object and set it exactly on the release point,
           aligned with the location (_put_down)
        3. Open the gripper, retreat straight back and stow the arm
        4. Check the object is at the release point; correct it if not

        Fails only if the robot is not within reach of the location or the
        object cannot be released after every attempt.

        Args expected:
          location (str): Gazebo entity name of the target location
          object   (str): name of the object being placed
        """
        def fail(detail: str) -> None:
            self.get_logger().error(f'[place] Task {task_id}: {detail}')
            self._publish_feedback(plan_id, task_id, robot, status='failed',
                                   stage='place', detail=detail)

        location = args.get('location')
        held = self._held[robot]
        object_name = args.get('object') or (held['name'] if held else None)
        if not location:
            fail("Missing 'location' in args.")
            return
        if held is None or held['name'] != object_name:
            fail(f"Not holding '{object_name}' "
                 f"(holding {held['name'] if held else 'nothing'}).")
            return
        loc = self._get_pose(location)
        robot_pose = self._robot_pose(robot)
        if loc is None:
            fail(f"Location '{location}' not in pose registry.")
            return
        if robot_pose is None:
            fail(f"Robot model '{self._robot_model_name(robot)}' not in pose registry.")
            return

        release = self._choose_release_point(location, loc, object_name, robot_pose)
        if release is None:
            fail(f"No free place for '{object_name}' at '{location}' (every "
                 'configured height is occupied).')
            return
        target = _world_to_robot(robot_pose, release)
        approach = _horizontal_approach(self._arm_base, target)
        grasp_offset = float(held['grasp_offset'])
        place_tcp = self._along(target, approach, -grasp_offset)
        above_tcp = (place_tcp[0], place_tcp[1], place_tcp[2] + self._lift_distance)
        pre_tcp = self._along(above_tcp, approach, -self._approach_distance)
        retreat_tcp = self._along(place_tcp, approach, -self._retreat_distance)

        self.get_logger().info(
            f"[place] Task {task_id}: placing '{object_name}' at '{location}' "
            f'({release[0]:.2f}, {release[1]:.2f}, {release[2]:.2f}) world = '
            f'({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) in '
            f'{self._planning_frame}')
        self._publish_feedback(plan_id, task_id, robot, status='in_progress',
                               stage='place_in_progress',
                               detail=f"Placing '{object_name}' at '{location}'.")

        err = self._reach_error(pre_tcp, approach) or self._reach_error(place_tcp, approach)
        if err:
            fail(f"'{location}' is out of the arm's reach: {err}")
            return
        arm_at_place, err = self._move_tcp_retry(robot, pre_tcp, approach)
        if arm_at_place:
            self._move_cartesian_best_effort(robot, [above_tcp, place_tcp], approach, 'lower')
        else:
            self.get_logger().warn(
                f'[place] MoveIt could not reach the pre-place pose ({err}); '
                'releasing the object onto the place from where it is.')

        yaw = loc[3]  # align the object with the location
        ok, err = self._put_down(robot, object_name, release, yaw)
        if not ok:
            fail(f"Could not release '{object_name}': {err}")
            return
        ok, err = self._send_gripper_goal(robot, self._gripper_open)
        if not ok:
            self.get_logger().warn(f'[place] Could not open the gripper ({err}).')
        if arm_at_place:
            self._move_cartesian_best_effort(robot, [retreat_tcp], approach, 'retreat')
        ok, err = self._stow_retry(robot)
        if not ok:
            self.get_logger().warn(f'[place] Could not stow the arm: {err}')
        ok, err = self._ensure_placed(robot, object_name, release, yaw)
        if not ok:
            fail(f"'{object_name}' did not stay at '{location}': {err}")
            return

        self.get_logger().info(f'[place] Task {task_id}: place complete.')
        self._publish_feedback(plan_id, task_id, robot, status='success',
                               stage='complete',
                               detail=f"Placed '{object_name}' at '{location}'.")

    def _choose_release_point(self, location: str, loc: EntityPose, object_name: str,
                              robot_pose: EntityPose) -> Optional[Vec3]:
        """
        Pick the world point at which the held object's centre is released.

        Horizontally: the location's origin, moved place_offsets[location]
        metres towards the robot (e.g. onto the near half of a deep table).
        Vertically: the first of place_heights[location] (heights above the
        location's origin, e.g. shelf tiers) with no other copy of the held
        object's model already there. None if all are occupied.
        """
        heights = _lookup_by_prefix(self._place_heights, location, [0.0])
        if not isinstance(heights, (list, tuple)):
            heights = [heights]
        offset = float(_lookup_by_prefix(self._place_offsets, location, 0.0))
        ux, uy = _horizontal_approach(loc[:3], robot_pose[:3])
        rx, ry = loc[0] + offset * ux, loc[1] + offset * uy
        others = self._same_model_nearby(object_name, (rx, ry), self._slot_radius)
        for height in heights:
            z = loc[2] + float(height)
            if any(abs(p[2] - z) < self._slot_half_height for p in others):
                self.get_logger().info(
                    f"[place] '{location}' is occupied at height {float(height):.2f}.")
                continue
            return rx, ry, z + self._place_clearance
        return None

    def _gripper_position_for(self, width: float) -> float:
        """
        Knuckle position that leaves the fingers just around `width` metres.

        The 2F-85's opening goes roughly linearly from gripper_max_opening at
        gripper_open_position to 0 at gripper_closed_position. Objects wider
        than the opening get the fully closed position (closing on air in
        front of them; they are held by the attachment).
        """
        opening = width + self._gripper_clearance
        if opening >= self._gripper_max_opening:
            return self._gripper_closed
        fraction = 1.0 - opening / self._gripper_max_opening
        return self._gripper_open + fraction * (self._gripper_closed - self._gripper_open)

    # Holding and releasing objects (DetachableJoint + set_pose)

    def _secure_object(self, robot: str, object_name: str, grasp_offset: float,
                       resting: bool, force: bool = False) -> Tuple[bool, str]:
        """
        Get `object_name` into the gripper and fix it to the wrist.

        The object belongs `grasp_offset` past the TCP. If it is further than
        snap_tolerance from there (the arm stopped short or off to a side),
        or `force` is set (a previous hold failed), it is detached if needed
        and teleported there before attaching. A `resting` object (still on
        its support) is never moved down, so it is not pushed into the
        support. Unless `force` is set, refuses if the gripper is more than
        snap_max_distance away: then the arm never got to the object.
        """
        expected = self._expected_held_position(robot, grasp_offset)
        if expected is None:
            return False, 'could not get the gripper pose from MoveIt (compute_fk).'
        pose = self._get_pose(object_name)
        if pose is None:
            return False, 'object not in pose registry.'
        dist = math.dist(pose[:3], expected)
        # (Only for the first grasp: an object that was already picked and
        # then dropped is always snapped back, however far it fell)
        if not force and dist > self._snap_max_distance:
            return False, (f'the gripper is {dist:.2f} m from it (max '
                           f'{self._snap_max_distance:.2f} m); the arm did not get there.')
        if not self._attach_enabled:
            return True, ''
        if force or dist > self._snap_tolerance:
            if force:
                ok, err = self._detach(robot, object_name)
                if not ok:
                    return False, err
                self._wait_sim(self._settle_time)
            z = max(expected[2], pose[2]) if resting else expected[2]
            snap = (expected[0], expected[1], z)
            if not self._set_object_pose(object_name, snap, pose[3]):
                return False, f'/world/{self._world_name}/set_pose failed.'
            self.get_logger().info(
                f"[attach] '{object_name}' was {dist:.3f} m from the gripper; "
                'snapped into it.')
            # Let Gazebo apply the teleport before the joint is created, so
            # the joint fixes the snapped (not the old) relative pose
            self._wait_sim(self._settle_time)
        return self._attach(robot, object_name)

    def _ensure_held(self, robot: str, object_name: str,
                     grasp_offset: float) -> Tuple[bool, str]:
        """
        Check that the object is in the gripper; re-secure it if not.

        Up to attach.attempts times: the object is snapped back into the
        gripper and attached again.
        """
        err = ''
        for attempt in range(self._attach_attempts + 1):
            err = self._hold_error(robot, object_name, grasp_offset)
            if not err:
                return True, ''
            if attempt == self._attach_attempts or not self._attach_enabled:
                break
            self.get_logger().warn(
                f"[attach] '{object_name}' is not held ({err}); re-attaching "
                f'(attempt {attempt + 1} of {self._attach_attempts}).')
            ok, secure_err = self._secure_object(
                robot, object_name, grasp_offset, resting=False, force=True)
            if not ok:
                err = secure_err
        return False, err

    def _hold_error(self, robot: str, object_name: str, grasp_offset: float) -> str:
        """Describe why the object is not in the gripper ('' if it is)."""
        self._wait_pose_updates(2)
        expected = self._expected_held_position(robot, grasp_offset)
        pose = self._get_pose(object_name)
        if expected is None or pose is None:
            return 'gripper or object pose unknown'
        dist = math.dist(pose[:3], expected)
        if dist > self._held_tolerance:
            return f'it is {dist:.2f} m from the gripper'
        return ''

    def _put_down(self, robot: str, object_name: str, release: Vec3,
                  yaw: float) -> Tuple[bool, str]:
        """Detach the held object and set it on the release point."""
        err = ''
        for _ in range(self._attach_attempts):
            ok, err = self._release(robot, object_name)
            if not ok:
                continue
            if not self._attach_enabled:
                return True, ''
            self._wait_sim(self._settle_time)
            if self._set_object_pose(object_name, release, yaw):
                self._wait_sim(self._settle_time)
                return True, ''
            err = f'/world/{self._world_name}/set_pose failed.'
        return False, err

    def _ensure_placed(self, robot: str, object_name: str, release: Vec3,
                       yaw: float) -> Tuple[bool, str]:
        """
        Check that the object rests at the release point; correct it if not.

        Placed means within place_tolerance of the release point horizontally
        and not fallen far below it. Otherwise (e.g. it is still attached and
        followed the arm, or it was knocked off) it is detached again and put
        back, up to attach.attempts times.
        """
        err = ''
        for attempt in range(self._attach_attempts + 1):
            self._wait_pose_updates(2)
            pose = self._get_pose(object_name)
            if pose is None:
                return False, 'object not in pose registry.'
            off = math.hypot(pose[0] - release[0], pose[1] - release[1])
            drop = release[2] - pose[2]
            if off <= self._place_tolerance and drop <= self._place_drop_limit:
                return True, ''
            err = f'it is {off:.2f} m from the release point and {drop:.2f} m below it'
            if attempt == self._attach_attempts or not self._attach_enabled:
                break
            self.get_logger().warn(
                f"[place] '{object_name}' is not in place ({err}); putting it back "
                f'(attempt {attempt + 1} of {self._attach_attempts}).')
            self._put_down(robot, object_name, release, yaw)
        return False, err

    def _expected_held_position(self, robot: str, grasp_offset: float) -> Optional[Vec3]:
        """World position of a held object's centre for the arm's current pose."""
        robot_pose = self._robot_pose(robot)
        tool = self._tool_in_planning_frame(robot)
        if robot_pose is None or tool is None:
            return None
        (px, py, pz), (zx, zy, zz) = tool
        d = self._tcp_offset + grasp_offset
        return _robot_to_world(robot_pose, (px + d * zx, py + d * zy, pz + d * zz))

    def _tool_in_planning_frame(self, robot: str) -> Optional[Tuple[Vec3, Vec3]]:
        """
        Return end_effector_link's (position, z axis) in the planning frame.

        From MoveIt's forward kinematics for the current joint states.
        """
        client = self._fk_clients[robot]
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = self._planning_frame
        request.fk_link_names = [self._end_effector_link]
        request.robot_state.is_diff = True
        future = client.call_async(request)
        if not self._wait_for_future(future, 30.0) or future.result() is None:
            return None
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            return None
        pose = response.pose_stamped[0].pose
        x, y, z, w = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        z_axis = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
        return (pose.position.x, pose.position.y, pose.position.z), z_axis

    def _set_object_pose(self, object_name: str, position: Vec3, yaw: float) -> bool:
        """Teleport a Gazebo model (upright, at `yaw`) with /world/<w>/set_pose."""
        req = GzPose()
        req.name = object_name
        req.position.x, req.position.y, req.position.z = position
        q = _yaw_to_quaternion(yaw)
        req.orientation.z, req.orientation.w = q.z, q.w
        ok, rep = self._gz_node.request(
            f'/world/{self._world_name}/set_pose', req, GzPose, Boolean,
            self._attach_timeout_ms)
        return bool(ok and rep.data)

    def _wait_sim(self, seconds: float) -> None:
        """Wait `seconds` of node-clock (simulation) time, at most 10 s wall time."""
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        waited = 0.0
        while self.get_clock().now() < deadline and waited < 10.0:
            threading.Event().wait(0.005)
            waited += 0.005

    def _wait_pose_updates(self, count: int, timeout_sec: float = 5.0) -> None:
        """Wait (wall time) until `count` new Gazebo pose messages have arrived."""
        with self._pose_lock:
            target = self._pose_seq + count
        waited = 0.0
        while waited < timeout_sec:
            with self._pose_lock:
                if self._pose_seq >= target:
                    return
            threading.Event().wait(0.02)
            waited += 0.02

    # MoveIt helpers
    # MoveIt helpers

    @staticmethod
    def _along(point: Vec3, approach: Tuple[float, float], distance: float) -> Vec3:
        """Move a point `distance` metres along a horizontal approach direction."""
        return (point[0] + distance * approach[0],
                point[1] + distance * approach[1], point[2])

    def _tool_pose(self, tcp: Vec3, approach: Tuple[float, float]) -> Pose:
        """Pose of end_effector_link (tool0) that puts the TCP at `tcp`."""
        tool = self._along(tcp, approach, -self._tcp_offset)
        pose = self._make_pose(*tool)
        pose.orientation = _tool_orientation(*approach)
        return pose

    def _reach_error(self, tcp: Vec3, approach: Tuple[float, float]) -> str:
        """Describe why a TCP target is out of reach ('' if it is reachable)."""
        tool = self._along(tcp, approach, -self._tcp_offset)
        dist = math.dist(tool, self._arm_base)
        if dist > self._max_reach:
            return (f'tool0 would be {dist:.2f} m from the shoulder (max '
                    f'{self._max_reach:.2f} m); navigate closer.')
        return ''

    def _move_tcp_retry(self, robot: str, tcp: Vec3,
                        approach: Tuple[float, float]) -> Tuple[bool, str]:
        """_move_tcp, retried motion_retries times (planning is randomized)."""
        ok, err = False, ''
        for attempt in range(self._motion_retries + 1):
            ok, err = self._move_tcp(robot, tcp, approach)
            if ok:
                return True, ''
            if attempt < self._motion_retries:
                self.get_logger().warn(f'[moveit] {robot}: {err} Retrying.')
        return False, err

    def _stow_retry(self, robot: str) -> Tuple[bool, str]:
        """_stow, retried motion_retries times."""
        ok, err = False, ''
        for _ in range(self._motion_retries + 1):
            ok, err = self._stow(robot)
            if ok:
                return True, ''
        return False, err

    def _move_cartesian_best_effort(self, robot: str, tcps: List[Vec3],
                                    approach: Tuple[float, float], label: str) -> bool:
        """
        Move as far along the straight path as is feasible; never fail the skill.

        Used where the hold does not depend on the arm being exact: the
        object is snapped into the gripper (pick) or onto the release point
        (place) afterwards.
        """
        ok, err = self._move_cartesian(robot, tcps, approach, partial=True)
        if not ok:
            self.get_logger().warn(f'[moveit] {robot}: {label} move incomplete ({err}).')
        return ok

    def _move_cartesian(self, robot: str, tcps: List[Vec3],
                        approach: Tuple[float, float],
                        partial: bool = False) -> Tuple[bool, str]:
        """
        Move the TCP in straight lines through `tcps`, keeping the tool orientation.

        Uses move_group's compute_cartesian_path service, then executes the
        trajectory with the execute_trajectory action. With `partial`, the
        feasible part of the path is executed even if it is shorter than
        min_cartesian_fraction.
        """
        # Straight moves enter fixtures on purpose (shelf tiers, table tops),
        # so they are checked against the floor, the robot itself and the
        # held object only
        ok, err = self._sync_scene(robot, fixtures=False)
        if not ok:
            return False, err
        client = self._cartesian_clients[robot]
        if not client.wait_for_service(timeout_sec=10.0):
            return False, 'compute_cartesian_path service not available.'
        request = GetCartesianPath.Request()
        request.header.frame_id = self._planning_frame
        request.start_state.is_diff = True
        request.group_name = self._move_group_name
        request.link_name = self._end_effector_link
        request.waypoints = [self._tool_pose(tcp, approach) for tcp in tcps]
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self._cartesian_scaling
        request.max_acceleration_scaling_factor = self._cartesian_scaling
        self.get_logger().info(
            f'[moveit] {robot}: cartesian {self._end_effector_link} -> '
            + ' -> '.join(f'({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f})'
                          for p in request.waypoints))
        future = client.call_async(request)
        if not self._wait_for_future(future, self._moveit_timeout) or future.result() is None:
            return False, 'compute_cartesian_path timed out.'
        response = future.result()
        if response.fraction < self._min_cartesian_fraction:
            message = (f'only {response.fraction * 100:.0f}% of the straight-line '
                       'path is feasible.')
            if not partial or len(response.solution.joint_trajectory.points) < 2:
                return False, message
            self.get_logger().warn(f'[moveit] {robot}: {message} Moving that far.')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        result, error = self._run_action(
            self._execute_clients[robot], goal, self._moveit_timeout, 'MoveIt execute')
        if result is None:
            return False, error
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return False, f'MoveIt error code {code} ({self._moveit_error_name(code)}).'
        return True, ''

    def _move_tcp(self, robot: str, tcp: Vec3,
                  approach: Tuple[float, float]) -> Tuple[bool, str]:
        """
        Move the gripper's TCP to `tcp` (planning frame), pointing along `approach`.

        MoveIt plans for end_effector_link (tool0); the TCP sits tcp_offset
        metres further along the tool's z axis, which is the approach.
        """
        tool_pose = self._tool_pose(tcp, approach)
        tool = (tool_pose.position.x, tool_pose.position.y, tool_pose.position.z)
        orientation = tool_pose.orientation

        position = PositionConstraint()
        position.header.frame_id = self._planning_frame
        position.link_name = self._end_effector_link
        position.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE,
                                dimensions=[self._position_tolerance])
        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [self._make_pose(*tool)]
        position.constraint_region = region
        position.weight = 1.0

        orient = OrientationConstraint()
        orient.header.frame_id = self._planning_frame
        orient.link_name = self._end_effector_link
        orient.orientation = orientation
        orient.absolute_x_axis_tolerance = self._orientation_tolerance
        orient.absolute_y_axis_tolerance = self._orientation_tolerance
        orient.absolute_z_axis_tolerance = self._orientation_tolerance
        orient.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orient]
        self.get_logger().info(
            f'[moveit] {robot}: {self._end_effector_link} -> '
            f'({tool[0]:.3f}, {tool[1]:.3f}, {tool[2]:.3f}) in {self._planning_frame}')
        return self._send_moveit_goal(robot, constraints)

    def _box_object(self, object_id: str, frame: str, size: Sequence[float],
                    pose: Pose) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=[float(v) for v in size])]
        obj.primitive_poses = [pose]
        return obj

    def _nearby_fixtures(self, robot: str) -> List[CollisionObject]:
        """
        Collision boxes, in the planning frame, of known fixtures near the robot.

        manipulation.fixture_sizes gives each fixture model's bounding box
        (x, y, z in its own frame, standing on its origin). The planning scene
        has no world frame (its root is base_link), so the boxes are placed
        from the Gazebo poses relative to the robot's current pose.
        """
        robot_pose = self._robot_pose(robot)
        if robot_pose is None or not self._fixture_sizes:
            return []
        with self._pose_lock:
            entities = list(self._pose_registry.items())
        objects = []
        for name, (fx, fy, fz, fyaw) in entities:
            size = _lookup_by_prefix(self._fixture_sizes, name, None)
            if size is None or _model_prefix(name) not in self._fixture_sizes:
                continue
            if math.hypot(fx - robot_pose[0], fy - robot_pose[1]) > self._fixture_radius:
                continue
            centre = _world_to_robot(robot_pose, (fx, fy, fz + size[2] / 2.0))
            pose = self._make_pose(*centre)
            pose.orientation = _yaw_to_quaternion(fyaw - robot_pose[3])
            objects.append(self._box_object(
                f'fixture/{name}', self._planning_frame, size, pose))
        return objects

    def _sync_scene(self, robot: str, fixtures: bool = True) -> Tuple[bool, str]:
        """
        Update MoveIt's planning scene before a planned (free) arm motion.

        * The floor: a slab below base_link. Without it, planned paths may
          sweep the arm through the floor, and the position-controlled arm
          then shoves the whole robot.
        * Known fixtures near the robot (shelves, tables, stands), so the arm
          and the held object are not swung through them.
        * The held object, attached to the tool, so it is not swung into the
          robot or the fixtures either.
        """
        client = self._scene_clients[robot]
        if not client.wait_for_service(timeout_sec=10.0):
            return False, 'apply_planning_scene service not available.'
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        if not self._acm_ready[robot] and self._allowed_pairs:
            acm = self._allowed_collision_matrix(robot)
            if acm is None:
                return False, 'could not read the allowed collision matrix.'
            scene.allowed_collision_matrix = acm
        world = [self._box_object(
            'ground', self._planning_frame, [6.0, 6.0, 0.2],
            self._make_pose(0.0, 0.0, self._ground_z - 0.1))]
        for old in self._scene_fixtures[robot]:
            world.append(CollisionObject(id=old, operation=CollisionObject.REMOVE))
        fixtures = self._nearby_fixtures(robot) if fixtures else []
        world += fixtures
        scene.world.collision_objects = world

        held = self._held[robot]
        attached = AttachedCollisionObject()
        attached.link_name = self._end_effector_link
        if held is not None:
            # Tool frame: x horizontal, y up, z along the approach (the
            # object's depth); the object's centre is past the TCP by the
            # grasp offset.
            width, depth, height = held['size']
            attached.object = self._box_object(
                'held_object', self._end_effector_link, [width, height, depth],
                self._make_pose(0.0, 0.0, self._tcp_offset + held['grasp_offset']))
            attached.touch_links = list(self._touch_links)
            scene.robot_state.attached_collision_objects = [attached]
        elif self._scene_holding[robot]:
            attached.object = CollisionObject(id='held_object',
                                              operation=CollisionObject.REMOVE)
            scene.robot_state.attached_collision_objects = [attached]
            scene.world.collision_objects.append(
                CollisionObject(id='held_object', operation=CollisionObject.REMOVE))

        request = ApplyPlanningScene.Request(scene=scene)
        future = client.call_async(request)
        if not self._wait_for_future(future, 30.0) or future.result() is None:
            return False, 'apply_planning_scene timed out.'
        if not future.result().success:
            return False, 'apply_planning_scene failed.'
        self._scene_fixtures[robot] = [f.id for f in fixtures]
        self._scene_holding[robot] = held is not None
        if scene.allowed_collision_matrix.entry_names:
            self._acm_ready[robot] = True
        return True, ''

    def _allowed_collision_matrix(self, robot: str):
        """
        Return move_group's allowed collision matrix plus allowed_collision_pairs.

        Clearpath's generated SRDF does not disable every adjacent pair of
        the sensors it adds (lidar2d_0_link touches top_plate_link), which
        makes MoveIt refuse plans from a start state "in collision". A
        planning-scene diff replaces the whole matrix, so it is read first.
        """
        client = self._get_scene_clients[robot]
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = client.call_async(request)
        if not self._wait_for_future(future, 30.0) or future.result() is None:
            return None
        acm = future.result().scene.allowed_collision_matrix
        names = list(acm.entry_names)
        rows = [list(e.enabled) for e in acm.entry_values]
        for pair in self._allowed_pairs:
            for link in pair:
                if link not in names:
                    names.append(link)
                    for row in rows:
                        row.append(False)
                    rows.append([False] * len(names))
            i, j = names.index(pair[0]), names.index(pair[1])
            rows[i][j] = rows[j][i] = True
        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        return acm

    def _stow(self, robot: str) -> Tuple[bool, str]:
        """Move the arm to the stow joint positions (skipped if not configured)."""
        if not self._arm_joints or len(self._arm_joints) != len(self._stow_positions):
            return True, ''
        constraints = Constraints()
        for joint, value in zip(self._arm_joints, self._stow_positions):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=joint, position=value, tolerance_above=0.01,
                tolerance_below=0.01, weight=1.0))
        self.get_logger().info(f'[moveit] {robot}: stowing the arm')
        return self._send_moveit_goal(robot, constraints)

    def _send_moveit_goal(self, robot: str,
                          constraints: Constraints) -> Tuple[bool, str]:
        """Plan and execute a MoveGroup goal; returns (success, error detail)."""
        ok, err = self._sync_scene(robot)
        if not ok:
            return False, err
        request = MotionPlanRequest()
        request.group_name = self._move_group_name
        request.goal_constraints = [constraints]
        request.start_state.is_diff = True
        request.num_planning_attempts = self._planning_attempts
        request.allowed_planning_time = self._planning_time
        request.max_velocity_scaling_factor = self._velocity_scaling
        request.max_acceleration_scaling_factor = self._velocity_scaling
        request.workspace_parameters.header.frame_id = self._planning_frame
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 3.0

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        result, error = self._run_action(
            self._moveit_clients[robot], goal, self._moveit_timeout, 'MoveIt')
        if result is None:
            return False, error
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return False, f'MoveIt error code {code} ({self._moveit_error_name(code)}).'
        return True, ''

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        for name in dir(MoveItErrorCodes):
            if name.isupper() and getattr(MoveItErrorCodes, name) == code:
                return name
        return 'UNKNOWN'

    # Gripper helper

    def _send_gripper_goal(self, robot: str, position: float) -> Tuple[bool, str]:
        """
        Command the gripper (0.0 = open, 0.8 = closed).

        Success when the position is reached or the fingers stall on an object.
        """
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self._gripper_max_effort
        result, error = self._run_action(
            self._gripper_clients[robot], goal, self._gripper_timeout, 'Gripper')
        if result is None:
            return False, error
        res = result.result
        if result.status == GoalStatus.STATUS_SUCCEEDED or res.reached_goal or res.stalled:
            return True, ''
        return False, (f'status {result.status}, position {res.position:.3f} '
                       f'(goal {position:.3f})')

    # Attach / detach (Gazebo DetachableJoint)

    def _gz_publish_empty(self, topic: str, timeout_sec: float = 5.0) -> bool:
        """
        Publish gz.msgs.Empty on a gz-transport topic once it has a subscriber.

        gz-transport drops messages published before discovery has connected
        the subscriber, so a new publisher waits (wall time) for a connection.
        Returns False if no subscriber appeared.
        """
        pub = self._gz_publishers.get(topic)
        if pub is None:
            pub = self._gz_node.advertise(topic, Empty)
            self._gz_publishers[topic] = pub
        waited = 0.0
        while not pub.has_connections() and waited < timeout_sec:
            threading.Event().wait(0.05)
            waited += 0.05
        if not pub.has_connections():
            return False
        pub.publish(Empty())
        return True

    def _attach_topics(self, robot: str, object_name: str) -> Tuple[str, str]:
        ns = self._robot_instances[robot].strip('/')
        token = _sanitize_topic_token(object_name)
        return f'/{ns}/attach/{token}/attach', f'/{ns}/attach/{token}/detach'

    def _watch_joint_state(self, object_name: str) -> None:
        """Track the DetachableJoint state of an object ('attached' / 'detached')."""
        if object_name in self._joint_state_watched:
            return
        topic = f'/model/{object_name}/detachable_joint/state'

        def callback(msg: StringMsg, name: str = object_name) -> None:
            self._joint_states[name] = msg.data

        if self._gz_node.subscribe(StringMsg, topic, callback):
            self._joint_state_watched.add(object_name)

    def _wait_joint_state(self, object_name: str, wanted: str) -> Optional[bool]:
        """
        Wait for an object's DetachableJoint to report `wanted`.

        True once it does; False if it reported something else until the
        timeout; None if it never reported a state (then the caller relies on
        the pose checks).
        """
        deadline = self.get_clock().now() + Duration(seconds=self._joint_state_timeout)
        waited = 0.0
        while self.get_clock().now() < deadline and waited < 10.0:
            if self._joint_states.get(object_name) == wanted:
                return True
            threading.Event().wait(0.02)
            waited += 0.02
        state = self._joint_states.get(object_name)
        return None if state is None else state == wanted

    def _attach(self, robot: str, object_name: str) -> Tuple[bool, str]:
        """
        Fix `object_name` to the wrist with a Gazebo DetachableJoint.

        The first time an object is picked, a DetachableJoint system is added to
        the robot model; it attaches the object at its current relative pose as
        soon as it loads. Later picks of the same object reuse that system
        through its attach topic, also when it was added by an earlier run of
        this node (the system stays loaded in Gazebo). The joint's state topic
        confirms the attachment; the request is repeated once if it did not.
        """
        attach_topic, detach_topic = self._attach_topics(robot, object_name)
        self._watch_joint_state(object_name)
        self._joint_states.pop(object_name, None)

        if object_name in self._attach_plugins[robot]:
            if not self._gz_publish_empty(attach_topic):
                return False, f'nothing subscribed to {attach_topic}.'
        elif self._gz_publish_empty(attach_topic, timeout_sec=2.0):
            self.get_logger().info(
                f"[attach] Reusing the DetachableJoint already loaded for '{object_name}'.")
            self._attach_plugins[robot][object_name] = (attach_topic, detach_topic)
        else:
            # By id: Gazebo does not resolve names with a '/' such as
            # 'a200_0000/robot' and would load the system on the world.
            with self._pose_lock:
                robot_id = self._entity_ids.get(self._robot_model_name(robot))
            if not robot_id:
                return False, f"no Gazebo entity id for '{self._robot_model_name(robot)}'."
            req = EntityPlugin_V()
            req.entity.id = robot_id
            req.entity.type = Entity.MODEL
            plugin = req.plugins.add()
            plugin.name = 'gz::sim::systems::DetachableJoint'
            plugin.filename = 'gz-sim-detachable-joint-system'
            plugin.innerxml = (
                f'<parent_link>{self._attach_parent_link}</parent_link>'
                f'<child_model>{object_name}</child_model>'
                f'<child_link>{self._attach_child_link}</child_link>'
                f'<detach_topic>{detach_topic}</detach_topic>'
                f'<attach_topic>{attach_topic}</attach_topic>'
            )
            service = f'/world/{self._world_name}/entity/system/add'
            ok, rep = self._gz_node.request(
                service, req, EntityPlugin_V, Boolean, self._attach_timeout_ms)
            if not ok or not rep.data:
                return False, f'{service} failed.'
            self._attach_plugins[robot][object_name] = (attach_topic, detach_topic)
            # Advertise the detach topic now, so it is connected at place time
            self._gz_publishers.setdefault(
                detach_topic, self._gz_node.advertise(detach_topic, Empty))

        state = self._wait_joint_state(object_name, 'attached')
        if state is False:
            self.get_logger().warn(f"[attach] '{object_name}' not attached yet; retrying.")
            self._gz_publish_empty(attach_topic)
            if self._wait_joint_state(object_name, 'attached') is False:
                return False, 'the DetachableJoint did not attach.'
        self._attached[robot][object_name] = (attach_topic, detach_topic)
        self.get_logger().info(f"[attach] '{object_name}' attached to {robot}.")
        return True, ''

    def _release(self, robot: str, object_name: str) -> Tuple[bool, str]:
        """Forget the held object and detach it in Gazebo (if attaching is on)."""
        held = self._held[robot]
        if held is not None and held['name'] == object_name:
            self._held[robot] = None
        if not self._attach_enabled:
            return True, ''
        return self._detach(robot, object_name)

    def _detach(self, robot: str, object_name: str) -> Tuple[bool, str]:
        """
        Release `object_name` from the wrist.

        Sends the detach request whenever a DetachableJoint for the object
        exists (also if this node thinks it is not attached, so a stale joint
        is always released) and confirms it on the joint's state topic,
        repeating the request once. Fails only if the joint still reports
        'attached'; then the object must not be teleported.
        """
        topics = self._attached[robot].pop(object_name, None)
        if topics is None and object_name not in self._attach_plugins[robot]:
            return True, ''
        _, detach_topic = self._attach_topics(robot, object_name)
        if not self._gz_publish_empty(detach_topic):
            # No DetachableJoint listens for this object, so none holds it
            self.get_logger().warn(f'[detach] Nothing subscribed to {detach_topic}.')
            return True, ''
        state = self._wait_joint_state(object_name, 'detached')
        if state is False:
            self._gz_publish_empty(detach_topic)
            if self._wait_joint_state(object_name, 'detached') is False:
                if topics is not None:
                    self._attached[robot][object_name] = topics
                return False, 'the DetachableJoint did not detach.'
        self.get_logger().info(f"[detach] '{object_name}' released by {robot}.")
        return True, ''

    # Feedback publisher

    def _publish_feedback(self, plan_id: str, task_id: str, robot: str,
                          status: str, stage: str, detail: str = '') -> None:
        """Publish a feedback message to /mission/feedback."""
        msg = String()
        msg.data = json.dumps({
            'plan_id': plan_id,
            'task_id': task_id,
            'robot': robot,
            'status': status,
            'stage': stage,
            'detail': detail,
        })
        self._feedback_pub.publish(msg)
        self.get_logger().info(
            f'[feedback] task={task_id} status={status} stage={stage} detail={detail}')

    @staticmethod
    def _make_pose(x: float, y: float, z: float) -> Pose:
        """Build a geometry_msgs/Pose at (x, y, z) with identity orientation."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MobileRobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
