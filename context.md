# Warehouse Workspace Project Context

> **Last Updated**: 2026-11-17  
> **Environment**: Ubuntu 24.04 (Noble), ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8 + DART physics)  
> **Docker Container**: `warehouse_ws` (network=host, Ubuntu 24.04, ROS 2 Jazzy)  
> **Host Path**: `/home/zain/warehouse_ws` (Mounted to container: `/root/warehouse_ws/src`)

---

## 1. Project Overview & Objectives

This workspace integrates a 7-DOF manipulator (`bcr_arm`) and a Robotiq 2F-85 parallel-jaw gripper into Gazebo Harmonic simulation with MoveIt 2. The primary objective is to implement an autonomous, front-facing pick-and-place manipulation pipeline capable of picking inventory boxes from warehouse shelves and placing them on destination surfaces.

The manipulation subsystem is designed to operate both:
1. In a minimal **unit test environment** ([test_shelf_world.sdf](file:///home/zain/warehouse_ws/src/warehouse_sim/worlds/test_shelf_world.sdf)) to isolate kinematics, collisions, and trajectory execution.
2. In the full **TEMPO (Task Execution & Motion Planning Orchestration)** warehouse world ([warehouse.sdf](file:///home/zain/warehouse_ws/src/warehouse_sim/worlds/warehouse.sdf)), receiving grounded subtasks from the LLM planner DAG dispatcher.

---

## 2. System Architecture & Workspaces

### Container & Build Architecture
* **Source code**: Host `/home/zain/warehouse_ws/src` is bind-mounted to Docker `/root/warehouse_ws/src`.
* **Build / Install**: Build artifacts (`build/` and `install/`) are container-local (not bind-mounted). When adding new Python modules, node entry points in [setup.py](file:///home/zain/warehouse_ws/src/warehouse_sim/setup.py), or dependencies in [package.xml](file:///home/zain/warehouse_ws/src/warehouse_sim/package.xml):
  * Build command inside container:
    ```bash
    docker exec -it -w /root/warehouse_ws warehouse_ws bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select warehouse_sim"
    ```
  * In active container terminal sessions, refresh the environment:
    ```bash
    source /root/warehouse_ws/install/setup.bash
    ```

### Key Packages
* [warehouse_sim](file:///home/zain/warehouse_ws/src/warehouse_sim): Simulation worlds, launch scripts, planning scene manager, pick-and-place nodes.
* [bcr_arm](file:///home/zain/warehouse_ws/src/bcr_arm): 7-DOF arm description, Gazebo simulation plugins, controller configurations.
* [bcr_arm_moveit_config](file:///home/zain/warehouse_ws/src/bcr_arm/bcr_arm_moveit_config): MoveIt 2 configuration, SRDF, kinematics (`KDLKinematicsPlugin`), OMPL planning pipelines.
* [robotiq_description](file:///home/zain/warehouse_ws/src/robotiq_description): Robotiq 2F-85 URDF/Xacro macros, collision primitives, and grasp TCP frame definitions.
* [llm_planner](file:///home/zain/warehouse_ws/src/llm_planner): TEMPO LLM orchestrator, scene graph grounder, SDF parser, and mission DAG builder.
* [dispatcher_node](file:///home/zain/warehouse_ws/src/dispatcher_node): Sequences and dispatches subtasks to robot execution workers.

---

## 3. End-Effector & Kinematic Conventions

### Coordinate Frames & Convention A
* **Approach Vector**: $+Z_{\text{tcp}}$ (points outward from the palm through the fingers).
* **Finger Opening Vector**: $+X_{\text{tcp}}$ (lateral axis along which fingers separate and close).
* **Normal / Vertical Vector**: $+Y_{\text{tcp}}$ (perpendicular to opening and approach).
* **Straight-Ahead Grasp Orientation** (in `world` / `base_link`):
  $$\mathbf{q}_{\text{grasp}} = [x, y, z, w] = [0.0, -0.7071, 0.0, 0.7071]$$

### Gripper Mounting & Grasp Frame
* **Flange Orientation**: Mounted at `rpy="1.5707963 0 1.5707963"` on the arm flange to keep jaws horizontal (side-by-side) when the wrist is in default orientation.
* **Grasp Center TCP**: Defined in [robotiq_2f_85_macro.urdf.xacro](file:///home/zain/warehouse_ws/src/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro) as `grasp_tcp` at offset `xyz="0 0 0.1246" rpy="0 0 0"` relative to `robotiq_85_base_link`.
* **Planning Group**: `bcr_arm` end-effector is configured in [bcr_arm.srdf](file:///home/zain/warehouse_ws/src/bcr_arm/bcr_arm_moveit_config/config/bcr_arm.srdf) as:
  ```xml
  <end_effector name="gripper" parent_link="flange" group="gripper" parent_group="bcr_arm"/>
  ```

---

## 4. Gazebo DART & ros2_control Configuration

### DART Physics Compatibility
* Complex STL meshes loaded as collision geometry inside Gazebo Harmonic DART physics engine can cause crashes or inaccurate contact resolution.
* **Fingertips**: Replaced mesh collision in [robotiq_2f_85_macro.urdf.xacro](file:///home/zain/warehouse_ws/src/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro) with primitive collision boxes `<box size="0.012 0.022 0.048"/>` offset to pad centers.
* **Base Link**: Replaced mesh collision with primitive cylinder `<cylinder radius="0.0375" length="0.075"/>`.

### Controller & Hardware Mappings
* Configured `<hold_joints>false</hold_joints>` and `<position_proportional_gain>20.0</position_proportional_gain>` in [gazebo_sim_ros2_control.urdf.xacro](file:///home/zain/warehouse_ws/src/bcr_arm/bcr_arm_description/urdf/control/gazebo_sim_ros2_control.urdf.xacro).
* Fixed relative path resolution in [bcr_arm.urdf.xacro](file:///home/zain/warehouse_ws/src/bcr_arm/bcr_arm_description/urdf/robots/bcr_arm.urdf.xacro) to use `$(find bcr_arm_moveit_config)/config/ros2_controllers.yaml`.
* MoveIt launch in [test_shelf.launch.py](file:///home/zain/warehouse_ws/src/warehouse_sim/launch/test_shelf.launch.py) passes explicit `mappings={"ros2_control_hardware_type": "mock_components"}` to `.robot_description(...)` and passes `moveit_config.to_dict()` to `rviz_node` so RViz MotionPlanning dropdowns populate properly.

---

## 5. MoveIt 2 & Planning Scene Management (Phase 1 Completed)

### Launch Sequencing ([test_shelf.launch.py](file:///home/zain/warehouse_ws/src/warehouse_sim/launch/test_shelf.launch.py))
To prevent premature startup before Gazebo joint state publishers are active:
1. Gazebo Harmonic spawns world and robot.
2. `robot_state_publisher` and controller spawners (`arm_controller`, `gripper_controller`) start.
3. MoveIt `move_group_node` and `rviz_node` are triggered via `OnProcessExit(spawn_gripper_controller_node)` (and are not duplicated in the top-level `LaunchDescription`).
4. `use_sim_time: true` is enforced across all MoveIt nodes.

### Planning Scene Manager ([planning_scene_manager.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/planning_scene_manager.py))
* Synchronous Python wrapper around `/apply_planning_scene` (`moveit_msgs/srv/ApplyPlanningScene`) and `/get_planning_scene` (`moveit_msgs/srv/GetPlanningScene`).
* Defines `DEFAULT_GRIPPER_TOUCH_LINKS`:
  * `grasp_tcp`, `robotiq_85_base_link`, `robotiq_85_left_finger_tip_link`, `robotiq_85_right_finger_tip_link`, `robotiq_85_left_finger_link`, `robotiq_85_right_finger_link`, `robotiq_85_left_inner_knuckle_link`, `robotiq_85_right_inner_knuckle_link`, `robotiq_85_left_knuckle_link`, `robotiq_85_right_knuckle_link`.
* Key methods:
  * `add_box(name, dimensions, position, orientation, frame_id="world")`
  * `add_cylinder(name, height, radius, position, orientation, frame_id="world")`
  * `remove_object(name)`
  * `attach_object(name, link_name="grasp_tcp", touch_links=None)`
  * `detach_object(name)`
  * `get_planning_scene(components)`
  * `set_allowed_collisions(target_name, other_names, allowed=True)` — queries current `AllowedCollisionMatrix`, preserves SRDF self-collision rules, expands matrix symmetrically for new world objects, and applies diff.
  * `allow_gripper_collisions(object_name, allowed=True, extra_links=None)`
  * `verify_planning_scene(expected_objects, acm_target, expected_allowed_links)`
  * `clear_all()`

### Test World Scene & Verification ([load_test_shelf_scene.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/load_test_shelf_scene.py))
* Injects the 4 test world collision geometries matching [test_shelf_world.sdf](file:///home/zain/warehouse_ws/src/warehouse_sim/worlds/test_shelf_world.sdf):
  1. `ground_plane`: size `[10.0, 10.0, 0.10]` at `[0.0, 0.0, -0.05]`
  2. `shelf_table`: size `[0.50, 0.80, 0.40]` at `[0.55, 0.00, 0.20]` (top surface at $z=0.40\text{ m}$, front edge at $x=0.30\text{ m}$)
  3. `pickable_cube`: size `[0.05, 0.05, 0.05]` at `[0.50, 0.00, 0.425]` (50 mm cube resting on `shelf_table`)
  4. `place_surface`: size `[0.30, 0.30, 0.40]` at `[0.00, 0.50, 0.20]` (top surface at $z=0.40\text{ m}$)
* Configures ACM via `allow_gripper_collisions("pickable_cube", allowed=True, extra_links=["shelf_table", "place_surface"])`.
* Automatically verifies scene geometries and ACM rules via `verify_test_shelf_scene()`.
* CLI executables registered in [setup.py](file:///home/zain/warehouse_ws/src/warehouse_sim/setup.py):
  * `ros2 run warehouse_sim planning_scene_manager`
  * `ros2 run warehouse_sim load_scene`
  * `ros2 run warehouse_sim verify_scene`

---

## 6. TEMPO Integration: Scaling to Full Warehouse

In the full TEMPO environment ([warehouse.sdf](file:///home/zain/warehouse_ws/src/warehouse_sim/worlds/warehouse.sdf)), static coordinates are replaced by dynamic queries:
1. **Hierarchical 3D Scene Graph**: [parse_sdf_to_scene_graph.py](file:///home/zain/warehouse_ws/src/llm_planner/tools/parse_sdf_to_scene_graph.py) parses the warehouse SDF into [warehouse_scene_graph.json](file:///home/zain/warehouse_ws/src/llm_planner/knowledge_base/warehouse_scene_graph.json) (`Warehouse -> Zone -> Aisle -> Fixture -> Object`), storing 6D poses, bounding boxes, and shelf tiers (e.g., `small_box_000_1` on `Picking_Shelves` in `aisle_1` of `picking_zone`).
2. **Stage 0 Grounder**: [scene_graph_grounder.py](file:///home/zain/warehouse_ws/src/llm_planner/llm_planner/scene_graph_grounder.py) resolves natural language instructions into grounded entity IDs, 6D poses, and parent fixtures, and [dispatcher_node.py](file:///home/zain/warehouse_ws/src/dispatcher_node/dispatcher_node/dispatcher_node.py) dispatches waves on `/mission/subtask`.
3. **Dynamic ROI Proximity Filtering**: Rather than loading hundreds of warehouse fixtures into MoveIt, the manipulation node filters the Scene Graph for fixtures and objects within $R \le 1.5 - 2.0\text{ m}$ of the robot arm base, injecting only the active shelf bay, target box, and nearby obstacles via `PlanningSceneManager`.
4. **TF Resolution**: World poses are transformed dynamically into `base_link` using `tf2_ros`.
5. **Scene Cleanup**: On wave completion, old shelf collision objects are removed via `remove_object()` before transiting to the next station.

---

## 7. DDS & Subnet Isolation Advisory

* The host machine is connected to a shared `/21` subnet (`10.20.4.221/21` — 2,046 host IPs).
* The `warehouse_ws` Docker container uses `--net=host` with `export ROS_DOMAIN_ID=0` in `/root/.bashrc`.
* **Advisory**: To prevent external ROS 2 nodes on the campus/office network from colliding with `/clock`, `/tf`, or MoveIt action servers:
  * Set `export ROS_LOCALHOST_ONLY=1` in both host and container `/root/.bashrc`, OR
  * Use a dedicated `export ROS_DOMAIN_ID=<ID>` (e.g., `42`).

---

## 8. Implementation Roadmap & Remaining Phases

### Phase 1: Planning Scene & Collision Representation (COMPLETED)
- [x] **Subtask 1.1**: Package dependencies (`moveit_msgs`, `geometry_msgs`, `shape_msgs`, `trajectory_msgs`, `sensor_msgs`) in [package.xml](file:///home/zain/warehouse_ws/src/warehouse_sim/package.xml).
- [x] **Subtask 1.2**: MoveIt 2 `move_group` & RViz integrated into [test_shelf.launch.py](file:///home/zain/warehouse_ws/src/warehouse_sim/launch/test_shelf.launch.py) with `use_sim_time:=true` and controller exit sequencing.
- [x] **Subtask 1.3**: `PlanningSceneManager` utility implemented in [planning_scene_manager.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/planning_scene_manager.py).
- [x] **Subtask 1.4**: Test world collision geometries (`ground_plane`, `shelf_table`, `pickable_cube`, `place_surface`) defined in [load_test_shelf_scene.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/load_test_shelf_scene.py).
- [x] **Subtask 1.5**: Allowed Collision Matrix (ACM) and `DEFAULT_GRIPPER_TOUCH_LINKS` configured for `pickable_cube` and support surfaces.
- [x] **Subtask 1.6**: Automated scene & ACM verification (`verify_planning_scene` / `ros2 run warehouse_sim verify_scene`) implemented.

### Phase 2: Gripper Control Client & Contact Detection (COMPLETED)
- [x] **Subtask 2.1**: Implemented `GripperInterface` in [gripper_interface.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/gripper_interface.py) publishing to `/gripper_controller/joint_trajectory` (`open()`: $q = 0.00\text{ rad}$, `close()`: $q_{\text{cmd}} = 0.80\text{ rad}$), plus Gazebo pad friction tags (`mu1=10.0, mu2=10.0`) and flat base collision box in [robotiq_2f_85_macro.urdf.xacro](file:///home/zain/warehouse_ws/src/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro).
- [x] **Subtask 2.2**: Added `/joint_states` stall-based grasp verification (`verify_grasp()`) checking smoothed $|\dot{q}| < 0.01\text{ rad/s}$ and partial-closure position (observed $q_{\text{final}} \approx 0.3627 - 0.3650\text{ rad}$ on the 50 mm cube).

### Phase 3: Pick Sequence Pipeline (COMPLETED)
- [x] **Subtask 3.1**: Implemented `PickAndPlacePipeline.pick_object()` in [pick_and_place.py](file:///home/zain/warehouse_ws/src/warehouse_sim/warehouse_sim/pick_and_place.py) with `BcrArmKinematics` + OMPL collision-free planning from `ready` to outward-wrist front-side stand-off ($\alpha = 18^\circ$ downward pitch, $\mathbf{p}_{\text{standoff}} = [0.42, 0.00, 0.432]$).
- [x] **Subtask 3.2**: Straight Cartesian approach via MoveIt `/compute_cartesian_path` (`link_name="flange"`, `max_cartesian_speed=0.045 m/s`) to $\mathbf{p}_{\text{grasp}} = [0.490, 0.00, 0.432]$ (compensating $-10\text{ mm}$ for Robotiq 4-bar closing arc).
- [x] **Subtask 3.3**: Close gripper, verify stall grasp, and call `attach_object("pickable_cube", link_name="grasp_tcp")`.
- [x] **Subtask 3.4**: Vertical Cartesian micro-lift ($\Delta z = +0.043\text{ m} \to z = 0.475\text{ m}$) followed by straight Cartesian retreat to $x = 0.40\text{ m}$.

### Phase 4: Place Sequence Pipeline (COMPLETED)
- [x] **Subtask 4.1**: Added `place_surface` to [test_shelf_world.sdf](file:///home/zain/warehouse_ws/src/warehouse_sim/worlds/test_shelf_world.sdf) and implemented OMPL free-space transit to $\mathbf{p}_{\text{pre\_place}} = [0.00, 0.489, 0.500]$ (`yaw = +1.571 rad`).
- [x] **Subtask 4.2**: Slow Cartesian linear descent (`max_cartesian_speed=0.04 m/s`) to $\mathbf{p}_{\text{place}} = [0.00, 0.489, 0.432]$.
- [x] **Subtask 4.3**: Detach `pickable_cube` in planning scene, open gripper ($q = 0.00\text{ rad}$), and re-add `pickable_cube` at drop pose with restored ACM rules.
- [x] **Subtask 4.4**: Post-place Cartesian lift to $z = 0.525\text{ m}$ and return arm to `ready` configuration.

### Phase 5: Verification & End-to-End Validation (COMPLETED)
- [x] **Subtask 5.1**: End-to-end simulation smoke test (`ros2 run warehouse_sim pick_and_place`) verified in Gazebo Harmonic: `pickable_cube` picked from `[0.50, 0.00, 0.425]` and placed onto `place_surface` at `[-0.000009, 0.506694, 0.425000]` ($\approx 6.7\text{ mm}$ error, within $\pm 1.5\text{ cm}$ tolerance).
- [x] **Subtask 5.2**: Validated TEMPO integration via live `DispatcherNode` + `TempoArmWorkerNode` (`ros2 run warehouse_sim tempo_arm_worker`) executing Wave 1 (`pick`) $\to$ Wave 2 (`place`) across `/mission/assigned_plan`, `/mission/subtask`, and `/mission/feedback` (final Gazebo pose `[-0.000001, 0.506693, 0.425000]`), with zero controller aborts or runtime collision errors.
- Registered CLI entry points in [setup.py](file:///home/zain/warehouse_ws/src/warehouse_sim/setup.py):
  - `ros2 run warehouse_sim gripper_interface`
  - `ros2 run warehouse_sim pick_and_place` (supports `--object-id`, `--cube-pos`, `--cube-dims`, `--place-pos`, `--pick-only`, `--skip-scene-load`)
  - `ros2 run warehouse_sim tempo_arm_worker`
