# Front-Side Shelf Pick-and-Place Implementation Plan

## 1. Overview & Objective
The goal is to implement an autonomous, collision-aware pick-and-place pipeline using **`bcr_arm`** (7-DOF), the **Robotiq 2F-85** gripper, and **MoveIt 2** in **Gazebo Harmonic**.

* **Target Object**: 5 cm × 5 cm × 5 cm pickable cube ($m = 0.1\text{ kg}$) located on the shelf at $(x=0.50, y=0.00, z=0.425\text{ m})$.
* **Tool Frame**: `grasp_tcp` (Convention A: $+Z_{\text{tcp}}$ approach, $+X_{\text{tcp}}$ horizontal jaw opening, $+Y_{\text{tcp}}$ up).
* **Target Drop Location**: Designated place location (e.g. adjacent table or drop zone $(x_d, y_d, z_d)$).

```mermaid
flowchart LR
    A["Ready Pose"] --> B["Pre-Grasp Stand-off"]
    B --> C["Cartesian Insertion"]
    C --> D["Grasp & Attach"]
    D --> E["Vertical Lift & Retract"]
    E --> F["Transit to Place Zone"]
    F --> G["Place Descent"]
    G --> H["Release & Detach"]
    H --> I["Retreat & Return Ready"]
```

---

## 2. Subtask Breakdown

### Phase 1: Planning Scene & Collision Representation
Before planning any motions near or inside the shelf, MoveIt's planning scene must reflect the actual environment geometries to prevent accidental collisions.

* [ ] **Subtask 1.1: Define Planning Scene Objects**
  * Add the shelf model as a collision object in MoveIt's `PlanningSceneInterface` (base floor $z=0.40\text{ m}$, roof, back, and side walls).
  * Add the cube as an initial collision object at $(0.50, 0.00, 0.425)$.
  * Add the place surface / bin at the desired drop destination.
* [ ] **Subtask 1.2: Planning Scene Verification**
  * Verify in RViz / MoveIt scene monitor that the planning scene accurately mirrors the Gazebo world.
  * Verify that allowed collision matrix (ACM) ignores false collisions between `pickable_cube` and `grasp_tcp` / fingertips during the grasp phase.

---

### Phase 2: Gripper Control Client & Contact Detection
A dedicated gripper controller interface is needed to command open/close states and detect physical grasp completion.

* [ ] **Subtask 2.1: Gripper Trajectory Action / Topic Client**
  * Implement an asynchronous helper class `GripperInterface` sending trajectories to `/gripper_controller/joint_trajectory`.
  * Define standard aperture poses:
    * `open()`: $q = 0.00\text{ rad}$ ($85\text{ mm}$ aperture).
    * `close()`: $q_{\text{cmd}} = 0.80\text{ rad}$ (commands full closure to apply gripping effort).
* [ ] **Subtask 2.2: Grasp Verification & Stall Check**
  * Subscribe to `/joint_states`.
  * Compute expected stall angle for a 50 mm cube:
    $$w_{\text{cube}} = 50\text{ mm} \implies q_{\text{expected}} \approx 0.45 - 0.55\text{ rad}$$
  * Verify that the gripper has gripped the cube if:
    1. Knuckle velocity drops to $|\dot{q}| < 0.01\text{ rad/s}$ while commanded effort is active.
    2. Final joint position falls within $0.35\text{ rad} \le q_{\text{final}} \le 0.65\text{ rad}$ (not fully open, not fully closed).

---

### Phase 3: Pick Sequence Pipeline
Execute a deterministic approach, grasp, and extraction sequence.

* [ ] **Subtask 3.1: Pre-Grasp Stand-off Pose**
  * Target pose for `grasp_tcp`:
    $$\mathbf{p}_{\text{standoff}} = [x_c - 0.12\text{ m}, y_c, z_c] = [0.38, 0.00, 0.425]$$
  * Orientation (Convention A straight-ahead):
    $$\mathbf{q}_{\text{grasp}} = [0.0, -0.7071, 0.0, 0.7071]$$
  * Plan collision-free trajectory from `ready` pose to `standoff` using OMPL / MoveIt `move_group`.
* [ ] **Subtask 3.2: Straight Cartesian Approach**
  * Generate a dense Cartesian path from $\mathbf{p}_{\text{standoff}}$ to $\mathbf{p}_{\text{grasp}} = [0.50, 0.00, 0.425]$ along $+Z_{\text{tcp}}$.
  * Validate path linearity ($J_1$ strictly constant, purely planar motion in $J_2, J_4, J_6$) to prevent lateral jaw collision with shelf walls.
  * Execute insertion trajectory.
* [ ] **Subtask 3.3: Close Gripper & Attach Object**
  * Command gripper `close()`.
  * Confirm grasp using stall detection (Subtask 2.2).
  * Attach `pickable_cube` collision object to the gripper group in MoveIt's planning scene so future motion planners treat the cube as part of the robot's collision geometry.
* [ ] **Subtask 3.4: Micro-Lift & Cartesian Retract**
  * Compute clearance to upper shelf shelf roof:
    $$\Delta z_{\text{lift}} = +0.03\text{ m} \implies z_{\text{lift}} = 0.455\text{ m}$$
  * Execute vertical Cartesian lift of $3\text{ cm}$.
  * Execute straight Cartesian retreat along $-Z_{\text{tcp}}$ back to $x \le 0.35\text{ m}$, completely clearing the shelf volume.

---

### Phase 4: Place Sequence Pipeline
Transfer the cube to the target drop location and release.

* [ ] **Subtask 4.1: Transit to Pre-Place Pose**
  * Plan free-space collision-avoiding trajectory from shelf exit to pre-place pose:
    $$\mathbf{p}_{\text{pre\_place}} = [x_d, y_d, z_d + 0.10\text{ m}]$$
* [ ] **Subtask 4.2: Cartesian Place Descent**
  * Compute Cartesian linear descent from $\mathbf{p}_{\text{pre\_place}}$ down to $\mathbf{p}_{\text{place}} = [x_d, y_d, z_d + 0.025\text{ m}]$.
  * Execute descent smoothly at low velocity ($v \le 0.05\text{ m/s}$).
* [ ] **Subtask 4.3: Detach & Release**
  * Detach `pickable_cube` from the gripper in the planning scene.
  * Command gripper `open()`.
  * Confirm joint position returns to $q \approx 0.00\text{ rad}$.
* [ ] **Subtask 4.4: Post-Place Retreat & Home**
  * Linear Cartesian lift $+10\text{ cm}$ above place position.
  * Return arm to standard `ready` or `upright` configuration.

---

### Phase 5: Verification & End-to-End Validation
* [ ] **Subtask 5.1: Simulation Smoke Test**
  * Launch `test_shelf.launch.py`.
  * Execute the pick-and-place script.
  * Assert in Gazebo that the cube cleanly separates from the shelf, lifts without dropping, and comes to rest at the desired place coordinates within $\pm 1.5\text{ cm}$ tolerance.
* [ ] **Subtask 5.2: Repeatability & Log Validation**
  * Verify no controller error aborts or planning scene collision warnings occur in ROS logs.