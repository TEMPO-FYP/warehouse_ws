# robot_controllers

Executes TEMPO subtasks on the simulated mobile manipulator `mobile_robot_1`
(Clearpath Husky A200 + UR5e + Robotiq 2F-85, namespace `a200_0000`) with
Nav2, MoveIt and the gripper controller, in the warehouse or the empty test
world.

## Running

The world and the robot are launched by `warehouse_sim`; this package adds
Nav2, MoveIt and the controller on top. Two terminals, both with
`source ~/warehouse_ws/install/setup.bash`:

```bash
# 1. World + robot (warehouse_sim)
ros2 launch warehouse_sim warehouse_clearpath.launch.py                     # warehouse
ros2 launch warehouse_sim warehouse_clearpath.launch.py \
  world_file:=$(ros2 pkg prefix warehouse_sim)/share/warehouse_sim/worlds/empty.sdf   # empty world
#   headless:=true  Gazebo server only (no GUI)

# 2. Localization, Nav2, MoveIt and the controller
ros2 launch robot_controllers mobile_robot_bringup.launch.py world:=warehouse   # or world:=empty
#   controller:=false  without mobile_robot_controller
```

`mobile_robot_bringup.launch.py` can be started right away: it waits
(`wait_for_robot`) until the robot is up (lidar scan, joint states, the four
ros2_control controllers), then starts map_server + AMCL → Nav2 → MoveIt +
`mobile_robot_controller`, each stage waiting for the previous one to be
active. Starting everything at once floods DDS discovery and loses lifecycle
service replies, which leaves Nav2 half-configured. The controller stows the
arm once MoveIt is up (it spawns stretched out in the lidar plane). It is
ready when it logs `[startup] mobile_robot_1: arm stowed.`

If you spawn the robot somewhere else (x:=, y:=, yaw:= of
`warehouse_clearpath.launch.py`), pass the same pose to the bringup as
initial_x:=, initial_y:=, initial_yaw:=.

`config/worlds.yaml` holds each world's map and spawn pose. The maps are
in Gazebo world coordinates, so Gazebo entity poses are Nav2 goals and AMCL
starts at the spawn pose. `maps/empty_map.*` is generated from
`warehouse_sim/worlds/empty.sdf`:

```bash
python3 scripts/make_world_map.py ../warehouse_sim/worlds/empty.sdf maps/empty_map
```

## Skills

Subtasks arrive on `/mission/subtask` (JSON, see AGENTS.MD) and feedback goes
to `/mission/feedback` with `status` `in_progress` / `success` / `failed` and
`stage` `complete` when done. Names are Gazebo entity names.

| skill | args | what happens |
| --- | --- | --- |
| `navigate_to` | `location` | Nav2 goal at a standoff pose facing the entity (shelves from their open side, never from the side blocked by a back-to-back twin), then fine positioning |
| `pick` | `object` | open gripper → planned move to a pre-grasp pose → straight in → snap the object into the gripper if the arm is off, attach → close → straight lift and pull-back → hold checked (re-snapped and re-attached if lost) → base backs off 0.3 m → carry pose → hold checked again |
| `place` | `object`, `location` | planned move to a pre-place pose → straight down → detach and set the object exactly on the release point → open → straight pull-back → stow → placement checked (put back if off) |

Example (empty world), with `ros2 topic echo /mission/feedback` in another
terminal; send the next subtask once the previous one reports
`status: success, stage: complete`:

```bash
ros2 topic pub --once -w 1 /mission/subtask std_msgs/msg/String \
  '{data: "{\"plan_id\": \"test\", \"task_id\": \"T1\", \"robot\": \"mobile_robot_1\", \"required_skills\": [\"navigate_to\"], \"args\": {\"location\": \"pick_stand\"}}"}'
# then: pick {"object": "test_box"}, navigate_to {"location": "place_stand"},
#       place {"object": "test_box", "location": "place_spot"}
```

Warehouse: `navigate_to Picking_Shelves.027` → `pick small_box.027_2` →
`navigate_to Packing_Station.002` → `place small_box.027_2 Packing_Station.002`.

## How it works (and why)

* **Poses** come from Gazebo (`/world/default/pose/info` over gz-transport).
  Arm targets are computed from the robot's true pose, so they do not depend
  on localization.
* **Holding (attach / detach)**: the 2F-85 opens to 85 mm and the warehouse
  boxes are 0.2–0.3 m wide, and the arm never ends up exactly on target, so
  pick and place do not rely on the fingers or on arm precision:
  * Pick: the object is fixed to the wrist with a Gazebo `DetachableJoint`
    (added to the robot at the first pick through
    `/world/default/entity/system/add`, reused afterwards, also across
    controller restarts). If the object is more than `attach.snap_tolerance`
    from where the gripper holds it, it is first teleported there
    (`/world/default/set_pose`, never downwards into its support). After the
    lift and again at the carry pose the controller checks that the object
    is in the gripper and, if not, snaps and re-attaches it (up to
    `attach.attempts` times).
  * Place: the object is detached and teleported exactly onto the release
    point, aligned with the location. Planned and straight arm motions are
    best effort; if MoveIt cannot reach the place, the object is still put
    there. At the end the controller checks the object is at the release
    point and puts it back if it was dragged or knocked off.
  * Every attach / detach is confirmed on the joint's state topic
    (`/model/<object>/detachable_joint/state`) and repeated once if needed.
  * A pick or place still fails, by design, when the robot is not within
    reach (`navigate_to` first) or the arm cannot get within
    `attach.snap_max_distance` of the object.
  `manipulation.attach.enabled: false` turns all of this off.
* **Arm motions**: the gripper approaches horizontally. Approach, lift and
  pull-back are straight Cartesian moves (`compute_cartesian_path` +
  `execute_trajectory`); only the moves to pre-grasp / pre-place and the stow
  are planned. Before every planned move the controller updates MoveIt's
  planning scene with the floor, the known fixtures near the robot
  (`manipulation.fixture_sizes`) and the held object, so the arm cannot sweep
  through the floor or a shelf. (The URDF root is `base_link`, so the planning
  scene moves with the robot.)
* **Place height**: `manipulation.place_heights` lists candidate heights per
  location (e.g. shelf tiers); the first one without a copy of the held
  object's model already there is used.
* **Reach**: targets more than `max_reach` from the shoulder fail fast with
  "navigate closer".
* **Fine positioning** (`navigation.fine_positioning`): Nav2 stops within its
  goal tolerance of where AMCL thinks the robot is, which can be ~0.35 m off
  in the warehouse. The controller then turns the robot to face the target
  and creeps straight to the exact standoff distance on `cmd_vel`, using
  Gazebo's true pose (within 3 cm). Simulation only.

## Slow-simulation settings

The warehouse runs at ~0.2× real time on CPU. `mobile_robot_bringup.launch.py`
therefore relaxes Clearpath's Nav2 parameters (BT server timeout, progress
checker, transform tolerances, lifecycle bond timeout) and starts `move_group`
itself with trajectory execution-duration monitoring off (MoveIt measures it
in wall time and would cancel every arm motion). All controller timeouts are
in simulation time.

## Tests

```bash
cd src/robot_controllers && python3 -m pytest test/
```

No simulation needed: helpers, the pose registry, release-point selection
and the exact MoveIt / planning-scene messages are tested offline.
