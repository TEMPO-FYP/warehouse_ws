#!/bin/bash
# Generate Clearpath Husky A200 assets per namespace, inject Gazebo ground-truth
# OdometryPublisher, disable EKF TF broadcast, and spawn the robot into Gazebo.
#
# Usage:
#   prepare_and_spawn_husky.sh CONFIG_YAML NAMESPACE ROBOT_NAME WORLD_NAME X Y Z YAW TIMEOUT_S SPAWN_DELAY_S

set -euo pipefail

CONFIG_YAML="${1:?Usage: prepare_and_spawn_husky.sh CONFIG_YAML [NAMESPACE] [ROBOT_NAME] [WORLD_NAME] [X] [Y] [Z] [YAW] [TIMEOUT_S] [SPAWN_DELAY_S]}"
NAMESPACE="${2:-a200_0001}"
ROBOT_NAME="${3:-$NAMESPACE}"
WORLD_NAME="${4:-default}"
X="${5:-0.0}"
Y="${6:--5.0}"
Z="${7:-0.15}"
YAW="${8:-0.0}"
TIMEOUT_S="${9:-120}"
SPAWN_DELAY_S="${10:-0.0}"

NAMESPACE="${NAMESPACE#/}"
SETUP_DIR="/tmp/clearpath_${NAMESPACE}"
URDF_FILE="${SETUP_DIR}/robot_abs.urdf"
SDF_FILE="${SETUP_DIR}/robot_abs.sdf"
CREATE_SERVICE="/world/${WORLD_NAME}/create"

if [ ! -s "${CONFIG_YAML}" ]; then
    echo "[prepare_and_spawn_husky] ERROR: Config YAML '${CONFIG_YAML}' is missing or empty."
    exit 1
fi

rm -rf "${SETUP_DIR}"
mkdir -p "${SETUP_DIR}"

# Write per-namespace robot.yaml
python3 - "${CONFIG_YAML}" "${SETUP_DIR}/robot.yaml" "${NAMESPACE}" <<'PYEOF'
import sys
import yaml

src_path, dst_path, ns = sys.argv[1], sys.argv[2], sys.argv[3]
with open(src_path, 'r', encoding='utf-8') as f:
    cfg = yaml.safe_load(f) or {}

system = cfg.setdefault('system', {})
ros2 = system.setdefault('ros2', {})
ros2['namespace'] = ns

with open(dst_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump(cfg, f, sort_keys=False)
PYEOF

HAS_MANIPULATORS=$(python3 - "${SETUP_DIR}/robot.yaml" <<'PYEOF'
import sys
import yaml

with open(sys.argv[1], 'r', encoding='utf-8') as f:
    cfg = yaml.safe_load(f) or {}

arms = (cfg.get('manipulators') or {}).get('arms') or []
print('1' if arms else '0')
PYEOF
)

echo "[prepare_and_spawn_husky] Generating Clearpath files in ${SETUP_DIR} (namespace=${NAMESPACE}, manipulators=${HAS_MANIPULATORS})..."
ros2 run clearpath_generator_common generate_bash -s "${SETUP_DIR}"
ros2 run clearpath_generator_common generate_description -s "${SETUP_DIR}"
ros2 run clearpath_generator_gz generate_launch -s "${SETUP_DIR}"
ros2 run clearpath_generator_gz generate_param -s "${SETUP_DIR}"

if [ "${HAS_MANIPULATORS}" = "1" ]; then
    ros2 run clearpath_generator_common generate_semantic_description -s "${SETUP_DIR}"
    if [ ! -f "${SETUP_DIR}/robot.srdf" ]; then
        echo "[prepare_and_spawn_husky] ERROR: ${SETUP_DIR}/robot.srdf was not generated."
        exit 1
    fi
    python3 - "${SETUP_DIR}/robot.srdf" <<'PYEOF'
import sys
import xml.etree.ElementTree as ET

srdf_path = sys.argv[1]
tree = ET.parse(srdf_path)
root = tree.getroot()

if root.find('virtual_joint') is None:
    vj = ET.Element(
        'virtual_joint',
        {
            'name': 'virtual_joint',
            'type': 'floating',
            'parent_frame': 'odom',
            'child_link': 'base_link',
        },
    )
    root.insert(0, vj)

has_carry = any(
    gs.attrib.get('name') == 'carry' and gs.attrib.get('group') == 'arm_0'
    for gs in root.findall('group_state')
)
if not has_carry:
    carry = ET.SubElement(root, 'group_state', {'name': 'carry', 'group': 'arm_0'})
    carry_joints = [
        ('arm_0_shoulder_pan_joint', '0.0'),
        ('arm_0_shoulder_lift_joint', '-1.5707'),
        ('arm_0_elbow_joint', '0.7854'),
        ('arm_0_wrist_1_joint', '-2.3562'),
        ('arm_0_wrist_2_joint', '-1.5707'),
        ('arm_0_wrist_3_joint', '0.0'),
    ]
    for jname, jval in carry_joints:
        ET.SubElement(carry, 'joint', {'name': jname, 'value': jval})

ET.indent(tree, space='  ')
tree.write(srdf_path, encoding='utf-8', xml_declaration=True)
PYEOF
    echo "[prepare_and_spawn_husky] Patched ${SETUP_DIR}/robot.srdf with virtual_joint and carry state"
fi

if [ ! -f "${SETUP_DIR}/robot.urdf.xacro" ]; then
    echo "[prepare_and_spawn_husky] ERROR: ${SETUP_DIR}/robot.urdf.xacro was not generated."
    exit 1
fi

# Disable Clearpath EKF TF broadcast so Gazebo ground-truth OdometryPublisher owns odom -> base_link
LOC_YAML="${SETUP_DIR}/platform/config/localization.yaml"
if [ -f "${LOC_YAML}" ]; then
    python3 - "${LOC_YAML}" <<'PYEOF'
import sys
import yaml

loc_path = sys.argv[1]
with open(loc_path, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f) or {}

def disable_publish_tf(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == 'publish_tf':
                obj[k] = False
            else:
                disable_publish_tf(v)
    elif isinstance(obj, list):
        for item in obj:
            disable_publish_tf(item)

disable_publish_tf(data)
if isinstance(data.get('ekf_node'), dict):
    data['ekf_node'].setdefault('ros__parameters', {})['publish_tf'] = False

with open(loc_path, 'w', encoding='utf-8') as f:
    yaml.safe_dump(data, f, sort_keys=False)
PYEOF
    echo "[prepare_and_spawn_husky] Disabled EKF publish_tf in ${LOC_YAML}"
fi

# Convert xacro -> URDF -> SDF and resolve model://clearpath_* (and arm/gripper) URIs
USE_MANIP_CTRL="false"
if [ "${HAS_MANIPULATORS}" = "1" ]; then
    USE_MANIP_CTRL="true"
fi

ros2 run xacro xacro "${SETUP_DIR}/robot.urdf.xacro" \
    is_sim:=true \
    "namespace:=${NAMESPACE}" \
    "gazebo_controllers:=${SETUP_DIR}/platform/config/control.yaml" \
    "use_manipulation_controllers:=${USE_MANIP_CTRL}" \
    > "${URDF_FILE}"

if [ ! -s "${URDF_FILE}" ]; then
    echo "[prepare_and_spawn_husky] ERROR: xacro produced an empty URDF."
    exit 1
fi

if [ "${HAS_MANIPULATORS}" = "1" ]; then
    python3 - "${URDF_FILE}" <<'PYEOF'
import sys
import xml.etree.ElementTree as ET

urdf_path = sys.argv[1]
tree = ET.parse(urdf_path)
root = tree.getroot()

initial_positions = {
    'arm_0_shoulder_pan_joint': '0.0',
    'arm_0_shoulder_lift_joint': '-1.5707',
    'arm_0_elbow_joint': '1.5707',
    'arm_0_wrist_1_joint': '-1.5707',
    'arm_0_wrist_2_joint': '-1.5707',
    'arm_0_wrist_3_joint': '0.0',
}

updated = 0
for rc in root.findall('ros2_control'):
    for joint in rc.findall('joint'):
        jname = joint.attrib.get('name', '')
        if jname not in initial_positions:
            continue
        for si in joint.findall('state_interface'):
            if si.attrib.get('name') != 'position':
                continue
            param = next(
                (p for p in si.findall('param') if p.attrib.get('name') == 'initial_value'),
                None,
            )
            if param is None:
                param = ET.SubElement(si, 'param', {'name': 'initial_value'})
            param.text = initial_positions[jname]
            updated += 1

ET.indent(tree, space='  ')
tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
print(f'[prepare_and_spawn_husky] Set initial_value on {updated} UR5e state_interface(s)')
PYEOF
fi

gz sdf -p "${URDF_FILE}" > "${SDF_FILE}"
if [ ! -s "${SDF_FILE}" ]; then
    echo "[prepare_and_spawn_husky] ERROR: gz sdf produced an empty SDF."
    exit 1
fi

ROS_SHARE_PATH=$(ros2 pkg prefix clearpath_platform_description)/share
sed -i -E "s|model://(clearpath_[a-z0-9_]+)|${ROS_SHARE_PATH}/\1|g" "${SDF_FILE}"

if [ "${HAS_MANIPULATORS}" = "1" ]; then
    UR_SHARE_PATH=$(ros2 pkg prefix ur_description)/share/ur_description
    ROBOTIQ_SHARE_PATH=$(ros2 pkg prefix robotiq_description)/share/robotiq_description
    sed -i -E "s|model://ur_description|${UR_SHARE_PATH}|g" "${SDF_FILE}"
    sed -i -E "s|model://robotiq_description|${ROBOTIQ_SHARE_PATH}|g" "${SDF_FILE}"
fi

# Inject OdometryPublisher, deduplicate GazeboSimROS2ControlPlugin, and add DynamicAttachSystem
python3 - "${SDF_FILE}" "${NAMESPACE}" "${HAS_MANIPULATORS}" <<'PYEOF'
import sys
import xml.etree.ElementTree as ET

sdf_path, ns, has_manip = sys.argv[1], sys.argv[2], sys.argv[3]
tree = ET.parse(sdf_path)
root = tree.getroot()
model = root.find('model')
if model is None:
    raise RuntimeError(f'No <model> element found in {sdf_path}')

changed = False

# Deduplicate bare GazeboSimROS2ControlPlugin elements (Clearpath universal_robots.urdf.xacro
# emits a second bare plugin without <parameters> alongside a200.urdf.xacro's parameterized one).
ros2_ctrl_plugins = [
    p for p in model.findall('plugin')
    if p.attrib.get('name') == 'gz_ros2_control::GazeboSimROS2ControlPlugin'
]
has_parameterized = any(p.find('parameters') is not None for p in ros2_ctrl_plugins)
if has_parameterized and len(ros2_ctrl_plugins) > 1:
    for p in ros2_ctrl_plugins:
        if p.find('parameters') is None:
            model.remove(p)
            changed = True

already_present = any(
    p.attrib.get('name') == 'gz::sim::systems::OdometryPublisher'
    for p in model.findall('plugin')
)
if not already_present:
    plugin = ET.SubElement(
        model,
        'plugin',
        {
            'filename': 'gz-sim-odometry-publisher-system',
            'name': 'gz::sim::systems::OdometryPublisher',
        },
    )
    entries = [
        ('odom_frame', 'odom'),
        ('robot_base_frame', 'base_link'),
        ('odom_topic', f'/{ns}/ground_truth/odom'),
        ('tf_topic', f'/{ns}/ground_truth/tf'),
        ('odom_publish_frequency', '50'),
        ('dimensions', '3'),
    ]
    for tag, text in entries:
        el = ET.SubElement(plugin, tag)
        el.text = text
    changed = True

if has_manip == '1':
    link_names = {lnk.attrib.get('name', '') for lnk in model.findall('link')}
    attach_link = 'arm_0_wrist_3_link'
    if attach_link not in link_names:
        arm_links = sorted(n for n in link_names if n.startswith('arm_0_'))
        raise RuntimeError(
            f"Expected attach parent link '{attach_link}' not found in {sdf_path}; "
            f'available arm_0 links: {arm_links}'
        )
    has_attach = any(
        p.attrib.get('name') == 'b1z1::DynamicAttachSystem'
        for p in model.findall('plugin')
    )
    if not has_attach:
        attach_plugin = ET.SubElement(
            model,
            'plugin',
            {
                'filename': 'libdynamic_attach_system.so',
                'name': 'b1z1::DynamicAttachSystem',
            },
        )
        for tag, text in (
            ('parent_link', attach_link),
            ('attach_topic', f'/{ns}/attach'),
            ('detach_topic', f'/{ns}/detach'),
        ):
            el = ET.SubElement(attach_plugin, tag)
            el.text = text
        changed = True

if changed:
    ET.indent(tree, space='  ')
    tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
PYEOF

echo "[prepare_and_spawn_husky] Waiting for ${CREATE_SERVICE} (timeout ${TIMEOUT_S}s)..."
start=$(date +%s)
until gz service -l 2>/dev/null | grep -qx "${CREATE_SERVICE}"; do
    if (( $(date +%s) - start >= TIMEOUT_S )); then
        echo "[prepare_and_spawn_husky] ERROR: ${CREATE_SERVICE} not available after ${TIMEOUT_S}s."
        exit 1
    fi
    sleep 1
done
echo "[prepare_and_spawn_husky] ${CREATE_SERVICE} ready after $(( $(date +%s) - start ))s."

if python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > 0.0 else 1)" "${SPAWN_DELAY_S}"; then
    echo "[prepare_and_spawn_husky] Waiting ${SPAWN_DELAY_S}s for Gazebo GUI settle..."
    sleep "${SPAWN_DELAY_S}"
fi

read -r QZ QW < <(python3 -c "import math, sys; y=float(sys.argv[1]); print(f'{math.sin(y/2.0):.8f} {math.cos(y/2.0):.8f}')" "${YAW}")

echo "[prepare_and_spawn_husky] Spawning ${ROBOT_NAME} at (${X}, ${Y}, ${Z}, yaw=${YAW})..."
response=$(gz service -s "${CREATE_SERVICE}" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 10000 \
    --req "sdf_filename: \"${SDF_FILE}\", name: \"${ROBOT_NAME}\", pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}")

if echo "${response}" | grep -q "data: true"; then
    echo "[prepare_and_spawn_husky] ${ROBOT_NAME} spawned successfully."
    exit 0
fi

echo "[prepare_and_spawn_husky] ERROR: spawn failed. Response: ${response:-<none>}"
exit 1
