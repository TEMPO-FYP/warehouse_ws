#!/usr/bin/env python3
"""Systematic PID and CHAMP gait tuning harness for Unitree b1z1 in Gazebo Sim."""

import argparse
import glob
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import Parameter as RosParameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry


LEGS = ("FL", "FR", "RL", "RR")
JOINT_TYPES = ("hip", "thigh", "calf")

TORQUE_LIMITS = {
    "hip": 91.0035,
    "thigh": 93.33,
    "calf": 140.0,
}

DEFAULT_RUNS_DIR = "/root/warehouse_ws/tuning_runs"
if not os.path.isdir("/root/warehouse_ws") and os.path.isdir("/home/taha/Desktop/FYP/warehouse_ws"):
    DEFAULT_RUNS_DIR = "/home/taha/Desktop/FYP/warehouse_ws/tuning_runs"


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rms(vals: List[float]) -> float:
    if not vals:
        return 0.0
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def joint_type_of(name: str) -> Optional[str]:
    for jt in JOINT_TYPES:
        if f"_{jt}_" in name or name.endswith(f"_{jt}_joint"):
            return jt
    return None


def make_double_param(name: str, val: float) -> RosParameter:
    p = RosParameter()
    p.name = name
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_DOUBLE
    pv.double_value = float(val)
    p.value = pv
    return p


class LegTuneNode(Node):
    def __init__(self):
        super().__init__(
            "b1z1_leg_tune",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.latest_sim_time: Optional[float] = None
        self.clock_samples: List[Tuple[float, float]] = []  # (wall, sim)
        self.odom_samples: List[Tuple[float, float, float, float, float, float, float, float, float, float]] = []
        self.state_samples: List[Tuple[float, Dict[str, Tuple[float, float, float, float]]]] = []
        self.latest_odom = None
        self.latest_state_map: Dict[str, Tuple[float, float, float, float]] = {}
        self.recording = False

        self.create_subscription(Clock, "/clock", self._clock_cb, 50)
        self.create_subscription(Odometry, "/odom_gz", self._odom_cb, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/leg_trajectory_controller/controller_state",
            self._state_cb,
            50,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.body_pose_pub = self.create_publisher(Pose, "/body_pose", 10)

        self.jtc_set_cli = self.create_client(SetParameters, "/leg_trajectory_controller/set_parameters")
        self.jtc_get_cli = self.create_client(GetParameters, "/leg_trajectory_controller/get_parameters")
        self.champ_set_cli = self.create_client(SetParameters, "/quadruped_controller_node/set_parameters")
        self.champ_get_cli = self.create_client(GetParameters, "/quadruped_controller_node/get_parameters")

    def _clock_cb(self, msg: Clock):
        sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        wall_t = time.monotonic()
        self.latest_sim_time = sim_t
        if self.recording:
            self.clock_samples.append((wall_t, sim_t))

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        r, pi, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0 and self.latest_sim_time is not None:
            stamp = self.latest_sim_time
        sample = (stamp, p.x, p.y, p.z, r, pi, yaw, tw.linear.x, tw.linear.y, tw.angular.z)
        self.latest_odom = sample
        if self.recording:
            self.odom_samples.append(sample)

    def _state_cb(self, msg: JointTrajectoryControllerState):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0 and self.latest_sim_time is not None:
            stamp = self.latest_sim_time
        ref = msg.reference.positions
        fb = msg.feedback.positions
        err = msg.error.positions
        eff = msg.output.effort if len(msg.output.effort) == len(msg.joint_names) else msg.output.positions
        jmap: Dict[str, Tuple[float, float, float, float]] = {}
        for idx, jname in enumerate(msg.joint_names):
            r_v = float(ref[idx]) if idx < len(ref) else 0.0
            f_v = float(fb[idx]) if idx < len(fb) else 0.0
            e_v = float(err[idx]) if idx < len(err) else (r_v - f_v)
            u_v = float(eff[idx]) if idx < len(eff) else 0.0
            jmap[jname] = (r_v, f_v, e_v, u_v)
        self.latest_state_map = jmap
        if self.recording:
            self.state_samples.append((stamp, jmap))

    def wait_for_topics(self, timeout_wall: float = 30.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_wall:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.latest_sim_time is not None
                and self.latest_odom is not None
                and self.latest_state_map
            ):
                return True
        return False

    def spin_sim(self, sim_secs: float, wall_timeout: Optional[float] = None):
        if sim_secs <= 0.0:
            return
        if self.latest_sim_time is None:
            self.wait_for_topics()
        start_sim = self.latest_sim_time or 0.0
        start_wall = time.monotonic()
        max_wall = wall_timeout if wall_timeout is not None else max(sim_secs * 3.0 + 5.0, 15.0)
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_sim_time is not None and (self.latest_sim_time - start_sim) >= sim_secs:
                break
            if time.monotonic() - start_wall > max_wall:
                break

    def start_recording(self):
        self.clock_samples.clear()
        self.odom_samples.clear()
        self.state_samples.clear()
        self.recording = True

    def stop_recording(self):
        self.recording = False

    def call_set_params(self, client, service_name: str, params: List[RosParameter]) -> bool:
        if not params:
            return True
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service {service_name} not available")
        req = SetParameters.Request()
        req.parameters = params
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"SetParameters call to {service_name} timed out")
        for p, res in zip(params, fut.result().results):
            if not res.successful:
                raise RuntimeError(f"Failed to set {p.name} on {service_name}: {res.reason}")
        return True

    def call_get_params(self, client, service_name: str, names: List[str]) -> Dict[str, float]:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service {service_name} not available")
        req = GetParameters.Request()
        req.names = names
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not fut.done() or fut.result() is None:
            raise RuntimeError(f"GetParameters call to {service_name} timed out")
        out = {}
        for name, val in zip(names, fut.result().values):
            if val.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = val.double_value
            elif val.type == ParameterType.PARAMETER_INTEGER:
                out[name] = float(val.integer_value)
        return out

    def set_joint_gains(
        self,
        joint_types: List[str],
        p: Optional[float] = None,
        i: Optional[float] = None,
        d: Optional[float] = None,
        i_clamp: Optional[float] = None,
        ff_velocity_scale: Optional[float] = None,
    ):
        params: List[RosParameter] = []
        for jt in joint_types:
            for leg in LEGS:
                prefix = f"gains.{leg}_{jt}_joint"
                if p is not None:
                    params.append(make_double_param(f"{prefix}.p", p))
                if i is not None:
                    params.append(make_double_param(f"{prefix}.i", i))
                if d is not None:
                    params.append(make_double_param(f"{prefix}.d", d))
                if i_clamp is not None:
                    params.append(make_double_param(f"{prefix}.i_clamp", i_clamp))
                if ff_velocity_scale is not None:
                    params.append(make_double_param(f"{prefix}.ff_velocity_scale", ff_velocity_scale))
        self.call_set_params(self.jtc_set_cli, "/leg_trajectory_controller/set_parameters", params)

    def get_all_leg_gains(self) -> Dict[str, Dict[str, float]]:
        names = []
        for jt in JOINT_TYPES:
            prefix = f"gains.FL_{jt}_joint"
            for k in ("p", "i", "d", "i_clamp", "ff_velocity_scale"):
                names.append(f"{prefix}.{k}")
        raw = self.call_get_params(self.jtc_get_cli, "/leg_trajectory_controller/get_parameters", names)
        summary: Dict[str, Dict[str, float]] = {}
        for jt in JOINT_TYPES:
            prefix = f"gains.FL_{jt}_joint"
            summary[jt] = {k: raw.get(f"{prefix}.{k}", float("nan")) for k in ("p", "i", "d", "i_clamp", "ff_velocity_scale")}
        return summary

    def set_champ_params(self, kv_map: Dict[str, float]):
        params = [make_double_param(k, v) for k, v in kv_map.items()]
        self.call_set_params(self.champ_set_cli, "/quadruped_controller_node/set_parameters", params)

    def get_champ_params(self) -> Dict[str, float]:
        names = [
            "joint_command_time",
            "gait.nominal_height",
            "gait.swing_height",
            "gait.com_x_translation",
            "gait.stance_depth",
            "gait.stance_duration",
            "gait.max_linear_velocity_x",
            "gait.max_linear_velocity_y",
            "gait.max_angular_velocity_z",
            "gait.odom_scaler",
        ]
        return self.call_get_params(self.champ_get_cli, "/quadruped_controller_node/get_parameters", names)

    def publish_cmd_vel(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def publish_body_pose_z(self, delta_z: float):
        msg = Pose()
        msg.position.z = float(delta_z)
        msg.orientation.w = 1.0
        self.body_pose_pub.publish(msg)


def compute_rtf(clock_samples: List[Tuple[float, float]]) -> float:
    if len(clock_samples) < 2:
        return 0.0
    dw = clock_samples[-1][0] - clock_samples[0][0]
    ds = clock_samples[-1][1] - clock_samples[0][1]
    if dw <= 1e-6:
        return 0.0
    return ds / dw


def analyze_error_oscillation(series: List[Tuple[float, float]]) -> Tuple[float, float]:
    """Return (zero_crossing_freq_hz, peak_to_half_amplitude_rad) on de-meaned signal."""
    if len(series) < 20:
        return 0.0, 0.0
    duration = series[-1][0] - series[0][0]
    if duration <= 0.2:
        return 0.0, 0.0
    mean_v = sum(v for _, v in series) / len(series)
    centered = [(t, v - mean_v) for t, v in series]
    amp = max(abs(v) for _, v in centered)
    if amp < 1e-4:
        return 0.0, amp
    # Hysteresis threshold to ignore tiny sensor noise
    hyst = max(0.1 * amp, 0.0005)
    state = 0
    crossings = 0
    for _, v in centered:
        if state >= 0 and v < -hyst:
            state = -1
            crossings += 1
        elif state <= 0 and v > hyst:
            state = 1
            crossings += 1
    freq_hz = (crossings / 2.0) / duration
    return freq_hz, amp


def summarize_stand(node: LegTuneNode, label: str, extra: Optional[dict] = None) -> dict:
    rtf = compute_rtf(node.clock_samples)
    odoms = node.odom_samples
    states = node.state_samples

    zs = [s[3] for s in odoms]
    rolls = [s[4] for s in odoms]
    pitches = [s[5] for s in odoms]

    z_mean = sum(zs) / len(zs) if zs else 0.0
    z_min = min(zs) if zs else 0.0
    z_max = max(zs) if zs else 0.0
    z_ripple = z_max - z_min

    roll_rms = rms(rolls)
    roll_peak = max((abs(r) for r in rolls), default=0.0)
    pitch_rms = rms(pitches)
    pitch_peak = max((abs(p) for p in pitches), default=0.0)
    fell = (z_min < 0.38) or (roll_peak > 0.45) or (pitch_peak > 0.45)

    joint_metrics = {}
    for jt in JOINT_TYPES:
        all_errs: List[float] = []
        all_effs: List[float] = []
        front_effs: List[float] = []
        rear_effs: List[float] = []
        fl_series: List[Tuple[float, float]] = []
        for stamp, jmap in states:
            for jname, (_, _, err_v, eff_v) in jmap.items():
                if joint_type_of(jname) != jt:
                    continue
                all_errs.append(err_v)
                all_effs.append(eff_v)
                if jname.startswith(("FL_", "FR_")):
                    front_effs.append(eff_v)
                else:
                    rear_effs.append(eff_v)
                if jname == f"FL_{jt}_joint":
                    fl_series.append((stamp, err_v))
        osc_hz, osc_amp = analyze_error_oscillation(fl_series)
        peak_eff = max((abs(u) for u in all_effs), default=0.0)
        joint_metrics[jt] = {
            "err_rms": rms(all_errs),
            "err_peak": max((abs(e) for e in all_errs), default=0.0),
            "err_mean": sum(all_errs) / len(all_errs) if all_errs else 0.0,
            "eff_mean_abs": sum(abs(u) for u in all_effs) / len(all_effs) if all_effs else 0.0,
            "eff_peak_abs": peak_eff,
            "eff_front_mean_abs": sum(abs(u) for u in front_effs) / len(front_effs) if front_effs else 0.0,
            "eff_rear_mean_abs": sum(abs(u) for u in rear_effs) / len(rear_effs) if rear_effs else 0.0,
            "saturated": peak_eff >= 0.98 * TORQUE_LIMITS[jt],
            "osc_hz": osc_hz,
            "osc_amp": osc_amp,
        }

    gains = node.get_all_leg_gains()
    champ_p = node.get_champ_params()

    res = {
        "kind": "stand",
        "label": label,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "rtf": rtf,
        "n_odom": len(odoms),
        "n_state": len(states),
        "fell": fell,
        "z_mean": z_mean,
        "z_min": z_min,
        "z_max": z_max,
        "z_ripple": z_ripple,
        "roll_rms": roll_rms,
        "roll_peak": roll_peak,
        "pitch_rms": pitch_rms,
        "pitch_peak": pitch_peak,
        "joints": joint_metrics,
        "gains": gains,
        "champ": champ_p,
    }
    if extra:
        res.update(extra)
    return res


def save_result(runs_dir: str, data: dict) -> str:
    os.makedirs(runs_dir, exist_ok=True)
    safe_label = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in data.get("label", "run"))
    fname = f"{int(time.time() * 10)}_{data.get('kind', 'trial')}_{safe_label}.json"
    fpath = os.path.join(runs_dir, fname)
    with open(fpath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return fpath


def print_stand_summary(res: dict):
    jm = res["joints"]
    print(
        f"[{res['label']}] RTF={res['rtf']:.2f} fell={res['fell']} | "
        f"z={res['z_mean']:.3f}m (rip={res['z_ripple']:.4f}m, min={res['z_min']:.3f}) | "
        f"roll={res['roll_peak']:.4f}(rms {res['roll_rms']:.4f}) pitch={res['pitch_peak']:.4f}(rms {res['pitch_rms']:.4f}) | "
        f"hip(err={jm['hip']['err_rms']:.4f}, umax={jm['hip']['eff_peak_abs']:.1f}, osc={jm['hip']['osc_hz']:.1f}Hz/{jm['hip']['osc_amp']:.4f}) "
        f"thigh(err={jm['thigh']['err_rms']:.4f}, umax={jm['thigh']['eff_peak_abs']:.1f}, osc={jm['thigh']['osc_hz']:.1f}Hz/{jm['thigh']['osc_amp']:.4f}) "
        f"calf(err={jm['calf']['err_rms']:.4f}, umax={jm['calf']['eff_peak_abs']:.1f}, osc={jm['calf']['osc_hz']:.1f}Hz/{jm['calf']['osc_amp']:.4f})",
        flush=True,
    )


def run_stand(node: LegTuneNode, secs: float, settle: float, label: str, runs_dir: str) -> dict:
    node.publish_cmd_vel(0.0, 0.0, 0.0)
    node.spin_sim(settle)
    node.start_recording()
    node.spin_sim(secs)
    node.stop_recording()
    res = summarize_stand(node, label)
    fpath = save_result(runs_dir, res)
    res["_file"] = fpath
    print_stand_summary(res)
    return res


def run_step(
    node: LegTuneNode,
    joint: str,
    delta_z: float,
    jct_values: List[float],
    runs_dir: str,
    label: str,
):
    """Compare step response under CHAMP trajectory horizon (joint_command_time) values."""
    orig_champ = node.get_champ_params()
    orig_jct = orig_champ.get("joint_command_time", 1.0 / 60.0)
    trials = []

    try:
        for jct in jct_values:
            node.set_champ_params({"joint_command_time": jct})
            node.publish_body_pose_z(0.0)
            node.spin_sim(1.5)

            # Baseline pre-step mean over 0.5 s
            node.start_recording()
            node.spin_sim(0.5)
            node.stop_recording()
            pre_vals = [
                jmap[f"FL_{joint}_joint"][1]
                for _, jmap in node.state_samples
                if f"FL_{joint}_joint" in jmap
            ]
            pre_pos = sum(pre_vals) / len(pre_vals) if pre_vals else 0.0
            pre_z = sum(s[3] for s in node.odom_samples) / len(node.odom_samples) if node.odom_samples else 0.0

            # Trigger body height step via /body_pose
            node.start_recording()
            t0 = node.latest_sim_time or 0.0
            node.publish_body_pose_z(delta_z)
            node.spin_sim(2.5)
            node.stop_recording()

            rtf = compute_rtf(node.clock_samples)
            series = [
                (t - t0, jmap[f"FL_{joint}_joint"][0], jmap[f"FL_{joint}_joint"][1], jmap[f"FL_{joint}_joint"][2], jmap[f"FL_{joint}_joint"][3])
                for t, jmap in node.state_samples
                if f"FL_{joint}_joint" in jmap
            ]
            target_final = sum(x[1] for x in series[-10:]) / max(1, len(series[-10:]))
            actual_final = sum(x[2] for x in series[-10:]) / max(1, len(series[-10:]))
            step_size = actual_final - pre_pos
            if abs(step_size) > 1e-4:
                sgn = 1.0 if step_size > 0 else -1.0
                norm = [(t, sgn * (fb - pre_pos) / abs(step_size)) for t, _, fb, _, _ in series]
                t10 = next((t for t, n in norm if n >= 0.1), float("nan"))
                t90 = next((t for t, n in norm if n >= 0.9), float("nan"))
                rise_time = t90 - t10 if not (math.isnan(t10) or math.isnan(t90)) else float("nan")
                peak_n = max(n for _, n in norm)
                overshoot_pct = max(0.0, (peak_n - 1.0) * 100.0)
                settle_time = 0.0
                for t, n in norm:
                    if abs(n - 1.0) > 0.05:
                        settle_time = t
            else:
                rise_time = float("nan")
                overshoot_pct = 0.0
                settle_time = float("nan")

            errs = [x[3] for x in series]
            effs = [abs(x[4]) for x in series]
            eff_first_03 = [abs(x[4]) for x in series if x[0] <= 0.3]
            z_end = sum(s[3] for s in node.odom_samples[-10:]) / max(1, len(node.odom_samples[-10:]))
            roll_peak = max((abs(s[4]) for s in node.odom_samples), default=0.0)
            pitch_peak = max((abs(s[5]) for s in node.odom_samples), default=0.0)

            tr = {
                "joint_command_time": jct,
                "rtf": rtf,
                "pre_z": pre_z,
                "post_z": z_end,
                "delta_z_actual": z_end - pre_z,
                "pre_joint_pos": pre_pos,
                "ref_joint_final": target_final,
                "fb_joint_final": actual_final,
                "ss_err": target_final - actual_final,
                "step_joint_rad": step_size,
                "rise_time_s": rise_time,
                "overshoot_pct": overshoot_pct,
                "settle_time_s": settle_time,
                "err_rms": rms(errs),
                "eff_peak": max(effs, default=0.0),
                "eff_peak_03s": max(eff_first_03, default=0.0),
                "roll_peak": roll_peak,
                "pitch_peak": pitch_peak,
            }
            trials.append(tr)
            print(
                f"[step jct={jct:.4f}s] RTF={rtf:.2f} | dz={tr['delta_z_actual']:.4f}m d_theta={step_size:.4f}rad | "
                f"rise={rise_time:.3f}s OS={overshoot_pct:.1f}% settle={settle_time:.3f}s ss_err={tr['ss_err']:.4f} | "
                f"err_rms={tr['err_rms']:.4f} eff_pk={tr['eff_peak']:.1f}Nm eff_03s={tr['eff_peak_03s']:.1f}Nm",
                flush=True,
            )

            # Return back cleanly
            node.publish_body_pose_z(0.0)
            node.spin_sim(1.5)
    finally:
        node.publish_body_pose_z(0.0)
        node.set_champ_params({"joint_command_time": orig_jct})

    out = {
        "kind": "step",
        "label": label,
        "joint": joint,
        "delta_z_cmd": delta_z,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "trials": trials,
    }
    fpath = save_result(runs_dir, out)
    print(f"Saved step report: {fpath}", flush=True)


def run_walk(
    node: LegTuneNode,
    vx: float,
    vy: float,
    wz: float,
    secs: float,
    label: str,
    runs_dir: str,
) -> dict:
    # Ensure upright before starting
    node.publish_cmd_vel(0.0, 0.0, 0.0)
    node.spin_sim(1.0)
    if node.latest_odom is None or node.latest_odom[3] < 0.40 or abs(node.latest_odom[4]) > 0.45 or abs(node.latest_odom[5]) > 0.45:
        print(f"[{label}] ABORT: robot not upright before walk (odom={node.latest_odom})", flush=True)
        return {"kind": "walk", "label": label, "fell": True, "aborted": True}

    ramp_up = 1.5
    ramp_dn = 1.0
    dt = 0.05
    total_hold = secs

    node.start_recording()
    sim0 = node.latest_sim_time or 0.0
    fell = False

    def step_cmd(elapsed: float) -> Tuple[float, float, float]:
        if elapsed < ramp_up:
            alpha = max(0.0, min(1.0, elapsed / ramp_up))
            return vx * alpha, vy * alpha, wz * alpha
        elif elapsed < ramp_up + total_hold:
            return vx, vy, wz
        elif elapsed < ramp_up + total_hold + ramp_dn:
            alpha = max(0.0, min(1.0, 1.0 - (elapsed - ramp_up - total_hold) / ramp_dn))
            return vx * alpha, vy * alpha, wz * alpha
        return 0.0, 0.0, 0.0

    total_sim = ramp_up + total_hold + ramp_dn + 1.0
    start_wall = time.monotonic()
    while True:
        cur_sim = node.latest_sim_time or sim0
        elapsed = cur_sim - sim0
        if elapsed >= total_sim:
            break
        if time.monotonic() - start_wall > total_sim * 3.0 + 10.0:
            break
        cvx, cvy, cwz = step_cmd(elapsed)
        node.publish_cmd_vel(cvx, cvy, cwz)
        node.spin_sim(dt, wall_timeout=2.0)
        lo = node.latest_odom
        if lo is not None and (lo[3] < 0.40 or abs(lo[4]) > 0.45 or abs(lo[5]) > 0.45):
            fell = True
            break

    node.publish_cmd_vel(0.0, 0.0, 0.0)
    node.stop_recording()

    rtf = compute_rtf(node.clock_samples)
    odoms = node.odom_samples
    states = node.state_samples

    zs = [s[3] for s in odoms]
    rolls = [s[4] for s in odoms]
    pitches = [s[5] for s in odoms]
    z_mean = sum(zs) / len(zs) if zs else 0.0
    z_min = min(zs) if zs else 0.0
    roll_rms = rms(rolls)
    roll_peak = max((abs(r) for r in rolls), default=0.0)
    pitch_rms = rms(pitches)
    pitch_peak = max((abs(p) for p in pitches), default=0.0)

    # Steady-state window for velocity achieved: [ramp_up + 1.0, ramp_up + total_hold]
    ss_start = sim0 + ramp_up + 1.0
    ss_end = sim0 + ramp_up + total_hold
    ss_odoms = [s for s in odoms if ss_start <= s[0] <= ss_end]
    if len(ss_odoms) < 5:
        ss_odoms = [s for s in odoms if (sim0 + ramp_up) <= s[0] <= ss_end]

    vx_ach, vy_ach, wz_ach = 0.0, 0.0, 0.0
    if len(ss_odoms) >= 2:
        # Compute body-frame velocities from ground-truth pose increments
        sum_dx_b = 0.0
        sum_dy_b = 0.0
        sum_dyaw = 0.0
        total_dt = 0.0
        for k in range(1, len(ss_odoms)):
            t_prev, x0, y0, _, _, _, yaw0, _, _, _ = ss_odoms[k - 1]
            t_cur, x1, y1, _, _, _, yaw1, _, _, _ = ss_odoms[k]
            dt_k = t_cur - t_prev
            if dt_k <= 1e-5:
                continue
            dx = x1 - x0
            dy = y1 - y0
            mid_yaw = yaw0 + 0.5 * wrap_angle(yaw1 - yaw0)
            c = math.cos(mid_yaw)
            s = math.sin(mid_yaw)
            sum_dx_b += c * dx + s * dy
            sum_dy_b += -s * dx + c * dy
            sum_dyaw += wrap_angle(yaw1 - yaw0)
            total_dt += dt_k
        if total_dt > 0.1:
            vx_ach = sum_dx_b / total_dt
            vy_ach = sum_dy_b / total_dt
            wz_ach = sum_dyaw / total_dt

    ratios = []
    if abs(vx) > 0.02:
        ratios.append(vx_ach / vx)
    if abs(vy) > 0.02:
        ratios.append(vy_ach / vy)
    if abs(wz) > 0.02:
        ratios.append(wz_ach / wz)
    speed_ratio = sum(ratios) / len(ratios) if ratios else 1.0

    # Net yaw drift when commanding pure straight motion (wz == 0)
    yaw_drift = 0.0
    if abs(wz) < 1e-4 and len(ss_odoms) >= 2:
        yaw_drift = wrap_angle(ss_odoms[-1][6] - ss_odoms[0][6])

    joint_metrics = {}
    for jt in JOINT_TYPES:
        errs = []
        effs = []
        for _, jmap in states:
            for jname, (_, _, err_v, eff_v) in jmap.items():
                if joint_type_of(jname) == jt:
                    errs.append(err_v)
                    effs.append(abs(eff_v))
        peak_eff = max(effs, default=0.0)
        joint_metrics[jt] = {
            "err_rms": rms(errs),
            "eff_peak_abs": peak_eff,
            "saturated": peak_eff >= 0.98 * TORQUE_LIMITS[jt],
        }

    res = {
        "kind": "walk",
        "label": label,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "cmd": {"vx": vx, "vy": vy, "wz": wz, "secs": secs},
        "rtf": rtf,
        "fell": fell,
        "vx_achieved": vx_ach,
        "vy_achieved": vy_ach,
        "wz_achieved": wz_ach,
        "speed_ratio": speed_ratio,
        "yaw_drift_rad": yaw_drift,
        "z_mean": z_mean,
        "z_min": z_min,
        "roll_rms": roll_rms,
        "roll_peak": roll_peak,
        "pitch_rms": pitch_rms,
        "pitch_peak": pitch_peak,
        "pitch_mean": sum(pitches) / len(pitches) if pitches else 0.0,
        "joints": joint_metrics,
        "gains": node.get_all_leg_gains(),
        "champ": node.get_champ_params(),
    }
    fpath = save_result(runs_dir, res)
    res["_file"] = fpath
    jm = res["joints"]
    print(
        f"[{label}] RTF={rtf:.2f} fell={fell} | cmd=({vx:.2f},{vy:.2f},{wz:.2f}) "
        f"ach=({vx_ach:.3f},{vy_ach:.3f},{wz_ach:.3f}) ratio={speed_ratio:.2f} | "
        f"z_mean={z_mean:.3f} z_min={z_min:.3f} | roll={roll_peak:.3f}(rms {roll_rms:.3f}) "
        f"pitch={pitch_peak:.3f}(mean {res['pitch_mean']:.3f}) | "
        f"eff_pk=(H:{jm['hip']['eff_peak_abs']:.1f}, T:{jm['thigh']['eff_peak_abs']:.1f}, C:{jm['calf']['eff_peak_abs']:.1f})",
        flush=True,
    )
    return res


def run_sweep(
    node: LegTuneNode,
    joint: str,
    param: str,
    values: List[float],
    secs: float,
    settle: float,
    runs_dir: str,
    label_prefix: str,
):
    results = []
    for v in values:
        kw = {param: v}
        node.set_joint_gains([joint], **kw)
        lbl = f"{label_prefix}_{joint}_{param}_{v:g}"
        res = run_stand(node, secs, settle, lbl, runs_dir)
        results.append(res)
        if res["fell"]:
            print(f"STOPPING SWEEP: robot fell at {joint}.{param}={v}", flush=True)
            break

    print("\n--- SWEEP SUMMARY ---", flush=True)
    print("val | RTF | fell | z_mean | z_rip | roll_pk | pitch_pk | target_err_rms | osc_Hz | osc_amp | u_peak", flush=True)
    for v, r in zip(values, results):
        jm = r["joints"][joint]
        print(
            f"{v:6.2f} | {r['rtf']:.2f} | {str(r['fell']):5s} | {r['z_mean']:.4f} | {r['z_ripple']:.4f} | "
            f"{r['roll_peak']:.4f} | {r['pitch_peak']:.4f} | {jm['err_rms']:.4f} | "
            f"{jm['osc_hz']:.2f} | {jm['osc_amp']:.4f} | {jm['eff_peak_abs']:.1f}",
            flush=True,
        )


def generate_report(runs_dir: str, out_path: str):
    files = sorted(glob.glob(os.path.join(runs_dir, "*.json")))
    stands = []
    walks = []
    steps = []
    for f in files:
        try:
            with open(f, "r", encoding="utf-8") as fh:
                d = json.load(fh)
        except Exception:
            continue
        k = d.get("kind")
        if k == "stand":
            stands.append(d)
        elif k == "walk":
            walks.append(d)
        elif k == "step":
            steps.append(d)

    lines = ["# Unitree b1z1 Leg-PID + CHAMP Gait Tuning Report\n"]
    lines.append(f"Generated at: `{time.strftime('%Y-%m-%d %H:%M:%S')}` from `{runs_dir}` ({len(files)} trials).\n")

    if steps:
        lines.append("\n## Phase 0: Trajectory Horizon (`joint_command_time`) Step Comparison\n")
        for st in steps:
            lines.append(f"\n### {st.get('label')} (joint=`{st.get('joint')}`, delta_z=`{st.get('delta_z_cmd')}` m)\n")
            lines.append("| jct (s) | RTF | d_z (m) | rise (s) | OS (%) | settle (s) | err_rms (rad) | eff_peak (Nm) | eff_0.3s (Nm) |")
            lines.append("|---|---|---|---|---|---|---|---|---|")
            for t in st.get("trials", []):
                lines.append(
                    f"| {t['joint_command_time']:.4f} | {t['rtf']:.2f} | {t['delta_z_actual']:.4f} | "
                    f"{t['rise_time_s']:.3f} | {t['overshoot_pct']:.1f} | {t['settle_time_s']:.3f} | "
                    f"{t['err_rms']:.4f} | {t['eff_peak']:.1f} | {t['eff_peak_03s']:.1f} |"
                )

    if stands:
        lines.append("\n## Standing Trials (Phase 0 & Phase 1)\n")
        lines.append("| Label | RTF | Fell | z_mean (m) | z_rip (m) | roll_pk | pitch_pk | H/T/C err_rms (rad) | H/T/C u_pk (Nm) | H/T/C P/I/D |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for r in stands:
            jm = r["joints"]
            g = r["gains"]
            errs = f"{jm['hip']['err_rms']:.4f}/{jm['thigh']['err_rms']:.4f}/{jm['calf']['err_rms']:.4f}"
            upks = f"{jm['hip']['eff_peak_abs']:.1f}/{jm['thigh']['eff_peak_abs']:.1f}/{jm['calf']['eff_peak_abs']:.1f}"
            gstr = (
                f"H({g['hip']['p']:.0f},{g['hip']['i']:.1f},{g['hip']['d']:.1f}) "
                f"T({g['thigh']['p']:.0f},{g['thigh']['i']:.1f},{g['thigh']['d']:.1f}) "
                f"C({g['calf']['p']:.0f},{g['calf']['i']:.1f},{g['calf']['d']:.1f})"
            )
            lines.append(
                f"| {r['label']} | {r['rtf']:.2f} | {r['fell']} | {r['z_mean']:.3f} | {r['z_ripple']:.4f} | "
                f"{r['roll_peak']:.4f} | {r['pitch_peak']:.4f} | {errs} | {upks} | {gstr} |"
            )

    if walks:
        lines.append("\n## Walking Trials (Phase 2)\n")
        lines.append("| Label | RTF | Fell | cmd (vx,vy,wz) | ach (vx,vy,wz) | ratio | z_min (m) | roll_pk | pitch_pk | H/T/C u_pk (Nm) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for r in walks:
            if r.get("aborted"):
                lines.append(f"| {r['label']} | - | True (aborted) | - | - | - | - | - | - | - |")
                continue
            c = r["cmd"]
            jm = r["joints"]
            cmd_s = f"({c['vx']:.2f},{c['vy']:.2f},{c['wz']:.2f})"
            ach_s = f"({r['vx_achieved']:.2f},{r['vy_achieved']:.2f},{r['wz_achieved']:.2f})"
            upks = f"{jm['hip']['eff_peak_abs']:.1f}/{jm['thigh']['eff_peak_abs']:.1f}/{jm['calf']['eff_peak_abs']:.1f}"
            lines.append(
                f"| {r['label']} | {r['rtf']:.2f} | {r['fell']} | {cmd_s} | {ach_s} | {r['speed_ratio']:.2f} | "
                f"{r['z_min']:.3f} | {r['roll_peak']:.3f} | {r['pitch_peak']:.3f} | {upks} |"
            )

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote tuning report to {out_path}", flush=True)


def parse_kv(items: List[str]) -> Dict[str, float]:
    out = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Expected key=val, got: {item}")
        k, v = item.split("=", 1)
        out[k.strip()] = float(v.strip())
    return out


def main():
    parser = argparse.ArgumentParser(description="b1z1 PID & CHAMP tuning harness")
    parser.add_argument("--runs-dir", default=DEFAULT_RUNS_DIR)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_stand = sub.add_parser("stand")
    p_stand.add_argument("--secs", type=float, default=10.0)
    p_stand.add_argument("--settle", type=float, default=2.0)
    p_stand.add_argument("--label", default="stand")

    p_step = sub.add_parser("step")
    p_step.add_argument("--joint", choices=JOINT_TYPES, default="calf")
    p_step.add_argument("--delta", type=float, default=0.15)
    p_step.add_argument("--delta-z", type=float, default=0.03)
    p_step.add_argument("--jct-values", type=float, nargs="+", default=[1.0 / 60.0, 0.04, 0.08, 0.20])
    p_step.add_argument("--label", default="step_jct")

    p_sweep = sub.add_parser("sweep")
    p_sweep.add_argument("--joint", choices=JOINT_TYPES, required=True)
    p_sweep.add_argument("--param", choices=["p", "i", "d", "i_clamp", "ff_velocity_scale"], required=True)
    p_sweep.add_argument("--values", type=float, nargs="+", required=True)
    p_sweep.add_argument("--secs", type=float, default=8.0)
    p_sweep.add_argument("--settle", type=float, default=2.0)
    p_sweep.add_argument("--label", default="sweep")

    p_walk = sub.add_parser("walk")
    p_walk.add_argument("--vx", type=float, default=0.0)
    p_walk.add_argument("--vy", type=float, default=0.0)
    p_walk.add_argument("--wz", type=float, default=0.0)
    p_walk.add_argument("--secs", type=float, default=12.0)
    p_walk.add_argument("--label", default="walk")

    p_setg = sub.add_parser("set-gains")
    p_setg.add_argument("--joint", choices=[*JOINT_TYPES, "all"], required=True)
    p_setg.add_argument("--p", type=float, default=None)
    p_setg.add_argument("--i", type=float, default=None)
    p_setg.add_argument("--d", type=float, default=None)
    p_setg.add_argument("--i-clamp", type=float, default=None)
    p_setg.add_argument("--ff", type=float, default=None)

    sub.add_parser("get-gains")

    p_setc = sub.add_parser("set-champ")
    p_setc.add_argument("kv", nargs="+", help="key=val pairs like gait.nominal_height=0.50")

    sub.add_parser("get-champ")

    p_rep = sub.add_parser("report")
    p_rep.add_argument("--out", default=None)

    args = parser.parse_args()

    if args.cmd == "report":
        out_path = args.out or os.path.join(os.path.dirname(args.runs_dir), "tuning_report.md")
        generate_report(args.runs_dir, out_path)
        return

    rclpy.init()
    node = LegTuneNode()
    try:
        if not node.wait_for_topics(timeout_wall=30.0):
            print("ERROR: Timed out waiting for /clock, /odom_gz, and /leg_trajectory_controller/controller_state", file=sys.stderr)
            sys.exit(1)

        if args.cmd == "stand":
            run_stand(node, args.secs, args.settle, args.label, args.runs_dir)
        elif args.cmd == "step":
            run_step(node, args.joint, args.delta_z, args.jct_values, args.runs_dir, args.label)
        elif args.cmd == "sweep":
            run_sweep(node, args.joint, args.param, args.values, args.secs, args.settle, args.runs_dir, args.label)
        elif args.cmd == "walk":
            run_walk(node, args.vx, args.vy, args.wz, args.secs, args.label, args.runs_dir)
        elif args.cmd == "set-gains":
            jts = list(JOINT_TYPES) if args.joint == "all" else [args.joint]
            node.set_joint_gains(jts, p=args.p, i=args.i, d=args.d, i_clamp=args.i_clamp, ff_velocity_scale=args.ff)
            print(json.dumps(node.get_all_leg_gains(), indent=2), flush=True)
        elif args.cmd == "get-gains":
            print(json.dumps(node.get_all_leg_gains(), indent=2), flush=True)
        elif args.cmd == "set-champ":
            kv = parse_kv(args.kv)
            node.set_champ_params(kv)
            print(json.dumps(node.get_champ_params(), indent=2), flush=True)
        elif args.cmd == "get-champ":
            print(json.dumps(node.get_champ_params(), indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
