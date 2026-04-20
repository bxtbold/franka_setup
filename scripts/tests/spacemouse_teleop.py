#!/usr/bin/env python3
"""SpaceMouse teleoperation for franka_setup bridge."""

from __future__ import annotations

import argparse
import threading
import time
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from franka_bridge_client import FrankaClient

try:
    import pyspacemouse
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "pyspacemouse is required. Install with: python3 -m pip install pyspacemouse"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SpaceMouse teleop for Franka bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--spacemouse-hz", type=float, default=250.0)
    parser.add_argument("--pos-scale", type=float, default=0.1, help="meters per unit input step")
    parser.add_argument("--rot-scale", type=float, default=0.2, help="radians per unit input step")
    parser.add_argument(
        "--max-step-m",
        type=float,
        default=0.05,
        help="maximum absolute xyz delta per control step (meters)",
    )
    parser.add_argument(
        "--max-rot-step-rad",
        type=float,
        default=0.1,
        help="maximum absolute roll/pitch/yaw delta per control step (radians)",
    )
    parser.add_argument(
        "--rotation-frame",
        choices=["tool", "world"],
        default="world",
        help="compose incremental rotation in tool frame (current*delta) or world frame (delta*current)",
    )
    parser.add_argument("--deadband", type=float, default=0.07)
    parser.add_argument("--enable-rotation", action="store_true")
    parser.add_argument(
        "--require-hold-left",
        action="store_true",
        help="Require holding left button to send motion (extra safety).",
    )
    parser.add_argument("--frame-id", default="0")
    parser.add_argument(
        "--xyz-min",
        type=str,
        default=None,
        help="workspace cube lower bound as 'x,y,z' in meters",
    )
    parser.add_argument(
        "--xyz-max",
        type=str,
        default=None,
        help="workspace cube upper bound as 'x,y,z' in meters",
    )
    parser.add_argument(
        "--rpy-cone-deg",
        type=float,
        default=None,
        help="max roll/pitch cone half-angle in degrees around initial orientation",
    )
    return parser.parse_args()


def read_spacemouse() -> Tuple[list[float], bool, bool]:
    """Return (6d_action, left_button, right_button)."""
    state = pyspacemouse.read()
    if state is None:
        return [0.0] * 6, False, False

    # pyspacemouse usually returns a SpaceNavigator object with attributes.
    if hasattr(state, "x") and hasattr(state, "buttons"):
        action = [
            -float(state.y),      # x
            float(state.x),       # y
            float(state.z),       # z
            -float(state.roll),   # roll
            -float(state.pitch),  # pitch
            -float(state.yaw),    # yaw
        ]
        buttons = list(state.buttons) if state.buttons is not None else [0, 0]
        left_pressed = bool(buttons[0]) if len(buttons) > 0 else False
        right_pressed = bool(buttons[1]) if len(buttons) > 1 else False
        return action, left_pressed, right_pressed

    # Fallback for tuple/list return shape.
    action = [
        -float(state[2]),  # x
        float(state[1]),   # y
        float(state[3]),   # z
        -float(state[4]),  # roll
        -float(state[5]),  # pitch
        -float(state[6]),  # yaw
    ]
    left_pressed = bool(state[7]) if len(state) > 7 else False
    right_pressed = bool(state[8]) if len(state) > 8 else False
    return action, left_pressed, right_pressed


class SpaceMouseReader:
    """Background SpaceMouse polling with latest-sample cache."""

    def __init__(self, read_hz: float = 250.0) -> None:
        self._dt = 1.0 / max(read_hz, 1.0)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest: Tuple[list[float], bool, bool] = ([0.0] * 6, False, False)

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            sample = read_spacemouse()
            with self._lock:
                self._latest = sample
            time.sleep(self._dt)

    def latest(self) -> Tuple[list[float], bool, bool]:
        with self._lock:
            action, left, right = self._latest
            return list(action), bool(left), bool(right)

    def close(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.2)


def get_action_from_spacemouse(
    action_6d: list[float],
    deadband: float = 0.07,
    use_orientation: bool = False,
) -> np.ndarray | None:
    """Collect-demos style conversion: returns None when inactive."""
    command = np.array(action_6d, dtype=np.float32)
    if not use_orientation:
        command[3:6] = 0.0

    if np.linalg.norm(command) <= deadband:
        return None

    # Match collect_demos helper behavior.
    command[:3] = np.clip(command[:3], -1.0, 1.0)
    return command


def _parse_vec3_arg(name: str, value: str | None) -> np.ndarray | None:
    if value is None:
        return None
    parts = [p.strip() for p in value.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be 'x,y,z', got: {value}")
    try:
        return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)
    except ValueError as exc:
        raise ValueError(f"{name} must contain numeric values, got: {value}") from exc


def main() -> None:
    args = parse_args()
    dt = 1.0 / max(args.hz, 1.0)
    xyz_min = _parse_vec3_arg("--xyz-min", args.xyz_min)
    xyz_max = _parse_vec3_arg("--xyz-max", args.xyz_max)
    if (xyz_min is None) != (xyz_max is None):
        raise ValueError("Set both --xyz-min and --xyz-max together.")
    if xyz_min is not None and np.any(xyz_min > xyz_max):
        raise ValueError("--xyz-min must be <= --xyz-max on each axis.")
    cone_max_rad = None if args.rpy_cone_deg is None else np.deg2rad(float(args.rpy_cone_deg))
    if cone_max_rad is not None and cone_max_rad < 0.0:
        raise ValueError("--rpy-cone-deg must be >= 0.")

    print("Opening SpaceMouse...")
    if not pyspacemouse.open():
        raise RuntimeError("Failed to open SpaceMouse device.")
    sm_reader = SpaceMouseReader(read_hz=args.spacemouse_hz)
    sm_reader.start()

    client = FrankaClient(
        cmd_address=f"tcp://{args.host}:{args.cmd_port}",
        state_address=f"tcp://{args.host}:{args.state_port}",
        timeout_ms=2500,
    )

    print("\nSpaceMouse teleop ready")
    if args.require_hold_left:
        print("- Hold LEFT button to enable motion commands")
    else:
        print("- Motion commands are sent when SpaceMouse action exceeds deadband")
    print("- LEFT button closes gripper")
    print("- RIGHT button opens gripper")
    if xyz_min is not None and xyz_max is not None:
        print(f"- XYZ cube clamp active: min={xyz_min.tolist()} max={xyz_max.tolist()}")
    if cone_max_rad is not None:
        print(f"- RPY cone clamp active: {args.rpy_cone_deg:.2f} deg around initial orientation")
    print("- Ctrl+C to quit\n")

    prev_left = False
    prev_right = False
    orientation_ref: R | None = None

    try:
        while True:
            raw_action, left, right = sm_reader.latest()
            action = get_action_from_spacemouse(
                raw_action,
                deadband=args.deadband,
                use_orientation=args.enable_rotation,
            )

            # Edge-triggered gripper commands.
            if left and not prev_left:
                client.close_gripper()
                print("gripper -> close")
            if right and not prev_right:
                client.open_gripper()
                print("gripper -> open")
            prev_left = left
            prev_right = right

            motion_enabled = action is not None and (left if args.require_hold_left else True)
            if motion_enabled:
                state = client.get_latest_state(max_wait_s=0.1) or client.get_state()
                ee_pose = state.get("ee_pose")
                if not isinstance(ee_pose, dict):
                    print("No ee_pose in bridge state yet; waiting...")
                    time.sleep(dt)
                    continue

                pos = [float(v) for v in ee_pose.get("position", [])]
                quat = [float(v) for v in ee_pose.get("orientation", [])]
                if len(pos) != 3 or len(quat) != 4:
                    print("Invalid ee_pose format; waiting...")
                    time.sleep(dt)
                    continue

                if orientation_ref is None:
                    orientation_ref = R.from_quat(quat)

                delta_xyz = np.array(action[:3], dtype=np.float64) * float(args.pos_scale)
                delta_xyz = np.clip(delta_xyz, -float(args.max_step_m), float(args.max_step_m))
                pos_target_np = np.array(
                    [pos[0] + delta_xyz[0], pos[1] + delta_xyz[1], pos[2] + delta_xyz[2]],
                    dtype=np.float64,
                )
                if xyz_min is not None and xyz_max is not None:
                    pos_target_np = np.clip(pos_target_np, xyz_min, xyz_max)
                pos_target = [float(pos_target_np[0]), float(pos_target_np[1]), float(pos_target_np[2])]

                quat_target = quat
                if args.enable_rotation:
                    delta_rpy = np.array(
                        [
                            action[3] * args.rot_scale,
                            action[4] * args.rot_scale,
                            action[5] * args.rot_scale,
                        ],
                        dtype=np.float64,
                    )
                    delta_rpy = np.clip(
                        delta_rpy,
                        -float(args.max_rot_step_rad),
                        float(args.max_rot_step_rad),
                    )
                    delta_rot = R.from_euler(
                        "xyz",
                        delta_rpy,
                    )
                    current_rot = R.from_quat(quat)
                    if args.rotation_frame == "tool":
                        target_rot = current_rot * delta_rot
                    else:
                        target_rot = delta_rot * current_rot
                    if cone_max_rad is not None and orientation_ref is not None:
                        rel = orientation_ref.inv() * target_rot
                        rel_rpy = rel.as_euler("xyz", degrees=False)
                        rp = np.array([rel_rpy[0], rel_rpy[1]], dtype=np.float64)
                        rp_norm = float(np.linalg.norm(rp))
                        if rp_norm > cone_max_rad and rp_norm > 1e-9:
                            rp = rp * (cone_max_rad / rp_norm)
                            rel = R.from_euler("xyz", [rp[0], rp[1], rel_rpy[2]], degrees=False)
                            target_rot = orientation_ref * rel
                    quat_target = target_rot.as_quat().tolist()
                    print(target_rot.as_euler("xyz"))


                client.move_pose(position=pos_target, orientation=quat_target, frame_id=args.frame_id)
            # else:
            #     print("No motion")

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopping teleop.")
    finally:
        client.close()
        sm_reader.close()
        try:
            pyspacemouse.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
