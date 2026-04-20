# franka_setup

`franka_setup` provides a Dockerized ROS Noetic (Ubuntu 20.04) stack for Franka control, plus a ROS-free Python client that talks over ZeroMQ.

## Design

- Controller stack runs inside Docker (ROS + `serl_franka_controllers`).
- Non-ROS apps use Python client sockets.
- Ports are split to avoid conflicts:
  - Arm command port (`REQ/REP`): `5555`
  - Arm state stream port (`PUB/SUB`): `5556`
  - Optional gripper command port (`REQ/REP`): `5557`
  - Optional gripper state stream port (`PUB/SUB`): `5558`
- `FRANKA_STATE_HZ` / `FRANKA_GRIPPER_STATE_HZ` control ZMQ stream rates.
- `FRANKA_ROS_PUBLISH_RATE` controls ROS topic publish rates (e.g. `/franka_state_controller/joint_states`).

## Project Layout

- `docker/Dockerfile`: Noetic image + Franka deps + bridge packages.
- `docker-compose.yml`: host-network runtime config.
- `python/franka_bridge`: bridge server package.
- `python/franka_bridge_client`: ROS-free client SDK.
- `scripts/start_robot_stack.sh`: starts ROS/controller and bridge.
- `scripts/smoke_test_client.py`: client connectivity test.

## Prerequisites

- Docker + Docker Compose plugin.
- Franka robot reachable from this machine (`ROBOT_IP`).
- Robot in FCI mode.
- Host setup compatible with `franka_ros` runtime needs.

## Configure

```bash
cd /home/tactile/mpail/franka_setup
cp .env.example .env
```

Update `.env` as needed, especially `ROBOT_IP`.

Controller compliance defaults are also configurable in `.env` and will be
re-applied on startup after `impedance.launch`:
- `FRANKA_TRANSLATIONAL_STIFFNESS` (default `2000.0`)
- `FRANKA_ROTATIONAL_STIFFNESS` (default `150.0`)
- `FRANKA_COMPLIANCE_NODE` (default `/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node`)

If you want GUI tools from inside the container (for example RViz), keep
`DISPLAY`/`QT_X11_NO_MITSHM` set in `.env` and allow local Docker root access:

```bash
xhost +local:root
```

## Build

```bash
cd /home/tactile/mpail/franka_setup
docker compose build
```

## Run

```bash
cd /home/tactile/mpail/franka_setup
docker compose up
```

To verify visualization forwarding, from another terminal run:

```bash
cd /home/tactile/mpail/franka_setup
# Rebuild once after pulling Dockerfile changes that add RViz:
# docker compose build --no-cache franka_setup
docker compose exec franka_setup bash -lc "source /opt/ros/noetic/setup.bash && rviz"
```

To launch `rqt`:

```bash
cd /home/tactile/mpail/franka_setup
docker compose exec franka_setup bash -lc "source /opt/ros/noetic/setup.bash && rqt"
```

To visualize commanded target pose in RViz, add:
- `Pose` display on topic `/franka_bridge/target_pose`
- `TF` display and enable frame `franka_target`

By default this starts:
1. `roscore`
2. `roslaunch serl_franka_controllers impedance.launch ...`
3. split-port bridge server

If you want bridge only (skip controller launch), set:

```bash
LAUNCH_SERL_CONTROLLER=0 docker compose up
```

If you want to manually run `roslaunch` in the container while keeping the bridge
available for local clients, set in `.env`:

```bash
LAUNCH_SERL_CONTROLLER=0
LAUNCH_ROSCORE=1
```

Then start the container and run launch manually:

```bash
cd /home/tactile/mpail/franka_setup
docker compose up -d
docker compose exec franka_setup bash
source /opt/ros/noetic/setup.bash
source /opt/franka_setup/ros_ws/devel/setup.bash
roslaunch serl_franka_controllers impedance.launch robot_ip:=${ROBOT_IP} load_gripper:=${LOAD_GRIPPER}
```

In another host terminal, run your local client:

```bash
cd /home/tactile/mpail/franka_setup
python3 scripts/spacemouse_teleop.py --host 127.0.0.1 --cmd-port 5555 --state-port 5556
```

If your controller expects frame `"0"` (as in some SERL setups), add:

```bash
python3 scripts/spacemouse_teleop.py --host 127.0.0.1 --cmd-port 5555 --state-port 5556 --frame-id 0
```

### Troubleshooting: controller starts but robot does not move

If you see:

```text
libfranka: Move command rejected: command not possible in the current mode!
```

check the following:

1. Robot is in FCI/external-control mode on the Desk UI and not in a fault state.
2. Clear controller error once from inside container:

```bash
docker compose exec franka_setup bash -lc "source /opt/ros/noetic/setup.bash && rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal '{}'"
```

3. Relaunch impedance controller after recovery.
4. Use the default teleop settings in this repo (smaller Cartesian increments) or lower `--pos-scale` further, e.g. `--pos-scale 0.001`.

### Relax collision/reflex sensitivity

If you get reflex aborts like `motion aborted by reflex! ["cartesian_reflex"]`,
you can increase Franka collision thresholds after controller startup.

Inside container:

```bash
source /opt/ros/noetic/setup.bash
source /opt/franka_setup/ros_ws/devel/setup.bash
/opt/franka_setup/scripts/set_collision_behavior.sh --profile relaxed
```

Profiles:
- `normal`: baseline thresholds
- `relaxed`: +20%
- `relaxed_more`: +40%
- default printed command: `set_force_torque_collision_behavior`

Check currently tracked values:

```bash
docker compose exec franka_setup bash -lc "source /opt/ros/noetic/setup.bash && /opt/franka_setup/scripts/set_collision_behavior.sh --print-current"
```

This prints live values from `/franka_control/collision_config/*` when available.

Print command for a specific service:

```bash
/opt/franka_setup/scripts/set_collision_behavior.sh --profile relaxed --service force_torque
```

Start with `relaxed`, then test slowly. Re-apply after controller restart.

## Smoke Test From Host

Install client package locally (optional):

```bash
cd /home/tactile/mpail/franka_setup
python3 -m pip install -e python/franka_bridge_client
python3 scripts/smoke_test_client.py
```

Test optional dedicated gripper ports:

```bash
python3 scripts/smoke_test_client.py --test-gripper
```

## Safe Controller Test

For a guarded first motion test (observe-first, explicit confirmations, tiny single-joint nudge, return-to-start):

```bash
python3 scripts/safe_controller_test.py
```

Enable tiny motion phase only when ready:

```bash
python3 scripts/safe_controller_test.py --allow-motion
```

Joint-mode fallback (if you explicitly want joint nudge):

```bash
python3 scripts/safe_controller_test.py --allow-motion --mode joint --joint-index 6 --delta-rad 0.01
```

## SpaceMouse Teleop

Install local SpaceMouse dependency (host side):

```bash
python3 -m pip install pyspacemouse scipy
```

Run teleop:

```bash
python3 scripts/spacemouse_teleop.py
```

Example with workspace limits (XYZ cube + RPY cone):

```bash
python3 scripts/spacemouse_teleop.py \
  --xyz-min "0.35,-0.35,0.10" \
  --xyz-max "0.75,0.35,0.65" \
  --rpy-cone-deg 25
```

Controls:
- Collect-demos-style deadbanded motion (no command when inactive).
- Left SpaceMouse button closes gripper (edge-triggered).
- Right SpaceMouse button opens gripper (edge-triggered).
- Use `--enable-rotation` to apply rotational input (disabled by default for safety).
- Rotation is composed in tool frame by default (`--rotation-frame tool`); try `--rotation-frame world` if preferred.
- For gentler orientation updates, lower `--rot-scale` (e.g. `0.01`) and `--max-rot-step-rad` (e.g. `0.01`).
- Use `--require-hold-left` if you want hold-to-move safety gating.

## Python Client Example

```python
from franka_bridge_client import FrankaClient

client = FrankaClient(
    cmd_address="tcp://127.0.0.1:5555",
    state_address="tcp://127.0.0.1:5556",
)

print(client.ping())
print(client.get_state())
latest = client.get_latest_state(max_wait_s=1.0)
print(latest)
client.close()
```

## Notes

- Gripper command/state methods in this initial scaffold are wired through the bridge and state model, but likely need project-specific ROS gripper action/service bindings for your hardware.
- The bridge validates that all enabled ports are unique at startup and exits if duplicates are configured.
