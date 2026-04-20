#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts can assume these exist; define safe defaults first.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

source /opt/ros/noetic/setup.bash
if [[ -f /opt/franka_setup/ros_ws/devel/setup.bash ]]; then
  source /opt/franka_setup/ros_ws/devel/setup.bash
fi

ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
LOAD_GRIPPER="${LOAD_GRIPPER:-false}"
LAUNCH_SERL_CONTROLLER="${LAUNCH_SERL_CONTROLLER:-1}"
LAUNCH_ROSCORE="${LAUNCH_ROSCORE:-1}"
FRANKA_CMD_PORT="${FRANKA_CMD_PORT:-5555}"
FRANKA_STATE_PORT="${FRANKA_STATE_PORT:-5556}"
FRANKA_ENABLE_GRIPPER_PORTS="${FRANKA_ENABLE_GRIPPER_PORTS:-0}"
FRANKA_GRIPPER_CMD_PORT="${FRANKA_GRIPPER_CMD_PORT:-5557}"
FRANKA_GRIPPER_STATE_PORT="${FRANKA_GRIPPER_STATE_PORT:-5558}"
FRANKA_STATE_HZ="${FRANKA_STATE_HZ:-60}"
FRANKA_GRIPPER_STATE_HZ="${FRANKA_GRIPPER_STATE_HZ:-60}"
FRANKA_ROS_PUBLISH_RATE="${FRANKA_ROS_PUBLISH_RATE:-60}"
FRANKA_COMPLIANCE_NODE="${FRANKA_COMPLIANCE_NODE:-/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node}"
FRANKA_TRANSLATIONAL_STIFFNESS="${FRANKA_TRANSLATIONAL_STIFFNESS:-800.0}"
FRANKA_ROTATIONAL_STIFFNESS="${FRANKA_ROTATIONAL_STIFFNESS:-50.0}"

# If the robot gripper is enabled, expose dedicated gripper sockets by default.
# Set FRANKA_ENABLE_GRIPPER_PORTS explicitly to 0 to force-disable.
if [[ "${LOAD_GRIPPER}" == "true" && "${FRANKA_ENABLE_GRIPPER_PORTS}" != "1" ]]; then
  FRANKA_ENABLE_GRIPPER_PORTS=1
fi

mkdir -p /opt/franka_setup/logs

cleanup() {
  jobs -p | xargs -r kill
}
trap cleanup EXIT INT TERM

if [[ "${LAUNCH_ROSCORE}" == "1" ]]; then
  roscore >/opt/franka_setup/logs/roscore.log 2>&1 &
  sleep 3
fi

if [[ "${LAUNCH_SERL_CONTROLLER}" == "1" ]]; then
  # Ensure non-realtime hosts can still start franka_control.
  for cfg in \
    /opt/franka_setup/ros_ws/src/franka_ros/franka_control/config/franka_control_node.yaml \
    /opt/ros/noetic/share/franka_control/config/franka_control_node.yaml; do
    if [[ -f "${cfg}" ]]; then
      sed -i 's/^realtime_config:.*/realtime_config: ignore/' "${cfg}"
    fi
  done

  # Apply publish rate before launch; many nodes read this only at startup.
  for cfg in \
    /opt/franka_setup/ros_ws/src/franka_ros/franka_control/config/franka_control_node.yaml \
    /opt/ros/noetic/share/franka_control/config/franka_control_node.yaml; do
    if [[ -f "${cfg}" ]]; then
      sed -i -E "s/^([[:space:]]*publish_rate:[[:space:]]*).*/\1${FRANKA_ROS_PUBLISH_RATE}/" "${cfg}"
    fi
  done

  roslaunch serl_franka_controllers impedance.launch \
    robot_ip:="${ROBOT_IP}" \
    load_gripper:="${LOAD_GRIPPER}" \
    >/opt/franka_setup/logs/serl_controller.log 2>&1 &
  sleep 5

  # FRANKA_STATE_HZ controls ZMQ state streaming only. ROS topics are configured separately.
  rosparam set /franka_state_controller/publish_rate "${FRANKA_ROS_PUBLISH_RATE}" || true
  rosparam set /joint_state_publisher/rate "${FRANKA_ROS_PUBLISH_RATE}" || true
  rosparam set /joint_state_desired_publisher/rate "${FRANKA_ROS_PUBLISH_RATE}" || true

  # Re-apply compliance values after controller startup.
  compliance_param="${FRANKA_COMPLIANCE_NODE}/translational_stiffness"
  for _ in $(seq 1 20); do
    if rosparam get "${compliance_param}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  if rosparam get "${compliance_param}" >/dev/null 2>&1; then
    rosrun dynamic_reconfigure dynparam set "${FRANKA_COMPLIANCE_NODE}" \
      "{translational_stiffness: ${FRANKA_TRANSLATIONAL_STIFFNESS}, rotational_stiffness: ${FRANKA_ROTATIONAL_STIFFNESS}}" \
      || true
  else
    echo "Warning: compliance node '${FRANKA_COMPLIANCE_NODE}' not ready; skipped stiffness set."
  fi
fi

# Bridge requires a reachable ROS master even when controller launch is manual.
until rosparam list >/dev/null 2>&1; do
  echo "Waiting for ROS master at ${ROS_MASTER_URI}..."
  sleep 2
done

bridge_args=(
  --cmd-port "${FRANKA_CMD_PORT}"
  --state-port "${FRANKA_STATE_PORT}"
  --state-hz "${FRANKA_STATE_HZ}"
)

if [[ "${FRANKA_ENABLE_GRIPPER_PORTS}" == "1" ]]; then
  bridge_args+=(
    --enable-gripper-ports
    --gripper-cmd-port "${FRANKA_GRIPPER_CMD_PORT}"
    --gripper-state-port "${FRANKA_GRIPPER_STATE_PORT}"
    --gripper-state-hz "${FRANKA_GRIPPER_STATE_HZ}"
  )
fi

exec python3 -m franka_bridge.main "${bridge_args[@]}"
