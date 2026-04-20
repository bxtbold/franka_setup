#!/usr/bin/env bash
set -euo pipefail

# Set Franka contact/collision thresholds via franka_control services.
# Run this after the controller is up.
#
# Example:
#   set_collision_behavior.sh --profile relaxed
#
# Profiles are multipliers over Franka example baseline thresholds:
# - normal: 1.0
# - relaxed: 1.2
# - relaxed_more: 1.4

PROFILE="relaxed"
PRINT_CURRENT=0
PREFERRED_SERVICE="force_torque"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --print-current)
      PRINT_CURRENT=1
      shift
      ;;
    --service)
      PREFERRED_SERVICE="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--profile normal|relaxed|relaxed_more] [--print-current] [--service force_torque|full]" >&2
      exit 2
      ;;
  esac
done

case "${PROFILE}" in
  normal) SCALE="1.0" ;;
  relaxed) SCALE="1.2" ;;
  relaxed_more) SCALE="1.4" ;;
  *)
    echo "Invalid --profile '${PROFILE}'. Use normal|relaxed|relaxed_more." >&2
    exit 2
    ;;
esac

case "${PREFERRED_SERVICE}" in
  force_torque|full) ;;
  *)
    echo "Invalid --service '${PREFERRED_SERVICE}'. Use force_torque|full." >&2
    exit 2
    ;;
esac

source /opt/ros/noetic/setup.bash
if [[ -f /opt/franka_setup/ros_ws/devel/setup.bash ]]; then
  source /opt/franka_setup/ros_ws/devel/setup.bash
fi

if ! rosparam list >/dev/null 2>&1; then
  echo "ROS master is not reachable. Start roscore/controller first." >&2
  exit 1
fi

if [[ "${PRINT_CURRENT}" == "1" ]]; then
  if rosparam get /franka_control/collision_config/lower_torque_thresholds_nominal >/dev/null 2>&1; then
    echo "Current franka_control collision_config:"
    for key in \
      lower_torque_thresholds_acceleration \
      upper_torque_thresholds_acceleration \
      lower_torque_thresholds_nominal \
      upper_torque_thresholds_nominal \
      lower_force_thresholds_acceleration \
      upper_force_thresholds_acceleration \
      lower_force_thresholds_nominal \
      upper_force_thresholds_nominal; do
      echo "- ${key}: $(rosparam get /franka_control/collision_config/${key})"
    done
    if rosparam get /franka_setup/collision_behavior/last_applied/profile >/dev/null 2>&1; then
      echo
      echo "Last applied via set_collision_behavior.sh:"
      rosparam get /franka_setup/collision_behavior/last_applied
    fi
  else
    echo "No /franka_control/collision_config params found yet."
    echo "Is franka_control running?"
  fi
  exit 0
fi

if ! rosservice list | rg -q "^/franka_control/set_(full_collision_behavior|force_torque_collision_behavior)$"; then
  echo "No Franka collision service found yet. Is franka_control running?" >&2
  exit 1
fi

scale_list() {
  local csv="$1"
  local scale="$2"
  python3 - "$csv" "$scale" <<'PY'
import sys
values = [float(x.strip()) for x in sys.argv[1].split(",")]
scale = float(sys.argv[2])
print("[" + ", ".join(f"{v*scale:.3f}" for v in values) + "]")
PY
}

# Franka example baseline thresholds.
BASE_TORQUE="20,20,18,18,16,14,12"
BASE_FORCE="20,20,20,25,25,25"

TORQUE="$(scale_list "${BASE_TORQUE}" "${SCALE}")"
FORCE="$(scale_list "${BASE_FORCE}" "${SCALE}")"

if [[ "${PREFERRED_SERVICE}" == "force_torque" ]]; then
  cat <<EOF
rosservice call /franka_control/set_force_torque_collision_behavior "{
  lower_torque_thresholds_nominal: ${TORQUE},
  upper_torque_thresholds_nominal: ${TORQUE},
  lower_force_thresholds_nominal: ${FORCE},
  upper_force_thresholds_nominal: ${FORCE}
}"
EOF
else
  cat <<EOF
rosservice call /franka_control/set_full_collision_behavior "{
  lower_torque_thresholds_acceleration: ${TORQUE},
  upper_torque_thresholds_acceleration: ${TORQUE},
  lower_torque_thresholds_nominal: ${TORQUE},
  upper_torque_thresholds_nominal: ${TORQUE},
  lower_force_thresholds_acceleration: ${FORCE},
  upper_force_thresholds_acceleration: ${FORCE},
  lower_force_thresholds_nominal: ${FORCE},
  upper_force_thresholds_nominal: ${FORCE}
}"
EOF
fi


rosparam set /franka_setup/collision_behavior/last_applied/profile "${PROFILE}"
rosparam set /franka_setup/collision_behavior/last_applied/scale "${SCALE}"
rosparam set /franka_setup/collision_behavior/last_applied/torque_thresholds "${TORQUE}"
rosparam set /franka_setup/collision_behavior/last_applied/force_thresholds "${FORCE}"
rosparam set /franka_setup/collision_behavior/last_applied/timestamp "$(date -Iseconds)"
rosparam set /franka_setup/collision_behavior/last_applied/service_preference "${PREFERRED_SERVICE}"

echo "Printed collision service command for profile '${PROFILE}' (scale ${SCALE})."
