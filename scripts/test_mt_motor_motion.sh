#!/usr/bin/env bash
set -euo pipefail

# MT 运动测试：速度模式 + MIT 位置模式（增强版：MIT 往返位移更明显）
# 用法：
#   bash scripts/test_mt_motor_motion.sh [profile] [motor_id|auto] [vel] [vel_duration] [pos_rad] [mit_hold_sec]
# 示例：
#   bash scripts/test_mt_motor_motion.sh car_a 0x141 5.0 2.0 2.5 2.0
#   bash scripts/test_mt_motor_motion.sh car_a auto 5.0 2.0 2.5 2.0

PROFILE="${1:-car_a}"
MOTOR_ID_INPUT="${2:-auto}"
VEL="${3:-5.0}"
VEL_DURATION="${4:-2.0}"
POS_RAD="${5:-2.5}"
MIT_HOLD_SEC="${6:-2.0}"
MIT_STREAM_HZ="${MT_TEST_MIT_STREAM_HZ:-20}"
MIT_VERIFY_POS_RAD="${MT_TEST_MIT_VERIFY_POS_RAD:-2.5}"
MT_TEST_INIT_LOOPBACK="${MT_TEST_INIT_LOOPBACK:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MT_IF="${SCRIPT_DIR}/mt_motor_interface.py"

if [[ ! -x "${MT_IF}" ]]; then
  echo "[ERR] 未找到可执行接口脚本: ${MT_IF}"
  echo "请先执行: chmod +x scripts/mt_motor_interface.py"
  exit 2
fi

run_if() {
  python3 "${MT_IF}" --profile "${PROFILE}" --motor-id "${MOTOR_ID}" "$@"
}

run_if_motor() {
  local motor_id="$1"
  shift
  python3 "${MT_IF}" --profile "${PROFILE}" --motor-id "${motor_id}" "$@"
}

resolve_profile_field() {
  local field="$1"
  local profiles_file="${SCRIPT_DIR}/../config/mt_control_profiles.yaml"
  if [[ ! -f "${profiles_file}" ]]; then
    return 0
  fi
  python3 - "${profiles_file}" "${PROFILE}" "${field}" <<'PY'
import sys, yaml
path, profile, field = sys.argv[1], sys.argv[2], sys.argv[3]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
cfg = (data.get("profiles") or {}).get(profile) or {}
value = cfg.get(field, "")
if isinstance(value, str):
    print(value)
PY
}

DRIVER_NS="${MT_TEST_DRIVER_NS:-$(resolve_profile_field driver_ns)}"
if [[ -z "${DRIVER_NS}" ]]; then
  DRIVER_NS="/can_driver_node"
fi
if [[ "${DRIVER_NS}" == "/" ]]; then
  DRIVER_NS=""
fi
CAN_DEVICE="${MT_TEST_INIT_DEVICE:-$(resolve_profile_field can_device)}"

INIT_SRV="${DRIVER_NS}/init"
ENABLE_SRV="${DRIVER_NS}/enable"
RESUME_SRV="${DRIVER_NS}/resume"
RECOVER_SRV="${DRIVER_NS}/recover"
LIFECYCLE_STATE_TOPIC="${DRIVER_NS}/lifecycle_state"

log() {
  echo "[MT-TEST] $*"
}

err() {
  echo "[MT-TEST][ERR] $*" >&2
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    err "missing command: ${cmd}"
    exit 2
  fi
}

require_cmd rosservice
require_cmd rostopic
require_cmd python3

ensure_ros_online() {
  if ! rosservice list >/dev/null 2>&1; then
    err "ROS master unavailable. Please run roscore/bringup first."
    exit 2
  fi
}

call_trigger_srv() {
  local service_name="$1"
  local output
  if ! output="$(rosservice call "${service_name}")"; then
    err "lifecycle service failed: ${service_name}"
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    err "lifecycle service rejected: ${service_name}"
    echo "${output}" >&2
    exit 4
  fi
}

call_init_srv() {
  local output
  if [[ -z "${CAN_DEVICE}" ]]; then
    err "cannot resolve init device. set MT_TEST_INIT_DEVICE or profile.can_device"
    exit 4
  fi
  if ! output="$(rosservice call "${INIT_SRV}" "{device: '${CAN_DEVICE}', loopback: ${MT_TEST_INIT_LOOPBACK}}")"; then
    err "lifecycle init failed: ${INIT_SRV}"
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    err "lifecycle init rejected: ${INIT_SRV}"
    echo "${output}" >&2
    exit 4
  fi
}

call_recover_srv() {
  local output
  if ! output="$(rosservice call "${RECOVER_SRV}" "{motor_id: 65535}")"; then
    err "lifecycle recover failed: ${RECOVER_SRV}"
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    err "lifecycle recover rejected: ${RECOVER_SRV}"
    echo "${output}" >&2
    exit 4
  fi
}

get_lifecycle_state() {
  timeout 3s rostopic echo "${LIFECYCLE_STATE_TOPIC}" 2>/dev/null | awk '
    $1=="data:" {
      gsub(/"/, "", $2)
      print $2
      exit
    }'
}

wait_lifecycle_state() {
  local expected="$1"
  local timeout_sec="${2:-5.0}"
  python3 - "$expected" "$timeout_sec" "$LIFECYCLE_STATE_TOPIC" <<'PY'
import subprocess
import sys
import time

expected = sys.argv[1]
timeout_sec = float(sys.argv[2])
topic = sys.argv[3]
deadline = time.time() + timeout_sec

while time.time() < deadline:
    proc = subprocess.run(
        ["timeout", "2s", "rostopic", "echo", topic],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    for line in proc.stdout.splitlines():
        parts = line.split(":", 1)
        if len(parts) == 2 and parts[0].strip() == "data":
            value = parts[1].strip().strip('"')
            if value == expected:
                sys.exit(0)
    time.sleep(0.1)

sys.exit(1)
PY
}

ensure_lifecycle_running() {
  ensure_ros_online
  local state
  state="$(get_lifecycle_state || true)"
  log "lifecycle state=${state:-<unknown>} ns=${DRIVER_NS:-/}"

  if [[ "${state}" == "Running" ]]; then
    return 0
  fi

  if [[ "${state}" == "Faulted" ]]; then
    log "lifecycle Faulted: recover -> Standby"
    call_recover_srv
    wait_lifecycle_state "Standby" 8.0 || true
    state="$(get_lifecycle_state || true)"
  fi

  if [[ "${state}" == "Inactive" || "${state}" == "Configured" || -z "${state}" ]]; then
    log "lifecycle init device=${CAN_DEVICE} loopback=${MT_TEST_INIT_LOOPBACK}"
    call_init_srv
    wait_lifecycle_state "Armed" 8.0 || true
    state="$(get_lifecycle_state || true)"
  fi

  if [[ "${state}" == "Standby" ]]; then
    log "lifecycle enable"
    call_trigger_srv "${ENABLE_SRV}"
    wait_lifecycle_state "Armed" 8.0 || true
    state="$(get_lifecycle_state || true)"
  fi

  if [[ "${state}" == "Armed" ]]; then
    log "lifecycle resume"
    call_trigger_srv "${RESUME_SRV}"
    wait_lifecycle_state "Running" 8.0 || true
    state="$(get_lifecycle_state || true)"
  fi

  if [[ "${state}" != "Running" ]]; then
    err "lifecycle did not reach Running (current=${state:-<unknown>})"
    err "check services: ${INIT_SRV} ${ENABLE_SRV} ${RESUME_SRV} and topic ${LIFECYCLE_STATE_TOPIC}"
    exit 4
  fi
}

discover_runtime_mt_ids() {
  python3 "${MT_IF}" --profile "${PROFILE}" --action discover 2>/dev/null \
    | awk '
      /motor_id=/ {
        for (i = 1; i <= NF; ++i) {
          if ($i ~ /^motor_id=/) {
            split($i, a, "=");
            print a[2];
          }
        }
      }'
}

discover_profile_mt_ids() {
  local profiles_file="${SCRIPT_DIR}/../config/mt_control_profiles.yaml"
  if [[ ! -f "${profiles_file}" ]]; then
    return 0
  fi
  python3 - "${profiles_file}" "${PROFILE}" <<'PY'
import sys, yaml
path, profile = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
cfg = (data.get("profiles") or {}).get(profile) or {}
ids = cfg.get("mt_motor_ids") or []
for item in ids:
    try:
        val = int(item, 0) if isinstance(item, str) else int(item)
    except Exception:
        continue
    print(hex(val))
PY
}

resolve_equivalent_runtime_id() {
  local requested="$1"
  shift
  python3 - "${requested}" "$@" <<'PY'
import sys
if len(sys.argv) < 2:
    raise SystemExit(0)
req = int(sys.argv[1], 0)
runtime = set(int(x, 0) for x in sys.argv[2:] if x)
low = req & 0xFF
candidates = [req, low, (0x100 | low)]
for c in candidates:
    if c in runtime:
        print(hex(c))
        raise SystemExit(0)
raise SystemExit(1)
PY
}

send_mit_position_hold() {
  local target_rad="$1"
  local hold_sec="$2"

  local step_sec
  step_sec="$(python3 - <<PY
hz=float('${MIT_STREAM_HZ}')
print(1.0/max(1.0,hz))
PY
)"

  python3 - <<PY
import time, subprocess, shlex
target='${target_rad}'
hold=float('${hold_sec}')
step=float('${step_sec}')
cmd="python3 ${MT_IF} --profile ${PROFILE} --motor-id ${MOTOR_ID} --action position --value " + target
cmd += " --no-auto-mode"
end=time.time()+max(0.0, hold)
while time.time()<end:
    subprocess.run(shlex.split(cmd), check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(step)
# 保底再打一帧，避免边界时刻丢最后目标
subprocess.run(shlex.split(cmd), check=True)
PY
}

select_motor_id() {
  local req_id="${MOTOR_ID_INPUT}"
  local -a runtime_ids=()
  local -a profile_ids=()
  local -a candidates=()
  local -A seen=()

  mapfile -t runtime_ids < <(discover_runtime_mt_ids || true)
  mapfile -t profile_ids < <(discover_profile_mt_ids || true)

  add_candidate() {
    local id="$1"
    [[ -z "${id}" ]] && return 0
    if [[ -z "${seen[${id}]:-}" ]]; then
      seen["${id}"]=1
      candidates+=("${id}")
    fi
  }

  if [[ "${req_id}" != "auto" && "${req_id}" != "AUTO" ]]; then
    if [[ "${#runtime_ids[@]}" -gt 0 ]]; then
      local resolved=""
      if resolved="$(resolve_equivalent_runtime_id "${req_id}" "${runtime_ids[@]}" 2>/dev/null)"; then
        add_candidate "${resolved}"
      else
        add_candidate "${req_id}"
      fi
    else
      add_candidate "${req_id}"
    fi
  else
    local id
    for id in "${runtime_ids[@]}"; do
      add_candidate "${id}"
    done
    for id in "${profile_ids[@]}"; do
      add_candidate "${id}"
    done
    add_candidate "0x141"
    add_candidate "0x142"
  fi

  if [[ "${#candidates[@]}" -eq 0 ]]; then
    return 1
  fi
  printf '%s\n' "${candidates[0]}"
  return 0
}

if ! MOTOR_ID="$(select_motor_id)"; then
  err "未找到可用 MT 电机ID（runtime/profile/default 均为空）"
  err "可先执行: rosrun can_driver mt_motor_interface.py --profile ${PROFILE} --action list"
  err "或执行: rosrun can_driver mt_motor_interface.py --profile ${PROFILE} --action discover"
  exit 3
fi

log "profile=${PROFILE}, motor=${MOTOR_ID}, ns=${DRIVER_NS}, init_device=${CAN_DEVICE}"
ensure_lifecycle_running

echo "[1/10] Enable"
run_if --action enable

echo "[2/10] Velocity mode"
run_if --action mode --value 1

echo "[3/10] Velocity +${VEL}"
run_if --action velocity --value "${VEL}"
sleep "${VEL_DURATION}"

echo "[4/10] Velocity -${VEL}"
run_if --action velocity --value "-$(python3 - <<PY
v=float('${VEL}')
print(abs(v))
PY
)"
sleep "${VEL_DURATION}"

echo "[5/10] 速度归零（验证位置模式前必须停转）"
run_if --action mode --value 1
run_if --action velocity --value 0
sleep 1.0

echo "[6/10] Stop（保险）"
run_if --action stop
sleep 0.5

echo "[7/10] Position mode(MIT)"
run_if --action mode --value 0
sleep 0.2

echo "[8/10] MIT 位置 +${POS_RAD} rad"
send_mit_position_hold "${POS_RAD}" "${MIT_HOLD_SEC}"

echo "[9/10] MIT 位置 -${POS_RAD} rad"
NEG_POS="-$(python3 - <<PY
p=float('${POS_RAD}')
print(abs(p))
PY
)"
send_mit_position_hold "${NEG_POS}" "${MIT_HOLD_SEC}"

echo "[10/10] MIT 回零 (0 rad)"
send_mit_position_hold "0" "${MIT_HOLD_SEC}"

echo "[MIT-VERIFY] 强测: 重复置位模式 + 固定大位移 ${MIT_VERIFY_POS_RAD} rad"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "${MIT_VERIFY_POS_RAD}" "${MIT_HOLD_SEC}"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "-${MIT_VERIFY_POS_RAD}" "${MIT_HOLD_SEC}"
run_if --action mode --value 0
sleep 0.1
send_mit_position_hold "0" "${MIT_HOLD_SEC}"

echo "[End] Final stop (保持可继续调试)"
run_if --action stop

if [[ "${MT_TEST_SEND_DISABLE:-0}" == "1" ]]; then
  echo "[End+] Disable (按需)"
  run_if --action disable
fi

echo "[MT-TEST] 完成"
