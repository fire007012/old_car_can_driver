#!/usr/bin/env bash
set -euo pipefail

# ECB 网络电机测试脚本（位置+速度）
#
# 设计目标：
# 1) 一条命令完成 Enable -> 速度模式 -> 位置模式 的联调闭环；
# 2) 尽量降低人工排障成本（自动选电机、输出关键状态、统一错误提示）；
# 3) 保持对命名空间和参数可配置，便于不同现场环境复用。
#
# 用法：
#   bash scripts/test_ecb_motor_motion.sh [motor_id|auto] [vel_rad_s] [vel_hold_s] [pos_rad] [pos_hold_s] [driver_ns]
# 示例：
#   bash scripts/test_ecb_motor_motion.sh auto 0.8 2.0 1.2 2.0 /can_driver_node

MOTOR_ID_INPUT="${1:-auto}"
VEL_CMD="${2:-0.8}"
VEL_HOLD_SEC="${3:-2.0}"
POS_CMD="${4:-1.2}"
POS_HOLD_SEC="${5:-2.0}"
DRIVER_NS="${6:-/can_driver_node}"

# 统一服务和状态话题命名，避免脚本里多处硬编码。
MOTOR_SRV="${DRIVER_NS}/motor_command"
MOTOR_STATE_TOPIC="${DRIVER_NS}/motor_states"
LIFECYCLE_STATE_TOPIC="${DRIVER_NS}/lifecycle_state"
HALT_SRV="${DRIVER_NS}/halt"
RESUME_SRV="${DRIVER_NS}/resume"
ENABLE_SRV="${DRIVER_NS}/enable"
RECOVER_SRV="${DRIVER_NS}/recover"

log() {
  echo "[ECB-TEST] $*"
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "[ECB-TEST][ERR] missing command: ${cmd}" >&2
    exit 2
  fi
}

require_cmd rosservice
require_cmd rostopic
require_cmd rosparam
require_cmd rossrv
require_cmd python3

ensure_can_driver_srv_type() {
  if ! rossrv show can_driver/MotorCommand >/dev/null 2>&1; then
    echo "[ECB-TEST][ERR] cannot load ROS service type can_driver/MotorCommand." >&2
    echo "[ECB-TEST][ERR] Please build/source the workspace in this terminal:" >&2
    echo "[ECB-TEST][ERR]   cd /home/rera/catkin_ws && catkin build can_driver && source devel/setup.bash" >&2
    echo "[ECB-TEST][ERR] If you use catkin_make: cd /home/rera/catkin_ws && catkin_make --pkg can_driver && source devel/setup.bash" >&2
    exit 2
  fi
}

ensure_ros_online() {
  # 这里用 rosservice list 作为 master 存活探针，失败就直接退出，
  # 防止后续命令逐个超时导致排障体验很差。
  if ! rosservice list >/dev/null 2>&1; then
    echo "[ECB-TEST][ERR] ROS master unavailable. Please run roscore/bringup first." >&2
    exit 2
  fi
}

ensure_can_driver_srv_type

resolve_ecb_joint_and_motor() {
  local req_motor_id="$1"
  # 用内嵌 Python 解析 rosparam 中的 joints，原因：
  # - shell 对 YAML 的结构化解析能力弱；
  # - 这里还需要做 protocol 过滤、motor_id 进制兼容和 auto 选择策略。
  python3 - "$req_motor_id" "$DRIVER_NS" <<'PY'
import collections
import subprocess
import sys
import yaml

req_motor = sys.argv[1]
ns = sys.argv[2].rstrip('/')
primary_param = f"{ns}/joints" if ns else "/joints"

candidates = []
for p in [primary_param, "/can_driver_node/joints", "/joints"]:
  if p and p not in candidates:
    candidates.append(p)

loaded = None
loaded_param = ""
protocol_counter = collections.Counter()

for param_name in candidates:
  try:
    output = subprocess.check_output(["rosparam", "get", param_name], text=True)
  except subprocess.CalledProcessError:
    continue

  try:
    joints = yaml.safe_load(output)
  except Exception:
    continue

  if isinstance(joints, list):
    loaded = joints
    loaded_param = param_name
    for j in joints:
      if isinstance(j, dict):
        protocol_counter[str(j.get("protocol", "")).upper()] += 1
    break

if loaded is None:
  print("ERR: joints parameter not found or invalid yaml. tried: " + ", ".join(candidates))
  sys.exit(1)

joints = loaded

def parse_mid(v):
    if isinstance(v, int):
        return v
    if isinstance(v, str):
        return int(v, 0)
    raise ValueError("motor_id must be int or string")

ecb = []
for j in joints:
    if not isinstance(j, dict):
        continue
    if str(j.get("protocol", "")).upper() != "ECB":
        continue
    try:
        mid = parse_mid(j.get("motor_id"))
    except Exception:
        continue
    pos_scale = float(j.get("position_scale", 1.0))
    vel_scale = float(j.get("velocity_scale", 1.0))
    ecb.append((mid, str(j.get("name", "")), str(j.get("can_device", "")), pos_scale, vel_scale))

if not ecb:
  proto_text = ", ".join(f"{k}:{v}" for k, v in sorted(protocol_counter.items())) if protocol_counter else "<none>"
  print("ERR: no ECB joint found in joints config"
      f" (loaded={loaded_param}, protocols={proto_text}). "
      "Please add at least one joint with protocol: ECB and relaunch can_driver.")
  sys.exit(1)

if req_motor.lower() != "auto":
    try:
        req_mid = int(req_motor, 0)
    except Exception:
        print(f"ERR: invalid motor id: {req_motor}")
        sys.exit(1)
    for mid, name, dev, pos_scale, vel_scale in ecb:
        if mid == req_mid:
            print(f"{mid} {name} {dev} {pos_scale} {vel_scale}")
            sys.exit(0)
    print(f"ERR: motor_id {req_motor} not found in ECB joints")
    sys.exit(1)

# auto: 优先固定IP设备（ecb://<ip>），其次其它 ECB。
# 经验：固定 IP 通常比自动扫描更稳定，适合作为优先测试目标。
ecb_sorted = sorted(ecb, key=lambda x: (x[2].endswith('auto'), x[0]))
mid, name, dev, pos_scale, vel_scale = ecb_sorted[0]
print(f"{mid} {name} {dev} {pos_scale} {vel_scale}")
PY
}

call_motor_cmd() {
  # MotorCommand 协议常量：
  # 0=ENABLE, 1=DISABLE, 2=STOP, 3=SET_MODE(value:0/1)
  local motor_id="$1"
  local command="$2"
  local value="$3"
  local output
  if ! output="$(rosservice call "${MOTOR_SRV}" "{motor_id: ${motor_id}, command: ${command}, value: ${value}}")"; then
    echo "[ECB-TEST][ERR] motor_command failed: motor_id=${motor_id}, command=${command}, value=${value}" >&2
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    echo "[ECB-TEST][ERR] motor_command rejected: motor_id=${motor_id}, command=${command}, value=${value}" >&2
    echo "${output}" >&2
    exit 4
  fi
}

call_trigger_srv() {
  local service_name="$1"
  local output
  if ! output="$(rosservice call "${service_name}")"; then
    echo "[ECB-TEST][ERR] lifecycle service failed: ${service_name}" >&2
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    echo "[ECB-TEST][ERR] lifecycle service rejected: ${service_name}" >&2
    echo "${output}" >&2
    exit 4
  fi
}

call_recover_srv() {
  local output
  if ! output="$(rosservice call "${RECOVER_SRV}" "{motor_id: 65535}")"; then
    echo "[ECB-TEST][ERR] lifecycle recover failed: ${RECOVER_SRV}" >&2
    echo "${output}" >&2
    exit 4
  fi
  if printf '%s\n' "${output}" | grep -q "success: False"; then
    echo "[ECB-TEST][ERR] lifecycle recover rejected: ${RECOVER_SRV}" >&2
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
    }
  '
}

wait_lifecycle_state() {
  local expected="$1"
  local timeout_sec="${2:-3.0}"
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

prepare_mode_switch() {
  local label="$1"
  recover_if_faulted "${label} mode switch"
  local state
  state="$(get_lifecycle_state || true)"
  if [[ "${state}" == "Armed" ]]; then
    log "lifecycle already Armed before ${label} mode switch"
    return
  fi

  log "Pause lifecycle before ${label} mode switch"
  call_trigger_srv "${HALT_SRV}"
  if wait_lifecycle_state "Armed" 3.0; then
    log "lifecycle Armed"
  else
    log "lifecycle not confirmed Armed within timeout; continue but watch roslaunch logs"
  fi
}

resume_after_mode_switch() {
  local label="$1"
  local state
  state="$(get_lifecycle_state || true)"
  if [[ "${state}" == "Running" ]]; then
    log "lifecycle already Running after ${label} mode switch"
    return
  fi

  log "Resume lifecycle after ${label} mode switch"
  call_trigger_srv "${RESUME_SRV}"
  if wait_lifecycle_state "Running" 3.0; then
    log "lifecycle Running"
  else
    log "lifecycle not confirmed Running within timeout; continue but watch roslaunch logs"
  fi
}

recover_if_faulted() {
  local label="$1"
  local state
  state="$(get_lifecycle_state || true)"
  if [[ "${state}" != "Faulted" ]]; then
    return
  fi

  log "Lifecycle is Faulted before ${label}; use safety recover path"
  call_recover_srv
  if wait_lifecycle_state "Standby" 5.0; then
    log "lifecycle Standby after recover"
  else
    log "lifecycle not confirmed Standby within timeout; continue but watch roslaunch logs"
  fi

  log "Enable lifecycle after recover"
  call_trigger_srv "${ENABLE_SRV}"
  if wait_lifecycle_state "Armed" 5.0; then
    log "lifecycle Armed after recover"
  else
    log "lifecycle not confirmed Armed after recover; continue but watch roslaunch logs"
  fi
}

publish_once() {
  # -1 表示只发一帧，适合 stop 或归零边界点。
  local topic="$1"
  local value="$2"
  rostopic pub -1 "${topic}" std_msgs/Float64 "data: ${value}" >/dev/null
}

hold_publish() {
  # ECB 位置/速度测试采用“流式重复下发”，而不是只发一帧：
  # - 可更稳定复现实机行为；
  # - 可覆盖控制环中瞬时丢包/抖动场景；
  # - 便于观察模式切换后的持续跟踪效果。
  local topic="$1"
  local value="$2"
  local hold_sec="$3"
  local hz="${ECB_TEST_STREAM_HZ:-20}"
  python3 - "$topic" "$value" "$hold_sec" "$hz" <<'PY'
import subprocess
import sys
import time

topic = sys.argv[1]
value = float(sys.argv[2])
hold = max(0.0, float(sys.argv[3]))
hz = max(1.0, float(sys.argv[4]))
period = 1.0 / hz
end = time.time() + hold

cmd = [
    "rostopic",
    "pub",
    "-1",
    topic,
    "std_msgs/Float64",
    f"data: {value}",
]
while time.time() < end:
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(period)
subprocess.run(cmd, check=True)
PY
}

show_last_state() {
  log "采样目标电机的 motor_states 最近一条（用于快速排错）"
  timeout 3s rostopic echo "${MOTOR_STATE_TOPIC}" 2>/dev/null | awk -v id="${MOTOR_ID}" '
    /^motor_id:/ {
      current_id=$2
      block=$0 ORS
      next
    }
    current_id != "" {
      block=block $0 ORS
      if ($0 == "---") {
        if (current_id == id) {
          printf "%s", block
          exit
        }
        current_id=""
        block=""
      }
    }
  ' || true
}

get_motor_position_raw() {
  timeout 5s rostopic echo "${MOTOR_STATE_TOPIC}" 2>/dev/null | awk -v id="${MOTOR_ID}" '
    $1=="motor_id:" { current_id=$2; next }
    $1=="position:" && current_id==id {
      print $2
      exit
    }
  '
}

get_motor_state_field() {
  local field_name="$1"
  timeout 5s rostopic echo "${MOTOR_STATE_TOPIC}" 2>/dev/null | awk -v id="${MOTOR_ID}" -v field="${field_name}" '
    $1=="motor_id:" { current_id=$2; next }
    current_id==id && $1==(field ":") {
      print $2
      exit
    }
  '
}

wait_motor_state_equals() {
  local field_name="$1"
  local expected="$2"
  local timeout_sec="${3:-3.0}"
  python3 - "$field_name" "$expected" "$timeout_sec" "$MOTOR_STATE_TOPIC" "$MOTOR_ID" <<'PY'
import subprocess
import sys
import time

field = sys.argv[1]
expected = sys.argv[2]
timeout_sec = float(sys.argv[3])
topic = sys.argv[4]
motor_id = sys.argv[5]
deadline = time.time() + timeout_sec

awk_script = (
    '$1=="motor_id:" { current_id=$2; next } '
    'current_id==id && $1==(field ":") { print $2; exit }'
)

while time.time() < deadline:
    try:
        proc = subprocess.run(
            ["timeout", "2s", "rostopic", "echo", topic],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if proc.stdout:
            current_id = ""
            for line in proc.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2 and parts[0] == "motor_id:":
                    current_id = parts[1]
                    continue
                if current_id == motor_id and len(parts) >= 2 and parts[0] == f"{field}:":
                    value = parts[1]
                    if value == expected:
                        sys.exit(0)
                    break
    except Exception:
        pass
    time.sleep(0.1)

sys.exit(1)
PY
}

to_rad_from_raw() {
  local raw="$1"
  python3 - "$raw" "$POSITION_SCALE" <<'PY'
import sys

raw = float(sys.argv[1])
scale = float(sys.argv[2])
print(raw * scale)
PY
}

add_rad() {
  local base="$1"
  local offset="$2"
  python3 - "$base" "$offset" <<'PY'
import sys

base = float(sys.argv[1])
offset = float(sys.argv[2])
print(base + offset)
PY
}

ensure_ros_online
log "使用 driver_ns=${DRIVER_NS}"

RESOLVE_OUT="$(resolve_ecb_joint_and_motor "${MOTOR_ID_INPUT}")" || {
  echo "[ECB-TEST][ERR] ${RESOLVE_OUT}" >&2
  exit 3
}

MOTOR_ID="$(echo "${RESOLVE_OUT}" | awk '{print $1}')"
JOINT_NAME="$(echo "${RESOLVE_OUT}" | awk '{print $2}')"
CAN_DEVICE="$(echo "${RESOLVE_OUT}" | awk '{print $3}')"
POSITION_SCALE="$(echo "${RESOLVE_OUT}" | awk '{print $4}')"
VELOCITY_SCALE="$(echo "${RESOLVE_OUT}" | awk '{print $5}')"

if [[ -z "${MOTOR_ID}" || -z "${JOINT_NAME}" || -z "${POSITION_SCALE}" || -z "${VELOCITY_SCALE}" ]]; then
  echo "[ECB-TEST][ERR] resolve failed: ${RESOLVE_OUT}" >&2
  exit 3
fi

VEL_TOPIC="${DRIVER_NS}/motor/${JOINT_NAME}/cmd_velocity"
POS_TOPIC="${DRIVER_NS}/motor/${JOINT_NAME}/cmd_position"

log "target: motor_id=${MOTOR_ID}, joint=${JOINT_NAME}, device=${CAN_DEVICE}"
log "注意：本脚本默认命令单位为 SI（rad / rad/s），是否正确取决于 can_driver.yaml 中该 joint 的 scale 配置。"
python3 - "${VEL_CMD}" "${POS_CMD}" "${VELOCITY_SCALE}" "${POSITION_SCALE}" <<'PY'
import sys
vel = abs(float(sys.argv[1]))
pos = abs(float(sys.argv[2]))
vel_scale = float(sys.argv[3])
pos_scale = float(sys.argv[4])
vel_raw = vel / vel_scale if vel_scale else 0.0
pos_raw = pos / pos_scale if pos_scale else 0.0
print(
    "[ECB-TEST] command conversion: "
    f"{vel:g} rad/s -> raw={vel_raw:.1f} -> {vel_raw / 10.0:.2f} RPM; "
    f"{pos:g} rad -> raw={pos_raw:.1f} -> {pos_raw / 10000.0:.4f} rev"
)
PY

recover_if_faulted "test"

log "[1/14] Prepare lifecycle for velocity mode switch"
prepare_mode_switch "velocity"

log "[2/14] Disable before velocity mode switch"
call_motor_cmd "${MOTOR_ID}" 1 0.0
sleep 0.3

log "[3/14] Set velocity mode"
# 模式切换必须在失能状态执行；切换后先发一帧对齐命令，再发真实目标。
call_motor_cmd "${MOTOR_ID}" 3 1.0
if wait_motor_state_equals "mode" "2" 3.0; then
  log "velocity mode confirmed by motor_states"
else
  log "velocity mode not confirmed within timeout; continue but watch roslaunch logs"
fi

log "[4/14] Enable velocity mode"
call_motor_cmd "${MOTOR_ID}" 0 0.0
if wait_motor_state_equals "enabled" "True" 3.0; then
  log "enable confirmed by motor_states"
else
  log "enable not confirmed within timeout; continue but watch roslaunch logs"
fi

log "[5/14] Resume velocity mode"
resume_after_mode_switch "velocity"

log "[6/14] Align velocity command to current feedback"
CURRENT_VEL_RAW="$(get_motor_state_field "velocity" || true)"
if [[ -n "${CURRENT_VEL_RAW}" ]]; then
  CURRENT_VEL="$(python3 - "${CURRENT_VEL_RAW}" "${VELOCITY_SCALE}" <<'PY'
import sys
raw = float(sys.argv[1])
scale = float(sys.argv[2])
print(raw * scale)
PY
)"
else
  CURRENT_VEL="0.0"
fi
hold_publish "${VEL_TOPIC}" "${CURRENT_VEL}" 0.5

log "[7/14] Velocity +${VEL_CMD} rad/s"
hold_publish "${VEL_TOPIC}" "${VEL_CMD}" "${VEL_HOLD_SEC}"

log "[8/14] Velocity -${VEL_CMD} rad/s"
NEG_VEL="-$(python3 - <<PY
v=float('${VEL_CMD}')
print(abs(v))
PY
)"
hold_publish "${VEL_TOPIC}" "${NEG_VEL}" "${VEL_HOLD_SEC}"

log "[9/14] Velocity zero"
publish_once "${VEL_TOPIC}" 0.0
sleep 0.5

log "[10/14] Stop and prepare lifecycle for position mode switch"
call_motor_cmd "${MOTOR_ID}" 2 0.0
prepare_mode_switch "position"
call_motor_cmd "${MOTOR_ID}" 1 0.0
sleep 0.5

log "[11/14] Set position mode"
call_motor_cmd "${MOTOR_ID}" 3 0.0
if wait_motor_state_equals "mode" "1" 3.0; then
  log "position mode confirmed by motor_states"
else
  log "position mode not confirmed within timeout; continue but watch roslaunch logs"
fi

log "[12/14] Enable position mode"
call_motor_cmd "${MOTOR_ID}" 0 0.0
if wait_motor_state_equals "enabled" "True" 3.0; then
  log "enable confirmed by motor_states"
else
  log "enable not confirmed within timeout; continue but watch roslaunch logs"
fi

log "[13/14] Resume position mode"
resume_after_mode_switch "position"

CURRENT_POS_RAW="$(get_motor_position_raw || true)"
if [[ -n "${CURRENT_POS_RAW}" ]]; then
  CURRENT_POS_RAD="$(to_rad_from_raw "${CURRENT_POS_RAW}")"
  POS_OFFSET="$(python3 - <<PY
p=float('${POS_CMD}')
print(abs(p))
PY
)"
  POS_TARGET_POS="$(add_rad "${CURRENT_POS_RAD}" "${POS_OFFSET}")"
  POS_TARGET_NEG="$(add_rad "${CURRENT_POS_RAD}" "-${POS_OFFSET}")"

  log "[14/14] Align position command to current feedback (target=${CURRENT_POS_RAD} rad)"
  hold_publish "${POS_TOPIC}" "${CURRENT_POS_RAD}" 0.5

  log "Position +${POS_OFFSET} rad relative (target=${POS_TARGET_POS} rad)"
  hold_publish "${POS_TOPIC}" "${POS_TARGET_POS}" "${POS_HOLD_SEC}"

  log "Position -${POS_OFFSET} rad relative (target=${POS_TARGET_NEG} rad)"
  hold_publish "${POS_TOPIC}" "${POS_TARGET_NEG}" "${POS_HOLD_SEC}"

  log "Position back to start (target=${CURRENT_POS_RAD} rad)"
  hold_publish "${POS_TOPIC}" "${CURRENT_POS_RAD}" "${POS_HOLD_SEC}"
else
  log "[14/14] Position alignment skipped: no current state"
  log "Position +${POS_CMD} rad absolute (fallback: no current state)"
  hold_publish "${POS_TOPIC}" "${POS_CMD}" "${POS_HOLD_SEC}"

  log "Position -${POS_CMD} rad absolute (fallback)"
  NEG_POS="-$(python3 - <<PY
p=float('${POS_CMD}')
print(abs(p))
PY
)"
  hold_publish "${POS_TOPIC}" "${NEG_POS}" "${POS_HOLD_SEC}"

  log "Position back to zero (fallback)"
  hold_publish "${POS_TOPIC}" 0.0 "${POS_HOLD_SEC}"
fi

show_last_state

log "Final stop"
# 脚本默认保留使能，方便你继续手工调试；
# 如需完成后自动失能，导出 ECB_TEST_SEND_DISABLE=1。
call_motor_cmd "${MOTOR_ID}" 2 0.0
if [[ "${ECB_TEST_SEND_DISABLE:-0}" == "1" ]]; then
  log "Disable"
  call_motor_cmd "${MOTOR_ID}" 1 0.0
fi

log "测试完成"
