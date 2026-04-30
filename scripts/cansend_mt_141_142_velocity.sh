#!/usr/bin/env bash
set -euo pipefail

CAN_DEV="can0"
LEFT_ID="141"
RIGHT_ID="142"
LEFT_VEL=""
RIGHT_VEL=""
RAW_MODE=0
REPEAT=1
INTERVAL_SEC="0.02"
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage:
  cansend_mt_141_142_velocity.sh [left_rad_s] [right_rad_s]
  cansend_mt_141_142_velocity.sh --raw [left_raw] [right_raw]

Examples:
  # Send +1.0 rad/s to 0x141 and -1.0 rad/s to 0x142 on can0.
  ./cansend_mt_141_142_velocity.sh 1.0 -1.0

  # Stop both motors.
  ./cansend_mt_141_142_velocity.sh 0 0

  # Send raw protocol values directly. Raw unit is 0.01 deg/s per LSB.
  ./cansend_mt_141_142_velocity.sh --raw 5729 -5729

Options:
  -d, --device IFACE       SocketCAN interface, default: can0
  --left-id HEX           Left motor CAN ID without 0x, default: 141
  --right-id HEX          Right motor CAN ID without 0x, default: 142
  --same VALUE            Use the same velocity for both motors
  --raw                   Treat velocity arguments as signed int32 raw values
  -n, --repeat COUNT      Send the pair COUNT times, default: 1
  -i, --interval SEC      Delay between repeated pairs, default: 0.02
  --dry-run               Print cansend commands without sending
  -h, --help              Show this help

Frame format:
  MT speed closed-loop command 0xA2, DATA[4..7] = int32 little-endian velocity.
  In normal mode, rad/s is converted using 0.01 deg/s per LSB.
EOF
}

die() {
  echo "[ERR] $*" >&2
  exit 2
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -d|--device)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        CAN_DEV="$2"
        shift 2
        ;;
      --left-id)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        LEFT_ID="${2#0x}"
        LEFT_ID="${LEFT_ID#0X}"
        shift 2
        ;;
      --right-id)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        RIGHT_ID="${2#0x}"
        RIGHT_ID="${RIGHT_ID#0X}"
        shift 2
        ;;
      --same)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        LEFT_VEL="$2"
        RIGHT_VEL="$2"
        shift 2
        ;;
      --raw)
        RAW_MODE=1
        shift
        ;;
      -n|--repeat)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        REPEAT="$2"
        shift 2
        ;;
      -i|--interval)
        [[ $# -ge 2 ]] || die "$1 requires a value"
        INTERVAL_SEC="$2"
        shift 2
        ;;
      --dry-run)
        DRY_RUN=1
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        break
        ;;
      -*|*)
        if [[ "$1" == -* && ! "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
          die "unknown option: $1"
        fi
        if [[ -z "${LEFT_VEL}" ]]; then
          LEFT_VEL="$1"
        elif [[ -z "${RIGHT_VEL}" ]]; then
          RIGHT_VEL="$1"
        else
          die "too many velocity arguments"
        fi
        shift
        ;;
    esac
  done
}

validate_number_args() {
  [[ -n "${LEFT_VEL}" ]] || die "missing left velocity"
  [[ -n "${RIGHT_VEL}" ]] || die "missing right velocity"
  [[ "${REPEAT}" =~ ^[0-9]+$ ]] || die "repeat must be a positive integer"
  (( REPEAT >= 1 )) || die "repeat must be >= 1"
}

raw_from_value() {
  local value="$1"
  if (( RAW_MODE )); then
    awk -v v="${value}" 'BEGIN {
      if (v !~ /^[-+]?[0-9]+$/) exit 1;
      if (v < -2147483648 || v > 2147483647) exit 1;
      printf "%d", v;
    }'
  else
    awk -v v="${value}" 'BEGIN {
      if (v !~ /^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$/) exit 1;
      raw = v * 18000.0 / atan2(0, -1);
      raw = (raw >= 0) ? int(raw + 0.5) : int(raw - 0.5);
      if (raw < -2147483648 || raw > 2147483647) exit 1;
      printf "%d", raw;
    }'
  fi
}

payload_from_raw() {
  local raw="$1"
  awk -v raw="${raw}" 'BEGIN {
    if (raw < 0) raw += 4294967296;
    printf "A2000000%02X%02X%02X%02X", \
      raw % 256, int(raw / 256) % 256, int(raw / 65536) % 256, int(raw / 16777216) % 256;
  }'
}

send_frame() {
  local can_id="$1"
  local payload="$2"
  local frame="${can_id}#${payload}"
  if (( DRY_RUN )); then
    printf 'cansend %q %q\n' "${CAN_DEV}" "${frame}"
  else
    cansend "${CAN_DEV}" "${frame}"
  fi
}

main() {
  parse_args "$@"
  validate_number_args

  if ! (( DRY_RUN )) && ! command -v cansend >/dev/null 2>&1; then
    die "missing command: cansend"
  fi

  local left_raw right_raw left_payload right_payload
  left_raw="$(raw_from_value "${LEFT_VEL}")" || die "invalid left velocity: ${LEFT_VEL}"
  right_raw="$(raw_from_value "${RIGHT_VEL}")" || die "invalid right velocity: ${RIGHT_VEL}"
  left_payload="$(payload_from_raw "${left_raw}")"
  right_payload="$(payload_from_raw "${right_raw}")"

  echo "[MT-CANSEND] ${CAN_DEV} 0x${LEFT_ID}: input=${LEFT_VEL} raw=${left_raw} payload=${left_payload}"
  echo "[MT-CANSEND] ${CAN_DEV} 0x${RIGHT_ID}: input=${RIGHT_VEL} raw=${right_raw} payload=${right_payload}"

  local i
  for ((i = 1; i <= REPEAT; ++i)); do
    send_frame "${LEFT_ID}" "${left_payload}"
    send_frame "${RIGHT_ID}" "${right_payload}"
    if (( i < REPEAT )); then
      sleep "${INTERVAL_SEC}"
    fi
  done
}

main "$@"
