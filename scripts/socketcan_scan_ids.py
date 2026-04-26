#!/usr/bin/env python3

import argparse
import os
import select
import socket
import struct
import sys
import time

import yaml


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)

PP_READ_COMMAND = 0x03
PP_READ_RESPONSE = 0x04
PP_READ_SERIAL_SUBCOMMAND = 0x02
PP_SUBCOMMANDS = (0x07, 0x06, 0x0F, 0x10, 0x15)

DM_FEEDBACK_FRAME_ID = 0x000
DM_SPEED_FRAME_BASE = 0x200

MT_SEND_FRAME_BASE = 0x140
MT_RESPONSE_FRAME_BASE = 0x240
MT_READ_STATE_COMMAND = 0x9C
DEFAULT_ARM_MT_MIN_ID = 1
DEFAULT_ARM_MT_MAX_ID = 31


def build_can_frame(can_id, payload):
    data = bytes(payload[:8])
    dlc = len(data)
    if dlc < 8:
        data = data + bytes(8 - dlc)
    return struct.pack(CAN_FRAME_FORMAT, can_id, dlc, data)


def parse_can_frame(frame_bytes):
    can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame_bytes)
    return can_id & 0x1FFFFFFF, dlc, data[:dlc]


def open_socketcan(device, timeout_sec):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 1)
    except OSError:
        pass
    sock.settimeout(timeout_sec)
    sock.bind((device,))
    return sock


def default_can_driver_config_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "can_driver.yaml"))


def send_pp_probe(sock, node_id):
    for subcommand in PP_SUBCOMMANDS:
        sock.send(build_can_frame(node_id, bytes((PP_READ_COMMAND, subcommand))))


def send_pp_serial_query(sock, node_id):
    sock.send(build_can_frame(node_id, bytes((PP_READ_COMMAND, PP_READ_SERIAL_SUBCOMMAND))))


def send_dm_probe(sock, node_id):
    payload = struct.pack("<f", 0.0)
    sock.send(build_can_frame(DM_SPEED_FRAME_BASE + node_id, payload))


def send_mt_probe(sock, node_id):
    sock.send(build_can_frame(MT_SEND_FRAME_BASE + node_id, bytes((MT_READ_STATE_COMMAND,))))


def debug_log_send(enabled, protocol, node_id, can_id, payload):
    if not enabled:
        return
    payload_text = " ".join(f"{byte:02X}" for byte in payload)
    print(
        f"[scan][tx] protocol={protocol} node_id={node_id} can_id=0x{can_id:03X} data={payload_text}"
    )


def collect_responses(sock, deadline, protocol, online):
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return

        readable, _, _ = select.select([sock], [], [], remaining)
        if not readable:
            return

        frame = sock.recv(CAN_FRAME_SIZE)
        can_id, dlc, data = parse_can_frame(frame)
        if protocol in ("pp", "both") and can_id < 0x100 and dlc >= 2:
            response_type = data[0]
            subcommand = data[1]
            if response_type == PP_READ_RESPONSE and subcommand == PP_READ_SERIAL_SUBCOMMAND and dlc >= 6:
                node_id = can_id & 0xFF
                serial_number = struct.unpack(">I", data[2:6])[0]
                online.setdefault(("pp", node_id), {}).setdefault("replies", set())
                online[("pp", node_id)]["serial"] = serial_number
            elif response_type in (PP_READ_RESPONSE, 0x02) and subcommand in PP_SUBCOMMANDS:
                node_id = can_id & 0xFF
                online.setdefault(("pp", node_id), {}).setdefault("replies", set()).add(subcommand)

        if protocol in ("dm", "both") and can_id == DM_FEEDBACK_FRAME_ID and dlc >= 1:
            node_id = data[0] & 0x0F
            state = (data[0] >> 4) & 0x0F
            online.setdefault(("dm", node_id), {}).setdefault("states", set()).add(state)

        if protocol in ("mt", "both") and MT_RESPONSE_FRAME_BASE <= can_id < MT_RESPONSE_FRAME_BASE + 0x100 and dlc >= 1:
            command = data[0]
            if command == MT_READ_STATE_COMMAND:
                node_id = can_id - MT_RESPONSE_FRAME_BASE
                entry = online.setdefault(("mt", node_id), {})
                entry.setdefault("replies", set()).add(command)
                if dlc >= 8:
                    entry["temperature_c"] = struct.unpack("<b", data[1:2])[0]
                    entry["current_a"] = struct.unpack("<h", data[2:4])[0] / 100.0
                    entry["velocity_dps"] = struct.unpack("<h", data[4:6])[0]
                    entry["encoder_raw"] = struct.unpack("<H", data[6:8])[0]


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Scan SocketCAN node IDs for PP, MT and DM motors on a single CAN bus."
    )
    parser.add_argument("--device", default="can1", help="SocketCAN device, default: can1")
    parser.add_argument(
        "--protocol",
        choices=("pp", "mt", "dm", "both", "arm-mt"),
        default="both",
        help="Which protocol probes to send",
    )
    parser.add_argument("--min-id", type=int, default=1, help="Minimum node ID to probe")
    parser.add_argument("--max-id", type=int, default=31, help="Maximum node ID to probe")
    parser.add_argument(
        "--settle-ms",
        type=int,
        default=25,
        help="Delay between node probes in milliseconds",
    )
    parser.add_argument(
        "--listen-ms",
        type=int,
        default=120,
        help="Listen window after each node probe in milliseconds",
    )
    parser.add_argument(
        "--read-pp-serial",
        action="store_true",
        help="After PP discovery, query serial number for each discovered PP node",
    )
    parser.add_argument(
        "--show-can-id-hex",
        action="store_true",
        help="Print full CAN motor ID in hex for MT devices (for example 0x143)",
    )
    parser.add_argument(
        "--debug-tx",
        action="store_true",
        help="Print each probe frame before send, useful when comparing with candump",
    )
    parser.add_argument(
        "--can-driver-config",
        default=default_can_driver_config_path(),
        help="can_driver joints config used to print configured arm MT IDs for comparison",
    )
    return parser.parse_args(argv)


def validate_args(args):
    if args.min_id < 0 or args.max_id > 255 or args.min_id > args.max_id:
        raise ValueError("Require 0 <= min-id <= max-id <= 255.")
    if args.settle_ms < 0 or args.listen_ms <= 0:
        raise ValueError("settle-ms must be >= 0 and listen-ms must be > 0.")


def normalize_protocol(protocol):
    if protocol == "arm-mt":
        return "mt"
    return protocol


def format_mt_identity(node_id, show_can_id_hex):
    motor_id = MT_SEND_FRAME_BASE + node_id
    if show_can_id_hex:
        return f"node_id={node_id} motor_id=0x{motor_id:03X}"
    return f"node_id={node_id}"


def load_configured_mt_ids(config_path, device):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        return []
    except yaml.YAMLError:
        return []

    joints = data.get("can_driver_node", {}).get("joints", [])
    if not isinstance(joints, list):
        return []

    configured = []
    for joint in joints:
        if not isinstance(joint, dict):
            continue
        if str(joint.get("protocol", "")).strip().upper() != "MT":
            continue
        if str(joint.get("can_device", "")).strip() != device:
            continue
        try:
            motor_id = int(joint.get("motor_id"), 0) if isinstance(joint.get("motor_id"), str) else int(joint.get("motor_id"))
        except (TypeError, ValueError):
            continue
        configured.append((str(joint.get("name", "")), motor_id))
    configured.sort(key=lambda item: item[1])
    return configured


def load_arm_mt_configured_ids(config_path, device):
    return load_configured_mt_ids(config_path, device)


def main(argv):
    args = parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"[scan] invalid arguments: {exc}", file=sys.stderr)
        return 2

    try:
        sock = open_socketcan(args.device, args.listen_ms / 1000.0)
    except OSError as exc:
        print(f"[scan] failed to open {args.device}: {exc}", file=sys.stderr)
        return 2

    online = {}
    protocol = normalize_protocol(args.protocol)
    settle_sec = args.settle_ms / 1000.0
    listen_sec = args.listen_ms / 1000.0

    min_id = args.min_id
    max_id = args.max_id
    if args.protocol == "arm-mt":
        min_id = DEFAULT_ARM_MT_MIN_ID
        max_id = DEFAULT_ARM_MT_MAX_ID

    configured_mt = []
    if args.protocol == "arm-mt":
        configured_mt = load_arm_mt_configured_ids(args.can_driver_config, args.device)
    elif protocol in ("mt", "both"):
        configured_mt = load_configured_mt_ids(args.can_driver_config, args.device)

    with sock:
        for node_id in range(min_id, max_id + 1):
            if protocol in ("pp", "both"):
                debug_log_send(args.debug_tx, "pp", node_id, node_id, bytes((PP_READ_COMMAND,)) )
                send_pp_probe(sock, node_id)
            if protocol in ("mt", "both"):
                debug_log_send(
                    args.debug_tx,
                    "mt",
                    node_id,
                    MT_SEND_FRAME_BASE + node_id,
                    bytes((MT_READ_STATE_COMMAND,)),
                )
                send_mt_probe(sock, node_id)
            if protocol in ("dm", "both"):
                debug_log_send(
                    args.debug_tx,
                    "dm",
                    node_id,
                    DM_SPEED_FRAME_BASE + node_id,
                    struct.pack("<f", 0.0),
                )
                send_dm_probe(sock, node_id)

            collect_responses(sock, time.monotonic() + listen_sec, protocol, online)

            if settle_sec > 0.0:
                time.sleep(settle_sec)

        if args.read_pp_serial and protocol in ("pp", "both"):
            pp_discovered = sorted(node_id for proto, node_id in online if proto == "pp")
            for node_id in pp_discovered:
                send_pp_serial_query(sock, node_id)
                collect_responses(sock, time.monotonic() + listen_sec, protocol, online)

    pp_hits = sorted(node_id for proto, node_id in online if proto == "pp")
    mt_hits = sorted(node_id for proto, node_id in online if proto == "mt")
    dm_hits = sorted(node_id for proto, node_id in online if proto == "dm")

    print(f"[scan] device={args.device} protocol={args.protocol} range={min_id}-{max_id}")
    if protocol in ("mt", "both"):
        if configured_mt:
            configured_text = ", ".join(
                f"{name}=0x{motor_id:03X}" for name, motor_id in configured_mt
            )
            label = "configured arm MT IDs" if args.protocol == "arm-mt" else "configured MT IDs on device"
            print(f"[scan] {label}: {configured_text}")
        else:
            label = "configured arm MT IDs" if args.protocol == "arm-mt" else "configured MT IDs on device"
            print(f"[scan] {label}: none")

    if protocol in ("pp", "both"):
        if pp_hits:
            print("[scan] PP online IDs:", ", ".join(str(node_id) for node_id in pp_hits))
            for node_id in pp_hits:
                entry = online[("pp", node_id)]
                replies = sorted(entry.get("replies", set()))
                reply_text = ", ".join(f"0x{sub:02X}" for sub in replies)
                serial_number = entry.get("serial")
                if serial_number is None:
                    print(f"  - pp id={node_id}: replies={reply_text}")
                else:
                    print(
                        f"  - pp id={node_id}: replies={reply_text} serial=0x{serial_number:08X} ({serial_number})"
                    )
        else:
            print("[scan] PP online IDs: none")

    if protocol in ("mt", "both"):
        if mt_hits:
            summary = ", ".join(
                f"0x{MT_SEND_FRAME_BASE + node_id:03X}" if args.show_can_id_hex else str(node_id)
                for node_id in mt_hits
            )
            print("[scan] MT online IDs:", summary)
            for node_id in mt_hits:
                entry = online[("mt", node_id)]
                replies = sorted(entry.get("replies", set()))
                reply_text = ", ".join(f"0x{command:02X}" for command in replies)
                details = []
                if "temperature_c" in entry:
                    details.append(f"temp={entry['temperature_c']}C")
                if "current_a" in entry:
                    details.append(f"current={entry['current_a']:.2f}A")
                if "velocity_dps" in entry:
                    details.append(f"velocity={entry['velocity_dps']}dps")
                if "encoder_raw" in entry:
                    details.append(f"encoder={entry['encoder_raw']}")
                suffix = f" {' '.join(details)}" if details else ""
                print(
                    f"  - mt {format_mt_identity(node_id, args.show_can_id_hex)}: "
                    f"replies={reply_text}{suffix}"
                )

            if configured_mt:
                configured_motor_ids = {motor_id for _, motor_id in configured_mt}
                online_motor_ids = {MT_SEND_FRAME_BASE + node_id for node_id in mt_hits}
                missing = sorted(configured_motor_ids - online_motor_ids)
                unexpected = sorted(online_motor_ids - configured_motor_ids)
                if missing:
                    print(
                        "[scan] configured but offline MT IDs: "
                        + ", ".join(f"0x{motor_id:03X}" for motor_id in missing)
                    )
                if unexpected:
                    print(
                        "[scan] online but not configured MT IDs: "
                        + ", ".join(f"0x{motor_id:03X}" for motor_id in unexpected)
                    )
        else:
            print("[scan] MT online IDs: none")
            if configured_mt:
                print(
                    "[scan] configured but offline MT IDs: "
                    + ", ".join(f"0x{motor_id:03X}" for _, motor_id in configured_mt)
                )

    if protocol in ("dm", "both"):
        if dm_hits:
            print("[scan] DM online IDs:", ", ".join(str(node_id) for node_id in dm_hits))
            for node_id in dm_hits:
                states = sorted(online[("dm", node_id)].get("states", set()))
                state_text = ", ".join(str(state) for state in states)
                print(f"  - dm id={node_id}: feedback_states={state_text}")
        else:
            print("[scan] DM online IDs: none")

    if not online:
        print("[scan] no replies observed; if tx exists but rx stays zero, check motor power, bitrate, termination, and node IDs.")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))