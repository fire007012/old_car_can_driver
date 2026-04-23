#!/usr/bin/env python3

import argparse
import select
import socket
import struct
import sys
import time


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
    sock.settimeout(timeout_sec)
    sock.bind((device,))
    return sock


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
        choices=("pp", "mt", "dm", "both"),
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
    return parser.parse_args(argv)


def validate_args(args):
    if args.min_id < 0 or args.max_id > 255 or args.min_id > args.max_id:
        raise ValueError("Require 0 <= min-id <= max-id <= 255.")
    if args.settle_ms < 0 or args.listen_ms <= 0:
        raise ValueError("settle-ms must be >= 0 and listen-ms must be > 0.")


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
    settle_sec = args.settle_ms / 1000.0
    listen_sec = args.listen_ms / 1000.0

    with sock:
        for node_id in range(args.min_id, args.max_id + 1):
            if args.protocol in ("pp", "both"):
                send_pp_probe(sock, node_id)
            if args.protocol in ("mt", "both"):
                send_mt_probe(sock, node_id)
            if args.protocol in ("dm", "both"):
                send_dm_probe(sock, node_id)

            collect_responses(sock, time.monotonic() + listen_sec, args.protocol, online)

            if settle_sec > 0.0:
                time.sleep(settle_sec)

        if args.read_pp_serial and args.protocol in ("pp", "both"):
            pp_discovered = sorted(node_id for proto, node_id in online if proto == "pp")
            for node_id in pp_discovered:
                send_pp_serial_query(sock, node_id)
                collect_responses(sock, time.monotonic() + listen_sec, args.protocol, online)

    pp_hits = sorted(node_id for proto, node_id in online if proto == "pp")
    mt_hits = sorted(node_id for proto, node_id in online if proto == "mt")
    dm_hits = sorted(node_id for proto, node_id in online if proto == "dm")

    print(f"[scan] device={args.device} protocol={args.protocol} range={args.min_id}-{args.max_id}")
    if args.protocol in ("pp", "both"):
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

    if args.protocol in ("mt", "both"):
        if mt_hits:
            print("[scan] MT online IDs:", ", ".join(str(node_id) for node_id in mt_hits))
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
                print(f"  - mt id={node_id}: replies={reply_text}{suffix}")
        else:
            print("[scan] MT online IDs: none")

    if args.protocol in ("dm", "both"):
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