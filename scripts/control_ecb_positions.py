#!/usr/bin/env python3
"""Command ECB swing-arm motors in position mode.

Usage examples:
  scripts/control_ecb_positions.py 0.2 0.2 0.2 0.2
  scripts/control_ecb_positions.py --relative 0.1 -0.1 0.1 -0.1 --hold 3
  scripts/control_ecb_positions.py --ids 2,3 --relative 0.2 0.2
"""

import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import rospy
import yaml
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from can_driver.msg import MotorState
from can_driver.srv import MotorCommand, MotorCommandRequest, Recover


@dataclass
class EcbJoint:
    motor_id: int
    name: str
    can_device: str
    position_scale: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Set ECB motors to position mode and publish per-motor position targets in rad."
    )
    parser.add_argument(
        "positions",
        nargs="+",
        type=float,
        help="Target positions in rad. With --relative, offsets from current feedback.",
    )
    parser.add_argument(
        "--ids",
        default="2,3,4,5",
        help="Comma-separated ECB motor IDs, in the same order as positions. Default: 2,3,4,5",
    )
    parser.add_argument(
        "--driver-ns",
        default="/can_driver_node",
        help="Driver namespace. Default: /can_driver_node",
    )
    parser.add_argument(
        "--relative",
        action="store_true",
        help="Treat positions as relative offsets from current motor feedback.",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=2.0,
        help="Seconds to keep streaming targets. Default: 2.0",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=20.0,
        help="Publish rate in Hz. Default: 20",
    )
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="Do not send ENABLE after switching position mode.",
    )
    parser.add_argument(
        "--skip-lifecycle",
        action="store_true",
        help="Do not halt/resume the can_driver lifecycle around mode switching.",
    )
    parser.add_argument(
        "--verify-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for final feedback before printing target error. Default: 2.0",
    )
    return parser.parse_args()


def normalize_ns(driver_ns: str) -> str:
    value = driver_ns.strip()
    if not value:
        return ""
    return "/" + value.strip("/")


def parse_motor_ids(ids_text: str) -> List[int]:
    ids = [int(part.strip(), 0) for part in ids_text.split(",") if part.strip()]
    if not ids:
        raise ValueError("--ids must contain at least one motor id")
    return ids


def load_joints(driver_ns: str) -> List[EcbJoint]:
    candidates = []
    primary = f"{driver_ns}/joints" if driver_ns else "/joints"
    for param_name in [primary, "/can_driver_node/joints", "/joints"]:
        if param_name not in candidates:
            candidates.append(param_name)

    loaded = None
    loaded_param = ""
    for param_name in candidates:
        try:
            raw = subprocess.check_output(["rosparam", "get", param_name], text=True)
        except subprocess.CalledProcessError:
            continue
        try:
            value = yaml.safe_load(raw)
        except yaml.YAMLError:
            continue
        if isinstance(value, list):
            loaded = value
            loaded_param = param_name
            break

    if loaded is None:
        raise RuntimeError("joints parameter not found; tried: " + ", ".join(candidates))

    joints: List[EcbJoint] = []
    for item in loaded:
        if not isinstance(item, dict):
            continue
        if str(item.get("protocol", "")).upper() != "ECB":
            continue
        raw_motor_id = item.get("motor_id")
        if isinstance(raw_motor_id, str):
            motor_id = int(raw_motor_id, 0)
        elif isinstance(raw_motor_id, int):
            motor_id = raw_motor_id
        else:
            continue
        joints.append(
            EcbJoint(
                motor_id=motor_id,
                name=str(item.get("name", "")),
                can_device=str(item.get("can_device", "")),
                position_scale=float(item.get("position_scale", 1.0)),
            )
        )

    if not joints:
        raise RuntimeError(f"no ECB joints found in {loaded_param}")
    return joints


def wait_current_positions(topic: str, targets: Iterable[EcbJoint], timeout_sec: float = 5.0) -> Dict[int, float]:
    wanted = {joint.motor_id: joint for joint in targets}
    positions: Dict[int, float] = {}

    def callback(msg: MotorState) -> None:
        if msg.motor_id in wanted and msg.position_valid:
            joint = wanted[msg.motor_id]
            positions[msg.motor_id] = float(msg.position) * joint.position_scale

    subscriber = rospy.Subscriber(topic, MotorState, callback, queue_size=20)
    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline and not rospy.is_shutdown():
            if all(motor_id in positions for motor_id in wanted):
                return positions
            rospy.sleep(0.05)
    finally:
        subscriber.unregister()

    missing = sorted(set(wanted) - set(positions))
    raise RuntimeError("missing fresh position feedback for motor_id=" + ",".join(map(str, missing)))


def sample_positions(topic: str, targets: Iterable[EcbJoint], timeout_sec: float) -> Dict[int, float]:
    wanted = {joint.motor_id: joint for joint in targets}
    positions: Dict[int, float] = {}

    def callback(msg: MotorState) -> None:
        if msg.motor_id in wanted and msg.position_valid:
            joint = wanted[msg.motor_id]
            positions[msg.motor_id] = float(msg.position) * joint.position_scale

    subscriber = rospy.Subscriber(topic, MotorState, callback, queue_size=20)
    deadline = time.time() + max(0.0, timeout_sec)
    try:
        while time.time() < deadline and not rospy.is_shutdown():
            if all(motor_id in positions for motor_id in wanted):
                break
            rospy.sleep(0.05)
    finally:
        subscriber.unregister()
    return positions


def call_motor_command(proxy: rospy.ServiceProxy, motor_id: int, command: int, value: float) -> None:
    req = MotorCommandRequest()
    req.motor_id = motor_id
    req.command = command
    req.value = value
    resp = proxy(req)
    if not resp.success:
        raise RuntimeError(f"motor_id={motor_id} command={command} rejected: {resp.message}")


class LifecycleClient:
    def __init__(self, driver_ns: str) -> None:
        self.driver_ns = driver_ns
        self.state = ""
        state_topic = f"{driver_ns}/lifecycle_state" if driver_ns else "/lifecycle_state"
        halt_srv = f"{driver_ns}/halt" if driver_ns else "/halt"
        enable_srv = f"{driver_ns}/enable" if driver_ns else "/enable"
        resume_srv = f"{driver_ns}/resume" if driver_ns else "/resume"
        recover_srv = f"{driver_ns}/recover" if driver_ns else "/recover"

        rospy.Subscriber(state_topic, String, self._on_state, queue_size=1)
        self.halt = rospy.ServiceProxy(halt_srv, Trigger)
        self.enable = rospy.ServiceProxy(enable_srv, Trigger)
        self.resume = rospy.ServiceProxy(resume_srv, Trigger)
        self.recover = rospy.ServiceProxy(recover_srv, Recover)
        for service_name, proxy in [
            (halt_srv, self.halt),
            (enable_srv, self.enable),
            (resume_srv, self.resume),
            (recover_srv, self.recover),
        ]:
            proxy.wait_for_service(timeout=5.0)
            print(f"[ECB-POS] lifecycle service ready: {service_name}")

    def _on_state(self, msg: String) -> None:
        self.state = (msg.data or "").strip()

    def wait_state(self, expected: str, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.state == expected:
                return True
            rospy.sleep(0.05)
        return self.state == expected

    def call_trigger(self, name: str, proxy: rospy.ServiceProxy) -> None:
        resp = proxy()
        if not resp.success:
            raise RuntimeError(f"lifecycle {name} rejected: {resp.message}")
        print(f"[ECB-POS] lifecycle {name}: {resp.message}")

    def recover_if_faulted(self) -> None:
        self.wait_state("Faulted", 0.5)
        if self.state != "Faulted":
            return
        resp = self.recover(motor_id=0xFFFF)
        if not resp.success:
            raise RuntimeError(f"lifecycle recover rejected: {resp.message}")
        print(f"[ECB-POS] lifecycle recover: {resp.message}")
        self.wait_state("Standby", 5.0)

    def prepare_mode_switch(self) -> None:
        self.recover_if_faulted()
        if self.state == "Armed":
            print("[ECB-POS] lifecycle already Armed")
            return
        print(f"[ECB-POS] lifecycle halt before mode switch, state={self.state or 'unknown'}")
        self.call_trigger("halt", self.halt)
        if not self.wait_state("Armed", 5.0):
            print(f"[ECB-POS][WARN] lifecycle not confirmed Armed, state={self.state or 'unknown'}")

    def resume_after_mode_switch(self) -> None:
        if self.state == "Faulted":
            self.recover_if_faulted()
        if self.state == "Standby":
            self.call_trigger("enable", self.enable)
            self.wait_state("Armed", 5.0)
        if self.state == "Running":
            print("[ECB-POS] lifecycle already Running")
            return
        print(f"[ECB-POS] lifecycle resume after mode switch, state={self.state or 'unknown'}")
        self.call_trigger("resume", self.resume)
        if not self.wait_state("Running", 5.0):
            print(f"[ECB-POS][WARN] lifecycle not confirmed Running, state={self.state or 'unknown'}")


def main() -> int:
    args = parse_args()
    driver_ns = normalize_ns(args.driver_ns)
    motor_ids = parse_motor_ids(args.ids)
    if len(args.positions) != len(motor_ids):
        print("positions count must match --ids count", file=sys.stderr)
        return 2

    rospy.init_node("ecb_position_commander", anonymous=True)
    motor_srv = f"{driver_ns}/motor_command" if driver_ns else "/motor_command"
    state_topic = f"{driver_ns}/motor_states" if driver_ns else "/motor_states"

    joints_by_id = {joint.motor_id: joint for joint in load_joints(driver_ns)}
    try:
        selected = [joints_by_id[motor_id] for motor_id in motor_ids]
    except KeyError as exc:
        print(f"motor_id={exc.args[0]} is not an ECB joint in the active config", file=sys.stderr)
        return 2

    rospy.wait_for_service(motor_srv, timeout=5.0)
    command_proxy = rospy.ServiceProxy(motor_srv, MotorCommand)
    lifecycle = None if args.skip_lifecycle else LifecycleClient(driver_ns)

    current_positions: Optional[Dict[int, float]] = None
    if args.relative:
        current_positions = wait_current_positions(state_topic, selected)

    targets: Dict[int, float] = {}
    for joint, requested in zip(selected, args.positions):
        base = current_positions[joint.motor_id] if current_positions is not None else 0.0
        targets[joint.motor_id] = base + requested

    print(f"[ECB-POS] driver_ns={driver_ns or '/'}")
    for joint in selected:
        print(
            f"[ECB-POS] motor_id={joint.motor_id} joint={joint.name} "
            f"target={targets[joint.motor_id]:.6f} rad"
        )

    if lifecycle is not None:
        lifecycle.prepare_mode_switch()

    for joint in selected:
        call_motor_command(command_proxy, joint.motor_id, MotorCommandRequest.CMD_DISABLE, 0.0)
        call_motor_command(command_proxy, joint.motor_id, MotorCommandRequest.CMD_SET_MODE, 0.0)
        if not args.no_enable:
            call_motor_command(command_proxy, joint.motor_id, MotorCommandRequest.CMD_ENABLE, 0.0)

    if lifecycle is not None:
        lifecycle.resume_after_mode_switch()

    publishers = []
    messages = []
    for joint in selected:
        topic = f"{driver_ns}/motor/{joint.name}/cmd_position" if driver_ns else f"/motor/{joint.name}/cmd_position"
        pub = rospy.Publisher(topic, Float64, queue_size=1)
        msg = Float64(data=targets[joint.motor_id])
        publishers.append(pub)
        messages.append(msg)
        print(f"[ECB-POS] publish {topic}")

    start_wait = time.time()
    while time.time() - start_wait < 1.0 and not rospy.is_shutdown():
        if all(pub.get_num_connections() > 0 for pub in publishers):
            break
        rospy.sleep(0.05)

    rate = rospy.Rate(max(1.0, args.hz))
    deadline = time.time() + max(0.0, args.hold)
    while time.time() < deadline and not rospy.is_shutdown():
        for pub, msg in zip(publishers, messages):
            pub.publish(msg)
        rate.sleep()

    for pub, msg in zip(publishers, messages):
        pub.publish(msg)

    final_positions = sample_positions(state_topic, selected, args.verify_timeout)
    if final_positions:
        print("[ECB-POS] final feedback:")
        for joint in selected:
            actual = final_positions.get(joint.motor_id)
            if actual is None:
                print(f"[ECB-POS] motor_id={joint.motor_id} joint={joint.name} actual=missing")
                continue
            target = targets[joint.motor_id]
            error = actual - target
            print(
                f"[ECB-POS] motor_id={joint.motor_id} joint={joint.name} "
                f"target={target:.6f} actual={actual:.6f} error={error:+.6f} rad"
            )

    print("[ECB-POS] done")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (rospy.ROSException, rospy.ROSInterruptException, RuntimeError, ValueError) as exc:
        print(f"[ECB-POS][ERR] {exc}", file=sys.stderr)
        raise SystemExit(1)
