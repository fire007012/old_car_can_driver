#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time
from typing import Any, Dict, List, Optional

import rospkg
import rospy
import yaml
from std_msgs.msg import Float64

from can_driver.srv import MotorCommand, MotorCommandRequest


def parse_motor_id(raw: str) -> int:
    return int(raw, 0)


def parse_joint_motor_id(raw: Any) -> Optional[int]:
    if isinstance(raw, int):
        return raw
    if isinstance(raw, str):
        try:
            return int(raw, 0)
        except ValueError:
            return None
    return None


def normalize_protocol(value: Any) -> str:
    if not isinstance(value, str):
        return ""
    return value.strip().upper()


def default_profile_path() -> str:
    pkg = rospkg.RosPack().get_path("can_driver")
    return os.path.join(pkg, "config", "mt_control_profiles.yaml")


def load_profiles(path: str) -> Dict[str, Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    profiles = data.get("profiles", {})
    if not isinstance(profiles, dict):
        raise RuntimeError("profiles 节点不存在或格式错误")
    return profiles


class MtMotorInterface:
    def __init__(self, profile_name: str, profile: Dict[str, Any], driver_ns_override: str = ""):
        self.profile_name = profile_name
        self.profile = profile
        ns = driver_ns_override.strip() or str(profile.get("driver_ns", "/can_driver_node"))
        self.driver_ns = ns.rstrip("/") if ns != "/" else ""

        self.motor_cmd_srv = f"{self.driver_ns}/motor_command"
        self.joints_param = f"{self.driver_ns}/joints"

    def _load_joints(self):
        if not rospy.has_param(self.joints_param):
            raise RuntimeError(f"参数不存在: {self.joints_param}")
        joints = rospy.get_param(self.joints_param)
        if not isinstance(joints, list):
            raise RuntimeError(f"参数格式错误(非列表): {self.joints_param}")
        return joints

    def discover_mt_joints(self) -> List[Dict[str, Any]]:
        can_device_expect = str(self.profile.get("can_device", ""))
        joints = self._load_joints()
        discovered: List[Dict[str, Any]] = []
        for j in joints:
            if not isinstance(j, dict):
                continue
            if normalize_protocol(j.get("protocol", "")) != "MT":
                continue
            jid = parse_joint_motor_id(j.get("motor_id"))
            if jid is None:
                continue
            can_device = str(j.get("can_device", ""))
            discovered.append(
                {
                    "motor_id": jid,
                    "name": str(j.get("name", "")),
                    "can_device": can_device,
                    "device_match": (not can_device_expect) or (can_device == can_device_expect),
                }
            )
        discovered.sort(key=lambda item: int(item["motor_id"]))
        return discovered

    def resolve_joint_name(self, motor_id: int, joint_name_override: str = "") -> str:
        if joint_name_override:
            return joint_name_override

        can_device_expect = str(self.profile.get("can_device", ""))
        discovered = self.discover_mt_joints()

        # 优先：ID + can_device 精确匹配
        for item in discovered:
            if item["motor_id"] == motor_id and item["device_match"]:
                name = item["name"]
                if name:
                    return name

        # 兼容：若 profile 的 can_device 已变化，允许按 ID 退化匹配。
        fallback = [item for item in discovered if item["motor_id"] == motor_id and item["name"]]
        if len(fallback) == 1:
            selected = fallback[0]
            rospy.logwarn(
                "[MT-IF] profile can_device=%s 与运行时关节 can_device=%s 不一致，"
                "按 motor_id=0x%X 退化匹配 joint=%s。",
                can_device_expect,
                selected["can_device"],
                motor_id,
                selected["name"],
            )
            return selected["name"]

        if len(fallback) > 1:
            raise RuntimeError(
                f"motor_id={hex(motor_id)} 在 {self.joints_param} 中存在多个 MT 关节同名候选，"
                "请使用 --joint 指定。"
            )

        raise RuntimeError(
            f"无法在 {self.joints_param} 中找到 MT 关节: motor_id={hex(motor_id)} "
            f"(profile={self.profile_name}, can_device={can_device_expect})"
        )

    def call_motor_command(self, motor_id: int, command: int, value: float = 0.0) -> None:
        rospy.wait_for_service(self.motor_cmd_srv, timeout=3.0)
        proxy = rospy.ServiceProxy(self.motor_cmd_srv, MotorCommand)

        req = MotorCommandRequest()
        req.motor_id = motor_id
        req.command = command
        req.value = value

        resp = proxy(req)
        if not resp.success:
            raise RuntimeError(f"service 失败: {resp.message}")

    def publish_direct(self, joint_name: str, is_velocity: bool, value: float) -> None:
        suffix = "cmd_velocity" if is_velocity else "cmd_position"
        topic = f"{self.driver_ns}/motor/{joint_name}/{suffix}"

        pub = rospy.Publisher(topic, Float64, queue_size=1)
        start = time.time()
        while pub.get_num_connections() == 0 and (time.time() - start) < 1.0 and not rospy.is_shutdown():
            rospy.sleep(0.05)

        msg = Float64()
        msg.data = value
        for _ in range(3):
            pub.publish(msg)
            rospy.sleep(0.03)

    def check_motor_in_profile(self, motor_id: int) -> None:
        ids = self.profile.get("mt_motor_ids", [])
        if not isinstance(ids, list) or not ids:
            return
        normalized = set()
        for x in ids:
            try:
                normalized.add(int(x, 0) if isinstance(x, str) else int(x))
            except (ValueError, TypeError):
                continue
        if normalized and motor_id not in normalized:
            rospy.logwarn(
                "[MT-IF] motor_id=0x%X 不在 profile '%s' 的 mt_motor_ids 中: %s，"
                "继续执行（兼容适配后的现场配置）。",
                motor_id,
                self.profile_name,
                [hex(i) for i in sorted(normalized)],
            )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MT 电机控制接口（含 MIT 位置模式）")
    p.add_argument("--profile", required=True, help="车配置名，如 car_a/car_b")
    p.add_argument(
        "--action",
        required=True,
        choices=["enable", "disable", "stop", "mode", "velocity", "position", "list", "discover"],
        help="控制动作",
    )
    p.add_argument("--motor-id", default="0x141", help="目标电机ID，支持十进制/十六进制")
    p.add_argument("--value", type=float, default=0.0, help="动作参数：mode用0/1，速度/位置用浮点")
    p.add_argument("--joint", default="", help="可选：直接指定 joint 名，跳过自动匹配")
    p.add_argument("--profiles-file", default=default_profile_path(), help="profile 配置文件路径")
    p.add_argument("--driver-ns", default="", help="可选：覆盖 profile 内 driver_ns")
    p.add_argument(
        "--no-auto-mode",
        action="store_true",
        help="禁用 velocity/position 动作中的自动模式切换（默认会自动切换）",
    )
    return p


def print_profiles(profiles: Dict[str, Dict[str, Any]]) -> None:
    print("可用 MT profiles:")
    for name, cfg in profiles.items():
        desc = cfg.get("description", "")
        dev = cfg.get("can_device", "")
        ids = cfg.get("mt_motor_ids", [])
        print(f"- {name}: {desc}")
        print(f"    can_device: {dev}")
        print(f"    mt_motor_ids: {ids}")


def main() -> int:
    args = build_parser().parse_args()

    try:
        profiles = load_profiles(args.profiles_file)
    except Exception as e:
        print(f"[MT-IF] 加载 profiles 失败: {e}", file=sys.stderr)
        return 2

    if args.action == "list":
        print_profiles(profiles)
        return 0

    if args.profile not in profiles:
        print(f"[MT-IF] 未找到 profile: {args.profile}", file=sys.stderr)
        print_profiles(profiles)
        return 2

    try:
        motor_id = parse_motor_id(args.motor_id)
    except ValueError:
        print(f"[MT-IF] motor-id 无法解析: {args.motor_id}", file=sys.stderr)
        return 2

    rospy.init_node("mt_motor_interface_cli", anonymous=True)

    iface = MtMotorInterface(args.profile, profiles[args.profile], args.driver_ns)

    try:
        if args.action == "discover":
            joints = iface.discover_mt_joints()
            if not joints:
                print("[MT-IF] 未发现 MT joints")
                return 0
            for item in joints:
                marker = "*" if item["device_match"] else " "
                print(
                    f"{marker} motor_id=0x{int(item['motor_id']):X} "
                    f"joint={item['name']} can_device={item['can_device']}"
                )
            return 0

        iface.check_motor_in_profile(motor_id)

        if args.action == "enable":
            iface.call_motor_command(motor_id, MotorCommandRequest.CMD_ENABLE, 0.0)
            print("OK enable")
            return 0

        if args.action == "disable":
            iface.call_motor_command(motor_id, MotorCommandRequest.CMD_DISABLE, 0.0)
            print("OK disable")
            return 0

        if args.action == "stop":
            iface.call_motor_command(motor_id, MotorCommandRequest.CMD_STOP, 0.0)
            print("OK stop")
            return 0

        if args.action == "mode":
            if args.value not in (0.0, 1.0):
                raise RuntimeError("mode 动作要求 --value 只能是 0(position/MIT) 或 1(velocity)")
            iface.call_motor_command(motor_id, MotorCommandRequest.CMD_SET_MODE, args.value)
            print("OK mode")
            return 0

        if args.action in ("velocity", "position"):
            joint = iface.resolve_joint_name(motor_id, args.joint)
            if not args.no_auto_mode:
                # 明确设置模式：velocity=1，position(MIT)=0
                mode_value = 1.0 if args.action == "velocity" else 0.0
                iface.call_motor_command(motor_id, MotorCommandRequest.CMD_SET_MODE, mode_value)
            iface.publish_direct(joint, args.action == "velocity", args.value)
            print(f"OK {args.action} joint={joint} value={args.value}")
            return 0

        raise RuntimeError(f"不支持动作: {args.action}")
    except Exception as e:
        print(f"[MT-IF] 执行失败: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
