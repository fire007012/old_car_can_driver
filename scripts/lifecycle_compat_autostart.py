#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from can_driver.srv import Init, Recover


class LifecycleCompatAutostart:
    def __init__(self) -> None:
        self.driver_ns = rospy.get_param("~driver_ns", "/can_driver_node").rstrip("/")
        self.init_device = rospy.get_param("~init_device", "")
        self.init_loopback = bool(rospy.get_param("~init_loopback", False))
        self.auto_enable = bool(rospy.get_param("~auto_enable", True))
        self.auto_resume = bool(rospy.get_param("~auto_resume", True))
        self.auto_recover = bool(rospy.get_param("~auto_recover", False))
        self.startup_delay = max(0.0, float(rospy.get_param("~startup_delay", 1.0)))
        self.service_timeout = max(1.0, float(rospy.get_param("~service_timeout", 10.0)))
        self.retry_period = max(0.2, float(rospy.get_param("~retry_period", 1.0)))
        self.max_attempts = max(0, int(rospy.get_param("~max_attempts", 0)))

        self.lifecycle_state = ""
        rospy.Subscriber(f"{self.driver_ns}/lifecycle_state", String, self._on_lifecycle_state)

        self.init_srv = rospy.ServiceProxy(f"{self.driver_ns}/init", Init)
        self.enable_srv = rospy.ServiceProxy(f"{self.driver_ns}/enable", Trigger)
        self.resume_srv = rospy.ServiceProxy(f"{self.driver_ns}/resume", Trigger)
        self.recover_srv = rospy.ServiceProxy(f"{self.driver_ns}/recover", Recover)

    def _resolve_init_device(self) -> str:
        if self.init_device:
            return self.init_device

        joints_param = f"{self.driver_ns}/joints"
        joints = rospy.get_param(joints_param, None)
        if not isinstance(joints, list):
            return ""

        for joint in joints:
            if not isinstance(joint, dict):
                continue
            can_device = str(joint.get("can_device", "")).strip()
            if can_device:
                return can_device
        return ""

    def _on_lifecycle_state(self, msg: String) -> None:
        self.lifecycle_state = (msg.data or "").strip()

    def _wait_services(self) -> bool:
        services = [
            (f"{self.driver_ns}/init", self.init_srv),
            (f"{self.driver_ns}/enable", self.enable_srv),
            (f"{self.driver_ns}/resume", self.resume_srv),
            (f"{self.driver_ns}/recover", self.recover_srv),
        ]
        for service_name, proxy in services:
            try:
                proxy.wait_for_service(timeout=self.service_timeout)
            except rospy.ROSException:
                rospy.logerr("compat autostart: wait service timeout: %s", service_name)
                return False
        return True

    def _call_init(self) -> bool:
        response = self.init_srv(device=self.init_device, loopback=self.init_loopback)
        if not response.success:
            rospy.logwarn("compat autostart: init failed: %s", response.message)
            return False
        rospy.loginfo("compat autostart: init ok: %s", response.message)
        return True

    def _call_trigger(self, name: str, service: rospy.ServiceProxy) -> bool:
        response = service()
        if not response.success:
            rospy.logwarn("compat autostart: %s failed: %s", name, response.message)
            return False
        rospy.loginfo("compat autostart: %s ok: %s", name, response.message)
        return True

    def _call_recover(self) -> bool:
        response = self.recover_srv(motor_id=0xFFFF)
        if not response.success:
            rospy.logwarn("compat autostart: recover failed: %s", response.message)
            return False
        rospy.loginfo("compat autostart: recover ok: %s", response.message)
        return True

    def run(self) -> None:
        time.sleep(self.startup_delay)
        if not self._wait_services():
            return

        self.init_device = self._resolve_init_device()
        attempts = 0

        rate = rospy.Rate(1.0 / self.retry_period)
        while not rospy.is_shutdown():
            if self.max_attempts > 0 and attempts >= self.max_attempts:
                rospy.logerr(
                    "compat autostart: reached max_attempts=%d, stop retrying",
                    self.max_attempts,
                )
                return

            state = self.lifecycle_state
            if state == "Running":
                rospy.loginfo_throttle(30.0, "compat autostart: lifecycle already Running")
                return

            ok = True
            if state in ("", "Inactive", "Configured"):
                if not self.init_device:
                    rospy.logwarn_throttle(5.0, "compat autostart: waiting for ~init_device")
                    ok = False
                else:
                    attempts += 1
                    ok = self._call_init()
                    state = "Armed" if ok else state
            elif state == "Faulted":
                attempts += 1
                ok = self._call_recover() if self.auto_recover else False
                state = "Standby" if ok else state

            if ok and self.auto_enable and state == "Standby":
                ok = self._call_trigger("enable", self.enable_srv)
                state = "Armed" if ok else state

            if ok and self.auto_resume and state == "Armed":
                ok = self._call_trigger("resume", self.resume_srv)

            if ok:
                time.sleep(0.3)
                if self.lifecycle_state == "Running" or (
                    not self.auto_resume and self.lifecycle_state == "Armed"
                ):
                    rospy.loginfo("compat autostart: reached lifecycle state %s", self.lifecycle_state)
                    return

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lifecycle_compat_autostart", anonymous=False)
    LifecycleCompatAutostart().run()
