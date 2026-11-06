#!/usr/bin/env python3
"""Read-only Nav2 lifecycle/odom/health report for ROS 2 Jazzy.

Does not configure or activate lifecycle nodes, command motion, or bypass health.
Use from a freshly sourced ROS_DOMAIN_ID=188 shell while simulation is running.
"""
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

NAV_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
)
DASHBOARD_REQUIRED = (
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
)


class Nav2Report(Node):
    def __init__(self):
        super().__init__("niihan_nav2_readonly_diagnostic")
        self.state_clients = {
            name: self.create_client(GetState, "/" + name + "/get_state")
            for name in NAV_NODES
        }
        self.samples = {}
        self.create_subscription(
            Odometry, "/glim_ros/odom",
            lambda msg: self.samples.setdefault("glim_odom", msg),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odom",
            lambda msg: self.samples.setdefault("odom", msg),
            qos_profile_sensor_data)
        self.create_subscription(
            String, "/niihan/health",
            lambda msg: self.samples.__setitem__("health", msg.data),
            qos_profile_sensor_data)
        self.create_subscription(
            String, "/niihan/slam/status",
            lambda msg: self.samples.__setitem__("slam", msg.data),
            qos_profile_sensor_data)
        self.nav_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def read_states(self):
        results = {}
        futures = {}
        for name, client in self.state_clients.items():
            if client.service_is_ready():
                futures[name] = client.call_async(GetState.Request())
            else:
                results[name] = "SERVICE MISSING"
        deadline = time.monotonic() + 7.0
        while time.monotonic() < deadline:
            if all(f.done() for f in futures.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.15)
        for name, future in futures.items():
            if not future.done():
                results[name] = "GET_STATE TIMEOUT"
                continue
            try:
                state = future.result().current_state
                results[name] = f"{state.label or 'UNKNOWN'} [{state.id}]"
            except Exception as exc:
                results[name] = f"GET_STATE ERROR: {exc}"
        return results

    def collect_topics(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if all(key in self.samples for key in ("glim_odom", "odom", "health", "slam")):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        for topic, key in (
            ("/glim_ros/odom", "glim_odom"),
            ("/odom", "odom"),
        ):
            count = self.count_publishers(topic)
            sample = self.samples.get(key)
            if sample is None:
                print(f"{topic}: publishers={count}; NO SAMPLE (3 seconds)")
            else:
                stamp = sample.header.stamp
                print(
                    f"{topic}: publishers={count}; sample stamp="
                    f"{stamp.sec}.{stamp.nanosec:09d}, frame={sample.header.frame_id}")
        for name in ("health", "slam"):
            value = self.samples.get(name)
            if value is None:
                print(f"/niihan/{name if name == 'health' else 'slam/status'}: NO SAMPLE")
            else:
                try:
                    print(f"{name}: " + json.dumps(json.loads(value), sort_keys=True))
                except ValueError:
                    print(f"{name}: {value}")


def main():
    rclpy.init()
    node = Nav2Report()
    try:
        print("==== NAV2 LIFECYCLE STATES (READ-ONLY) ====", flush=True)
        states = node.read_states()
        for name in NAV_NODES:
            print(f"{name}: {states[name]}")
        action = node.nav_action.server_is_ready()
        print(f"/navigate_to_pose action server ready: {action}")
        print("==== SENSOR / SLAM DATA ====", flush=True)
        node.collect_topics()
        ready = action and all(states[name] == "active [3]" for name in DASHBOARD_REQUIRED)
        print(f"Dashboard Nav2 readiness equivalent: {ready}", flush=True)
        if not ready:
            print("Nav2 is NOT READY: inspect launch terminal for the first configure/activate failure.")
            print("Do not manually activate everything or bypass motion permission.")
        return 0 if ready else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
