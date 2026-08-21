#!/usr/bin/env python3
"""Minimal ROS 2 FMS for the local integration smoke test.

Run with the normal ROS 2 Python environment after the Isaac Sim world is ready.
It reads `cuopt_route.json`, sends one goal at a time, observes AMR pose/status,
and finishes when every route point has been visited.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_ROUTE_PATH = BASE_DIR / "cuopt_route.json"
ARRIVAL_TOLERANCE_M = 0.20
GOAL_REPUBLISH_SEC = 1.0


class SimpleFMS(Node):
    def __init__(self, route_file: Path) -> None:
        super().__init__("simple_fms")

        data = json.loads(route_file.read_text(encoding="utf-8"))
        self.route = [int(v) for v in data["route"]]
        self.locations = {int(k): v for k, v in data["locations"].items()}

        if len(self.route) < 2:
            raise ValueError("Route must contain a depot and at least one target.")

        self.goal_pub = self.create_publisher(Point, "/fms/goal", 10)
        self.pose_sub = self.create_subscription(
            Point, "/amr/pose", self._on_pose, 10
        )
        self.status_sub = self.create_subscription(
            String, "/amr/status", self._on_status, 10
        )

        self.latest_pose: tuple[float, float] | None = None
        self.latest_status = "UNKNOWN"
        self.route_index = 1  # route[0] is the starting depot
        self.current_target_id: int | None = None
        self.last_goal_publish = 0.0
        self.done = False

        self.timer = self.create_timer(0.10, self._control_loop)

        self.get_logger().info(f"Solver: {data.get('solver', 'unknown')}")
        self.get_logger().info(f"Route : {self.route}")
        self.get_logger().info("Waiting for /amr/pose from Isaac Sim ...")

    def _on_pose(self, msg: Point) -> None:
        self.latest_pose = (float(msg.x), float(msg.y))

    def _on_status(self, msg: String) -> None:
        if msg.data != self.latest_status:
            self.latest_status = msg.data
            self.get_logger().info(f"AMR status: {msg.data}")

    def _publish_target(self, target_id: int) -> None:
        loc = self.locations[target_id]
        msg = Point()
        msg.x = float(loc["x"])
        msg.y = float(loc["y"])
        msg.z = 0.0
        self.goal_pub.publish(msg)
        self.last_goal_publish = time.monotonic()

    def _control_loop(self) -> None:
        if self.done or self.latest_pose is None:
            return

        if self.route_index >= len(self.route):
            self.done = True
            self.get_logger().info("MISSION COMPLETE")
            return

        target_id = self.route[self.route_index]
        target = self.locations[target_id]

        if self.current_target_id != target_id:
            self.current_target_id = target_id
            self.get_logger().info(
                f"Dispatch -> {target_id}:{target['name']} "
                f"({target['x']:.2f}, {target['y']:.2f})"
            )
            self._publish_target(target_id)

        px, py = self.latest_pose
        distance = math.hypot(target["x"] - px, target["y"] - py)

        if distance <= ARRIVAL_TOLERANCE_M:
            self.get_logger().info(
                f"Reached -> {target_id}:{target['name']} (distance={distance:.3f} m)"
            )
            self.route_index += 1
            self.current_target_id = None
            return

        # Republish to make startup timing / occasional DDS packet loss harmless.
        if time.monotonic() - self.last_goal_publish >= GOAL_REPUBLISH_SEC:
            self._publish_target(target_id)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--route",
        type=Path,
        default=DEFAULT_ROUTE_PATH,
        help="Path to cuopt_route.json",
    )
    args = parser.parse_args()

    if not args.route.exists():
        raise FileNotFoundError(
            f"Route file not found: {args.route}\n"
            "Run 01_local_cuopt_solver.py first."
        )

    rclpy.init()
    node = SimpleFMS(args.route)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
