"""ROS2 <-> 관제 서버 브릿지 노드 (ROS2 Humble, TurtleBot3 기준).

로봇 -> 서버: /odom(위치·속도), /scan(라이다)을 구독해서 10Hz로 묶어 서버에 전송
서버 -> 로봇: 서버의 최신 명령을 가져와 안전장치를 거친 뒤 /cmd_vel로 발행

실기체로 바꿀 때: 같은 TurtleBot3 실기체는 토픽 이름이 같아서 이 코드를 그대로 쓴다.
다른 로봇은 토픽 이름(파라미터)과 메시지 변환 부분만 바꾸면 된다.

실행: ros2 run 없이 바로 -> python3 bridge/ros_bridge.py --ros-args -p server_url:=http://api:8000
"""
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.core import CommandTracker, Uplink, decide_command, min_obstacle, to_reading  # noqa: E402


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__("telemetry_bridge")
        self.robot_id = self.declare_parameter("robot_id", "TB3-01").value
        self.source = self.declare_parameter("source", "sim").value          # 실기체면 real
        server_url = self.declare_parameter("server_url", "http://localhost:8000").value

        self.uplink = Uplink(server_url)
        self.tracker = CommandTracker()
        self.odom = None
        self.min_obs = None
        self.last_reason = None
        self.safety_stop = False

        # 라이다처럼 빠른 센서는 best effort로 받음: 몇 개 빠져도 최신 값이 더 중요
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_timer(0.1, self.sample)     # 10Hz: 센서 값을 서버 형식으로 쌓기
        self.create_timer(0.05, self.control)   # 20Hz: 로봇에 명령 발행 (안전장치 포함)

        # 네트워크는 별도 스레드: 서버가 느리거나 죽어도 로봇 제어 루프는 멈추지 않게
        threading.Thread(target=self.network_loop, daemon=True).start()
        self.get_logger().info(f"bridge up: {self.robot_id} -> {server_url}")

    # ----- 로봇 -> 서버 -----
    def on_odom(self, msg: Odometry):
        self.odom = msg

    def on_scan(self, msg: LaserScan):
        self.min_obs = min_obstacle(msg.ranges, msg.range_min, msg.range_max)

    def sample(self):
        if self.odom is None:
            return
        p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
        self.uplink.push(to_reading(self.robot_id, self.source, p.x, p.y, v.x,
                                    self.min_obs, self.safety_stop))

    def network_loop(self):
        last_flush = 0.0
        while rclpy.ok():
            now = time.monotonic()
            self.tracker.update(self.uplink.latest_command(self.robot_id), now)
            if now - last_flush >= 1.0:          # 1초마다 묶어서 전송
                before = len(self.uplink.buffer)
                sent = self.uplink.flush()
                if sent == 0 and before:
                    self.get_logger().warn(f"server unreachable, buffered={len(self.uplink.buffer)}")
                last_flush = now
            time.sleep(0.1)

    # ----- 서버 -> 로봇 -----
    def control(self):
        cmd = decide_command(self.tracker.latest, self.min_obs,
                             time.monotonic(), self.tracker.last_new_time)
        self.safety_stop = cmd.reason == "obstacle"
        if cmd.reason != self.last_reason:
            self.get_logger().info(f"command state: {cmd.reason}")
            self.last_reason = cmd.reason
        t = Twist()
        t.linear.x, t.angular.z = cmd.linear, cmd.angular
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # 종료할 때 반드시 정지 명령
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
