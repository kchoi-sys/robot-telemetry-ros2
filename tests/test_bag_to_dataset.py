"""가짜 rosbag을 직접 만들어서 변환기를 검증 (ROS 설치 없이)."""
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

rosbags = pytest.importorskip("rosbags")
from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
from bag_to_dataset import build_dataset, read_bag  # noqa: E402

S = get_typestore(Stores.ROS2_HUMBLE)
T = S.types
NS = 1_000_000_000


def header(t):
    return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=int(t), nanosec=int(t % 1 * 1e9)),
                                    frame_id="odom")


def vec(x=0.0, y=0.0, z=0.0):
    return T["geometry_msgs/msg/Vector3"](x=x, y=y, z=z)


def odom(t, x, yaw, v):
    q = T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    pose = T["geometry_msgs/msg/PoseWithCovariance"](
        pose=T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=x, y=0.0, z=0.0), orientation=q),
        covariance=np.zeros(36))
    twist = T["geometry_msgs/msg/TwistWithCovariance"](
        twist=T["geometry_msgs/msg/Twist"](linear=vec(v), angular=vec()), covariance=np.zeros(36))
    return T["nav_msgs/msg/Odometry"](header=header(t), child_frame_id="base_footprint", pose=pose, twist=twist)


def scan(t, d):
    return T["sensor_msgs/msg/LaserScan"](header=header(t), angle_min=0.0, angle_max=6.28, angle_increment=0.0175,
                                          time_increment=0.0, scan_time=0.2, range_min=0.12, range_max=3.5,
                                          ranges=np.array([d, math.inf, 3.0], dtype=np.float32),
                                          intensities=np.zeros(3, dtype=np.float32))


def twist(lin):
    return T["geometry_msgs/msg/Twist"](linear=vec(lin), angular=vec())


@pytest.fixture(scope="module")
def bag():
    path = Path(tempfile.mkdtemp()) / "run"
    with Writer(path, version=8) as w:
        c_odom = w.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=S)
        c_scan = w.add_connection("/scan", "sensor_msgs/msg/LaserScan", typestore=S)
        c_cmd = w.add_connection("/cmd_vel", "geometry_msgs/msg/Twist", typestore=S)
        put = lambda c, t, m: w.write(c, int(t * NS), S.serialize_cdr(m, c.msgtype))  # noqa: E731
        for k in range(100):                       # 0~5초, odom 20Hz
            t = 100 + k * 0.05
            if 2.0 <= t - 100 < 3.0:               # 1초 동안 odom 끊김
                continue
            put(c_odom, t, odom(t, 0.2 * (t - 100), 0.1, 0.2))
        for k in range(25):                        # 라이다 5Hz, 점점 가까워짐
            t = 100 + k * 0.2
            put(c_scan, t, scan(t, 2.0 - 0.06 * k))
        for k in range(15):                        # 명령은 0~3초만, 그 뒤 끊김
            put(c_cmd, 100 + k * 0.2, twist(0.2))
        put(c_odom, 100.5, odom(100.5, 0.1, 0.1, 0.2))   # 순서 뒤바뀐 메시지 (같은 시각 중복)
    return path


def test_conversion_aligns_and_reports(bag):
    series, issues = read_bag(bag)
    assert len(series["/odom"]) == 80                     # 100 - 20 (끊긴 1초)
    assert issues["/odom"]["out_of_order"] + issues["/odom"]["duplicate_ts"] >= 1

    rows, stats = build_dataset(series, hz=10)
    # odom 끊긴 1초 중 0.5초 넘게 지난 구간은 버려야 함
    assert stats["dropped_odom_gap"] >= 4
    assert stats["rows"] == len(rows) == stats["grid_points"] - stats["dropped_odom_gap"]

    by_t = {r[0]: r for r in rows}
    assert by_t[1.0][7] == pytest.approx(0.2)              # 명령 있는 구간: 행동 = 전진
    assert by_t[4.5][7] == 0.0                             # 명령 끊긴 뒤: 행동 = 정지
    assert stats["cmd_stale_as_stop"] > 0
    assert by_t[1.0][6] == pytest.approx(2.0 - 0.06 * 5, abs=1e-3)   # 라이다 inf는 무시하고 가장 가까운 값
    assert by_t[0.0][3] == pytest.approx(0.1, abs=1e-4)              # yaw 복원
