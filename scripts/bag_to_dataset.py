"""ROS2 주행 로그(rosbag) -> 학습용 데이터셋(CSV) 변환.

로봇을 조종하면서 녹화한 로그를 '그 순간 로봇이 본 것(관측)'과 '그때 내린 명령(행동)' 쌍으로 바꾼다.
이런 형태가 사람 조종을 따라 배우는 학습(모방 학습)의 기본 데이터 형식이다.

센서마다 들어오는 주기가 다르기 때문에(odom, 라이다, 명령) 일정한 시간 간격(기본 10Hz)으로 맞춰 정렬하고,
센서가 끊긴 구간이나 순서가 뒤바뀐 메시지는 품질 리포트로 따로 남긴다.

ROS 설치 없이 동작한다 (rosbags 라이브러리 사용).

녹화 (Codespaces, 로봇 컨테이너 안에서):
    ros2 bag record -o /ws/bags/run1 /odom /scan /cmd_vel
변환:
    python scripts/bag_to_dataset.py bags/run1 -o dataset_run1.csv
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TOPICS = {"/odom", "/scan", "/cmd_vel"}
SENSOR_STALE_S = 0.5     # 이보다 오래된 센서 값은 '끊김'으로 봄
CMD_STALE_S = 0.5        # 브릿지 watchdog과 같은 기준: 명령이 끊기면 행동은 정지(0)로 기록
COLUMNS = ["t_s", "x", "y", "yaw", "v", "w", "min_obstacle_m", "action_linear", "action_angular"]


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def min_obstacle(scan):
    valid = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    return min(valid) if valid else None


def read_bag(path):
    """토픽별로 (시각, 값) 목록을 읽고, 순서 뒤바뀜·중복 시각을 센다."""
    store = get_typestore(Stores.ROS2_HUMBLE)
    series = {t: [] for t in TOPICS}
    issues = {t: {"out_of_order": 0, "duplicate_ts": 0} for t in TOPICS}
    with AnyReader([Path(path)], default_typestore=store) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, ts_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = ts_ns / 1e9
            if conn.topic == "/odom":
                p, tw = msg.pose.pose, msg.twist.twist
                val = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation),
                       tw.linear.x, tw.angular.z)
            elif conn.topic == "/scan":
                val = min_obstacle(msg)
            else:
                val = (msg.linear.x, msg.angular.z)
            s = series[conn.topic]
            if s and t < s[-1][0]:
                issues[conn.topic]["out_of_order"] += 1
            elif s and t == s[-1][0]:
                issues[conn.topic]["duplicate_ts"] += 1
                continue
            s.append((t, val))
    for s in series.values():
        s.sort(key=lambda tv: tv[0])      # 뒤바뀐 순서는 시각 기준으로 바로잡음
    return series, issues


def latest_before(s, times, t):
    """시각 t 이전의 가장 최근 값과 그 나이(초)."""
    i = bisect.bisect_right(times, t) - 1
    return (None, None) if i < 0 else (s[i][1], t - s[i][0])


def build_dataset(series, hz=10.0):
    odom = series["/odom"]
    if not odom:
        raise SystemExit("/odom 메시지가 없음: 녹화할 때 /odom 토픽을 포함했는지 확인")
    times = {k: [t for t, _ in v] for k, v in series.items()}
    t0, t1 = odom[0][0], odom[-1][0]
    rows, stats = [], {"grid_points": 0, "dropped_odom_gap": 0, "scan_missing": 0, "cmd_stale_as_stop": 0}
    n = int((t1 - t0) * hz) + 1
    for k in range(n):
        t = t0 + k / hz
        stats["grid_points"] += 1
        o, age = latest_before(odom, times["/odom"], t)
        if o is None or age > SENSOR_STALE_S:
            stats["dropped_odom_gap"] += 1        # 위치를 모르는 순간은 학습에 쓰지 않음
            continue
        scan, s_age = latest_before(series["/scan"], times["/scan"], t)
        if scan is None or s_age > SENSOR_STALE_S:
            scan = None
            stats["scan_missing"] += 1
        cmd, c_age = latest_before(series["/cmd_vel"], times["/cmd_vel"], t)
        if cmd is None or c_age > CMD_STALE_S:
            if cmd is not None:
                stats["cmd_stale_as_stop"] += 1
            cmd = (0.0, 0.0)
        x, y, yaw, v, w = o
        rows.append([round(t - t0, 3), round(x, 4), round(y, 4), round(yaw, 4), round(v, 4), round(w, 4),
                     None if scan is None else round(scan, 3), round(cmd[0], 4), round(cmd[1], 4)])
    stats["rows"] = len(rows)
    stats["duration_s"] = round(t1 - t0, 2)
    return rows, stats


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag", help="rosbag 폴더 (ros2 bag record -o 로 만든 것)")
    p.add_argument("-o", "--out", default="dataset.csv")
    p.add_argument("--hz", type=float, default=10.0)
    a = p.parse_args()

    series, issues = read_bag(a.bag)
    rows, stats = build_dataset(series, a.hz)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerows(rows)
    report = {"messages": {k: len(v) for k, v in series.items()}, "issues": issues, **stats, "output": a.out}
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
