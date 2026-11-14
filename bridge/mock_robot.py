"""ROS 없이 돌아가는 가짜 로봇. ros_bridge.py와 같은 core 로직(버퍼, 안전장치, watchdog)을 쓴다.

4m x 4m 방 안에서 움직이는 바퀴 로봇을 단순 계산으로 흉내 내고, 벽까지 거리를 라이다 값처럼 만든다.
Gazebo가 무거워서 못 돌릴 때나, 브릿지 로직만 빠르게 확인할 때 쓴다.

실행: python bridge/mock_robot.py --url http://localhost:8000 --seconds 60
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bridge.core import CommandTracker, Uplink, decide_command, to_reading  # noqa: E402

ROOM = 2.0   # 방 중심에서 벽까지 2m


def wall_distance(x, y, yaw):
    """로봇 앞쪽 ±30도 방향으로 가장 가까운 벽까지 거리 (라이다 흉내)."""
    best = 3.5
    for deg in range(-30, 31, 5):
        a = yaw + math.radians(deg)
        dx, dy = math.cos(a), math.sin(a)
        for wall, d in ((ROOM - x, dx), (ROOM + x, -dx), (ROOM - y, dy), (ROOM + y, -dy)):
            if d > 1e-6:
                best = min(best, wall / d)
    return best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="http://localhost:8000")
    p.add_argument("--robot-id", default="MOCK-01")
    p.add_argument("--seconds", type=float, default=0, help="0이면 계속 실행")
    a = p.parse_args()

    up, tracker = Uplink(a.url, timeout=0.5), CommandTracker()
    x = y = yaw = 0.0
    battery, safety_stop = 100.0, False
    dt, t0, last_flush, step = 0.05, time.monotonic(), 0.0, 0
    reasons = {}
    while not a.seconds or time.monotonic() - t0 < a.seconds:
        now = time.monotonic()
        if step % 2 == 0:                                   # 10Hz 명령 확인
            tracker.update(up.latest_command(a.robot_id), now)
        min_obs = wall_distance(x, y, yaw)
        cmd = decide_command(tracker.latest, min_obs, now, tracker.last_new_time)
        reasons[cmd.reason] = reasons.get(cmd.reason, 0) + 1
        safety_stop = cmd.reason == "obstacle"

        yaw += cmd.angular * dt                             # 바퀴 로봇 움직임
        x += cmd.linear * math.cos(yaw) * dt
        y += cmd.linear * math.sin(yaw) * dt
        battery = max(0.0, battery - (0.002 + 0.02 * abs(cmd.linear)) * dt)

        if step % 2 == 0:                                   # 10Hz 기록
            up.push(to_reading(a.robot_id, "sim", x, y, cmd.linear, min_obs, safety_stop,
                               battery=round(battery, 2)))
        if now - last_flush >= 1.0:                         # 1초마다 전송
            up.flush()
            last_flush = now
        step += 1
        time.sleep(dt)
    up.flush()
    print({"recorded": up.pushed, "sent": up.sent, "buffered": len(up.buffer), "send_failures": up.failures,
           "dropped": up.dropped, "control_states": reasons})


if __name__ == "__main__":
    main()
