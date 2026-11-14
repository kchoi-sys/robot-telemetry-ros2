"""관제 화면 대신 스크립트로 로봇을 조종해서 안전장치와 watchdog을 확인하는 시나리오.

1) 벽을 향해 계속 전진 명령 -> 0.3m 앞에서 안전장치가 멈춰야 함
2) 회전 후 다시 전진
3) 명령을 끊음 -> 0.5초 안에 watchdog이 로봇을 세워야 함

실행: python scripts/drive_scenario.py --robot MOCK-01   (서버와 로봇/브릿지가 켜져 있어야 함)
"""
import argparse
import json
import time
import urllib.request


def call(url, body=None):
    req = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2) as r:
        return json.loads(r.read())


def hold(base, robot, linear, angular, seconds):
    """버튼을 누르고 있는 것처럼 0.2초마다 같은 명령을 보냄."""
    end = time.time() + seconds
    while time.time() < end:
        try:
            call(f"{base}/commands", {"robot_id": robot, "linear": linear, "angular": angular})
        except OSError:
            pass   # 서버가 잠깐 죽어 있어도 시나리오는 계속
        time.sleep(0.2)


def state(base, robot):
    try:
        return {r["robot_id"]: r for r in call(f"{base}/robots")}.get(robot)
    except OSError:
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="http://localhost:8000")
    p.add_argument("--robot", default="TB3-01")
    a = p.parse_args()

    print("1) 벽 쪽으로 전진 12초")
    hold(a.url, a.robot, 0.2, 0.0, 12)
    s = state(a.url, a.robot)
    print("   장애물 거리", s and s["min_obstacle_m"], "안전장치", s and s["safety_stop"], "속도", s and s["speed"])
    print("2) 좌회전 1.5초 후 전진 3초")
    hold(a.url, a.robot, 0.0, 1.2, 1.5)
    hold(a.url, a.robot, 0.2, 0.0, 3)
    print("3) 명령 끊기")
    time.sleep(1.5)
    s = state(a.url, a.robot)
    print("   1.5초 뒤 속도", s and s["speed"], "(0이면 watchdog 정상)")


if __name__ == "__main__":
    main()
