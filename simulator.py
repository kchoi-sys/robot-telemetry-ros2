"""가짜 로봇 3대가 센서 데이터를 보냄. 일부러 중복·이상치·순서 뒤바뀐 데이터를 섞어서
파이프라인이 걸러내는지 확인한다.

실행: python simulator.py            (서버가 켜져 있어야 함)
옵션: python simulator.py --steps 200 --url http://localhost:8000
"""
import argparse
import json
import math
import random
import urllib.request
from datetime import datetime, timedelta, timezone

ROBOTS = [("R-01", "real"), ("R-02", "real"), ("SIM-01", "sim")]


def make_reading(robot_id, source, t, step):
    angle = step / 15 + hash(robot_id) % 10
    return {
        "robot_id": robot_id,
        "ts": t.isoformat(),
        "source": source,
        "x": round(10 * math.cos(angle), 3),
        "y": round(10 * math.sin(angle), 3),
        "speed": round(abs(random.gauss(1.2, 0.3)), 3),
        "battery": round(min(100.0, max(0.0, 100 - step * 0.2 + random.uniform(-0.5, 0.5))), 2),
        "temp_c": round(random.gauss(38, 2), 2),
    }


def post(url, batch):
    req = urllib.request.Request(
        url + "/telemetry",
        data=json.dumps(batch).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="http://localhost:8000")
    p.add_argument("--steps", type=int, default=120)
    p.add_argument("--noise", type=float, default=0.05, help="이상 데이터 섞는 비율")
    a = p.parse_args()

    start = datetime.now(timezone.utc)
    total = {"accepted": 0, "duplicates": 0, "rejected": 0}
    for step in range(a.steps):
        t = start + timedelta(seconds=step)
        batch = [make_reading(rid, src, t, step) for rid, src in ROBOTS]

        if random.random() < a.noise:                      # 같은 데이터 두 번 전송
            batch.append(dict(batch[0]))
        if random.random() < a.noise:                      # 센서 튐 (속도 이상치)
            batch[1]["speed"] = 42.0
        if random.random() < a.noise:                      # 온도 센서 고장
            batch[2]["temp_c"] = 300.0
        random.shuffle(batch)                              # 도착 순서 섞임

        for k, v in post(a.url, batch).items():
            total[k] += v
    print("done:", total)


if __name__ == "__main__":
    main()
