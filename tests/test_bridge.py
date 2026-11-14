import math
import os
import sys
import tempfile
import urllib.error

os.environ.setdefault("TELEMETRY_DB", os.path.join(tempfile.mkdtemp(), "test.db"))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fastapi.testclient import TestClient  # noqa: E402

from app.main import app  # noqa: E402
from bridge.core import (CommandTracker, Uplink, decide_command, min_obstacle,  # noqa: E402
                         to_reading)

client = TestClient(app)
CMD = {"id": 1, "linear": 0.2, "angular": 0.0}


# ----- 안전장치 / watchdog -----
def test_forward_allowed_when_clear():
    c = decide_command(CMD, min_obs=1.0, now_s=10.0, last_cmd_time_s=9.9)
    assert (c.linear, c.reason) == (0.2, "ok")


def test_watchdog_stops_when_commands_stop():
    c = decide_command(CMD, min_obs=1.0, now_s=10.0, last_cmd_time_s=9.0)
    assert (c.linear, c.angular, c.reason) == (0.0, 0.0, "watchdog")


def test_obstacle_blocks_forward_but_allows_turn():
    c = decide_command({"id": 1, "linear": 0.2, "angular": 1.0}, 0.2, 10.0, 9.9)
    assert (c.linear, c.angular, c.reason) == (0.0, 1.0, "obstacle")
    back = decide_command({"id": 2, "linear": -0.1, "angular": 0.0}, 0.2, 10.0, 9.9)
    assert back.linear == -0.1   # 막힌 곳에서 후진으로 빠져나오는 건 허용


def test_no_command_means_stop():
    assert decide_command(None, 1.0, 10.0, -1e9).reason == "none"


def test_lidar_ignores_invalid_ranges():
    assert min_obstacle([math.inf, 0.01, float("nan"), 0.8, 2.0]) == 0.8
    assert min_obstacle([math.inf, math.inf]) is None


def test_stale_command_not_treated_as_new():
    t = CommandTracker()
    t.update({"id": 5, "linear": 0.2, "angular": 0, "age_s": 30.0}, now_s=100.0)
    # 30초 전에 내려진 명령: 브릿지가 막 켜졌어도 로봇을 움직이면 안 됨
    assert decide_command(t.latest, 1.0, 100.0, t.last_new_time).reason == "watchdog"


# ----- 끊김 대비: 버퍼와 재전송 -----
def test_buffer_survives_outage_and_resends_without_loss():
    server_up = {"v": False}
    received = []

    def fake_post(path, batch):
        if not server_up["v"]:
            raise urllib.error.URLError("down")
        received.extend(batch)

    up = Uplink("http://x", post=fake_post)
    for i in range(30):
        up.push({"i": i})
    assert up.flush() == 0 and len(up.buffer) == 30      # 서버 다운: 하나도 안 버림
    server_up["v"] = True
    assert up.flush() == 30 and len(up.buffer) == 0      # 복구: 전부 전송
    assert [r["i"] for r in received] == list(range(30))  # 순서도 유지


def test_resend_after_lost_response_is_deduplicated_by_server():
    r = to_reading("TB3-T", "sim", 0.1, 0.2, 0.1, 0.9, False)
    first = client.post("/telemetry", json=[r]).json()
    again = client.post("/telemetry", json=[r]).json()   # 응답을 못 받아 다시 보낸 상황
    assert first["accepted"] == 1 and again["duplicates"] == 1


# ----- 명령 API -----
def test_command_roundtrip_and_limits():
    assert client.post("/commands", json={"robot_id": "TB3-T", "linear": 0.2, "angular": 0}).status_code == 200
    latest = client.get("/commands/TB3-T/latest").json()
    assert latest["linear"] == 0.2 and latest["age_s"] < 5
    # 로봇 사양보다 빠른 명령은 서버에서 거부
    assert client.post("/commands", json={"robot_id": "TB3-T", "linear": 3.0, "angular": 0}).status_code == 422


def test_robot_without_battery_sensor_and_latency_report():
    r = client.post("/telemetry", json=[to_reading("TB3-N", "sim", 0, 0, 0.1, None, False,
                                                   ts=None)]).json()
    assert r["accepted"] == 1
    robots = {x["robot_id"]: x for x in client.get("/robots").json()}
    assert robots["TB3-N"]["online"] and robots["TB3-N"]["battery"] is None
    assert client.get("/quality").json()["latency"]["n"] >= 1
