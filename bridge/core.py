"""브릿지의 핵심 로직. ROS2에 의존하지 않아서 ROS 없이도 테스트할 수 있다.

ros_bridge.py(ROS2 노드)와 mock_robot.py(ROS 없는 가짜 로봇)가 둘 다 이걸 쓴다.

하는 일
1. 센서 값 -> 서버 데이터 형식으로 변환
2. 버퍼: 서버가 죽어도 데이터를 쌓아뒀다가 살아나면 재전송 (유실 방지)
3. 안전장치: 장애물이 가까우면 전진 명령 차단
4. watchdog: 명령이 일정 시간 안 오면 로봇 정지
"""
import json
import math
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone

STOP_DISTANCE_M = 0.30      # 이보다 가까운 장애물이 있으면 전진 금지
WATCHDOG_S = 0.5            # 이 시간 동안 새 명령이 없으면 정지
BUFFER_MAX = 5000           # 서버가 오래 죽어 있으면 가장 오래된 것부터 버림 (메모리 보호)


def min_obstacle(ranges, range_min=0.12, range_max=3.5):
    """라이다 거리 배열에서 유효한 값 중 가장 가까운 거리. 유효값이 없으면 None."""
    valid = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    return min(valid) if valid else None


FRONT_HALF_WIDTH_DEG = 30   # 전진 차단 판단에 쓰는 정면 범위 (좌우 각 30도)


def front_min_obstacle(ranges, angle_min, angle_increment, range_min=0.12, range_max=3.5,
                       half_width_deg=FRONT_HALF_WIDTH_DEG):
    """정면 ±half_width_deg 안에서만 가장 가까운 장애물 거리.

    처음엔 360도 전체의 최솟값을 썼더니, 뒤나 옆에 기둥이 있어도 전진을 막아서
    로봇이 갇히는 문제가 실제 주행에서 생겼다. 전진을 막을지는 앞쪽만 보고 판단한다.
    """
    limit = math.radians(half_width_deg)
    front = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        a = math.atan2(math.sin(a), math.cos(a))      # -pi ~ pi 로 정규화
        if abs(a) <= limit:
            front.append(r)
    return min_obstacle(front, range_min, range_max)


def to_reading(robot_id, source, x, y, speed, min_obs, safety_stop, battery=None, ts=None):
    ts = ts or datetime.now(timezone.utc)
    return {
        "robot_id": robot_id,
        "ts": ts.isoformat(),
        "source": source,
        "x": round(x, 4),
        "y": round(y, 4),
        "speed": round(abs(speed), 4),
        "battery": battery,
        "min_obstacle_m": None if min_obs is None else round(min_obs, 3),
        "safety_stop": safety_stop,
    }


@dataclass
class SafeCommand:
    linear: float
    angular: float
    reason: str          # ok | watchdog | obstacle | none


def decide_command(latest, min_obs, now_s, last_cmd_time_s):
    """로봇에 실제로 보낼 명령을 결정.

    latest: 서버에서 받은 최신 명령 (dict 또는 None)
    min_obs: 가장 가까운 장애물 거리 (m)
    now_s, last_cmd_time_s: 현재 시각, 마지막으로 '새' 명령을 받은 시각 (초)
    """
    if latest is None:
        return SafeCommand(0.0, 0.0, "none")
    if now_s - last_cmd_time_s > WATCHDOG_S:
        return SafeCommand(0.0, 0.0, "watchdog")
    linear, angular = latest["linear"], latest["angular"]
    if linear > 0 and min_obs is not None and min_obs < STOP_DISTANCE_M:
        # 회전·후진은 허용해서 막힌 곳에서 빠져나올 수 있게 함
        return SafeCommand(0.0, angular, "obstacle")
    return SafeCommand(linear, angular, "ok")


class Uplink:
    """서버로 보내는 쪽. 실패하면 버퍼에 남겨두고 다음에 다시 보낸다.
    재전송으로 같은 데이터가 두 번 가도 서버의 (robot_id, ts) 기본키가 중복을 막는다."""

    def __init__(self, base_url, timeout=2.0, post=None):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.buffer = deque(maxlen=BUFFER_MAX)
        self.pushed = 0
        self.sent = 0
        self.failures = 0
        self.dropped = 0
        self._post = post or self._http_post
        self._flush_lock = threading.Lock()   # 전송 스레드와 종료 시 마지막 전송이 겹치지 않게

    def push(self, reading):
        if len(self.buffer) == self.buffer.maxlen:
            self.dropped += 1
        self.buffer.append(reading)
        self.pushed += 1

    def flush(self, batch_size=500):
        """버퍼를 비우려고 시도. 성공한 건수를 반환. 서버 응답이 없으면 그대로 남겨둠."""
        with self._flush_lock:
            return self._flush(batch_size)

    def _flush(self, batch_size):
        total = 0
        while self.buffer:
            batch = list(self.buffer)[:batch_size]
            try:
                self._post("/telemetry", batch)
            except (urllib.error.URLError, ConnectionError, TimeoutError, OSError):
                self.failures += 1
                return total
            for _ in batch:
                self.buffer.popleft()
            total += len(batch)
            self.sent += len(batch)
        return total

    def latest_command(self, robot_id):
        try:
            with urllib.request.urlopen(f"{self.base_url}/commands/{robot_id}/latest",
                                        timeout=self.timeout) as r:
                return json.loads(r.read())
        except (urllib.error.URLError, ConnectionError, TimeoutError, OSError, ValueError):
            return None   # 서버와 끊기면 명령 없음 -> watchdog이 로봇을 세움

    def _http_post(self, path, body):
        req = urllib.request.Request(self.base_url + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())


class CommandTracker:
    """서버의 최신 명령이 '새' 명령인지 추적 (같은 명령을 계속 읽는 것과 구분)."""

    def __init__(self):
        self.last_id = None
        self.last_new_time = -1e9
        self.latest = None

    def update(self, cmd, now_s):
        if cmd is None:
            return
        if cmd["id"] != self.last_id:
            self.last_id = cmd["id"]
            # 서버에 쌓여 있던 오래된 명령은 새 명령으로 치지 않음
            self.last_new_time = now_s - cmd.get("age_s", 0.0)
            self.latest = cmd


class NetworkWorker:
    """서버와의 통신을 두 스레드로 나눠서 돌린다.

    - 명령 확인 스레드: 0.1초마다 최신 명령을 가져옴
    - 데이터 전송 스레드: 1초마다 버퍼를 서버로 보냄

    처음엔 한 스레드에서 번갈아 했다. 이러면 서버가 느려 전송이 오래 걸리는 동안 명령 확인도 멈춘다.
    실제 주행에서 1초 주기로 watchdog이 반복해서 걸리는 현상이 나왔고, 원인 후보 중 하나가 이것이었다.
    둘을 분리해서 전송이 아무리 느려도 명령 확인은 제때 되게 했다.
    """

    def __init__(self, uplink, tracker, robot_id, poll_s=0.1, flush_s=1.0, on_warn=None):
        self.uplink, self.tracker, self.robot_id = uplink, tracker, robot_id
        self.poll_s, self.flush_s = poll_s, flush_s
        self.on_warn = on_warn or (lambda msg: None)
        self._stop = threading.Event()
        self._threads = []

    def start(self):
        self._threads = [threading.Thread(target=f, daemon=True)
                         for f in (self._command_loop, self._upload_loop)]
        for t in self._threads:
            t.start()
        return self

    def stop(self, timeout=3.0):
        self._stop.set()
        for t in self._threads:
            t.join(timeout)

    def _command_loop(self):
        while not self._stop.is_set():
            self.tracker.update(self.uplink.latest_command(self.robot_id), time.monotonic())
            self._stop.wait(self.poll_s)

    def _upload_loop(self):
        while not self._stop.wait(self.flush_s):
            before = len(self.uplink.buffer)
            if self.uplink.flush() == 0 and before:
                self.on_warn(f"server unreachable, buffered={len(self.uplink.buffer)}")
