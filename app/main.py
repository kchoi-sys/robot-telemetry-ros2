"""Robot Telemetry Pipeline: 로봇 센서 데이터 수집 -> 검증·정제 -> 학습용 데이터셋, 그리고 로봇 원격 명령.

실행: uvicorn app.main:app --reload
대시보드: http://localhost:8000/
API 문서: http://localhost:8000/docs
"""
import csv
import io
import os
import sqlite3
import statistics
from contextlib import contextmanager
from datetime import datetime, timezone

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field

DB_PATH = os.environ.get("TELEMETRY_DB", "telemetry.db")
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")
OFFLINE_AFTER_S = 3.0          # 이 시간 동안 데이터가 없으면 오프라인으로 표시

app = FastAPI(title="Robot Telemetry Pipeline")

COLUMNS = ["robot_id", "ts", "source", "x", "y", "speed", "battery", "temp_c",
           "min_obstacle_m", "safety_stop", "received_at"]


def now_utc():
    return datetime.now(timezone.utc)


# ---------- 저장소 ----------
@contextmanager
def db():
    conn = sqlite3.connect(DB_PATH, timeout=5)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    finally:
        conn.close()


def init_db():
    with db() as c:
        # WAL: 데이터를 쓰는 중에도 관제 화면 조회가 기다리지 않게 (읽기/쓰기 동시 진행)
        c.execute("PRAGMA journal_mode=WAL")
        c.executescript(
            """
            CREATE TABLE IF NOT EXISTS telemetry (
                robot_id   TEXT NOT NULL,
                ts         TEXT NOT NULL,      -- 로봇 쪽에서 측정한 시각
                source     TEXT NOT NULL,      -- real | sim
                x REAL, y REAL, speed REAL,
                battery REAL, temp_c REAL,     -- 센서가 없는 로봇은 NULL
                min_obstacle_m REAL,           -- 라이다 기준 가장 가까운 장애물 거리
                safety_stop INTEGER DEFAULT 0, -- 안전장치가 전진을 막았는지
                received_at TEXT,              -- 서버가 받은 시각 (지연 계산용)
                PRIMARY KEY (robot_id, ts)     -- 같은 로봇·같은 시각 중복 차단 (재전송해도 안전)
            );
            CREATE TABLE IF NOT EXISTS rejected (
                received_at TEXT, robot_id TEXT, reason TEXT, payload TEXT
            );
            CREATE TABLE IF NOT EXISTS commands (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                robot_id TEXT NOT NULL,
                issued_at TEXT NOT NULL,
                linear REAL NOT NULL,          -- 전진 속도 m/s
                angular REAL NOT NULL          -- 회전 속도 rad/s
            );
            -- 데이터가 쌓여도 관제 화면 조회(최근 수신, 로봇별 마지막 수신)가 느려지지 않게
            CREATE INDEX IF NOT EXISTS idx_tel_received ON telemetry(received_at);
            CREATE INDEX IF NOT EXISTS idx_tel_robot_received ON telemetry(robot_id, received_at);
            CREATE INDEX IF NOT EXISTS idx_cmd_robot ON commands(robot_id, id);
            """
        )


init_db()


# ---------- 입력 스키마 (1차 검증) ----------
class Reading(BaseModel):
    robot_id: str = Field(min_length=1, max_length=32)
    ts: datetime
    source: str = Field(pattern="^(real|sim)$")
    x: float
    y: float
    speed: float = Field(ge=0)
    battery: float | None = Field(None, ge=0, le=100)
    temp_c: float | None = None
    min_obstacle_m: float | None = Field(None, ge=0)
    safety_stop: bool = False


# ---------- 2차 검증: 값은 형식상 맞지만 물리적으로 말이 안 되는 경우 ----------
MAX_SPEED = 5.0      # m/s
TEMP_RANGE = (-30.0, 90.0)


def physical_check(r: Reading) -> str | None:
    if r.speed > MAX_SPEED:
        return f"speed {r.speed} > {MAX_SPEED}"
    if r.temp_c is not None and not TEMP_RANGE[0] <= r.temp_c <= TEMP_RANGE[1]:
        return f"temp {r.temp_c} out of range"
    return None


def reject(c, r: Reading, reason: str):
    c.execute(
        "INSERT INTO rejected VALUES (?,?,?,?)",
        (now_utc().isoformat(), r.robot_id, reason, r.model_dump_json()),
    )


# ---------- 수집 API ----------
@app.post("/telemetry")
def ingest(readings: list[Reading]):
    """센서 데이터 일괄 수신. 중복·이상치는 버리지 않고 rejected 테이블에 사유와 함께 남김."""
    ok = dup = bad = 0
    received = now_utc().isoformat()
    with db() as c:
        for r in readings:
            reason = physical_check(r)
            if reason:
                reject(c, r, reason)
                bad += 1
                continue
            ts = r.ts.astimezone(timezone.utc).isoformat()
            try:
                c.execute(
                    f"INSERT INTO telemetry ({','.join(COLUMNS)}) VALUES ({','.join('?' * len(COLUMNS))})",
                    (r.robot_id, ts, r.source, r.x, r.y, r.speed, r.battery, r.temp_c,
                     r.min_obstacle_m, int(r.safety_stop), received),
                )
                ok += 1
            except sqlite3.IntegrityError:
                reject(c, r, "duplicate (robot_id, ts)")
                dup += 1
    return {"accepted": ok, "duplicates": dup, "rejected": bad}


@app.get("/robots")
def robots():
    with db() as c:
        rows = c.execute(
            """SELECT robot_id, source, COUNT(*) n, MAX(ts) last_ts, MAX(received_at) last_seen
               FROM telemetry GROUP BY robot_id ORDER BY robot_id"""
        ).fetchall()
        out = []
        for r in rows:
            last = dict(c.execute(
                "SELECT * FROM telemetry WHERE robot_id=? ORDER BY ts DESC LIMIT 1",
                (r["robot_id"],)).fetchone())
            age = (now_utc() - datetime.fromisoformat(r["last_seen"])).total_seconds()
            out.append({**dict(r), "online": age < OFFLINE_AFTER_S,
                        "battery": last["battery"], "speed": last["speed"],
                        "min_obstacle_m": last["min_obstacle_m"],
                        "safety_stop": bool(last["safety_stop"])})
    return out


@app.get("/telemetry/{robot_id}")
def history(robot_id: str, limit: int = Query(100, le=5000)):
    with db() as c:
        rows = c.execute(
            "SELECT * FROM telemetry WHERE robot_id=? ORDER BY ts DESC LIMIT ?",
            (robot_id, limit),
        ).fetchall()
    if not rows:
        raise HTTPException(404, "unknown robot")
    return [dict(r) for r in reversed(rows)]


@app.get("/quality")
def quality():
    """데이터 품질 리포트: 수용/거부 건수, 거부 사유 분포, 수신 지연."""
    with db() as c:
        accepted = c.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        reasons = c.execute(
            """SELECT CASE WHEN reason LIKE 'duplicate%' THEN 'duplicate'
                           WHEN reason LIKE 'speed%' THEN 'speed_outlier'
                           WHEN reason LIKE 'temp%' THEN 'temp_outlier'
                           ELSE 'other' END kind, COUNT(*) n
               FROM rejected GROUP BY kind"""
        ).fetchall()
        by_source = c.execute(
            "SELECT source, COUNT(*) n FROM telemetry GROUP BY source"
        ).fetchall()
        recent = c.execute(
            "SELECT ts, received_at FROM telemetry ORDER BY received_at DESC LIMIT 500"
        ).fetchall()
    # 지연 = 서버 수신 시각 - 로봇 측정 시각. 끊겼다가 재전송된 데이터는 지연이 크게 잡힘.
    lat = sorted(
        (datetime.fromisoformat(r["received_at"]) - datetime.fromisoformat(r["ts"])).total_seconds() * 1000
        for r in recent)
    latency = None
    if lat:
        latency = {"p50_ms": round(statistics.median(lat), 1),
                   "p95_ms": round(lat[int(0.95 * (len(lat) - 1))], 1),
                   "n": len(lat)}
    return {
        "accepted": accepted,
        "rejected": {r["kind"]: r["n"] for r in reasons},
        "by_source": {r["source"]: r["n"] for r in by_source},
        "latency": latency,
    }


@app.get("/dataset.csv")
def export_dataset(source: str | None = Query(None, pattern="^(real|sim)$")):
    """검증을 통과한 데이터만 학습용 CSV로 내보내기 (시간순 정렬, 실제/합성 구분 컬럼 포함)."""
    sql, args = f"SELECT {','.join(COLUMNS)} FROM telemetry", []
    if source:
        sql += " WHERE source=?"
        args.append(source)
    sql += " ORDER BY robot_id, ts"
    with db() as c:
        rows = c.execute(sql, args).fetchall()
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(COLUMNS)
    w.writerows([tuple(r) for r in rows])
    buf.seek(0)
    return StreamingResponse(
        iter([buf.getvalue()]),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=dataset.csv"},
    )


# ---------- 명령 API (서버 -> 로봇) ----------
# TurtleBot3 Burger 사양 기준 최대치. 이보다 큰 명령은 서버에서 아예 받지 않음.
MAX_LINEAR = 0.22    # m/s
MAX_ANGULAR = 2.84   # rad/s


class Command(BaseModel):
    robot_id: str = Field(min_length=1, max_length=32)
    linear: float = Field(ge=-MAX_LINEAR, le=MAX_LINEAR)
    angular: float = Field(ge=-MAX_ANGULAR, le=MAX_ANGULAR)


@app.post("/commands")
def send_command(cmd: Command):
    """관제 화면에서 보낸 이동 명령을 저장. 브릿지가 가져가서 로봇에 전달.
    관제 쪽은 버튼을 누르고 있는 동안 계속 보내야 함 (안 보내면 브릿지 watchdog이 로봇을 세움)."""
    issued = now_utc().isoformat()
    with db() as c:
        cur = c.execute("INSERT INTO commands (robot_id, issued_at, linear, angular) VALUES (?,?,?,?)",
                        (cmd.robot_id, issued, cmd.linear, cmd.angular))
    return {"id": cur.lastrowid, "issued_at": issued}


@app.get("/commands/{robot_id}/latest")
def latest_command(robot_id: str):
    with db() as c:
        row = c.execute("SELECT * FROM commands WHERE robot_id=? ORDER BY id DESC LIMIT 1",
                        (robot_id,)).fetchone()
    if not row:
        return None
    age = (now_utc() - datetime.fromisoformat(row["issued_at"])).total_seconds()
    return {**dict(row), "age_s": round(age, 3)}


@app.get("/")
def dashboard():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))
