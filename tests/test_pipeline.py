import os
import tempfile

os.environ["TELEMETRY_DB"] = os.path.join(tempfile.mkdtemp(), "test.db")

from fastapi.testclient import TestClient  # noqa: E402

from app.main import app  # noqa: E402

client = TestClient(app)


def reading(**kw):
    base = dict(robot_id="R-01", ts="2026-11-15T00:00:00+00:00", source="real",
                x=1.0, y=2.0, speed=1.0, battery=90.0, temp_c=38.0)
    base.update(kw)
    return base


def test_accepts_valid_and_blocks_duplicate():
    r = client.post("/telemetry", json=[reading(), reading()]).json()
    assert r == {"accepted": 1, "duplicates": 1, "rejected": 0}


def test_rejects_physical_outliers():
    r = client.post("/telemetry", json=[
        reading(ts="2026-11-15T00:00:01+00:00", speed=42.0),
        reading(ts="2026-11-15T00:00:02+00:00", temp_c=300.0),
    ]).json()
    assert r["rejected"] == 2


def test_schema_validation():
    assert client.post("/telemetry", json=[reading(battery=150)]).status_code == 422
    assert client.post("/telemetry", json=[reading(source="fake")]).status_code == 422


def test_quality_and_export():
    q = client.get("/quality").json()
    assert q["accepted"] == 1 and q["rejected"]["duplicate"] == 1
    csv_text = client.get("/dataset.csv").text
    assert csv_text.splitlines()[0].startswith("robot_id,ts,source")
    assert len(csv_text.strip().splitlines()) == 2
