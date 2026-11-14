import os
import sqlite3
import tempfile

import pytest

os.environ.setdefault("TELEMETRY_DB", os.path.join(tempfile.mkdtemp(), "test.db"))


@pytest.fixture(autouse=True, scope="module")
def clean_db():
    """테스트 파일마다 빈 DB에서 시작."""
    from app.main import DB_PATH, init_db
    init_db()
    with sqlite3.connect(DB_PATH) as c:
        for t in ("telemetry", "rejected", "commands"):
            c.execute(f"DELETE FROM {t}")
    yield
