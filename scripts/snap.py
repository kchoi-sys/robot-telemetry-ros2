"""관제 화면을 PNG로 저장 (사지방처럼 화면 캡처가 막힌 환경용).

준비 (한 번만):
    pip install playwright && sudo $(which python) -m playwright install-deps chromium && python -m playwright install chromium
사용:
    python scripts/snap.py 이름        -> docs/screenshots/이름.png
"""
import os
import sys

from playwright.sync_api import sync_playwright

name = sys.argv[1] if len(sys.argv) > 1 else "dashboard"
os.makedirs("docs/screenshots", exist_ok=True)
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1200, "height": 760}, device_scale_factor=2)
    pg.goto("http://localhost:8000")
    pg.wait_for_timeout(2500)
    pg.screenshot(path=f"docs/screenshots/{name}.png", full_page=True)
    b.close()
print("saved", f"docs/screenshots/{name}.png")
