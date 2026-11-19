"""viz/scene.html 을 PNG로 렌더 (GPU 없는 환경에서도 소프트웨어 렌더링으로 동작).

준비 (snap.py 와 같음, 한 번만):
    pip install playwright && sudo $(which python) -m playwright install-deps chromium && python -m playwright install chromium
사용:
    python viz/render.py            -> docs/screenshots/3d_hero.png, docs/screenshots/3d_safety_stop.png
"""
import functools
import http.server
import os
import threading

from playwright.sync_api import sync_playwright

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(ROOT), "docs", "screenshots")
SHOTS = {"3d_hero.png": "hero", "3d_safety_stop.png": "clean"}

class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


handler = functools.partial(Quiet, directory=ROOT)
server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
port = server.server_address[1]

os.makedirs(OUT, exist_ok=True)
with sync_playwright() as p:
    b = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader",
                                "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
    for name, mode in SHOTS.items():
        pg = b.new_page(viewport={"width": 2400, "height": 1350})
        pg.goto(f"http://127.0.0.1:{port}/scene.html?mode={mode}")
        pg.wait_for_function("window.DONE === true", timeout=120_000)
        pg.wait_for_timeout(800)
        pg.screenshot(path=os.path.join(OUT, name))
        pg.close()
        print("saved", os.path.join("docs/screenshots", name))
    b.close()
server.shutdown()
