"""관제 화면 데모 GIF 녹화: 로봇이 전진 -> 장애물 앞 안전 정지 -> 후진·회전 -> 주행.

준비 (snap.py 와 같음, 한 번만):
    pip install playwright && sudo $(which python) -m playwright install-deps chromium && python -m playwright install chromium
    sudo apt-get install -y ffmpeg
사용 (서버와 로봇이 켜져 있어야 함):
    python scripts/record_demo.py --robot TB3-01     -> docs/screenshots/demo.gif
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
import urllib.request

from playwright.sync_api import sync_playwright

p = argparse.ArgumentParser()
p.add_argument("--url", default="http://localhost:8000")
p.add_argument("--robot", default="TB3-01")
p.add_argument("--out", default="docs/screenshots/demo.gif")
p.add_argument("--speed", type=float, default=2.0, help="GIF 재생 배속")
a = p.parse_args()


def call(path, body=None):
    req = urllib.request.Request(a.url + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2) as r:
        return json.loads(r.read())


cmd = {"linear": 0.0, "angular": 0.0}
alive = True


def sender():                      # 관제 화면 버튼을 누르고 있는 것처럼 0.2초마다 명령
    while alive:
        try:
            call("/commands", {"robot_id": a.robot, **cmd})
        except OSError:
            pass
        time.sleep(0.2)


def state():
    return {r["robot_id"]: r for r in call("/robots")}[a.robot]


def drive(linear, angular, seconds):
    cmd.update(linear=linear, angular=angular)
    time.sleep(seconds)


def scenario():
    drive(-0.1, 0.0, 2)                                # 장애물에서 떨어진 곳에서 시작
    for _ in range(6):                                 # 정면에 장애물이 나올 때까지 전진
        cmd.update(linear=0.2, angular=0.0)
        end = time.time() + 12
        while time.time() < end and not state()["safety_stop"]:
            time.sleep(0.3)
        if state()["safety_stop"]:
            print("안전 정지:", state()["min_obstacle_m"], "m")
            time.sleep(4)                              # 전진 명령을 계속 보내도 서 있는 모습
            break
        drive(0.0, 1.0, 0.6)
    drive(-0.1, 0.0, 2)                                # 후진으로 빠져나오기
    drive(0.0, 1.2, 1.5)                               # 회전
    drive(0.18, 0.35, 8)                               # 주행
    cmd.update(linear=0.0, angular=0.0)


tmp = tempfile.mkdtemp()
threading.Thread(target=sender, daemon=True).start()
with sync_playwright() as pw:
    b = pw.chromium.launch()
    ctx = b.new_context(viewport={"width": 1200, "height": 760}, record_video_dir=tmp,
                        record_video_size={"width": 1200, "height": 760})
    page = ctx.new_page()
    page.goto(a.url)
    page.wait_for_timeout(2500)
    scenario()
    page.wait_for_timeout(1500)
    ctx.close()
    b.close()
alive = False

video = glob.glob(os.path.join(tmp, "*.webm"))[0]
os.makedirs(os.path.dirname(a.out), exist_ok=True)
vf = (f"setpts=PTS/{a.speed},fps=8,scale=900:-1:flags=lanczos,"
      "split[s0][s1];[s0]palettegen=max_colors=96[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4")
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "2.5", "-i", video, "-vf", vf, a.out], check=True)
shutil.rmtree(tmp)
print("saved", a.out, f"({os.path.getsize(a.out) // 1024} KB)")
