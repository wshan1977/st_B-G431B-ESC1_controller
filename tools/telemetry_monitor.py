#!/usr/bin/env python3
"""B-G431B-ESC1 텔레메트리 모니터.

보드 USB(ST-LINK 가상 COM 포트, 115200 8N1)로 들어오는
"RPM=5010 IPH=1836mA IQ=-1830mA IBUS=250mA VBUS=12V" 형식의 라인을 파싱해
RPM / 전류 / 토크 / 브레이크력 / 버스전압을 실시간 표시하고 그래프로 그린다.

토크는 전기 입력 전력과 회전수로부터 T = P / ω = (Vbus x Ibus) / (RPM x 2π/60)
로 환산하고, 브레이크력은 입력한 작용 반경으로 F = (T - 0점 오프셋) / r 환산한다.
P/ω는 마찰·철손 등 무부하 손실까지 토크로 잡으므로, 무부하 회전 상태에서
[캘리브레이션] 버튼을 눌러 그 시점의 토크를 0점으로 저장해 차감한다.
Ibus(버스전류)가 음수이면 제동(회생), 양수이면 구동 상태다.
저속(|RPM| < 100)에서는 나눗셈이 발산하므로 표시하지 않는다.

의존성: pyserial  (pip install pyserial)
실행:   python telemetry_monitor.py
"""

import json
import math
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
from serial.tools import list_ports

BAUDRATE = 115200
LINE_RE = re.compile(
    r"RPM=(-?\d+)\s+IPH=(-?\d+)mA(?:\s+IQ=(-?\d+)mA)?\s+IBUS=(-?\d+)mA\s+VBUS=(\d+)V"
)
HISTORY_SEC = 60          # 그래프에 유지할 시간 범위
SAMPLE_PERIOD = 0.1       # 펌웨어 전송 주기 (100 ms)
MAX_POINTS = int(HISTORY_SEC / SAMPLE_PERIOD)

MIN_RPM_FOR_TORQUE = 100  # 이 미만에서는 T = P/ω 계산이 발산하므로 표시 안 함
DEFAULT_RADIUS_MM = 30.0  # 브레이크력 환산용 작용 반경 기본값
GF_PER_N = 1000 / 9.80665

# 캐스팅 시뮬레이션: 보드로 보내는 한 줄 명령
#   CAST1(살살) / CAST2(세게 멀리) / SPD n(커스텀 프로파일 스트리밍)
#   DIAL(다이얼 복귀) / STOP / START
PROFILE_STEP_MS = 100     # 커스텀 프로파일 재생 시 SPD 전송 주기 (펌웨어 워치독 1초)
PROFILE_MAX_SEC = 10.0    # 프로파일 에디터 시간축 범위
PROFILE_MAX_RPM = 9500    # 펌웨어 상한 (POT_SPEED_MAX_RPM)
PROFILE_MIN_RPM = 1200    # 펌웨어 하한 — 더 낮게 보내도 보드가 1200으로 클램프
BASE_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))
PROFILE_DIR = os.path.join(BASE_DIR, "profiles")


class SerialReader(threading.Thread):
    """시리얼 라인을 읽어 큐에 넣는 백그라운드 스레드."""

    def __init__(self, port: str, rx_queue: queue.Queue):
        super().__init__(daemon=True)
        self._queue = rx_queue
        self._stop_evt = threading.Event()
        self._ser = serial.Serial(port, BAUDRATE, timeout=0.5)

    def run(self):
        while not self._stop_evt.is_set():
            try:
                line = self._ser.readline().decode("ascii", errors="ignore")
            except (serial.SerialException, OSError):
                self._queue.put(("error", "시리얼 포트 연결이 끊어졌습니다."))
                break
            m = LINE_RE.search(line)
            if m:
                rpm, iph_ma, iq_ma, ibus_ma, vbus = (
                    int(g) if g is not None else None for g in m.groups())
                self._queue.put(
                    ("data", (time.time(), rpm, iph_ma, iq_ma, ibus_ma, vbus)))
        try:
            self._ser.close()
        except (serial.SerialException, OSError):
            pass

    def send(self, text: str):
        """보드로 ASCII 명령 한 줄 전송 (다른 스레드에서 호출해도 안전)."""
        self._ser.write(text.encode("ascii"))

    def stop(self):
        self._stop_evt.set()


class StripChart(tk.Canvas):
    """RPM(우축, 항상 표시)과 선택한 비교 신호(좌축) 두 개를 겹쳐 그리는 차트.

    상단 범례에서 RPM 외 신호 하나를 클릭해 선택하면 좌측 Y축에 그 신호의
    눈금이 표시된다. 토크는 0 기준선(점선) 중심의 ±대칭 스케일로 구동(양수)은
    녹색, 제동(음수)은 빨간색. 브레이크력은 0~BRK_SCALE_MAX(20gf) 고정 범위에
    크기(|F|)로 그리고 제동 구간만 빨간색으로 구분한다.
    """

    PAD_L, PAD_R, PAD_T, PAD_B = 62, 62, 30, 22
    DRIVE_COLOR, BRAKE_COLOR = "#2ca02c", "#d62728"
    RPM_COLOR = "#1f77b4"
    BRK_SCALE_MAX = 20  # 브레이크력 고정 표시 범위 0~20 gf (|F| 기준, 적색=제동)
    IBUS_TURN_EPS_A = 0.05  # 버스전류 하강→상승 전환점 판정 히스테리시스 (A)
    # key, 라벨, 색, 단위, 부호(±대칭) 여부, 최소 스케일
    SERIES = (
        ("rpm",  "RPM",      "#1f77b4", "rpm",  False, 1000),
        ("trq",  "토크",      "#2ca02c", "mN·m", True,  10),
        ("brk",  "브레이크력", "#9467bd", "gf",   False, 20),
        ("iph",  "상전류",    "#ff7f0e", "A",    False, 1),
        ("ibus", "버스전류",  "#8c564b", "A",    False, 1),
        ("vbus", "버스전압",  "#7f7f7f", "V",    False, 15),
    )

    def __init__(self, master, **kw):
        super().__init__(master, bg="white", highlightthickness=1,
                         highlightbackground="#cccccc", **kw)
        self.samples = []  # dict: t, rpm, trq, brk, iph, ibus, vbus
        self.selected = "trq"  # 좌축에 표시할 비교 신호 (RPM은 우축 고정)
        self.bind("<Configure>", lambda e: self.redraw())
        self.bind("<Button-1>", self._on_click)
        self._legend_boxes = []  # (x_min, x_max, y_min, y_max, key)

    def add(self, sample: dict):
        self.samples.append(sample)
        if len(self.samples) > MAX_POINTS:
            del self.samples[: len(self.samples) - MAX_POINTS]
        self.redraw()

    def clear(self):
        self.samples = []
        self.redraw()

    def _on_click(self, event):
        for xa, xb, ya, yb, key in self._legend_boxes:
            if xa <= event.x <= xb and ya <= event.y <= yb:
                if key != "rpm":  # RPM은 항상 표시, 선택 대상 아님
                    self.selected = key
                    self.redraw()
                return

    def _series_def(self, key):
        return next(s for s in self.SERIES if s[0] == key)

    @classmethod
    def _falling_to_rising(cls, vals):
        """하강하다 상승으로 전환되는 지점(극소점)의 인덱스 목록.

        노이즈 오검출을 막기 위해 IBUS_TURN_EPS_A 이상 반대로 움직여야
        방향 전환으로 인정한다 (히스테리시스).
        """
        marks = []
        dirn = 0          # +1 상승, -1 하강, 0 미정
        ext_v, ext_i = vals[0], 0
        for i, v in enumerate(vals):
            if dirn >= 0:                     # 상승/미정: 최대값 추적
                if v >= ext_v:
                    ext_v, ext_i = v, i
                elif ext_v - v >= cls.IBUS_TURN_EPS_A:
                    dirn = -1                 # 하강 시작
                    ext_v, ext_i = v, i
            else:                             # 하강: 최소값 추적
                if v <= ext_v:
                    ext_v, ext_i = v, i
                elif v - ext_v >= cls.IBUS_TURN_EPS_A:
                    marks.append(ext_i)       # 하강 → 상승 전환점
                    dirn = 1
                    ext_v, ext_i = v, i
        return marks

    @staticmethod
    def _nice_max(value, minimum):
        v = max(value, minimum)
        step = 10 ** max(len(str(int(v))) - 2, 0)
        return ((int(v) // step) + 1) * step

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        x0, x1 = self.PAD_L, w - self.PAD_R
        y0, y1 = self.PAD_T, h - self.PAD_B
        if x1 <= x0 or y1 <= y0:
            return

        # 신호별 스케일 계산 (브레이크력은 0~BRK_SCALE_MAX 고정)
        scales = {}
        for key, _label, _color, _unit, signed, min_scale in self.SERIES:
            if key == "brk":
                scales[key] = self.BRK_SCALE_MAX
                continue
            vals = [abs(s[key]) for s in self.samples if s.get(key) is not None]
            scales[key] = self._nice_max(max(vals, default=0), min_scale)

        sel = self.selected
        _, sel_label, sel_color, sel_unit, sel_signed, _ = self._series_def(sel)

        # 범례: RPM은 항상 켜짐(고정), 나머지 5개 중 클릭한 하나만 선택
        self._legend_boxes = []
        lx = x0 + 4
        for key, label, color, unit, _signed, _min in self.SERIES:
            on = (key == "rpm") or (key == sel)
            if key == "brk":
                text = f"■ {label} (0~{scales[key]:g}{unit})"
            else:
                text = f"■ {label} (≤{scales[key]:g}{unit})"
            if key == "rpm":
                text += " [우축]"
            elif key == sel:
                text += " [좌축]"
            item = self.create_text(
                lx, self.PAD_T / 2, anchor="w",
                fill=(color if on else "#bbbbbb"),
                text=text, font=("TkDefaultFont", 9, "bold" if on else "normal"))
            xa, ya, xb, yb = self.bbox(item)
            self._legend_boxes.append((xa, xb, ya, yb, key))
            lx = xb + 14

        # 격자 + 양쪽 축 눈금 (좌: 선택 신호, 우: RPM)
        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            self.create_line(x0, y, x1, y, fill="#eeeeee")
            if sel_signed:
                tick = scales[sel] * (2 - i) / 2   # +max .. 0 .. -max
                tick_txt = f"{tick:+g}" if tick else "0"
            else:
                tick = scales[sel] * (4 - i) / 4   # max .. 0
                tick_txt = f"{tick:g}"
            self.create_text(x0 - 6, y, anchor="e", fill=sel_color,
                             text=tick_txt, font=("TkDefaultFont", 8))
            self.create_text(x1 + 6, y, anchor="w", fill=self.RPM_COLOR,
                             text=f"{int(scales['rpm'] * (4 - i) / 4)}",
                             font=("TkDefaultFont", 8))
        y_mid = (y0 + y1) / 2
        if sel_signed:
            self.create_line(x0, y_mid, x1, y_mid, fill="#999999", dash=(4, 3))
        self.create_text((x0 + x1) / 2, h - 6, fill="#888888",
                         text=f"최근 {HISTORY_SEC}초 — 좌축: {sel_label}({sel_unit}), "
                              f"우축: RPM. 범례를 클릭해 좌축 신호를 선택하세요",
                         font=("TkDefaultFont", 8))
        self.create_rectangle(x0, y0, x1, y1, outline="#bbbbbb")

        if len(self.samples) < 2:
            return
        t_end = self.samples[-1]["t"]
        t_start = t_end - HISTORY_SEC

        def to_x(t):
            return x0 + (x1 - x0) * (t - t_start) / HISTORY_SEC

        half = (y1 - y0) / 2
        for key in ("rpm", sel):
            _, _label, color, _unit, signed, _min = self._series_def(key)
            vmax = scales[key]
            pts = []
            for s in self.samples:
                if s["t"] < t_start or s.get(key) is None:
                    continue
                v = s[key]
                if signed:
                    y = y_mid - half * max(min(v / vmax, 1.0), -1.0)
                elif key == "brk":  # 크기를 0~고정범위로, 부호는 색으로 구분
                    y = y1 - (y1 - y0) * min(abs(v) / vmax, 1.0)
                else:
                    y = y1 - (y1 - y0) * min(max(v, 0) / vmax, 1.0)
                pts.append((to_x(s["t"]), y, v))
            if len(pts) < 2:
                continue
            if key == "trq":  # 토크: 부호에 따라 녹/적 구간 분리
                for (xa, ya, va), (xb, yb, vb) in zip(pts, pts[1:]):
                    seg = self.BRAKE_COLOR if (va + vb) / 2 < 0 else self.DRIVE_COLOR
                    self.create_line(xa, ya, xb, yb, fill=seg, width=2)
            elif key == "brk":  # 브레이크력: 제동(음수)은 적색, 구동은 보라색
                for (xa, ya, va), (xb, yb, vb) in zip(pts, pts[1:]):
                    seg = self.BRAKE_COLOR if (va + vb) / 2 < 0 else color
                    self.create_line(xa, ya, xb, yb, fill=seg, width=2)
            else:
                self.create_line(*[c for p in pts for c in (p[0], p[1])],
                                 fill=color, width=2)
                if key == "ibus":  # 하강→상승 전환점(극소점)에 빨간 점 표시
                    for i in self._falling_to_rising([p[2] for p in pts]):
                        x, y = pts[i][0], pts[i][1]
                        self.create_oval(x - 4, y - 4, x + 4, y + 4,
                                         fill=self.BRAKE_COLOR, outline="")


class ProfileEditor(tk.Toplevel):
    """커스텀 캐스팅 프로파일 에디터 — 도형 점 편집 방식.

    캔버스의 빈 곳을 클릭하면 점 추가, 점을 드래그하면 이동, 점을 우클릭하면
    삭제된다(최소 2점 유지). 곡선은 점 사이 직선 보간(RPM-시간)이며 [실행]을
    누르면 100 ms마다 SPD 명령으로 보드에 스트리밍된다. JSON으로 저장/불러오기.
    """

    PAD_L, PAD_R, PAD_T, PAD_B = 56, 16, 16, 34
    HIT_PX = 9  # 점 선택 판정 반경(픽셀)
    DEFAULT_POINTS = [(0.0, 1200), (0.6, 8000), (1.2, 7500),
                      (4.0, 2500), (6.0, 1200)]

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("캐스팅 프로파일 에디터")
        self.geometry("760x440")
        self.minsize(560, 340)
        self.points = list(self.DEFAULT_POINTS)
        self.drag_idx = None
        self.file_name = None

        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=8, pady=6)
        ttk.Button(bar, text="▶ 실행", command=self._run).pack(side="left")
        ttk.Button(bar, text="■ 정지(다이얼 복귀)",
                   command=lambda: self.app.stop_profile(send_dial=True)
                   ).pack(side="left", padx=4)
        ttk.Button(bar, text="저장", command=self._save).pack(side="left", padx=(12, 0))
        ttk.Button(bar, text="불러오기", command=self._load).pack(side="left", padx=4)
        ttk.Button(bar, text="초기화",
                   command=self._reset).pack(side="left", padx=(12, 0))
        self.info = tk.StringVar(value="클릭=점 추가, 드래그=이동, 우클릭=삭제")
        ttk.Label(bar, textvariable=self.info).pack(side="right")

        self.canvas = tk.Canvas(self, bg="white", highlightthickness=1,
                                highlightbackground="#cccccc")
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.canvas.bind("<Configure>", lambda e: self.redraw())
        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda e: self._on_release())
        self.canvas.bind("<Button-3>", self._on_rclick)

    # ---- 좌표 변환 ----
    def _plot_rect(self):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        return self.PAD_L, w - self.PAD_R, self.PAD_T, h - self.PAD_B

    def _to_xy(self, t, rpm):
        x0, x1, y0, y1 = self._plot_rect()
        x = x0 + (x1 - x0) * t / PROFILE_MAX_SEC
        y = y1 - (y1 - y0) * rpm / PROFILE_MAX_RPM
        return x, y

    def _to_trpm(self, x, y):
        x0, x1, y0, y1 = self._plot_rect()
        t = (x - x0) / max(x1 - x0, 1) * PROFILE_MAX_SEC
        rpm = (y1 - y) / max(y1 - y0, 1) * PROFILE_MAX_RPM
        return (max(0.0, min(t, PROFILE_MAX_SEC)),
                max(0, min(int(round(rpm)), PROFILE_MAX_RPM)))

    def _hit(self, x, y):
        for i, (t, rpm) in enumerate(self.points):
            px, py = self._to_xy(t, rpm)
            if abs(px - x) <= self.HIT_PX and abs(py - y) <= self.HIT_PX:
                return i
        return None

    # ---- 마우스 편집 ----
    def _on_press(self, ev):
        idx = self._hit(ev.x, ev.y)
        if idx is None:
            t, rpm = self._to_trpm(ev.x, ev.y)
            self.points.append((t, rpm))
            self.points.sort()
            idx = self.points.index((t, rpm))
        self.drag_idx = idx
        self.redraw()

    def _on_drag(self, ev):
        if self.drag_idx is None:
            return
        t, rpm = self._to_trpm(ev.x, ev.y)
        i = self.drag_idx
        # 시간축은 이웃 점 사이로 제한해 순서를 유지
        if i > 0:
            t = max(t, self.points[i - 1][0] + 0.05)
        if i < len(self.points) - 1:
            t = min(t, self.points[i + 1][0] - 0.05)
        self.points[i] = (t, rpm)
        self.redraw()

    def _on_release(self):
        self.drag_idx = None

    def _on_rclick(self, ev):
        idx = self._hit(ev.x, ev.y)
        if idx is not None and len(self.points) > 2:
            del self.points[idx]
            self.redraw()

    # ---- 파일 ----
    def _save(self):
        os.makedirs(PROFILE_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(
            parent=self, defaultextension=".json", initialdir=PROFILE_DIR,
            filetypes=[("캐스팅 프로파일 JSON", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"points": self.points}, f, indent=2)
        self.file_name = os.path.basename(path)
        self.info.set(f"저장됨: {self.file_name}")

    def _load(self):
        os.makedirs(PROFILE_DIR, exist_ok=True)
        path = filedialog.askopenfilename(
            parent=self, initialdir=PROFILE_DIR,
            filetypes=[("캐스팅 프로파일 JSON", "*.json")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            pts = sorted((float(t), int(r)) for t, r in data["points"])
            if len(pts) < 2:
                raise ValueError("점이 2개 미만")
        except (OSError, ValueError, KeyError, TypeError) as e:
            messagebox.showerror("불러오기 실패", f"{path}\n{e}", parent=self)
            return
        self.points = pts
        self.file_name = os.path.basename(path)
        self.info.set(f"불러옴: {self.file_name}")
        self.redraw()

    def _reset(self):
        self.points = list(self.DEFAULT_POINTS)
        self.redraw()

    def _run(self):
        self.app.play_profile(self.points)

    # ---- 그리기 ----
    def redraw(self):
        c = self.canvas
        c.delete("all")
        x0, x1, y0, y1 = self._plot_rect()
        if x1 <= x0 or y1 <= y0:
            return
        for i in range(int(PROFILE_MAX_SEC) + 1):  # 1초 격자
            x, _ = self._to_xy(i, 0)
            c.create_line(x, y0, x, y1, fill="#eeeeee")
            c.create_text(x, y1 + 12, text=f"{i}s", fill="#888888",
                          font=("TkDefaultFont", 8))
        for rpm in range(0, PROFILE_MAX_RPM + 1, 1000):  # 1000rpm 격자
            _, y = self._to_xy(0, rpm)
            c.create_line(x0, y, x1, y, fill="#eeeeee")
            c.create_text(x0 - 6, y, anchor="e", text=f"{rpm}",
                          fill="#888888", font=("TkDefaultFont", 8))
        # 펌웨어 하한(1200rpm) 표시
        _, y_min = self._to_xy(0, PROFILE_MIN_RPM)
        c.create_line(x0, y_min, x1, y_min, fill="#d62728", dash=(3, 3))
        c.create_rectangle(x0, y0, x1, y1, outline="#bbbbbb")

        pts_xy = [self._to_xy(t, r) for t, r in self.points]
        if len(pts_xy) >= 2:
            c.create_line(*[v for p in pts_xy for v in p],
                          fill="#1f77b4", width=2)
        for i, (x, y) in enumerate(pts_xy):
            color = "#d62728" if i == self.drag_idx else "#1f77b4"
            c.create_rectangle(x - 4, y - 4, x + 4, y + 4,
                               fill="white", outline=color, width=2)
            t, rpm = self.points[i]
            c.create_text(x, y - 12, text=f"{t:.1f}s/{rpm}",
                          fill="#555555", font=("TkDefaultFont", 7))


class MonitorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("B-G431B-ESC1 텔레메트리 모니터")
        self.geometry("860x600")
        self.minsize(640, 460)

        self.reader = None
        self.rx_queue = queue.Queue()
        # 브레이크 0점 보정: 무부하 손실 토크(P/ω가 마찰·철손까지 잡음)를 빼는 오프셋
        self.brk_offset_mnm = 0.0
        self._last_trq_mnm = None
        # 커스텀 프로파일 재생 상태
        self._prof_job = None
        self._prof_points = None
        self._prof_t0 = 0.0
        self.editor = None

        self._build_toolbar()
        self._build_castbar()
        self._build_readouts()
        self.chart = StripChart(self)
        self.chart.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.status = tk.StringVar(value="연결 대기 — IQ 음수는 제동, 양수는 구동입니다")
        ttk.Label(self, textvariable=self.status, anchor="w").pack(
            fill="x", padx=10, pady=(0, 6))

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._poll_queue)

    def _build_toolbar(self):
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=10, pady=8)
        ttk.Label(bar, text="포트:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var,
                                       width=28, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(bar, text="새로고침", command=self._refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(bar, text="연결", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=8)
        ttk.Button(bar, text="그래프 지우기",
                   command=lambda: self.chart.clear()).pack(side="left")
        ttk.Label(bar, text="   작용 반경(mm):").pack(side="left")
        self.radius_var = tk.StringVar(value=f"{DEFAULT_RADIUS_MM:g}")
        ttk.Entry(bar, textvariable=self.radius_var, width=6).pack(side="left")
        self._refresh_ports()

    def _build_castbar(self):
        bar = ttk.LabelFrame(
            self, text="캐스팅 시뮬레이션 — 노멀 모드에서는 보드 다이얼로 속도 조절")
        bar.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Button(bar, text="살살 던지기", width=12,
                   command=lambda: self._send_cmd("CAST1")).pack(
                       side="left", padx=(8, 2), pady=4)
        ttk.Button(bar, text="세게 멀리 던지기", width=14,
                   command=lambda: self._send_cmd("CAST2")).pack(
                       side="left", padx=2, pady=4)
        ttk.Button(bar, text="커스텀 프로파일…", width=15,
                   command=self._open_editor).pack(side="left", padx=(12, 2), pady=4)
        ttk.Button(bar, text="모터 정지", width=9,
                   command=self._cmd_stop).pack(side="right", padx=(2, 8), pady=4)
        ttk.Button(bar, text="모터 시작", width=9,
                   command=lambda: self._send_cmd("START")).pack(
                       side="right", padx=2, pady=4)

    # ---- 보드 명령 / 캐스팅 ----
    def _send_cmd(self, cmd, quiet=False):
        if self.reader is None:
            self.status.set("보드 미연결 — 먼저 [연결]을 누르세요.")
            return False
        try:
            self.reader.send(cmd + "\r\n")
        except (serial.SerialException, OSError):
            self._disconnect("명령 전송 실패 — 연결이 끊어졌습니다.")
            return False
        if not quiet:
            self.status.set(f"명령 전송: {cmd}")
        return True

    def _cmd_stop(self):
        self.stop_profile()
        self._send_cmd("STOP")

    def _open_editor(self):
        if self.editor is not None and self.editor.winfo_exists():
            self.editor.lift()
            self.editor.focus_set()
        else:
            self.editor = ProfileEditor(self)

    def play_profile(self, points):
        """(t초, rpm) 점 목록을 선형 보간해 100ms마다 SPD로 스트리밍."""
        pts = sorted(points)
        if len(pts) < 2:
            self.status.set("프로파일에 점이 2개 이상 필요합니다.")
            return
        if self.reader is None:
            self.status.set("보드 미연결 — 먼저 [연결]을 누르세요.")
            return
        self.stop_profile()
        self._prof_points = pts
        self._prof_t0 = time.time()
        self._profile_tick()

    def _profile_tick(self):
        pts = self._prof_points
        t = time.time() - self._prof_t0
        if t > pts[-1][0]:  # 재생 완료 → 다이얼 제어 복귀
            self._prof_job = None
            self._send_cmd("DIAL", quiet=True)
            self.status.set("프로파일 재생 완료 — 다이얼 제어로 복귀")
            return
        rpm = pts[0][1]
        for (ta, ra), (tb, rb) in zip(pts, pts[1:]):
            if t <= ta:
                break
            if t <= tb:
                rpm = ra + (rb - ra) * (t - ta) / max(tb - ta, 1e-6)
                break
            rpm = rb
        if not self._send_cmd(f"SPD {int(round(rpm))}", quiet=True):
            self._prof_job = None
            return
        self.status.set(
            f"프로파일 재생 중 {t:.1f}/{pts[-1][0]:.1f}s — SPD {int(round(rpm))}")
        self._prof_job = self.after(PROFILE_STEP_MS, self._profile_tick)

    def stop_profile(self, send_dial=False):
        if self._prof_job is not None:
            self.after_cancel(self._prof_job)
            self._prof_job = None
            self.status.set("프로파일 재생 중단")
        if send_dial:
            self._send_cmd("DIAL")

    def _build_readouts(self):
        frame = ttk.Frame(self)
        frame.pack(fill="x", padx=10, pady=(0, 8))
        self.values = {}
        specs = (("RPM", "rpm", "#1f77b4"), ("상전류", "A", "#2ca02c"),
                 ("토크", "mN·m", "#d62728"), ("브레이크력", "gf", "#9467bd"),
                 ("버스전류", "A", "#8c564b"), ("버스전압", "V", "#7f7f7f"))
        for i, (name, unit, color) in enumerate(specs):
            row, col = divmod(i, 3)
            box = ttk.LabelFrame(frame, text=f"{name} [{unit}]")
            box.grid(row=row, column=col, sticky="nsew", padx=4, pady=2)
            frame.columnconfigure(col, weight=1)
            var = tk.StringVar(value="--")
            tk.Label(box, textvariable=var, font=("Consolas", 20, "bold"),
                     fg=color).pack(side="left", expand=True, padx=8, pady=2)
            self.values[name] = var
            if name == "브레이크력":
                btns = ttk.Frame(box)
                btns.pack(side="right", padx=4)
                ttk.Button(btns, text="캘리브레이션", width=11,
                           command=self._calibrate_brake).pack(pady=1)
                ttk.Button(btns, text="보정 해제", width=11,
                           command=self._clear_brake_cal).pack(pady=1)

    def _calibrate_brake(self):
        """현재(무부하) 토크를 0점으로 저장 — 이후 브레이크력에서 차감."""
        if self._last_trq_mnm is None:
            self.status.set("캘리브레이션 실패 — 토크 값이 아직 수신되지 않았습니다.")
            return
        self.brk_offset_mnm = self._last_trq_mnm
        self.status.set(
            f"브레이크 0점 보정 적용: 오프셋 {self.brk_offset_mnm:+.1f} mN·m "
            "(무부하 상태에서 누르세요)")

    def _clear_brake_cal(self):
        self.brk_offset_mnm = 0.0
        self.status.set("브레이크 0점 보정 해제됨")

    def _radius_mm(self):
        try:
            r = float(self.radius_var.get())
            return r if r > 0 else None
        except ValueError:
            return None

    def _refresh_ports(self):
        ports = [f"{p.device} - {p.description}" for p in list_ports.comports()]
        self.port_combo["values"] = ports
        stlink = next((p for p in ports if "STLink" in p or "STM32" in p), None)
        if stlink:
            self.port_var.set(stlink)
        elif ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.reader is not None:
            self._disconnect("연결 해제됨")
            return
        sel = self.port_var.get()
        if not sel:
            messagebox.showwarning("포트 없음", "연결할 COM 포트를 선택하세요.")
            return
        port = sel.split(" - ")[0]
        try:
            self.reader = SerialReader(port, self.rx_queue)
        except (serial.SerialException, OSError) as e:
            messagebox.showerror(
                "연결 실패",
                f"{port} 열기 실패:\n{e}\n\n다른 터미널이 포트를 사용 중인지 확인하세요.")
            return
        self.reader.start()
        self.conn_btn.config(text="해제")
        self.status.set(f"{port} 연결됨 ({BAUDRATE}bps) — 데이터 수신 대기")

    def _disconnect(self, msg):
        self.stop_profile()
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.conn_btn.config(text="연결")
        self.status.set(msg)

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "data":
                    t, rpm, iph_ma, iq_ma, ibus_ma, vbus = payload
                    self.values["RPM"].set(f"{rpm}")
                    self.values["상전류"].set(f"{iph_ma / 1000:.2f}")
                    self.values["버스전류"].set(f"{ibus_ma / 1000:.2f}")
                    self.values["버스전압"].set(f"{vbus}")

                    # T = P/ω : 전기 입력 전력(Vbus x Ibus)을 기계 각속도로 나눔
                    torque_mnm = None
                    force_gf = None
                    if abs(rpm) >= MIN_RPM_FOR_TORQUE:
                        omega = rpm * 2 * math.pi / 60          # rad/s
                        power_w = vbus * ibus_ma / 1000          # W
                        torque_mnm = 1000 * power_w / omega
                        self._last_trq_mnm = torque_mnm
                        self.values["토크"].set(f"{torque_mnm:+.1f}")
                        r = self._radius_mm()
                        if r is not None:
                            # 0점 보정(무부하 손실 토크) 차감 후 힘으로 환산.
                            # mN·m == N·mm 이므로 F[N] = T[mN·m]/r[mm]
                            trq_corr = torque_mnm - self.brk_offset_mnm
                            mode = "제동" if trq_corr < 0 else "구동"
                            force_gf = trq_corr / r * GF_PER_N
                            self.values["브레이크력"].set(f"{force_gf:+.0f} {mode}")
                        else:
                            self.values["브레이크력"].set("반경?")
                    else:
                        self._last_trq_mnm = None
                        self.values["토크"].set("--")
                        self.values["브레이크력"].set("--")

                    self.chart.add({
                        "t": t, "rpm": rpm,
                        "trq": torque_mnm,
                        "brk": force_gf,
                        "iph": iph_ma / 1000, "ibus": ibus_ma / 1000,
                        "vbus": vbus,
                    })
                    self.status.set(
                        f"수신 중 — RPM={rpm} IPH={iph_ma}mA IQ={iq_ma}mA "
                        f"IBUS={ibus_ma}mA VBUS={vbus}V")
                elif kind == "error":
                    self._disconnect(payload)
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    def _on_close(self):
        self._disconnect("종료")
        self.destroy()


if __name__ == "__main__":
    MonitorApp().mainloop()
