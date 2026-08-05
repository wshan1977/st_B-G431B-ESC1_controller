#!/usr/bin/env python3
"""B-G431B-ESC1 텔레메트리 모니터.

보드 USB(ST-LINK 가상 COM 포트, 115200 8N1)로 들어오는
"RPM=5010 IPH=1836mA IQ=-1830mA IBUS=250mA VBUS=12V" 형식의 라인을 파싱해
RPM / 전류 / 토크 / 브레이크력 / 버스전압을 실시간 표시하고 그래프로 그린다.

토크는 모터 상수(0.82 Vrms ph-ph/kRPM, pmsm_motor_parameters.h)로부터
T = Kt x Iq 로 환산하고, 브레이크력은 입력한 작용 반경으로 F = T / r 환산한다.
Iq(토크 전류)가 음수이면 제동(브레이크), 양수이면 구동 상태다.

의존성: pyserial  (pip install pyserial)
실행:   python telemetry_monitor.py
"""

import math
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports

BAUDRATE = 115200
LINE_RE = re.compile(
    r"RPM=(-?\d+)\s+IPH=(-?\d+)mA(?:\s+IQ=(-?\d+)mA)?\s+IBUS=(-?\d+)mA\s+VBUS=(\d+)V"
)
HISTORY_SEC = 60          # 그래프에 유지할 시간 범위
SAMPLE_PERIOD = 0.1       # 펌웨어 전송 주기 (100 ms)
MAX_POINTS = int(HISTORY_SEC / SAMPLE_PERIOD)

# 토크 상수 [mN·m / A(peak)] — 역기전력 상수 0.82 Vrms(선간)/kRPM 에서 유도:
#   p·λ = 0.82 / 104.72(rad/s per kRPM) / √3(선간→상) × √2(rms→피크)
#   Kt  = 1.5 × p·λ  →  약 9.59 mN·m/A
KE_VRMS_LL_PER_KRPM = 0.82
KT_MNM_PER_A = 1.5 * (KE_VRMS_LL_PER_KRPM / (1000 * 2 * math.pi / 60)
                      / math.sqrt(3) * math.sqrt(2)) * 1000
DEFAULT_RADIUS_MM = 30.0  # 브레이크력 환산용 작용 반경 기본값
GF_PER_N = 1000 / 9.80665


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

    def stop(self):
        self._stop_evt.set()


class StripChart(tk.Canvas):
    """여러 신호를 한 그래프에 합쳐 그리는 스트립 차트.

    상단 범례를 클릭해 신호를 켜고 끌 수 있다. 각 신호는 자기 최대값으로
    독립 스케일링해 겹쳐 그리고, 현재 스케일은 범례에 표시한다.
    토크는 0 기준선(점선) 중심의 ±대칭 스케일로, 구동(양수)은 녹색,
    제동(음수)은 빨간색으로 구간별 색을 바꿔 그린다.
    """

    PAD_L, PAD_R, PAD_T, PAD_B = 8, 8, 30, 22
    DRIVE_COLOR, BRAKE_COLOR = "#2ca02c", "#d62728"
    # key, 라벨, 색, 단위, 부호(±대칭) 여부, 최소 스케일
    SERIES = (
        ("rpm",  "RPM",      "#1f77b4", "rpm",  False, 1000),
        ("trq",  "토크",      "#2ca02c", "mN·m", True,  10),
        ("brk",  "브레이크력", "#9467bd", "gf",   True,  50),
        ("iph",  "상전류",    "#ff7f0e", "A",    False, 1),
        ("ibus", "버스전류",  "#8c564b", "A",    False, 1),
        ("vbus", "버스전압",  "#7f7f7f", "V",    False, 15),
    )

    def __init__(self, master, **kw):
        super().__init__(master, bg="white", highlightthickness=1,
                         highlightbackground="#cccccc", **kw)
        self.samples = []  # dict: t, rpm, trq, iph, ibus, vbus
        self.visible = {"rpm", "trq", "brk"}
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
                if key in self.visible:
                    self.visible.discard(key)
                else:
                    self.visible.add(key)
                self.redraw()
                return

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

        # 신호별 자동 스케일 계산
        scales = {}
        for key, _label, _color, _unit, signed, min_scale in self.SERIES:
            vals = [abs(s[key]) for s in self.samples if s.get(key) is not None]
            scales[key] = self._nice_max(max(vals, default=0), min_scale)

        # 클릭 가능한 범례 (켜짐: ■ 색상, 꺼짐: 회색 취소선)
        self._legend_boxes = []
        lx = x0 + 4
        for key, label, color, unit, _signed, _min in self.SERIES:
            on = key in self.visible
            text = f"■ {label} (≤{scales[key]:g}{unit})"
            item = self.create_text(
                lx, self.PAD_T / 2, anchor="w",
                fill=(color if on else "#bbbbbb"),
                text=text, font=("TkDefaultFont", 9, "bold" if on else "overstrike"))
            xa, ya, xb, yb = self.bbox(item)
            self._legend_boxes.append((xa, xb, ya, yb, key))
            lx = xb + 14

        # 격자
        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            self.create_line(x0, y, x1, y, fill="#eeeeee")
        y_mid = (y0 + y1) / 2
        if self.visible & {"trq", "brk"}:
            self.create_line(x0, y_mid, x1, y_mid, fill="#999999", dash=(4, 3))
        self.create_text((x0 + x1) / 2, h - 6, fill="#888888",
                         text=f"최근 {HISTORY_SEC}초 — 범례를 클릭하면 신호를 켜고 끕니다"
                              " (토크: 녹=구동, 적=제동)",
                         font=("TkDefaultFont", 8))
        self.create_rectangle(x0, y0, x1, y1, outline="#bbbbbb")

        if len(self.samples) < 2:
            return
        t_end = self.samples[-1]["t"]
        t_start = t_end - HISTORY_SEC

        def to_x(t):
            return x0 + (x1 - x0) * (t - t_start) / HISTORY_SEC

        half = (y1 - y0) / 2
        for key, _label, color, _unit, signed, _min in self.SERIES:
            if key not in self.visible:
                continue
            vmax = scales[key]
            pts = []
            for s in self.samples:
                if s["t"] < t_start or s.get(key) is None:
                    continue
                v = s[key]
                if signed:
                    y = y_mid - half * max(min(v / vmax, 1.0), -1.0)
                else:
                    y = y1 - (y1 - y0) * min(max(v, 0) / vmax, 1.0)
                pts.append((to_x(s["t"]), y, v))
            if len(pts) < 2:
                continue
            if key == "trq":  # 토크: 부호에 따라 녹/적 구간 분리
                for (xa, ya, va), (xb, yb, vb) in zip(pts, pts[1:]):
                    seg = self.BRAKE_COLOR if (va + vb) / 2 < 0 else self.DRIVE_COLOR
                    self.create_line(xa, ya, xb, yb, fill=seg, width=2)
            else:
                self.create_line(*[c for p in pts for c in (p[0], p[1])],
                                 fill=color, width=2)


class MonitorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("B-G431B-ESC1 텔레메트리 모니터")
        self.geometry("860x600")
        self.minsize(640, 460)

        self.reader = None
        self.rx_queue = queue.Queue()

        self._build_toolbar()
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
                     fg=color).pack(padx=8, pady=2)
            self.values[name] = var

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

                    torque_mnm = None
                    force_gf = None
                    if iq_ma is not None:
                        torque_mnm = KT_MNM_PER_A * iq_ma / 1000
                        mode = "제동" if torque_mnm < 0 else "구동"
                        self.values["토크"].set(f"{torque_mnm:+.1f}")
                        r = self._radius_mm()
                        if r is not None:
                            # mN·m == N·mm 이므로 F[N] = T[mN·m]/r[mm]
                            force_gf = torque_mnm / r * GF_PER_N
                            self.values["브레이크력"].set(f"{force_gf:+.0f} {mode}")
                        else:
                            self.values["브레이크력"].set("반경?")
                    else:
                        self.values["토크"].set("--")
                        self.values["브레이크력"].set("--")

                    self.chart.add({
                        "t": t, "rpm": rpm,
                        "trq": torque_mnm if torque_mnm is not None else 0,
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
