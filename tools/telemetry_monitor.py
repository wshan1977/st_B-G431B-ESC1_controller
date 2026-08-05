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
    """RPM(좌축)과 |토크|(우축)를 함께 그리는 스트립 차트."""

    PAD_L, PAD_R, PAD_T, PAD_B = 56, 64, 10, 22
    RPM_COLOR, TRQ_COLOR = "#1f77b4", "#d62728"

    def __init__(self, master, **kw):
        super().__init__(master, bg="white", highlightthickness=1,
                         highlightbackground="#cccccc", **kw)
        self.samples = []  # (t, rpm, torque_mNm_abs)
        self.bind("<Configure>", lambda e: self.redraw())

    def add(self, t, rpm, torque_abs):
        self.samples.append((t, rpm, torque_abs))
        if len(self.samples) > MAX_POINTS:
            del self.samples[: len(self.samples) - MAX_POINTS]
        self.redraw()

    def clear(self):
        self.samples = []
        self.redraw()

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

        rpm_max = self._nice_max(max((s[1] for s in self.samples), default=0), 1000)
        trq_max = self._nice_max(max((s[2] for s in self.samples), default=0), 10)

        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            self.create_line(x0, y, x1, y, fill="#eeeeee")
            self.create_text(x0 - 6, y, anchor="e", fill=self.RPM_COLOR,
                             text=f"{int(rpm_max * (4 - i) / 4)}", font=("TkDefaultFont", 8))
            self.create_text(x1 + 6, y, anchor="w", fill=self.TRQ_COLOR,
                             text=f"{trq_max * (4 - i) / 4:.0f}", font=("TkDefaultFont", 8))
        self.create_text((x0 + x1) / 2, h - 6, fill="#888888",
                         text=f"최근 {HISTORY_SEC}초   (좌: RPM, 우: |토크| mN·m)",
                         font=("TkDefaultFont", 8))
        self.create_rectangle(x0, y0, x1, y1, outline="#bbbbbb")

        if len(self.samples) < 2:
            return
        t_end = self.samples[-1][0]
        t_start = t_end - HISTORY_SEC

        def to_xy(t, val, vmax):
            x = x0 + (x1 - x0) * (t - t_start) / HISTORY_SEC
            y = y1 - (y1 - y0) * min(val / vmax, 1.0)
            return x, y

        for idx, color, vmax in ((1, self.RPM_COLOR, rpm_max),
                                 (2, self.TRQ_COLOR, trq_max)):
            pts = [to_xy(s[0], s[idx], vmax) for s in self.samples if s[0] >= t_start]
            if len(pts) >= 2:
                self.create_line(*[c for p in pts for c in p], fill=color, width=2)


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

                    self.chart.add(t, rpm, abs(torque_mnm) if torque_mnm else 0)
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
