"""UI 자체 테스트: 가짜 텔레메트리를 주입해 차트 렌더링을 스크린샷으로 검증."""
import math
import sys
import time
import traceback

from PIL import ImageGrab

import telemetry_monitor as tm


def main():
    app = tm.MonitorApp()
    t0 = time.time()
    n = [0]

    def inject():
        # 10Hz 가짜 데이터 5초분: RPM 스윕 + 토크 사인파(제동 구간 포함)
        t = time.time()
        rpm = 3000 + int(2000 * math.sin((t - t0) * 1.5))
        iq_ma = int(3000 * math.sin((t - t0) * 2.5))  # 음수 = 제동
        iph_ma = abs(iq_ma) + 50
        ibus_ma = max(iq_ma // 4, -800)
        app.rx_queue.put(("data", (t, rpm, iph_ma, iq_ma, ibus_ma, 12)))
        n[0] += 1
        if n[0] < 50:
            app.after(100, inject)
        else:
            app.after(300, shoot)

    def shoot():
        app.update_idletasks()
        x, y = app.winfo_rootx(), app.winfo_rooty()
        w, h = app.winfo_width(), app.winfo_height()
        ImageGrab.grab((x, y, x + w, y + h)).save("ui_selftest.png")
        app.destroy()

    app.after(200, inject)
    try:
        app.mainloop()
        print("SELFTEST DONE")
    except Exception:
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
