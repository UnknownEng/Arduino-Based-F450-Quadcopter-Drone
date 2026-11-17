"""Render a video of the simulator flying the real flight_controller.ino, with the checks ticking off live.

Usage (after python sim/run_sim.py):   python sim/make_video.py
Needs: matplotlib, numpy and ffmpeg on the PATH. Output: sim/results/flight_tests.mp4
"""
import csv
import os
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
OUT = os.path.join(RES, "flight_tests.mp4")
W, H, FPS = 1280, 720, 20

BG, PANEL, FG, DIM = "#0f1419", "#1a2129", "#e6e6e6", "#8a96a3"
GREEN, RED, BLUE, ORANGE, YELLOW = "#3fb950", "#f85149", "#58a6ff", "#f0883e", "#d29922"
ESC_COLORS = ["#58a6ff", "#3fb950", "#f0883e", "#bc8cff"]

SCENARIOS = [
    ("hover", "Test 1 - Normal flight", 1.5,
     "Arm, take off, hover in gusts, stick inputs, land, disarm",
     ["loop() at 250 Hz, auto-level, calculate_pid()",
      "Motor mixing esc_1..esc_4 (signs and directions)",
      "Arm / disarm stick commands (start == 2)",
      "D-term on measurement (no stick-flick spike)",
      "micros() overflow during flight (at t = 25 s)"]),
    ("failsafe_cut", "Test 2 - Radio signal lost", 1.0,
     "Transmitter switched off while hovering (default settings)",
     ["receiver_signal_ok(): RX_TIMEOUT_US = 100 ms",
      "failsafe_throttle = 1000 -> stop motors",
      "No arming without a signal, manual re-arm only"]),
    ("failsafe_descent", "Test 3 - Signal lost, controlled descent", 1.0,
     "failsafe_throttle set just below hover",
     ["Failsafe forces auto-level (stick input ignored)",
      "Descends at failsafe_throttle",
      "Stops motors after failsafe_descent_ms = 5 s"]),
    ("gyro_fail", "Test 4 - MPU-6050 wire comes loose", 1.0,
     "I2C stops answering in flight",
     ["gyro_signalen(): I2C read checked, Wire timeout",
      "GYRO_MAX_ERRORS = 25 -> gyro_failure, motors stop",
      "LED solid, refuses to re-arm until power cycle"]),
    ("ground_windup", "Test 5 - Armed on a 6 degree slope", 1.0,
     "Sits armed for 10 s on sloped ground, then takes off",
     ["I-term reset while armed with throttle < 1050",
      "Clean take-off and level-out, no flip"]),
    ("rx_glitch", "Test 6 - Noisy receiver", 1.0,
     "300 us noise spikes on every channel + a dropped frame",
     ["ISR accepts only 800-2200 us pulses",
      "One missing 20 ms frame must NOT trigger failsafe"]),
]


def load_csv(name):
    with open(os.path.join(RES, name + ".csv")) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_events(name):
    ev = []
    path = os.path.join(RES, name + ".events.tsv")
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) >= 3:
                ev.append((float(parts[0]), parts[1], parts[2], parts[3] if len(parts) > 3 else ""))
    return ev


class Writer:
    def __init__(self, path):
        self.p = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{W}x{H}",
             "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", path],
            stdin=subprocess.PIPE)
        self.frames = 0

    def write(self, fig, repeat=1):
        fig.canvas.draw()
        buf = bytes(fig.canvas.buffer_rgba())
        for _ in range(repeat):
            self.p.stdin.write(buf)
            self.frames += 1

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def new_fig():
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor=BG)
    return fig


def card(writer, title, subtitle, bullets, seconds, heading=None):
    fig = new_fig()
    if heading:
        fig.text(0.07, 0.86, heading, color=DIM, fontsize=14)
    fig.text(0.07, 0.74, title, color=FG, fontsize=30, weight="bold")
    fig.text(0.07, 0.66, subtitle, color=DIM, fontsize=16)
    if bullets:
        fig.text(0.07, 0.54, "Code under test:", color=BLUE, fontsize=15, weight="bold")
        for i, b in enumerate(bullets):
            fig.text(0.09, 0.47 - i * 0.065, "•  " + b, color=FG, fontsize=15, family="monospace")
    writer.write(fig, int(seconds * FPS))
    plt.close(fig)


def summary_card(writer, results, robust, seconds):
    fig = new_fig()
    fig.text(0.07, 0.88, "Results", color=FG, fontsize=30, weight="bold")
    fig.text(0.07, 0.82, "Real flight_controller.ino, compiled unchanged, flying a simulated F450", color=DIM, fontsize=14)
    y = 0.72
    for title, passed, total in results:
        ok = passed == total
        fig.text(0.08, y, "✔" if ok else "✘", color=GREEN if ok else RED, fontsize=20)
        fig.text(0.12, y, f"{title}", color=FG, fontsize=16)
        fig.text(0.70, y, f"{passed}/{total} checks", color=GREEN if ok else RED, fontsize=16)
        y -= 0.065
    if robust:
        ok = robust[0] == robust[1]
        fig.text(0.08, y, "✔" if ok else "✘", color=GREEN if ok else RED, fontsize=20)
        fig.text(0.12, y, "Robustness: same code on different F450 builds", color=FG, fontsize=16)
        fig.text(0.70, y, f"{robust[0]}/{robust[1]} builds", color=GREEN if ok else RED, fontsize=16)
        y -= 0.065
    fig.text(0.07, y - 0.04, "A simulation proves the logic, signs and safety features - not your real tuning.",
             color=YELLOW, fontsize=13)
    fig.text(0.07, y - 0.09, "On the real drone: follow FIRST_FLIGHT_GUIDE.md step by step (props off for all bench tests).",
             color=YELLOW, fontsize=13)
    writer.write(fig, int(seconds * FPS))
    plt.close(fig)


def draw_quad(ax, angle, left_pulse, right_pulse, alt, title, left_label, right_label):
    ax.clear()
    ax.set_facecolor(PANEL)
    ax.set_xlim(-1.7, 1.7)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#30363d")
    ax.set_title(title, color=DIM, fontsize=11, pad=4)
    # Ground: moves down out of view as the quad climbs.
    gy = -0.45 - alt * 0.5
    if gy > -1.2:
        ax.add_patch(Rectangle((-1.7, -1.2), 3.4, gy + 1.2, color="#3b2f22", zorder=0))
        ax.plot([-1.7, 1.7], [gy, gy], color="#8b6b4a", lw=2, zorder=1)
    a = np.radians(angle)
    rot = lambda x, y: (x * np.cos(a) + y * np.sin(a), -x * np.sin(a) + y * np.cos(a))
    x1, y1 = rot(-1.0, 0.0)
    x2, y2 = rot(1.0, 0.0)
    ax.plot([x1, x2], [y1, y2], color=FG, lw=6, solid_capstyle="round", zorder=3)
    ax.add_patch(Rectangle((-0.22, -0.12), 0.44, 0.24, color="#30363d", zorder=4,
                           transform=matplotlib.transforms.Affine2D().rotate(-a) + ax.transData))
    cmap = plt.get_cmap("inferno")
    for side, pulse in ((-1, left_pulse), (1, right_pulse)):
        mx, my = rot(side * 1.0, 0.16)
        level = np.clip((pulse - 1000) / 1000, 0, 1)
        col = "#444c56" if pulse <= 1000 else cmap(0.35 + 0.6 * level)
        px1, py1 = rot(side * 1.0 - 0.38, 0.2)
        px2, py2 = rot(side * 1.0 + 0.38, 0.2)
        ax.plot([px1, px2], [py1, py2], color=col, lw=5, solid_capstyle="round", zorder=5)
        ax.add_patch(Circle((mx, my), 0.07, color=col, zorder=6))
        ax.text(side * 1.25, 0.85, f"{int(pulse)} us", color=FG, fontsize=10, ha="center")
    ax.text(-1.25, -1.05, left_label, color=DIM, fontsize=9, ha="center")
    ax.text(1.25, -1.05, right_label, color=DIM, fontsize=9, ha="center")
    ax.text(0, 1.0, f"{angle:+.1f}°", color=YELLOW, fontsize=13, ha="center", weight="bold")


def render_scenario(writer, name, title, base_speed):
    d = load_csv(name)
    ev = load_events(name)
    t = d["t"]
    t_start, t_end = t[0], t[-1]
    # "Interesting" moments play at normal speed; boring ground time is fast-forwarded.
    marks = [e[0] for e in ev]
    for k in ("armed", "failsafe", "gyro_failure", "rx_on", "rx_roll", "rx_pitch", "rx_yaw"):
        ch = np.nonzero(np.diff(d[k]))[0]
        marks += list(t[ch + 1])
    marks = np.array(sorted(marks)) if marks else np.array([t_start])

    fig = new_fig()
    hdr = fig.text(0.02, 0.955, title, color=FG, fontsize=18, weight="bold")
    clock = fig.text(0.98, 0.955, "", color=DIM, fontsize=14, ha="right")
    ax_rear = fig.add_axes([0.015, 0.47, 0.235, 0.40])
    ax_side = fig.add_axes([0.26, 0.47, 0.235, 0.40])
    ax_ang = fig.add_axes([0.56, 0.66, 0.42, 0.24], facecolor=PANEL)
    ax_esc = fig.add_axes([0.56, 0.37, 0.42, 0.24], facecolor=PANEL)
    for ax in (ax_ang, ax_esc):
        ax.tick_params(colors=DIM, labelsize=9)
        for s in ax.spines.values():
            s.set_color("#30363d")
        ax.grid(color="#30363d", lw=0.6)
    ax_ang.set_ylabel("angle (deg)", color=DIM, fontsize=10)
    ax_esc.set_ylabel("ESC pulse (us)", color=DIM, fontsize=10)
    ax_ang.set_ylim(-20, 20)
    ax_esc.set_ylim(950, 2050)
    l_roll, = ax_ang.plot([], [], color=BLUE, lw=1.6, label="roll")
    l_pitch, = ax_ang.plot([], [], color=ORANGE, lw=1.6, label="pitch")
    ax_ang.legend(loc="upper left", fontsize=9, facecolor=PANEL, edgecolor="#30363d", labelcolor=FG, ncol=2)
    l_esc = [ax_esc.plot([], [], color=c, lw=1.1, label=f"ESC {i + 1}")[0] for i, c in enumerate(ESC_COLORS)]
    l_thr, = ax_esc.plot([], [], color=DIM, lw=1, ls="--", label="throttle stick")
    ax_esc.legend(loc="upper left", fontsize=8, facecolor=PANEL, edgecolor="#30363d", labelcolor=FG, ncol=5)

    status = fig.text(0.015, 0.405, "", color=FG, fontsize=12, family="monospace")
    status2 = fig.text(0.015, 0.365, "", color=DIM, fontsize=11, family="monospace")
    fig.add_artist(Rectangle((0.01, 0.01), 0.98, 0.335, transform=fig.transFigure, color=PANEL, zorder=-1))
    fig.text(0.02, 0.31, "Checks", color=BLUE, fontsize=13, weight="bold")
    check_texts = [fig.text(0.02, 0.265 - i * 0.042, "", color=FG, fontsize=12) for i in range(6)]
    setup_note = "setup(): EEPROM read, MPU-6050 configured, gyro calibrated (2000 samples), waited for throttle low"

    def frame_at(tc):
        i = min(np.searchsorted(t, tc), len(t) - 1)
        w0 = max(t_start, tc - 10)
        sl = slice(np.searchsorted(t, w0), i + 1)
        for line, key in ((l_roll, "roll"), (l_pitch, "pitch")):
            line.set_data(t[sl], d[key][sl])
        for k, line in enumerate(l_esc):
            line.set_data(t[sl], d[f"esc{k + 1}"][sl])
        l_thr.set_data(t[sl], d["rx_throttle"][sl])
        for ax in (ax_ang, ax_esc):
            ax.set_xlim(w0, max(w0 + 10, tc))
        lo, hi = min(-20, np.min(d["roll"][sl]) - 3, np.min(d["pitch"][sl]) - 3), max(20, np.max(d["roll"][sl]) + 3, np.max(d["pitch"][sl]) + 3)
        ax_ang.set_ylim(max(lo, -90), min(hi, 90))
        e = [d[f"esc{k}"][i] for k in range(1, 5)]
        draw_quad(ax_rear, d["roll"][i], (e[2] + e[3]) / 2, (e[0] + e[1]) / 2, d["alt"][i],
                  "rear view (roll)", "left: ESC 3+4", "right: ESC 1+2")
        draw_quad(ax_side, -d["pitch"][i], (e[1] + e[2]) / 2, (e[0] + e[3]) / 2, d["alt"][i],
                  "side view (pitch, nose right)", "rear: ESC 2+3", "front: ESC 1+4")
        # In the side view the nose is on the right: nose up must rotate counter-clockwise.
        ax_side.texts[-1].set_text(f"{d['pitch'][i]:+.1f}°")
        armed = int(d["armed"][i])
        st = ["DISARMED", "ARMING", "ARMED"][armed] if armed in (0, 1, 2) else str(armed)
        flags = []
        flags.append(("RADIO OK" if d["rx_on"][i] else "RADIO LOST"))
        if d["failsafe"][i]:
            flags.append("FAILSAFE")
        if d["gyro_failure"][i]:
            flags.append("GYRO FAILURE")
        flags.append("LED ON" if d["led"][i] else "LED off")
        status.set_text(f"{st:9}  |  " + "  |  ".join(flags) + f"  |  altitude {d['alt'][i]:.1f} m")
        status.set_color(RED if (d["failsafe"][i] or d["gyro_failure"][i] or not d["rx_on"][i]) else (GREEN if armed == 2 else FG))
        status2.set_text(f"sticks  roll {int(d['rx_roll'][i])}  pitch {int(d['rx_pitch'][i])}  "
                         f"throttle {int(d['rx_throttle'][i])}  yaw {int(d['rx_yaw'][i])}")
        done = [e_ for e_ in ev if e_[0] <= tc + 1e-6]
        lines = [("i", setup_note, "")] + [(e_[1], e_[2], e_[3]) for e_ in done]
        lines = lines[-6:]
        for k, txt in enumerate(check_texts):
            if k < len(lines):
                kind, what, detail = lines[k]
                if kind == "i":
                    txt.set_text("• " + what); txt.set_color(DIM)
                else:
                    sym = "✔" if kind == "PASS" else "✘"
                    txt.set_text(f"{sym} {kind}  {what}" + (f"   ({detail})" if detail else ""))
                    txt.set_color(GREEN if kind == "PASS" else RED)
            else:
                txt.set_text("")
        clock.set_text(f"sim time {tc:6.1f} s")

    tc = t_start
    while tc <= t_end:
        near = np.min(np.abs(marks - tc)) < 1.5 if len(marks) else False
        speed = base_speed if (d["alt"][min(np.searchsorted(t, tc), len(t) - 1)] > 0.02 or near) else 6.0
        frame_at(tc)
        writer.write(fig)
        tc += speed / FPS
    frame_at(t_end)
    writer.write(fig, int(2.5 * FPS))
    plt.close(fig)
    passed = sum(1 for e_ in ev if e_[1] == "PASS")
    return passed, len(ev)


def robust_summary():
    path = os.path.join(RES, "robustness.tsv")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.DictReader(f, delimiter="\t"))
    return sum(r["result"] == "PASS" for r in rows), len(rows)


def main():
    w = Writer(OUT)
    card(w, "F450 Flight Controller", "Simulator test of the real flight_controller.ino (YMFC-AL based)",
         ["The Arduino code is compiled unchanged on the PC",
          "Fake Uno: 16-bit int, 250 Hz loop, micros() overflow",
          "Simulated F450: motor lag, inertia, gusts, vibration",
          "Simulated MPU-6050 and 50 Hz PWM radio receiver"], 5, heading="Software-in-the-loop flight test")
    results = []
    for name, title, speed, sub, bullets in SCENARIOS:
        card(w, title, sub, bullets, 3.5)
        p, n = render_scenario(w, name, title, speed)
        results.append((title, p, n))
        print(f"{title}: {p}/{n} checks, {w.frames / FPS:.0f} s of video so far")
    summary_card(w, results, robust_summary(), 8)
    w.close()
    print(f"Wrote {OUT} ({w.frames / FPS:.0f} s)")


if __name__ == "__main__":
    main()
