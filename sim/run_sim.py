"""Build the flight-controller simulator, run every scenario and plot the flights.

Usage (from the project folder):   python sim/run_sim.py
Needs:  pip install ziglang matplotlib
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
EXE = os.path.join(OUT, "sim.exe" if os.name == "nt" else "sim")
SCENARIOS = ["hover", "failsafe_cut", "failsafe_descent", "gyro_fail", "ground_windup", "rx_glitch"]


def build():
    os.makedirs(OUT, exist_ok=True)
    cmd = [sys.executable, "-m", "ziglang", "c++", "-O2", "-std=gnu++17", "-Wall",
           "-Wno-unused-variable", "-Wno-unused-but-set-variable",
           "-I", os.path.join(HERE, "include"), os.path.join(HERE, "sim.cpp"), "-o", EXE]
    print("Building simulator...")
    subprocess.run(cmd, check=True)


def run():
    failed = []
    for name in SCENARIOS:
        csv = os.path.join(OUT, name + ".csv")
        events = os.path.join(OUT, name + ".events.tsv")
        result = subprocess.run([EXE, name, csv, events], capture_output=True, text=True)
        print(result.stdout, end="")
        if result.returncode != 0:
            failed.append(name)
    return failed


def plot():
    try:
        import csv as csvmod
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plots")
        return
    for name in SCENARIOS:
        path = os.path.join(OUT, name + ".csv")
        with open(path) as f:
            rows = list(csvmod.DictReader(f))
        if not rows:
            continue
        col = lambda k: [float(r[k]) for r in rows]
        t = col("t")
        fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        ax[0].plot(t, col("roll"), label="roll (true)")
        ax[0].plot(t, col("pitch"), label="pitch (true)")
        ax[0].plot(t, col("fc_angle_roll"), "--", lw=0.8, label="roll (flight controller estimate)")
        ax[0].set_ylabel("angle, deg"); ax[0].legend(loc="upper left", fontsize=8); ax[0].grid(alpha=0.3)
        ax[1].plot(t, col("alt"), color="tab:green")
        ax[1].set_ylabel("altitude, m"); ax[1].grid(alpha=0.3)
        for i in range(1, 5):
            ax[2].plot(t, col(f"esc{i}"), lw=0.8, label=f"ESC {i}")
        ax[2].plot(t, col("rx_throttle"), "k--", lw=0.8, label="throttle stick")
        ax[2].set_ylabel("pulse, us"); ax[2].legend(loc="upper left", fontsize=8, ncol=5); ax[2].grid(alpha=0.3)
        ax[3].step(t, col("armed"), label="armed state (2 = armed)")
        ax[3].step(t, col("failsafe"), label="failsafe")
        ax[3].step(t, col("gyro_failure"), label="gyro failure")
        ax[3].step(t, col("rx_on"), ":", label="radio on")
        ax[3].set_ylabel("flags"); ax[3].set_xlabel("time, s"); ax[3].legend(loc="upper left", fontsize=8, ncol=4)
        ax[3].grid(alpha=0.3)
        fig.suptitle(f"Simulated flight: {name}")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, name + ".png"), dpi=90)
        plt.close(fig)
    print(f"Plots written to {OUT}")


if __name__ == "__main__":
    build()
    failed = run()
    plot()
    print("FAILED scenarios: " + ", ".join(failed) if failed else "All scenarios passed.")
    sys.exit(1 if failed else 0)
