"""Fly the real flight code on many different F450 builds and report whether the default PID settings stay stable.

Usage (after sim/run_sim.py has built the simulator):   python sim/robustness.py
"""
import concurrent.futures as cf
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
EXE = os.path.join(HERE, "results", "sim.exe" if os.name == "nt" else "sim")

BASE = dict(mass=1.2, tmax=8.0, tau=40, inertia=0.0125, noise=1.0, gust=1.0, battery=11.8)

# (label, changes from the base build)
CASES = [
    ("Baseline F450 (1.2 kg, 2212 motors, 10\" props)", {}),
    ("Light build (0.9 kg)", dict(mass=0.9)),
    ("Heavy build (1.6 kg, camera + big battery)", dict(mass=1.6)),
    ("Very heavy (1.9 kg)", dict(mass=1.9)),
    ("Weak motors (6 N each)", dict(tmax=6.0)),
    ("Strong motors (11 N each)", dict(tmax=11.0)),
    ("Fast ESCs / small props (20 ms lag)", dict(tau=20)),
    ("Slow ESCs / big props (70 ms lag)", dict(tau=70)),
    ("Very slow motors (100 ms lag)", dict(tau=100)),
    ("Low inertia (compact, 0.008)", dict(inertia=0.008)),
    ("High inertia (long arms, 0.020)", dict(inertia=0.020)),
    ("Unbalanced props (2x vibration)", dict(noise=2.0)),
    ("Badly unbalanced props (4x vibration)", dict(noise=4.0)),
    ("Windy (2x gusts)", dict(gust=2.0)),
    ("Very windy (3x gusts)", dict(gust=3.0)),
    ("Full battery (12.4 V)", dict(battery=12.4)),
    ("Tired battery (11.0 V)", dict(battery=11.0)),
    ("Heavy + weak motors", dict(mass=1.6, tmax=6.5)),
    ("Light + strong + low inertia (twitchy)", dict(mass=0.9, tmax=11.0, inertia=0.008)),
    ("Slow motors + vibration", dict(tau=70, noise=2.0)),
    ("Heavy + slow + high inertia + wind", dict(mass=1.6, tau=70, inertia=0.020, gust=2.0)),
    ("Twitchy + vibration + wind", dict(mass=0.9, tmax=11.0, inertia=0.008, noise=2.0, gust=2.0)),
    ("Worst realistic mix", dict(mass=1.5, tmax=7.0, tau=60, inertia=0.016, noise=2.0, gust=2.0, battery=11.2)),
]


def run(case):
    label, changes = case
    params = dict(BASE, **changes)
    args = [EXE, "robust"] + [f"{k}={v}" for k, v in params.items()]
    out = subprocess.run(args, capture_output=True, text=True).stdout
    line = next((l for l in out.splitlines() if l.startswith("ROBUST|")), None)
    fails = [l.strip() for l in out.splitlines() if "[FAIL]" in l]
    return label, line, fails


def main():
    if not os.path.exists(EXE):
        raise SystemExit("Build the simulator first: python sim/run_sim.py")
    with cf.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        results = list(ex.map(run, CASES))
    header = f"{'Build':48} {'hover':>6} {'max tilt':>8} {'wobble':>7} {'jitter':>6} {'limit':>6} {'oversh':>6} {'settle':>6}  result"
    print(header)
    print("-" * len(header))
    bad = 0
    rows = []
    for label, line, fails in results:
        if line is None:
            print(f"{label:48} simulator crashed")
            bad += 1
            continue
        f = line.split("|")
        hover, max_ang, rate, jitter, sat, over, settle, res = f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]
        print(f"{label:48} {hover:>6} {max_ang + '°':>8} {rate + '°/s':>7} {jitter + 'us':>6} {sat + '%':>6} {over + '°':>6} {settle + 's':>6}  {res}")
        for x in fails:
            print("      " + x)
        bad += res != "PASS"
        rows.append((label, hover, max_ang, rate, jitter, sat, over, settle, res))
    print()
    print(f"{len(results) - bad}/{len(results)} builds passed")
    with open(os.path.join(HERE, "results", "robustness.tsv"), "w") as fh:
        fh.write("build\thover_stick\tmax_tilt_deg\twobble_deg_s\tjitter_us\tat_limit_pct\tovershoot_deg\tsettle_s\tresult\n")
        for r in rows:
            fh.write("\t".join(r) + "\n")
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
