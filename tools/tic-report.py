#!/usr/bin/env python3
"""Analyze a tic-capture.sh trace: per-(domain,rate)-entry tick statistics.

Usage: tic-report.py CAPTURE.txt [--svg OUT.svg] [--saw-ticks N]

Stdlib only. Groups ticks by tic_timer_entry pointer (one group per active
cadence), prints stats, and optionally renders a standalone SVG with the
deviation-from-grid sawtooth and the interval histogram per entry.

Interpretation guide (see MULTI_PCM_PLAN.md, W5 record + Appendix E):
- mean vs nominal      : ppm offset of the PTP GM vs the local crystal.
                         Steady single-digit/low-double-digit ppm = the
                         cadence is PTP-steered (healthy). Drifting = bad.
- bimodal histogram    : the 100us scheduling-grid staircase. Expected at
                         44.1k-family rates (1.0/1.1 ms mix); a single
                         spike is expected at 48k-family rates.
- residual p95         : tick-to-tick deviation from the nearest grid step
                         = ~wakeup-latency jitter (first difference, so it
                         slightly overstates per-tick jitter). This is the
                         number that competes with the +-90.7us Q/R edge
                         window (Appendix E risk 1). Watch it when tuning
                         CPU isolation / IRQ affinity.
- outliers             : late ticks + their catch-up partners. A few per
                         minute is normal; clusters mean scheduling trouble.
"""
import argparse
import re
import sys

PAT = re.compile(r"\s(\d+)\.(\d{6}): rav_tick:.*entry=(0x[0-9a-f]+)")
# valid cadences in us for tic_frame_size_at_1fs=48 deployments
NOMINALS = [1000.0, 48 / 44.1 * 1000.0, 4000.0 / 3.0]


def parse(path):
    groups = {}
    header = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") and "rav_tick" not in line:
                header.append(line.rstrip())
                continue
            m = PAT.search(line)
            if m:
                t = int(m.group(1)) * 1_000_000 + int(m.group(2))  # us
                groups.setdefault(m.group(3), []).append(t)
    return header, groups


def analyze(ts):
    n = len(ts)
    d = [ts[i + 1] - ts[i] for i in range(n - 1)]
    mean = (ts[-1] - ts[0]) / (n - 1)
    nominal = min(NOMINALS, key=lambda x: abs(x - mean))
    ppm = (mean - nominal) / nominal * 1e6
    # grid steps bracketing the nominal (multiples of the 100us quantum)
    lo = (int(nominal) // 100) * 100
    steps = [float(lo)] if abs(nominal - lo) < 1e-9 else [float(lo), float(lo + 100)]
    resid = sorted(abs(x - min(steps, key=lambda s: abs(x - s))) for x in d)
    outliers = [x for x in d if abs(x - nominal) > 250]
    hist = {}
    for x in d:
        hist[int(x // 20) * 20] = hist.get(int(x // 20) * 20, 0) + 1
    return {
        "n": n, "span_s": (ts[-1] - ts[0]) / 1e6, "mean": mean,
        "nominal": nominal, "ppm": ppm, "steps": steps,
        "resid_p95": resid[int(0.95 * len(resid))] if resid else 0,
        "d_min": min(d), "d_max": max(d), "outliers": outliers,
        "hist": hist, "d": d, "ts": ts,
    }


def report(tag, a):
    print(f"\nentry {tag}: {a['n']} ticks / {a['span_s']:.3f} s")
    print(f"  mean interval : {a['mean']:.4f} us  (nominal {a['nominal']:.4f} -> {a['ppm']:+.2f} ppm vs local clock)")
    print(f"  interval range: [{a['d_min']}, {a['d_max']}] us; outliers (>250 us off nominal): {len(a['outliers'])}")
    print(f"  residual p95  : {a['resid_p95']:.0f} us vs grid steps {[int(s) for s in a['steps']]}")
    peak = max(a["hist"].values())
    for b in sorted(a["hist"]):
        if a["hist"][b] >= peak * 0.005:
            print(f"  {b:>5}-{b+20:<5} us | {'#' * max(1, round(a['hist'][b] / peak * 50))} {a['hist'][b]}")


def svg_panels(tag, a, y0, saw_ticks):
    ts, mean = a["ts"], a["mean"]
    n = min(saw_ticks, a["n"])
    saw = [ts[i] - ts[0] - i * mean for i in range(n)]
    amp = max(1.0, max(abs(v) for v in saw))
    pts = " ".join(
        f"{40 + i * (640 / max(1, n - 1)):.1f},{y0 + 70 - v / amp * 55:.1f}"
        for i, v in enumerate(saw))
    bins = sorted(b for b in a["hist"])
    bmin, bmax = bins[0], bins[-1]
    nb = (bmax - bmin) // 20 + 1
    bw = min(30, 620 / nb)
    peak = max(a["hist"].values())
    bars = "".join(
        f'<rect x="{50 + (b - bmin) // 20 * bw:.1f}" y="{y0 + 250 - a["hist"][b] / peak * 90:.1f}" '
        f'width="{bw - 2:.1f}" height="{a["hist"][b] / peak * 90:.1f}" fill="#1D9E75"/>'
        for b in bins)
    return f"""
<text x="20" y="{y0 + 16}" font-size="13" fill="#222">entry {tag}: deviation from mean grid, first {n} ticks (amp ±{amp:.0f} us) — mean {mean:.3f} us ({a['ppm']:+.1f} ppm), residual p95 {a['resid_p95']:.0f} us</text>
<line x1="40" y1="{y0 + 70}" x2="680" y2="{y0 + 70}" stroke="#bbb" stroke-dasharray="3 4"/>
<polyline points="{pts}" fill="none" stroke="#D85A30" stroke-width="1.5"/>
<text x="20" y="{y0 + 156}" font-size="13" fill="#222">interval histogram, 20 us bins ({bmin}-{bmax + 20} us); {len(a['outliers'])} outliers beyond ±250 us</text>
<line x1="46" y1="{y0 + 250}" x2="684" y2="{y0 + 250}" stroke="#888"/>
{bars}
<text x="50" y="{y0 + 266}" font-size="11" fill="#555">{bmin} us</text>
<text x="660" y="{y0 + 266}" font-size="11" fill="#555" text-anchor="end">{bmax + 20} us</text>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--svg")
    ap.add_argument("--saw-ticks", type=int, default=90)
    args = ap.parse_args()

    header, groups = parse(args.capture)
    if not groups:
        sys.exit("no rav_tick events found — was the capture taken with tic-capture.sh?")
    for h in header:
        print(h)
    results = {tag: analyze(ts) for tag, ts in groups.items() if len(ts) > 2}
    for tag, a in sorted(results.items()):
        report(tag, a)

    if args.svg:
        h = 290 * len(results) + 20
        body = "".join(
            svg_panels(tag, a, 20 + i * 290, args.saw_ticks)
            for i, (tag, a) in enumerate(sorted(results.items())))
        with open(args.svg, "w") as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 700 {h}" '
                    f'font-family="sans-serif"><rect width="700" height="{h}" fill="#fff"/>'
                    f"{body}</svg>\n")
        print(f"\nwrote {args.svg}")


if __name__ == "__main__":
    main()
