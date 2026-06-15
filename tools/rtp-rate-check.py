#!/usr/bin/env python3
"""Per-SSRC RTP cadence/rate check for AES67 streams.

Reads tshark field output (one packet per line) and, for each RTP SSRC,
reports the packet rate, the RTP-timestamp step distribution, and the
effective media-clock rate (timestamp units per wall-clock second). Used
to verify the W7 divergent-rate milestone directly on the wire.

Capture + analyze (replace IFACE):

    nix-shell -p wireshark-cli --run \
      "tshark -i IFACE -f 'udp port 5004' -d udp.port==5004,rtp -a duration:20 \
         -T fields -e frame.time_epoch -e rtp.ssrc -e rtp.timestamp -e rtp.seq \
         -E separator=," > /tmp/rtp.csv
    ./rtp-rate-check.py /tmp/rtp.csv

Expected for tic_frame_size_at_1fs=48 (framecount 48, constant per packet):
  48 kHz stream : ts_step=48 (constant), media rate ~48000, ~1000 pkt/s
  44.1 kHz      : ts_step=48 (constant), media rate ~44100, ~919  pkt/s

Failure signatures (why this test exists):
  - ts_step alternating 44/45 instead of a constant 48  -> the per-rate
    SAC stepped by the scaled ratio, not the chip's own frame (the W5
    pre-fix bug class). RTP timestamps must advance by the frame size.
  - 44.1k stream showing ~1000 pkt/s / media rate ~48000 -> that engine
    ticked at the wrong cadence (+8.84% pitch error class).
  - media rate drifting vs nominal -> not PTP-locked / cadence wrong.
"""
import sys
from collections import defaultdict

TS_MOD = 1 << 32
VALID = [44100, 48000, 88200, 96000, 176400, 192000, 352800, 384000]


def load(path):
    rows = defaultdict(list)  # ssrc -> [(t, ts, seq)]
    f = open(path) if path and path != "-" else sys.stdin
    for line in f:
        parts = line.rstrip("\n").split(",")
        if len(parts) < 3 or not parts[0] or not parts[1] or not parts[2]:
            continue
        try:
            t = float(parts[0])
            ssrc = int(parts[1], 0)            # hex like 0x...
            ts = int(parts[2]) & 0xFFFFFFFF
            seq = int(parts[3]) if len(parts) > 3 and parts[3] else -1
        except ValueError:
            continue
        rows[ssrc].append((t, ts, seq))
    return rows


def analyze(samples):
    samples.sort(key=lambda r: r[0])
    n = len(samples)
    t0, tN = samples[0][0], samples[-1][0]
    dur = tN - t0
    # RTP timestamp steps (wrap-safe), and total advance
    steps = {}
    total_adv = 0
    for i in range(n - 1):
        d = (samples[i + 1][1] - samples[i][1]) % TS_MOD
        steps[d] = steps.get(d, 0) + 1
        total_adv += d
    # inter-arrival
    ias = [samples[i + 1][0] - samples[i][0] for i in range(n - 1)]
    ia_mean = sum(ias) / len(ias) if ias else 0.0
    ia_sorted = sorted(ias)
    ia_p95 = ia_sorted[int(0.95 * (len(ia_sorted) - 1))] if ias else 0.0
    media_rate = total_adv / dur if dur > 0 else 0.0
    pkt_rate = (n - 1) / dur if dur > 0 else 0.0
    nominal = min(VALID, key=lambda r: abs(r - media_rate)) if media_rate else 0
    ppm = (media_rate - nominal) / nominal * 1e6 if nominal else 0.0
    return {
        "n": n, "dur": dur, "pkt_rate": pkt_rate, "media_rate": media_rate,
        "nominal": nominal, "ppm": ppm, "steps": steps,
        "ia_mean_ms": ia_mean * 1e3, "ia_p95_ms": ia_p95 * 1e3,
    }


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "-"
    rows = load(path)
    if not rows:
        sys.exit("no RTP packets parsed — check the tshark field order "
                 "(frame.time_epoch,rtp.ssrc,rtp.timestamp,rtp.seq)")
    for ssrc, samples in sorted(rows.items()):
        if len(samples) < 3:
            continue
        a = analyze(samples)
        steps_str = ", ".join(f"{k}:{v}" for k, v in sorted(a["steps"].items()))
        const = len(a["steps"]) == 1
        print(f"\nSSRC 0x{ssrc:08x}: {a['n']} pkts / {a['dur']:.2f} s")
        print(f"  media-clock rate : {a['media_rate']:.1f} Hz  "
              f"(nominal {a['nominal']} -> {a['ppm']:+.1f} ppm)")
        print(f"  packet rate      : {a['pkt_rate']:.1f} pkt/s  "
              f"(inter-arrival mean {a['ia_mean_ms']:.4f} ms, p95 {a['ia_p95_ms']:.4f} ms)")
        print(f"  RTP ts step      : {{{steps_str}}}  "
              f"{'[constant - OK]' if const else '[NON-CONSTANT - investigate: SAC/cadence bug?]'}")


if __name__ == "__main__":
    main()
