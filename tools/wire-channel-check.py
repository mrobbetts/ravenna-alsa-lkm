#!/usr/bin/env python3
"""Per-channel SOURCE check from the wire (L24 RTP payload, Goertzel, stdlib).

Decodes captured RTP payloads from a daemon source and verifies each RTP
channel carries its assigned tone — proving source-side channel packing
with no receiver/loopback. Any rate. Diagonal energy matrix = PASS;
off-diagonal peak = a packing/map misroute.

  tshark ... -T fields -e rtp.seq -e rtp.timestamp -e rtp.payload -E separator=, > wire.csv
  wire-channel-check.py wire.csv RATE NCH f0 f1 [f2 ...]
"""
import sys, math

def goertzel_power(x, rate, freq):
    n = len(x); k = round(n * freq / rate)
    coeff = 2 * math.cos(2 * math.pi * k / n)
    s1 = s2 = 0.0
    for v in x:
        s0 = v + coeff * s1 - s2
        s2, s1 = s1, s0
    return (s1*s1 + s2*s2 - coeff*s1*s2) / (n*n)

def main():
    path, rate, nch = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    freqs = [float(f) for f in sys.argv[4:]]
    if len(freqs) != nch: sys.exit(f"{nch} channels but {len(freqs)} freqs")
    chans = [[] for _ in range(nch)]
    fb = nch * 3                                   # bytes per interleaved frame (L24)
    npkt = 0
    with open(path) as f:
        for line in f:                              # capture order == TX order on a local egress grab
            parts = line.rstrip("\n").split(",")
            if len(parts) < 3 or not parts[2]: continue
            pay = bytes.fromhex(parts[2].replace(":", ""))
            npkt += 1
            for fr in range(len(pay) // fb):
                base = fr * fb
                for c in range(nch):
                    b = pay[base + c*3 : base + c*3 + 3]
                    chans[c].append(int.from_bytes(b, "big", signed=True))   # AES67 L24 = big-endian
    if not npkt: sys.exit("no RTP payloads parsed — check the -d udp.port==5004,rtp decode")
    n = min(len(c) for c in chans); win = min(2*rate, n); start = max(0, n//2 - win//2)
    chans = [c[start:start+win] for c in chans]
    print(f"{nch} ch, {rate} Hz, {npkt} packets, {win}-sample window\n")
    head = "ch \\ tone |" + "".join(f"{int(f):>8}" for f in freqs)
    print(head); print("-"*len(head))
    ok = True
    for c in range(nch):
        p = [goertzel_power(chans[c], rate, f) for f in freqs]
        mx = max(p) or 1e-300; peak = p.index(mx)
        row = "".join(f"{10*math.log10(pi/mx+1e-12):>8.1f}" for pi in p)
        if peak != c: ok = False
        print(f"  ch{c:>2}   |{row}   {'OK' if peak==c else f'MISROUTE→{int(freqs[peak])}'}")
    print("\nresult:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
