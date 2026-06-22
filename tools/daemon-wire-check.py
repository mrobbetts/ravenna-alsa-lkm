#!/usr/bin/env python3
"""daemon-wire-check.py — end-to-end RTP wire conformance for the AES67 daemon.

Discovers the LIVE topology from the daemon's REST API (cards + their PTP
domains, PCMs + their rates, enabled sources + their SDPs), captures each
source off the wire, and asserts the RTP is well formed:

  * exactly one sender IP and one SSRC per group,
  * a strictly monotonic +1 sequence (no gaps),
  * a CONSTANT RTP-timestamp step == the source's samples-per-packet,
  * packet rate and media-clock rate == the source's configured sample rate.

These catch the bug classes this machinery has actually hit:
  * N x over-send (pktrate a multiple of the nominal; ts steps {0,48} / {16,-32})
    -> the W11 cross-domain same-rate pump-filter bug (lkm 24e5bcc),
  * ts step alternating 44/45 -> the pre-W5 SAC-scaling bug,
  * 44.1k stream at ~1000 pkt/s -> an engine ticking at the wrong cadence.

Read-only by default (mutates nothing). `--exercise` snapshots the config, runs
a destructive re-rate matrix (incl. forcing two domains onto a shared rate — the
direct W11 regression test), re-checks the wire after each step, then restores
the config. NB: recreated sources roll their SSRC and restart their sequence, so
external receivers (e.g. a Hapi) must RE-ACQUIRE after an --exercise run; that is
inherent to teardown/recreate, not something this tool can paper over.

Stdlib only. Capturing needs CAP_NET_RAW -> run with sudo (or grant tcpdump the
cap). Talks to the daemon over plain HTTP on localhost.

Examples:
    sudo ./daemon-wire-check.py                       # check every enabled source
    sudo ./daemon-wire-check.py --duration 5
    sudo ./daemon-wire-check.py --iface vl_audio
    sudo ./daemon-wire-check.py --exercise --yes      # destructive; restores after
"""
import argparse
import json
import re
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

VALID_RATES = (44100, 48000, 88200, 96000, 176400, 192000, 352800, 384000)


# ---------------------------------------------------------------- daemon API ---
def api(base, path, method="GET", body=None):
    url = base.rstrip("/") + path
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        raw = r.read().decode()
    return raw


def api_json(base, path, **kw):
    return json.loads(api(base, path, **kw))


def parse_sdp(sdp):
    """Pull (group, port, rate, spp, channels, sender) out of a source SDP."""
    out = {"group": None, "port": None, "rate": None, "spp": None,
           "channels": None, "sender": None}
    for line in sdp.splitlines():
        line = line.strip()
        if line.startswith("m=audio"):           # m=audio 5004 RTP/AVP 98
            out["port"] = int(line.split()[1])
        elif line.startswith("c=IN IP4"):        # c=IN IP4 239.69.0.3/15
            out["group"] = line.split()[2].split("/")[0]
        elif line.startswith("o="):              # o=- .. IN IP4 192.168.0.129
            out["sender"] = line.split()[-1]
        elif line.startswith("a=rtpmap:"):       # a=rtpmap:98 L24/48000/4
            m = re.search(r"\b[LF](?:16|24|32)/(\d+)(?:/(\d+))?", line)
            if m:
                out["rate"] = int(m.group(1))
                out["channels"] = int(m.group(2)) if m.group(2) else 1
        elif line.startswith("a=framecount:"):   # a=framecount:48
            out["spp"] = int(line.split(":")[1])
    return out


def discover(base):
    """Return (domains:set, sources:[dict]) for every ENABLED source."""
    cards = api_json(base, "/api/cards").get("cards", [])
    domain_of_card = {c["name"]: c.get("domain", 0) for c in cards}
    # pcm_id -> domain, via each card's pcm list
    domain_of_pcm = {}
    for c in cards:
        try:
            pcms = api_json(base, "/api/card/%s/pcm" % c["name"]).get("pcms", [])
        except Exception:
            pcms = []
        for p in pcms:
            domain_of_pcm[p.get("pcm_id")] = domain_of_card.get(c["name"], 0)

    sources = []
    for s in api_json(base, "/api/sources").get("sources", []):
        if not s.get("enabled", True):
            continue
        sdp = api(base, "/api/source/sdp/%d" % s["id"])
        info = parse_sdp(sdp)
        info.update(id=s["id"], name=s.get("name", ""), pcm=s.get("pcm"),
                    domain=domain_of_pcm.get(s.get("pcm")))
        sources.append(info)
    return set(domain_of_card.values()), sources


# ------------------------------------------------------------------- capture ---
def detect_iface(sender_ip):
    out = subprocess.run(["ip", "-o", "-4", "addr"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if sender_ip and (" " + sender_ip + "/") in line:
            return line.split()[1]
    return None


def capture(iface, group, port, duration):
    """tcpdump the group:port for `duration` s -> list of (wall, seq, ts, ssrc, src)."""
    with tempfile.NamedTemporaryFile(suffix=".pcap", delete=True) as tf:
        filt = "udp and dst host %s and dst port %d" % (group, port)
        subprocess.run(["timeout", str(duration), "tcpdump", "-ni", iface,
                        "-w", tf.name, filt],
                       capture_output=True)  # timeout exit 124 is expected
        data = open(tf.name, "rb").read()
    return _parse_pcap(data)


def _parse_pcap(data):
    if len(data) < 24:
        return []
    pkts = []
    off = 24  # global header
    while off + 16 <= len(data):
        sec, usec, incl = struct.unpack("<III", data[off:off + 12])
        off += 16
        pkt = data[off:off + incl]
        off += incl
        eth = 14
        if len(pkt) >= 14 and pkt[12:14] == b"\x81\x00":   # 802.1Q VLAN tag
            eth = 18
        rtp = eth + 20 + 8
        if len(pkt) < rtp + 12:
            continue
        src = ".".join(str(b) for b in pkt[eth + 12:eth + 16])
        r = pkt[rtp:]
        seq = struct.unpack(">H", r[2:4])[0]
        ts = struct.unpack(">I", r[4:8])[0]
        ssrc = struct.unpack(">I", r[8:12])[0]
        pkts.append((sec + usec / 1e6, seq, ts, ssrc, src))
    return pkts


def _unwrap_and_order(pkts):
    """pkts are (wall, seq16, ts, ssrc, src) in CAPTURE order. Extend the 16-bit
    sequence across wraps (a high->low drop > 32768 = a wrap), then order by the
    extended sequence — wrap-safe AND tolerant of the occasional reorder. Sorting
    raw 16-bit seq instead silently scrambles any stream long enough to wrap
    (e.g. a 3000 pkt/s source in a 3 s capture)."""
    ext = []
    epoch = 0
    prev = None
    for p in pkts:
        s = p[1]
        if prev is not None and s < prev and (prev - s) > 32768:
            epoch += 1 << 16
        prev = s
        ext.append((epoch + s, p))
    ext.sort(key=lambda x: x[0])
    return [e[0] for e in ext], [e[1] for e in ext]


def analyze(pkts, expect_rate, spp, tol=0.005):
    """Return (ok, metrics, failures[]). tol = fraction of off ts-steps tolerated
    (rare wire reordering) before flagging."""
    fails = []
    m = {}
    if len(pkts) < 8:
        return False, {"pkts": len(pkts)}, ["too few packets (%d) — source silent or wrong group/iface" % len(pkts)]
    eseq, ordered = _unwrap_and_order(pkts)
    senders = {p[4] for p in ordered}
    ssrcs = {p[3] for p in ordered}
    walls = [p[0] for p in ordered]
    tss = [p[2] for p in ordered]
    dur = max(walls) - min(walls)
    m["pkts"] = len(ordered)
    m["senders"] = senders
    m["ssrcs"] = {hex(s) for s in ssrcs}
    m["pktrate"] = len(ordered) / dur if dur > 0 else 0
    steps = {}
    seqgaps = sum(1 for i in range(len(eseq) - 1) if eseq[i + 1] - eseq[i] != 1)
    for i in range(len(ordered) - 1):
        st = (tss[i + 1] - tss[i]) & 0xFFFFFFFF
        steps[st] = steps.get(st, 0) + 1
    m["ts_steps"] = steps
    m["seqgaps"] = seqgaps
    m["mediaclock"] = ((max(tss) - min(tss)) & 0xFFFFFFFF) / dur if dur > 0 else 0
    total = sum(steps.values())
    offsteps = total - steps.get(spp, 0) if spp else total

    if len(senders) != 1:
        fails.append("%d senders (expect 1): %s" % (len(senders), senders))
    if len(ssrcs) != 1:
        fails.append("%d SSRCs (expect 1) — collision/leak: %s" % (len(ssrcs), m["ssrcs"]))
    if seqgaps > max(1, total * tol):
        fails.append("%d sequence gaps (expect ~0)" % seqgaps)
    if spp and offsteps > total * tol:
        fails.append("ts step not constant %d: %d/%d off (%s) — over-send / bad media clock"
                     % (spp, offsteps, total, _top(steps)))
    if expect_rate:
        nominal = expect_rate / spp if spp else 0
        if nominal and abs(m["pktrate"] - nominal) / nominal > 0.05:
            fails.append("pktrate %.0f/s (expect ~%.0f/s = %.1fx)"
                         % (m["pktrate"], nominal, m["pktrate"] / nominal if nominal else 0))
        if abs(m["mediaclock"] - expect_rate) / expect_rate > 0.01:
            fails.append("media clock %.0f Hz (expect ~%d)" % (m["mediaclock"], expect_rate))
    return (not fails), m, fails


def _signed(s):
    """Render a 32-bit ts-step as signed, so a backward step reads -608 not
    4294966688."""
    return s - (1 << 32) if s >= (1 << 31) else s


def _top(steps, n=4):
    """Top n ts-steps by count, as a compact 'step:count' string (signed)."""
    return ", ".join("%d:%d" % (_signed(s), c) for s, c in
                     sorted(steps.items(), key=lambda kv: -kv[1])[:n])


# --------------------------------------------------------------------- checks ---
def check_sources(base, iface, duration, sources):
    allok = True
    for s in sources:
        if not s["group"] or not s["port"]:
            print("  SKIP source %s (id %s): no group/port in SDP" % (s["name"], s["id"]))
            continue
        ifc = iface or detect_iface(s["sender"])
        if not ifc:
            print("  SKIP %s: cannot detect egress iface for sender %s (use --iface)"
                  % (s["name"], s["sender"]))
            allok = False
            continue
        pkts = capture(ifc, s["group"], s["port"], duration)
        ok, m, fails = analyze(pkts, s["rate"], s["spp"])
        tag = "OK  " if ok else "FAIL"
        print("  [%s] source %s  dom=%s  %s:%d  %s Hz x%s spp%s  ->  %.0f pkt/s, mclk %.0f, steps {%s}"
              % (tag, s["name"], s["domain"], s["group"], s["port"], s["rate"],
                 s["channels"], s["spp"], m.get("pktrate", 0), m.get("mediaclock", 0),
                 _top(m.get("ts_steps", {}))))
        for f in fails:
            print("         - " + f)
        allok = allok and ok
    return allok


def cmd_check(base, iface, duration):
    domains, sources = discover(base)
    print("daemon %s: %d enabled source(s) across PTP domain(s) %s"
          % (base, len(sources), sorted(d for d in domains)))
    if len(domains) > 1:
        rates = {}
        for s in sources:
            rates.setdefault(s["rate"], set()).add(s["domain"])
        shared = {r: d for r, d in rates.items() if len(d) > 1}
        if shared:
            print("  note: rate(s) %s run in multiple domains — the exact W11 over-send "
                  "condition; the per-source checks below confirm it's filtered correctly"
                  % {r: sorted(d) for r, d in shared.items()})
    ok = check_sources(base, iface, duration, sources)
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ------------------------------------------------------------------ exercise ---
def set_pcm_rate(base, card, pcm, rate, ins, outs):
    api(base, "/api/card/%s/pcm/%s" % (card, pcm), method="PUT",
        body={"name": pcm, "sample_rate": rate, "num_inputs": ins, "num_outputs": outs})


def cmd_exercise(base, iface, duration, args):
    print("!! DESTRUCTIVE: this re-rates live PCMs. Downstream receivers (Hapi) will\n"
          "!! need to re-acquire afterwards (recreated streams roll their SSRC).")
    if not args.yes:
        print("Refusing without --yes.")
        return 2
    only = set(args.only.split(",")) if args.only else None
    # snapshot every (selected) pcm's rate + channels so we can restore it.
    saved = []
    for c in api_json(base, "/api/cards").get("cards", []):
        for p in api_json(base, "/api/card/%s/pcm" % c["name"]).get("pcms", []):
            if only and p["name"] not in only and c["name"] not in only:
                continue
            saved.append((c["name"], p["name"], p.get("sample_rate"),
                          p.get("num_inputs", 0), p.get("num_outputs", 0)))
    if not saved:
        print("no PCMs selected (check --only).")
        return 2
    rates = [int(r) for r in args.rates.split(",")] if args.rates else list(VALID_RATES)
    bad = [r for r in rates if r not in VALID_RATES]
    if bad:
        print("invalid --rates: %s (valid: %s)" % (bad, list(VALID_RATES)))
        return 2
    print("snapshot: %d pcm(s) -> %s" % (len(saved), [s[1] for s in saved]))
    print("sweep rates %s x %d repeat(s)" % (rates, args.repeat))
    rc = 0
    try:
        # Walk each selected pcm through the rate ladder. Sweeping high rates
        # exercises per-rate cadence + frame size; as pcms in different domains
        # land on a shared rate this reproduces the W11 cross-domain condition;
        # --repeat soaks the recreate-card path to surface entry/stream leaks.
        for rep in range(args.repeat):
            for (card, pcm, rate, ins, outs) in saved:
                for target in rates:
                    print("\n-- [rep %d] re-rate %s/%s -> %d --" % (rep + 1, card, pcm, target))
                    set_pcm_rate(base, card, pcm, target, ins, outs)
                    time.sleep(args.settle)
                    _, sources = discover(base)
                    if not check_sources(base, iface, duration, sources):
                        rc = 1
    finally:
        print("\nrestoring original rates ...")
        for (card, pcm, rate, ins, outs) in saved:
            try:
                set_pcm_rate(base, card, pcm, rate, ins, outs)
            except Exception as e:
                print("  WARN restore %s/%s: %s" % (card, pcm, e))
        print("restore done (NB: external receivers must re-pair).")
    print("\nRESULT:", "PASS" if rc == 0 else "FAIL")
    return rc


# ----------------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--daemon", default="http://localhost:8080", help="daemon base URL")
    ap.add_argument("--iface", default=None, help="capture iface (auto-detect from SDP sender if omitted)")
    ap.add_argument("--duration", type=float, default=3.0, help="seconds to capture per source")
    ap.add_argument("--exercise", action="store_true", help="destructive re-rate matrix + restore")
    ap.add_argument("--settle", type=float, default=2.0, help="seconds to wait after a re-rate (--exercise)")
    ap.add_argument("--only", default=None, help="limit --exercise to these card/pcm names (comma-separated)")
    ap.add_argument("--rates", default=None,
                    help="--exercise rate ladder (comma-separated Hz); default = all valid rates 44100..384000")
    ap.add_argument("--repeat", type=int, default=1, help="--exercise: repeat the whole sweep N times (soak)")
    ap.add_argument("--yes", action="store_true", help="confirm destructive --exercise")
    args = ap.parse_args()
    try:
        if args.exercise:
            return cmd_exercise(args.daemon, args.iface, args.duration, args)
        return cmd_check(args.daemon, args.iface, args.duration)
    except urllib.error.URLError as e:
        print("daemon API error (%s): is the daemon up at %s?" % (e, args.daemon), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
