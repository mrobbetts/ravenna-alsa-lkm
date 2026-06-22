# tools/ — tick-timing instrumentation

Measurement tooling for the per-(domain, rate) TIC engines (multi-rate W5+).
See MULTI_PCM_PLAN.md (W5 record, Appendix E risk 1) for the first captured
baseline and its interpretation.

## tic-capture.sh (run as root on the target)

    ./tic-capture.sh 5 /tmp/ticks.txt

Places a kprobe on `manager_entry_tick`, records every tick's timestamp and
its registry-entry pointer for N seconds, cleans up after itself, and stamps
host/kernel/module-srcversion provenance into the output header. Safe during
live audio (one probe hit per ~1 ms tick).

## tic-report.py (run anywhere)

    ./tic-report.py /tmp/ticks.txt --svg report.svg

Per entry (i.e. per active cadence): mean interval and ppm offset vs the
nominal cadence (= PTP GM vs local crystal), interval histogram (shows the
100 us scheduling-grid staircase at 44.1k-family rates), tick-to-tick
residual p95 (~wakeup-latency jitter — the number that competes with the
servo's ±90.7 us classification window), and outlier count. `--svg` renders
the sawtooth + histogram panels, one pair per entry — W7's two-cadence
captures separate automatically.

Baseline (Linux 7.0.10, 2026-06-12, single 44.1k entry, live audio,
no CPU isolation): +9.0 ppm, residual p95 = 75 us, 12/4601 outliers.
Tuning knobs to evaluate against this baseline: dedicated CPUs for the
timer softirqs (isolcpus/nohz_full + IRQ affinity), threadirqs, disabling
deep C-states on the isolated cores.

## rtp-rate-check.py (run anywhere)

Per-SSRC RTP cadence/rate check for the W7 divergent-rate milestone. Reads
tshark field output and reports, per stream: packet rate, RTP-timestamp step
distribution (must be a constant = frame size; a 44/45 split is the SAC-scaling
bug), and effective media-clock rate (≈ the sample rate, PTP-locked). See the
header for the exact tshark capture command. Stdlib only.

## daemon-wire-check.py (run as root on the target)

    sudo ./daemon-wire-check.py                    # check every enabled source
    sudo ./daemon-wire-check.py --iface vl_audio --duration 5
    sudo ./daemon-wire-check.py --exercise --yes --only speakers   # destructive

The conformance harness `rtp-rate-check.py` always wanted: it asks the daemon's
REST API for the LIVE topology (cards + PTP domains, PCMs + rates, enabled
sources + SDPs), captures each source off the wire (tcpdump), and ASSERTS it
against that source's own configured rate — one sender/SSRC, monotonic +1
sequence, a constant RTP-timestamp step == samples-per-packet, and packet/media
rate == the sample rate. Per-source `spp` and rate come from each SDP, so a
16-ch/spp-16 source and a 4-ch/spp-48 source are each held to their own nominal.
Exit code 0/1 = PASS/FAIL, one line per source. Read-only by default; auto-flags
when a rate runs in >1 domain (the exact W11 over-send condition — the per-source
check then confirms the pump filter keeps them separate, lkm 24e5bcc).

`--exercise --yes` is DESTRUCTIVE: it snapshots the selected PCMs' rates, walks
each through the full valid rate ladder (44100..384000; `--rates 48000,96000` to
limit), re-checks the wire after each step, and restores the rates. High rates
exercise per-rate cadence/frame size; as PCMs in different domains land on a
shared rate it reproduces the cross-domain W11 condition; `--repeat N` soaks the
recreate-card path to surface entry/stream leaks. NB: recreated sources roll
their SSRC + restart their sequence, so external receivers (a Hapi) must
RE-ACQUIRE afterwards — inherent to teardown/recreate, not restorable. Scope with
`--only name,...` to leave chains you don't want touched alone (e.g. exercise
only Music, leave HT). Needs CAP_NET_RAW for the capture (run via sudo).

A `--stress` mode (synthetic build-to-cardinality-limits: up to MAX_CARDS /
MAX_PCMS across domains at a rate spread, a silent source per PCM — the rings are
zeroed and unfed, so they emit zeros — full snapshot/restore, single all-groups
capture) is planned next. Stdlib only.
