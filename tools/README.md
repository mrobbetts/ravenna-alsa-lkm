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
