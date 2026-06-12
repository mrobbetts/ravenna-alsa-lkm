#!/usr/bin/env bash
# Capture RAVENNA tick timestamps via a kprobe on manager_entry_tick.
#
# Usage: tic-capture.sh [seconds] [outfile]
#   default: 5 seconds, ./tic-capture-<timestamp>.txt
#
# Needs root (tracefs). Overhead: one kprobe per tick (~1 kHz per active
# (domain, rate) entry), sub-microsecond per hit — safe during live audio.
# The entry=$arg1 field is the tic_timer_entry pointer, so captures with
# multiple concurrent cadences (W7+) separate cleanly in tic-report.py.
set -euo pipefail
DUR="${1:-5}"
OUT="${2:-tic-capture-$(date +%Y%m%d-%H%M%S).txt}"
T=/sys/kernel/tracing
[ -w "$T/kprobe_events" ] || { echo "error: need root (tracefs at $T)" >&2; exit 1; }

cleanup() {
    echo 0 > "$T/events/kprobes/rav_tick/enable" 2>/dev/null || true
    echo > "$T/trace" 2>/dev/null || true
    echo > "$T/kprobe_events" 2>/dev/null || true
}
trap cleanup EXIT

echo 'p:rav_tick manager_entry_tick entry=$arg1' > "$T/kprobe_events"
echo > "$T/trace"
echo 1 > "$T/events/kprobes/rav_tick/enable"
sleep "$DUR"
echo 0 > "$T/events/kprobes/rav_tick/enable"

{
    echo "# rav-tic capture $(date -Is) host=$(hostname) kernel=$(uname -r)"
    echo "# module srcversion: $(cat /sys/module/MergingRavennaALSA/srcversion 2>/dev/null || echo '?')"
    echo "# duration: ${DUR}s"
    cat "$T/trace"
} > "$OUT"
chmod 644 "$OUT"
echo "wrote $OUT ($(grep -c 'rav_tick:' "$OUT" || true) ticks)"
