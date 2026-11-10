#!/usr/bin/env bash
# deploy/bench/rt_latency.sh — PREEMPT_RT latency validation (architecture decision 8).
#
# Answers one question: does the NVIDIA kernel module inject unbounded scheduling latency
# on a PREEMPT_RT kernel while control threads run on isolated cores? If it does, ARTS needs
# a second PC for the control path. The 2026-08-05 run on Ubuntu 22.04 said no (190 us worst
# case, 9.5% of a 500 Hz period), and this script reproduces that protocol exactly so later
# runs stay comparable.
#
# Three conditions, matching README.md's original hand-run procedure:
#   1. idle baseline, no isolation
#   2. CPU + GPU load, no isolation          — shows what isolation is worth
#   3. CPU + NVIDIA GPU load, WITH isolation — the configuration ARTS actually ships
#
# Condition 3 is the one that matters. Conditions 1 and 2 exist so the number has a
# comparison and a reviewer can see the effect size rather than one figure in isolation.
#
# Writes docs/benchmarks/rt_latency_<host>_<date>.{json,md}. Commit both.
#
# Usage:
#   sudo -v && ./deploy/bench/rt_latency.sh            # full protocol, ~12 min
#   ./deploy/bench/rt_latency.sh --duration 60         # shorter, NOT comparable to baseline
#   ./deploy/bench/rt_latency.sh --skip-gpu            # no display available
set -uo pipefail

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="$ARTS_ROOT/docs/benchmarks"

# The published baseline used 60 s idle and 300 s loaded. Changing these makes the result
# incomparable to it, which is why --duration warns rather than silently accepting.
DUR_IDLE=60
DUR_LOAD=300
SKIP_GPU=0
PRIORITY=80
INTERVAL=1000

bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
ok()    { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn()  { printf '  \033[33m!\033[0m %s\n' "$*"; }
fail()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }
info()  { printf '    %s\n' "$*"; }
head2() { printf '\n\033[1m── %s\033[0m\n' "$*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --duration) DUR_LOAD="$2"; DUR_IDLE="$2"; shift 2
                    warn "non-default duration — results are NOT comparable to the committed baseline" ;;
        --skip-gpu) SKIP_GPU=1; shift ;;
        -h|--help)  sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)          fail "unknown argument: $1"; exit 2 ;;
    esac
done

TMP="$(mktemp -d)"
BG_PIDS=()
cleanup() {
    for p in "${BG_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    pkill -f 'glmark2' 2>/dev/null
    pkill -f 'stress-ng' 2>/dev/null
    rm -rf "$TMP"
}
trap cleanup EXIT INT TERM

# ── CPU list helpers ─────────────────────────────────────────────────────────
# The isolated set comes from the kernel, not from a hardcoded map, so this script
# works on the production workstation without editing.

expand_list() {  # "4-5,10-11" -> "4 5 10 11"
    local part a b out=() parts
    IFS=',' read -ra parts <<< "$1"
    for part in "${parts[@]}"; do
        [ -z "$part" ] && continue
        if [[ "$part" == *-* ]]; then
            a="${part%-*}"; b="${part#*-}"
            for ((i=a; i<=b; i++)); do out+=("$i"); done
        else
            out+=("$part")
        fi
    done
    echo "${out[@]}"
}

join_commas() { local IFS=','; echo "$*"; }

# ── Preconditions ────────────────────────────────────────────────────────────
bold "ARTS PREEMPT_RT latency benchmark"

head2 "Preconditions"

KERNEL="$(uname -r)"
KERNEL_VER="$(uname -v)"
if echo "$KERNEL_VER" | grep -q PREEMPT_RT; then
    ok "PREEMPT_RT kernel: $KERNEL"
else
    fail "not a PREEMPT_RT kernel ($KERNEL) — boot the realtime entry first"
    exit 1
fi

ISOLATED="$(cat /sys/devices/system/cpu/isolated 2>/dev/null)"
if [ -z "$ISOLATED" ]; then
    fail "no isolated cores — condition 3 is the whole point of this test"
    info "run ./arts_install.sh --prereqs to set isolcpus, then reboot"
    exit 1
fi
ok "isolated cores: $ISOLATED"

read -ra ISO_ARR <<< "$(expand_list "$ISOLATED")"
NCPU="$(nproc --all)"
HOUSE_ARR=()
for ((c=0; c<NCPU; c++)); do
    skip=0
    for i in "${ISO_ARR[@]}"; do [ "$c" = "$i" ] && skip=1 && break; done
    [ "$skip" = 0 ] && HOUSE_ARR+=("$c")
done
ISO_CSV="$(join_commas "${ISO_ARR[@]}")"
HOUSE_CSV="$(join_commas "${HOUSE_ARR[@]}")"
info "housekeeping cores: $HOUSE_CSV  (${#HOUSE_ARR[@]} of $NCPU)"

MISSING=()
command -v cyclictest >/dev/null 2>&1 || MISSING+=(rt-tests)
command -v stress-ng  >/dev/null 2>&1 || MISSING+=(stress-ng)
if [ "$SKIP_GPU" = 0 ]; then
    command -v glmark2 >/dev/null 2>&1 || MISSING+=(glmark2)
fi
if [ ${#MISSING[@]} -gt 0 ]; then
    warn "missing test tools: ${MISSING[*]}"
    read -r -p "  → install them now? [y/N] " reply
    if [[ "$reply" =~ ^[Yy]$ ]]; then
        sudo apt-get install -y "${MISSING[@]}" || { fail "install failed"; exit 1; }
    else
        exit 1
    fi
fi
ok "cyclictest, stress-ng${SKIP_GPU:+,}$([ "$SKIP_GPU" = 0 ] && echo ", glmark2") present"

# glmark2 needs a display. Without one, condition 3 degrades to a CPU-only test, which
# does not answer the question this benchmark exists to answer.
if [ "$SKIP_GPU" = 0 ] && [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    warn "no DISPLAY — GPU load cannot run; re-run from a desktop session"
    info "continuing with --skip-gpu would not validate the NVIDIA interaction"
    exit 1
fi

NVIDIA_DRV="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
NVIDIA_GPU="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
NVIDIA_PKG="$(dpkg-query -W -f='${db:Status-Status} ${Package}\n' 'nvidia-driver-*' 2>/dev/null \
              | awk '/^installed/ {print $2; exit}')"
[ -n "$NVIDIA_DRV" ] && ok "NVIDIA $NVIDIA_DRV ($NVIDIA_PKG) on $NVIDIA_GPU"

sudo -v || { fail "cyclictest needs sudo"; exit 1; }

# ── cyclictest wrapper ───────────────────────────────────────────────────────
# Returns "min avg_worst max" in microseconds. avg_worst is the highest per-thread
# average, not the mean of averages: one bad core is what breaks a control loop.

run_cyclictest() {
    local tag="$1" duration="$2" affinity="$3"
    local log="$TMP/$tag.log" cmd

    if [ -n "$affinity" ]; then
        cmd=(sudo taskset -c "$affinity" cyclictest --mlockall --priority="$PRIORITY" \
             --interval="$INTERVAL" --duration="$duration" --threads="${#ISO_ARR[@]}" --quiet)
    else
        cmd=(sudo cyclictest --mlockall --smp --priority="$PRIORITY" \
             --interval="$INTERVAL" --distance=0 --duration="$duration" --quiet)
    fi

    "${cmd[@]}" > "$log" 2>&1
    printf '%s\n' "${cmd[*]}" > "$TMP/$tag.cmd"

    local mn av mx
    mn="$(grep -oP 'Min:\s*\K[0-9]+' "$log" | sort -n | head -1)"
    av="$(grep -oP 'Avg:\s*\K[0-9]+' "$log" | sort -n | tail -1)"
    mx="$(grep -oP 'Max:\s*\K[0-9]+' "$log" | sort -n | tail -1)"
    echo "${mn:-0} ${av:-0} ${mx:-0}"
}

start_cpu_load() {  # $1 = affinity ("" for all cores)
    if [ -n "$1" ]; then
        local n; n="$(awk -F, '{print NF}' <<< "$1")"
        taskset -c "$1" stress-ng --cpu "$n" --timeout "$((DUR_LOAD + 15))" >/dev/null 2>&1 &
    else
        stress-ng --cpu "$NCPU" --timeout "$((DUR_LOAD + 15))" >/dev/null 2>&1 &
    fi
    BG_PIDS+=("$!")
}

start_gpu_load() {  # $1 = "nvidia" to force PRIME offload onto the discrete GPU
    [ "$SKIP_GPU" = 1 ] && return 0
    if [ "$1" = "nvidia" ]; then
        __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
            glmark2 --run-forever >/dev/null 2>&1 &
    else
        glmark2 --run-forever >/dev/null 2>&1 &
    fi
    BG_PIDS+=("$!")
}

# On hybrid-graphics laptops glmark2 silently falls back to the Intel iGPU, which would make
# condition 3 a CPU-only test wearing a GPU label. Sample utilization so the report can prove
# the discrete GPU was actually busy.
start_gpu_sampler() {
    [ "$SKIP_GPU" = 1 ] && return 0
    ( while :; do
        nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null
        sleep 2
      done ) > "$TMP/gpu_util.log" 2>/dev/null &
    BG_PIDS+=("$!")
}

stop_load() {
    pkill -f 'glmark2' 2>/dev/null
    pkill -f 'stress-ng' 2>/dev/null
    for p in "${BG_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    BG_PIDS=()
    sleep 2
}

gpu_stats() {  # "mean peak" from the sampler log
    [ -s "$TMP/gpu_util.log" ] || { echo "0 0"; return; }
    awk '{s+=$1; if($1>m) m=$1; n++} END {printf "%d %d", (n? s/n : 0), m+0}' "$TMP/gpu_util.log"
}

# ── Condition 1 — idle baseline ──────────────────────────────────────────────
head2 "Condition 1/3 — idle baseline, no isolation (${DUR_IDLE}s)"
info "establishes the floor; every later number is read against this"
read -r C1_MIN C1_AVG C1_MAX <<< "$(run_cyclictest idle "$DUR_IDLE" "")"
ok "min ${C1_MIN} us / avg ${C1_AVG} us / max ${C1_MAX} us"

# ── Condition 2 — load, no isolation ─────────────────────────────────────────
head2 "Condition 2/3 — CPU + GPU load, no isolation (${DUR_LOAD}s)"
info "cyclictest competes with the load on every core — expect this to be the worst"
start_cpu_load ""
start_gpu_load "default"
sleep 3
read -r C2_MIN C2_AVG C2_MAX <<< "$(run_cyclictest load_noiso "$DUR_LOAD" "")"
stop_load
ok "min ${C2_MIN} us / avg ${C2_AVG} us / max ${C2_MAX} us"

# ── Condition 3 — load, with isolation (the shipping configuration) ──────────
head2 "Condition 3/3 — CPU + NVIDIA GPU load, isolated cores (${DUR_LOAD}s)"
info "load pinned to $HOUSE_CSV, cyclictest pinned to $ISO_CSV"
info "GPU work forced onto the discrete NVIDIA card via PRIME offload"
start_cpu_load "$HOUSE_CSV"
start_gpu_load "nvidia"
start_gpu_sampler
sleep 3
read -r C3_MIN C3_AVG C3_MAX <<< "$(run_cyclictest load_iso "$DUR_LOAD" "$ISO_CSV")"
read -r GPU_MEAN GPU_PEAK <<< "$(gpu_stats)"
stop_load
ok "min ${C3_MIN} us / avg ${C3_AVG} us / max ${C3_MAX} us"
if [ "$SKIP_GPU" = 0 ]; then
    if [ "${GPU_PEAK:-0}" -lt 20 ]; then
        warn "NVIDIA GPU peaked at only ${GPU_PEAK}% — PRIME offload probably did not take"
        info "glmark2 likely rendered on the iGPU, so this did NOT test the NVIDIA module"
        info "the result below is not valid evidence for decision 8"
    else
        ok "NVIDIA GPU utilization during the run: ${GPU_MEAN}% mean, ${GPU_PEAK}% peak"
    fi
fi

# ── Verdict ──────────────────────────────────────────────────────────────────
# Budget: worst-case latency must stay under 10% of the control period (200 us at 500 Hz).
PERIOD_US=2000
BUDGET_US=200
PCT="$(awk -v m="$C3_MAX" -v p="$PERIOD_US" 'BEGIN {printf "%.1f", 100*m/p}')"
if [ "${C3_MAX:-999999}" -le "$BUDGET_US" ]; then
    VERDICT="PASS"
else
    VERDICT="FAIL"
fi

head2 "Verdict"
if [ "$VERDICT" = "PASS" ]; then
    ok "PASS — ${C3_MAX} us worst case = ${PCT}% of a 500 Hz period (budget: 10%)"
else
    fail "FAIL — ${C3_MAX} us worst case = ${PCT}% of a 500 Hz period (budget: 10%)"
    info "Do not silently accept this. Check IRQ affinity, SMI/BIOS power management,"
    info "and whether anything is still scheduled on the isolated cores."
fi

# ── Report ───────────────────────────────────────────────────────────────────
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d)"
CPU_MODEL="$(awk -F': ' '/model name/ {print $2; exit}' /proc/cpuinfo)"

# The machine is identified by CLASS, never by hostname. This used to be `hostname -s`, which
# put a username-derived string into a COMMITTED filename and into the JSON — and since this
# tree becomes a double-anonymous review capsule, that is an identity leak generated fresh on
# every run, so scrubbing the files without changing this line would only postpone it.
# A class is also the more useful label: what a reader needs in order to judge a latency
# number is the core topology and the isolation map, not which laptop it was.
# Override ARTS_MACHINE_CLASS to something meaningful for the machine (e.g. "ws-16c32t" for
# the production workstation); the default is derived from the CPU topology so it is never
# personal and never empty.
if [ -z "${ARTS_MACHINE_CLASS:-}" ]; then
    _cores="$(awk -F': ' '/cpu cores/ {print $2; exit}' /proc/cpuinfo)"
    ARTS_MACHINE_CLASS="${_cores:-?}c${NCPU}t"
fi
HOST="$ARTS_MACHINE_CLASS"
BASE="$OUT_DIR/rt_latency_${HOST}_${STAMP}"
DISTRO="$(. /etc/os-release && echo "$PRETTY_NAME")"

cat > "$BASE.json" <<EOF
{
  "benchmark": "rt_latency",
  "date": "$(date -Iseconds)",
  "verdict": "$VERDICT",
  "host": {
    "machine_class": "$HOST",
    "cpu": "$CPU_MODEL",
    "logical_cpus": $NCPU,
    "distro": "$DISTRO",
    "kernel": "$KERNEL",
    "kernel_version": "$KERNEL_VER",
    "isolated_cores": "$ISOLATED",
    "housekeeping_cores": "$HOUSE_CSV",
    "nvidia_driver": "$NVIDIA_DRV",
    "nvidia_package": "$NVIDIA_PKG",
    "gpu": "$NVIDIA_GPU"
  },
  "parameters": {
    "priority": $PRIORITY,
    "interval_us": $INTERVAL,
    "duration_idle_s": $DUR_IDLE,
    "duration_load_s": $DUR_LOAD
  },
  "results_us": {
    "idle_no_isolation":        {"min": $C1_MIN, "avg": $C1_AVG, "max": $C1_MAX},
    "load_no_isolation":        {"min": $C2_MIN, "avg": $C2_AVG, "max": $C2_MAX},
    "load_nvidia_isolated":     {"min": $C3_MIN, "avg": $C3_AVG, "max": $C3_MAX}
  },
  "gpu_utilization_pct": {"mean": ${GPU_MEAN:-0}, "peak": ${GPU_PEAK:-0}},
  "budget": {"period_us": $PERIOD_US, "budget_us": $BUDGET_US, "used_pct": $PCT}
}
EOF

cat > "$BASE.md" <<EOF
# PREEMPT_RT latency — $HOST, $(date +%Y-%m-%d)

**Verdict: $VERDICT** — worst-case ${C3_MAX} us under load on isolated cores, ${PCT}% of a
500 Hz control period (budget: 10%, i.e. ${BUDGET_US} us).

This is the measurement behind architecture decision 8 (single-PC realtime). It asks whether
the NVIDIA kernel module injects unbounded scheduling latency on PREEMPT_RT while control
threads run on isolated cores.

## Host

| | |
|---|---|
| CPU | $CPU_MODEL ($NCPU logical) |
| GPU | $NVIDIA_GPU |
| Distro | $DISTRO |
| Kernel | \`$KERNEL\` |
| | \`$KERNEL_VER\` |
| Isolated cores | \`$ISOLATED\` |
| Housekeeping cores | \`$HOUSE_CSV\` |
| NVIDIA driver | $NVIDIA_DRV (\`$NVIDIA_PKG\`) |

## Results

cyclictest at priority $PRIORITY, ${INTERVAL} us interval. \`avg\` is the highest per-thread
average rather than the mean across threads — one bad core is what breaks a control loop.

| Condition | Min | Avg | **Max** |
|---|---|---|---|
| Idle, no isolation (${DUR_IDLE}s) | ${C1_MIN} us | ${C1_AVG} us | ${C1_MAX} us |
| CPU + GPU load, no isolation (${DUR_LOAD}s) | ${C2_MIN} us | ${C2_AVG} us | ${C2_MAX} us |
| CPU + **NVIDIA** GPU load, **isolated** (${DUR_LOAD}s) | ${C3_MIN} us | ${C3_AVG} us | **${C3_MAX} us** |

NVIDIA GPU utilization during the third condition: ${GPU_MEAN:-0}% mean, ${GPU_PEAK:-0}% peak.
This is recorded because glmark2 silently falls back to the integrated GPU on hybrid-graphics
laptops; a low number here means the discrete GPU was never exercised and the result does not
support any claim about the NVIDIA module.

## What this does and does not establish

It establishes that the NVIDIA kernel module servicing GPU interrupts does not inject
unbounded latency into SCHED_FIFO threads on isolated cores. That is the specific failure
mode decision 8 was worried about, and it is the reason the second row exists: without
isolation the same load produces a much larger tail on the same machine.

It does not establish that the shipping system meets its deadlines. glmark2 is a graphics
benchmark, not the ARTS GPU workload, and it exercises the driver at a moderate duty cycle
rather than saturating it. The real load — GPU JPEG encoding, the stereo display path, and
Isaac Lab — has a different interrupt profile. Nor does cyclictest measure the control loop
itself; it measures the scheduler's ability to wake a thread on time, which is a necessary
but not sufficient condition. The deadline-miss numbers that back the paper's control claims
come from controller_manager diagnostics during the integration-ladder runs, not from here.

## Reproducing

\`\`\`bash
./deploy/bench/rt_latency.sh
\`\`\`

Commands executed:

\`\`\`
$(cat "$TMP/idle.cmd" 2>/dev/null)
$(cat "$TMP/load_noiso.cmd" 2>/dev/null)
$(cat "$TMP/load_iso.cmd" 2>/dev/null)
\`\`\`
EOF

head2 "Report"
ok "$BASE.md"
ok "$BASE.json"
info "commit both — the RT claim in the paper cites these numbers"
