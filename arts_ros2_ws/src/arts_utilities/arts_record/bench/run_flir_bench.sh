#!/usr/bin/env bash
# run_flir_bench.sh — orchestrate a FLIR camera benchmark inside Docker.
#
# Records a single camera at 40 fps for DURATION seconds, then runs the
# offline analyzer on the resulting MCAP bag.  Docker-stats samples are
# collected every 5 s in the background for resource-usage reporting.
#
# Prerequisites:
#   - arts-flir:jazzy image built (./deploy/docker/build_flir.sh)
#   - Camera plugged in via USB3
#   - Host net.core.rmem_max >= 10 MB (see README "Network buffer" section)
#
# Usage:
#   ./run_flir_bench.sh [DURATION_S] [SERIAL] [OUTPUT_BASE]
#
# Outputs go to OUTPUT_BASE/<timestamp>/:
#   flir_bench/          rosbag2 MCAP bag directory
#   docker_stats.csv     container resource samples
#   image_digest.txt     image ID of the arts-flir:jazzy image used
#   host_config.json     kernel, isolcpus, driver, sysctls the run happened on
#   bench_report.json    machine-readable results
#   bench_report.md      human-readable report (copy to docs/benchmarks/)
set -euo pipefail

DURATION_S="${1:-600}"
SERIAL="${2:-24057131}"
OUTPUT_BASE="${3:-/tmp/arts_bench}"
IMAGE="arts-flir:jazzy"
BENCH_DIR="$(cd "$(dirname "$0")" && pwd)"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="$OUTPUT_BASE/$TIMESTAMP"
mkdir -p "$OUTPUT_DIR"

# ── image digest ──
DIGEST=$(docker inspect --format='{{.Id}}' "$IMAGE")
echo "$DIGEST" > "$OUTPUT_DIR/image_digest.txt"

# ── host configuration ──
# A benchmark number is only comparable against another run on a known host.
# Kernel flavor, core isolation, and the CycloneDDS receive buffer all move
# these results, so they are recorded next to the image digest.
# `nproc` reports CPUs available after the isolcpus mask, not the machine's
# total, so both are recorded — otherwise an isolated host looks smaller than
# it is and the CPU percentages below get read against the wrong denominator.
KERNEL_RELEASE="$(uname -r)"
KERNEL_PREEMPT="$(uname -v | grep -o 'PREEMPT_RT\|PREEMPT_DYNAMIC\|PREEMPT' | head -1 || echo unknown)"
ISOLATED_CPUS="$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo '')"
NVIDIA_DRIVER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo none)"
RMEM_MAX="$(sysctl -n net.core.rmem_max 2>/dev/null || echo unknown)"
USBFS_MB="$(cat /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || echo unknown)"
DOCKER_VERSION="$(docker --version 2>/dev/null | awk '{print $3}' | tr -d ,)"

cat > "$OUTPUT_DIR/host_config.json" <<EOF
{
  "kernel_release": "$KERNEL_RELEASE",
  "kernel_preempt": "$KERNEL_PREEMPT",
  "isolated_cpus": "$ISOLATED_CPUS",
  "cpu_model": "$(lscpu | sed -n 's/^Model name: *//p' | head -1)",
  "nproc": $(nproc --all),
  "nproc_available": $(nproc),
  "nvidia_driver": "$NVIDIA_DRIVER",
  "net_core_rmem_max": "$RMEM_MAX",
  "usbfs_memory_mb": "$USBFS_MB",
  "docker_version": "$DOCKER_VERSION",
  "image": "$IMAGE",
  "image_digest": "$DIGEST"
}
EOF

echo "════════════════════════════════════════════════"
echo " FLIR benchmark"
echo "   Image:    $IMAGE"
echo "   Digest:   ${DIGEST:0:19}..."
echo "   Kernel:   $KERNEL_RELEASE ($KERNEL_PREEMPT)"
echo "   Isolated: ${ISOLATED_CPUS:-none}"
echo "   rmem_max: $RMEM_MAX"
echo "   Serial:   $SERIAL"
echo "   Duration: ${DURATION_S}s"
echo "   Output:   $OUTPUT_DIR"
echo "════════════════════════════════════════════════"

if [ -z "$ISOLATED_CPUS" ]; then
    echo "[BENCH] NOTE: no isolated CPUs — results are not comparable with"
    echo "        baselines taken under isolcpus. Recorded in host_config.json."
fi

# ── start recording container ──
# Duration is controlled by `timeout --signal=INT` inside the container so
# ros2 launch gets a clean SIGINT and rosbag2 finalizes the bag (writes
# metadata.yaml and the MCAP summary section).  The 30 s grace period after
# the INT lets the writer flush large image buffers before we hard-stop.
GRACE=30
CONTAINER=$(docker run -d --init --privileged --net=host \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$BENCH_DIR":/bench:ro \
    -v "$OUTPUT_DIR":/bench_output \
    "$IMAGE" \
    bash -c "timeout --signal=INT --kill-after=${GRACE} $((DURATION_S + 5)) \
        ros2 launch /bench/flir_bench.launch.py \
            serial:='$SERIAL' \
            output:='/bench_output/flir_bench' \
            mcap_config:='/bench/mcap_image_config.yaml'")

echo "[BENCH] Container $CONTAINER started"

# ── background stats collector ──
# Poll .State.Running, not plain `docker inspect`: inspect keeps succeeding for an
# EXITED container and only fails once it is removed, so the naive loop never ends.
# It also appends 0%/0B rows after recording stops, which drags the reported mean
# CPU down and makes the run look cheaper than it was.
(
    echo "timestamp,cpu_pct,mem_usage,mem_pct"
    while [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; do
        LINE=$(docker stats --no-stream \
            --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}}' \
            "$CONTAINER" 2>/dev/null || true)
        # Skip the all-zero sample docker emits as a container tears down.
        case "$LINE" in
            ''|0.00%,0B\ /\ 0B*) ;;
            *) echo "$(date +%s),$LINE" ;;
        esac
        sleep 5
    done
) > "$OUTPUT_DIR/docker_stats.csv" &
STATS_PID=$!

# ── wait for recording + shutdown grace ──
TOTAL_WAIT=$((DURATION_S + 5 + GRACE + 10))
echo "[BENCH] Recording for ${DURATION_S}s (container timeout in $((DURATION_S + 5))s) ..."
docker wait "$CONTAINER" >/dev/null 2>&1 &
WAIT_PID=$!

ELAPSED=0
while kill -0 "$WAIT_PID" 2>/dev/null && [ "$ELAPSED" -lt "$TOTAL_WAIT" ]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
done

if kill -0 "$WAIT_PID" 2>/dev/null; then
    echo "[BENCH] Container still running after ${TOTAL_WAIT}s, force-stopping..."
    docker stop -t 5 "$CONTAINER" >/dev/null 2>&1 || true
fi
wait "$WAIT_PID" 2>/dev/null || true

# stop stats collector
kill "$STATS_PID" 2>/dev/null || true
wait "$STATS_PID" 2>/dev/null || true

# ── verify bag was written ──
if [ ! -d "$OUTPUT_DIR/flir_bench" ]; then
    echo "[BENCH] ERROR: bag directory not found at $OUTPUT_DIR/flir_bench" >&2
    echo "[BENCH] Container logs:" >&2
    docker logs "$CONTAINER" 2>&1 | tail -40 || true
    docker rm "$CONTAINER" >/dev/null 2>&1 || true
    exit 1
fi

# remove the stopped container
docker rm "$CONTAINER" >/dev/null 2>&1 || true

# ── run offline analyzer in a fresh container ──
echo "[BENCH] Analyzing ..."
docker run --rm \
    -v "$OUTPUT_DIR":/bench_output \
    -v "$BENCH_DIR":/bench:ro \
    "$IMAGE" \
    python3 /bench/analyze_bench.py /bench_output/flir_bench \
        --nominal-hz 40.0 \
        --docker-stats /bench_output/docker_stats.csv \
        --image-digest "$DIGEST" \
        --host-config /bench_output/host_config.json \
        --output-dir /bench_output

echo ""
echo "════════════════════════════════════════════════"
echo " Done.  Results in $OUTPUT_DIR/"
echo "   bench_report.md   — copy to docs/benchmarks/"
echo "   bench_report.json — machine-readable"
echo "════════════════════════════════════════════════"
