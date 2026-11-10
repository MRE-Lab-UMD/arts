# PREEMPT_RT latency — ultra9-285k-8p16e, 2026-10-26

**Verdict: PASS** — worst-case 19 us under load on isolated cores, 0.9% of a
500 Hz control period (budget: 10%, i.e. 200 us).

This is the measurement behind architecture decision 8 (single-PC realtime). It asks whether
the NVIDIA kernel module injects unbounded scheduling latency on PREEMPT_RT while control
threads run on isolated cores.

## Host

| | |
|---|---|
| CPU | Intel(R) Core(TM) Ultra 9 285K (24 logical) |
| GPU | NVIDIA GeForce RTX 5080 |
| Distro | Ubuntu 24.04.4 LTS |
| Kernel | `6.8.1-1062-realtime` |
| | `#63-Ubuntu SMP PREEMPT_RT Wed Sep 30 13:49:30 UTC 2026` |
| Isolated cores | `2,4,6-7` |
| Housekeeping cores | `0,1,3,5,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23` |
| NVIDIA driver | 595.99.02 (`nvidia-driver-595-open`) |

## Results

cyclictest at priority 80, 1000 us interval. `avg` is the highest per-thread
average rather than the mean across threads — one bad core is what breaks a control loop.

| Condition | Min | Avg | **Max** |
|---|---|---|---|
| Idle, no isolation (60s) | 0 us | 1 us | 13 us |
| CPU + GPU load, no isolation (300s) | 1 us | 1 us | 37 us |
| CPU + **NVIDIA** GPU load, **isolated** (300s) | 0 us | 1 us | **19 us** |

NVIDIA GPU utilization during the third condition: 18% mean, 69% peak.
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

```bash
./deploy/bench/rt_latency.sh
```

Commands executed:

```
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=60 --quiet
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=300 --quiet
sudo taskset -c 2,4,6,7 cyclictest --mlockall --priority=80 --interval=1000 --duration=300 --threads=4 --quiet
```
