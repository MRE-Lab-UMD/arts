# PREEMPT_RT latency — devlaptop-6c12t, 2026-08-10

**Verdict: PASS** — worst-case 73 us under load on isolated cores, 3.6% of a
500 Hz control period (budget: 10%, i.e. 200 us).

This is the measurement behind architecture decision 8 (single-PC realtime). It asks whether
the NVIDIA kernel module injects unbounded scheduling latency on PREEMPT_RT while control
threads run on isolated cores.

## Host

| | |
|---|---|
| CPU | Intel(R) Core(TM) i7-9750H CPU @ 2.60GHz (12 logical) |
| GPU | NVIDIA GeForce RTX 2070 with Max-Q Design |
| Distro | Ubuntu 24.04.4 LTS |
| Kernel | `6.8.1-1057-realtime` |
| | `#58-Ubuntu SMP PREEMPT_RT Mon Jul 20 15:39:03 UTC 2026` |
| Isolated cores | `4-5,10-11` |
| Housekeeping cores | `0,1,2,3,6,7,8,9` |
| NVIDIA driver | 595.84 (`nvidia-driver-595-open`) |

## Results

cyclictest at priority 80, 1000 us interval. `avg` is the highest per-thread
average rather than the mean across threads — one bad core is what breaks a control loop.

| Condition | Min | Avg | **Max** |
|---|---|---|---|
| Idle, no isolation (60s) | 2 us | 3 us | 211 us |
| CPU + GPU load, no isolation (300s) | 4 us | 7 us | 410 us |
| CPU + **NVIDIA** GPU load, **isolated** (300s) | 4 us | 5 us | **73 us** |

NVIDIA GPU utilization during the third condition: 33% mean, 41% peak.
This is recorded because glmark2 silently falls back to the integrated GPU on hybrid-graphics
laptops; a low number here means the discrete GPU was never exercised and the result does not
support any claim about the NVIDIA module.

## Comparison with the 22.04 baseline

Same machine, same protocol, same isolated cores. What changed is the distro (jammy → noble),
the realtime kernel (5.15.0-1081 → 6.8.1-1057) and the NVIDIA kernel-module flavour
(proprietary → `-open`, both built through DKMS with `IGNORE_PREEMPT_RT_PRESENCE`).

| Condition (worst case) | 22.04 / 5.15 | 24.04 / 6.8.1 | Change |
|---|---|---|---|
| Idle, no isolation | 678 us | 211 us | −69% |
| Load, no isolation | 800 us | 410 us | −49% |
| **Load + NVIDIA, isolated** | **190 us** | **73 us** | **−62%** |

The average on the shipping configuration is unchanged at 4–5 us; the entire improvement is
in the tail. That pattern — same central tendency, shorter tail — points at the kernel rather
than at the driver flavour, since a driver that behaved differently under load would move the
average too. The improvement was not the goal and is not claimed as one; what matters is that
migrating to 24.04 did not cost RT headroom.

Note that the un-isolated configurations exceed the 200 us budget even at idle. Isolation is
not a tuning refinement on this hardware, it is the difference between meeting and missing
the deadline, which is worth stating plainly whenever the `isolcpus` requirement is
questioned as an inconvenience for outside users.

## Reproducing

```bash
./deploy/bench/rt_latency.sh
```

Commands executed:

```
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=60 --quiet
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=300 --quiet
sudo taskset -c 4,5,10,11 cyclictest --mlockall --priority=80 --interval=1000 --duration=300 --threads=4 --quiet
```
