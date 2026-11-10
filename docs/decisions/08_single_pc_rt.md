# Decision 8 — Single-PC real time

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

8. **Single-PC RT — PROVEN, on two kernels (2026-08-05 and re-validated 2026-08-10).** On the
   development laptop (i7-9750H, RTX 2070 Mobile) with `isolcpus=4,5,10,11`, under simultaneous
   CPU saturation and NVIDIA GPU rendering, worst-case cyclictest latency on isolated cores was
   **73 us** (avg 5 us) over 5 minutes on the current stack — 3.6% of the 500 Hz period.
   The 22.04 predecessor (5.15.0-1081-realtime + proprietary 595.84) gave 190 us, so the
   conclusion survived the migration with room to spare.
   PRODUCTION WORKSTATION, 2026-10-26: Core Ultra 9 285K + RTX 5080, 6.8.1-1062-realtime,
   `nvidia-driver-595-open` 595.99.02, `isolcpus=2,4,6,7` (P-cores): **19 us** worst case
   (avg 1 us) = 0.9% of the 500 Hz period; 37 us on the same load without isolation.
   QUOTE 19 us FOR THE SYSTEM — it is the machine the paper's experiments run on — and the
   laptop's 73 us as the cross-machine replication. Caveat: glmark2 is a fixed workload, so
   the faster GPU ran at 18% mean / 69% peak against the laptop's 33%; the real ARTS GPU
   load is still to be measured on the integration ladder.
   The RT control path has no GPU dependency by design; GPU work runs in separate processes
   on non-isolated cores. The two-PC and lowlatency-baseline fallbacks are no longer needed.
   Re-run with `deploy/bench/rt_latency.sh` after any kernel, driver or BIOS change.
