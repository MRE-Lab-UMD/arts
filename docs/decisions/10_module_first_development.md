# Decision 10 — Module-first development and the benchmark ladder

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

10. **Module-first development, in Docker on the current PC.** The host stays minimal (kernel,
    NVIDIA driver, docker + nvidia-container-toolkit, PTP daemons); everything else runs in
    containers. Every hardware module gets its own docker compose profile with ONLY its driver
    + ROS wrapper, and must pass a standard benchmark before integration. Benchmark harness
    contract (every driver package honors it): a bench launch file + offline analyzer
    reporting achieved rate (mean ± std from MCAP stamps), CPU%, RSS, disk/net I/O, and drop
    count; results are committed under `docs/benchmarks/` together with the image digest they
    ran on. Integration climbs a ladder, re-running the identical benchmarks at each rung:
    module solo → contention pairs (both cameras; cameras + recorder; arms + recorder) → full
    patient cart → full system with teleop and display. Each module gets an up-front resource
    budget (CPU cores from the isolcpus map, NIC allocation, NVMe lane) and passes against its
    budget, not just in isolation. Reminder: the kernel is never inside Docker — containers
    share the host kernel, so the PREEMPT_RT question is a host dual-boot test (see decision
    8), not a container concern.
