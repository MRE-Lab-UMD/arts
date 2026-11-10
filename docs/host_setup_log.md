# Host setup log and Docker images

**Reference (moved out of README.md on 2026-11-07).** How the development hosts were set up (RT
kernel, NVIDIA driver on the RT kernel, Docker, DDS buffers, `isolcpus`, the cyclictest) and how the
Docker images are layered. The unpinnable host choices and the open host issues are in
[`host_prerequisites.md`](host_prerequisites.md).

## Docker images

ARTS uses a layered Docker image approach: one base image for all modules, then per-module
images that extend it.

### Base image (`arts-base:jazzy`)

Provides the foundation: ROS 2 Jazzy on Ubuntu 24.04 (noble) with CycloneDDS as the RMW
(a deliberate departure from the default Fast DDS — chosen for shared-memory simplicity
and better large-data throughput on GigE camera topics), rosbag2 with MCAP storage, and
ros2_control. MCAP is Jazzy's default bag format, so no `-s mcap` flag is needed anywhere
in the recording path.

```bash
# build (not `docker build` directly — the middleware layer is shared with the
# bare-metal path and has to be staged into the build context first)
./deploy/docker/build_base.sh

# verify
docker run --rm arts-base:jazzy bash -c 'echo $RMW_IMPLEMENTATION'
# → rmw_cyclonedds_cpp

# verify GPU passthrough
docker run --rm --gpus all arts-base:jazzy nvidia-smi
```

### Middleware (both paths)

CycloneDDS is configured by `deploy/cyclonedds.xml`, installed to
`/etc/cyclonedds/cyclonedds.xml` on the host and copied to the same path in the image.
It is one file rather than two on purpose: bare metal and Docker are equally first-class
(architecture decision 4) and the benchmark ladder compares results across them, which is
meaningless if the two run different middleware settings.

Both the package and the config come from `deploy/modules/dds.sh`, which the bare-metal
installer and `Dockerfile.base` each call, so they cannot drift:

```bash
# bare metal — installs rmw_cyclonedds_cpp, the config, the rmem sysctl, and
# /etc/profile.d/arts-dds.sh; needs sudo
bash -c 'source deploy/modules/dds.sh && dds_host_deps'
```

`deploy/bench/dds_throughput.py` checks the transport carries the patient cart's image
load (2 × 6.29 MB frames at 40 Hz = 503 MB/s) using synthetic frames, so it runs with no
cameras attached and works identically on both paths.

### Module images and services

Only FLIR has an image of its own (`arts-flir:jazzy`, which carries the Spinnaker SDK from
the user's own download). Everything else is built into one image, `arts:jazzy`, and each
hardware module is a service of it, selected by a Docker Compose profile in
`deploy/docker/compose.yaml`. No image is published: every user builds them locally
(`./arts_install.sh --path docker`), because FLIR's SDK may not be baked into a public image
(decision 4). As of 2026-11-08 the Docker path is verified on mock hardware only (smoke test,
three-arm bringup with RViz and the gamepad); the hardware profiles below have not been run
against the real devices.

```bash
docker compose --profile flir up        # FLIR camera module only
docker compose --profile ur up          # UR arm driver only
docker compose --profile patient-cart up # all patient cart modules
```

---

## Host setup log

Development PC setup steps and results. These are the reference for the
`deploy/host_setup.sh` deliverable and for reproducing the environment on another machine.

The choices below that depend on *your* hardware and vendor accounts — GPU driver branch,
PREEMPT_RT kernel, `isolcpus` map, account-walled SDKs — are documented separately in
[`docs/host_prerequisites.md`](host_prerequisites.md), and
`./arts_install.sh --prereqs` walks through them interactively. Do not copy the version
numbers in this log; they are what was validated here, not what is correct for your machine.

> **Re-validated on Ubuntu 24.04 / ROS 2 Jazzy on 2026-08-10.** Two things below differ from
> the current machine: the Ubuntu Pro realtime kernel is `6.8.1-1057-realtime` rather than
> `5.15.*`, and `ubuntu-drivers` recommends the `-open` NVIDIA kernel modules. The `-open`
> modules still require the `IGNORE_PREEMPT_RT_PRESENCE` DKMS bypass — they carry the same
> PREEMPT_RT refusal as the proprietary ones, contrary to what this file previously said.
> RT latency was re-measured on the new stack and improved; both result sets are below.
> For a fresh install, follow [`docs/host_prerequisites.md`](host_prerequisites.md)
> rather than this log — it is the maintained procedure, and `./arts_install.sh --prereqs`
> automates it.

### Development PC

- **CPU:** Intel i7-9750H — 6 cores / 12 threads
- **GPU:** NVIDIA GeForce RTX 2070 Mobile (8 GB)
- **OS:** Ubuntu 24.04.4 (noble) — was 22.04 for the results below

### Kernel: PREEMPT_RT (Ubuntu Pro)

Dual-boot: generic `6.8.0-136-generic` remains as the default GRUB entry; RT kernel
`5.15.0-1081-realtime` selected manually at boot via GRUB menu.

```bash
# enable Ubuntu Pro realtime kernel (requires Ubuntu Pro subscription)
sudo pro attach <token>            # skip if already attached
sudo pro enable realtime-kernel    # installs the RT kernel as a second GRUB entry

# reboot, select the realtime kernel from GRUB Advanced Options, then verify:
uname -r                           # should show *-realtime
uname -v                           # should contain PREEMPT_RT
```

```
$ uname -r
5.15.0-1081-realtime
```

### NVIDIA driver 595.84 on the RT kernel

`sudo apt install nvidia-driver-595` installs userspace libraries but DKMS fails on
the RT kernel — the closed-source driver includes a `PREEMPT_RT` sanity check that
explicitly refuses to build. The fix: NVIDIA's own `IGNORE_PREEMPT_RT_PRESENCE`
environment variable bypasses the check. Reboot after — do NOT `modprobe nvidia`
mid-session (it restarts the display server and black-screens).

```bash
sudo apt install nvidia-driver-595                                        # DKMS fails on RT — expected
sudo IGNORE_PREEMPT_RT_PRESENCE=1 dkms install nvidia/595.84 -k $(uname -r)
sudo reboot
nvidia-smi                                                                # should show 595.84
```

**Making the bypass permanent (required):** Any future `apt install` or `apt upgrade`
re-triggers `dpkg --configure nvidia-dkms-595`, which deletes and re-extracts the DKMS
source tree — wiping a per-module `dkms.conf` edit. The fix is the system-wide DKMS
config, which survives package operations:

```bash
echo 'export IGNORE_PREEMPT_RT_PRESENCE=1' | sudo tee -a /etc/dkms/framework.conf
sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg --configure -a      # finishes the stuck package
sudo apt-mark hold nvidia-dkms-595 nvidia-driver-595        # prevent apt from re-triggering
```

After this, DKMS successfully built all five kernel modules (`nvidia.ko`, `nvidia-modeset.ko`,
`nvidia-drm.ko`, `nvidia-uvm.ko`, `nvidia-peermem.ko`) for both `5.15.0-1081-realtime` and
`6.8.0-136-generic`. Verified 2026-08-05.

> **Does not carry over to 24.04 (found 2026-10-22).** Jammy's dkms 2.8 sources
> `framework.conf` directly; noble's dkms 3.0.11 re-exports only its own variables from it,
> so the `framework.conf` line above is silently ignored there. On 24.04 the bypass goes in
> `/etc/dkms/nvidia.conf` — see `docs/host_prerequisites.md` §1, "Where the bypass goes".

Result (2026-08-04):
```
NVIDIA-SMI 595.84    Driver Version: 595.84    CUDA Version: 13.2
GPU: NVIDIA GeForce RTX 2070 Mobile    55°C    5W/90W    99MiB/8192MiB
```

### Docker

On Ubuntu 24.04 (2026-08-10): Docker CE **29.7.2** from Docker's own repository,
nvidia-container-toolkit **1.19.1-1**. (The 22.04 host ran 29.1.3 and 1.13.5.)
`./arts_install.sh --prereqs` installs both.

```bash
# install (pin all four packages to the same stable version)
sudo apt install -y nvidia-container-toolkit=1.19.1-1 \
  nvidia-container-toolkit-base=1.19.1-1 \
  libnvidia-container-tools=1.19.1-1 \
  libnvidia-container1=1.19.1-1

# configure Docker runtime and restart
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# verify GPU access inside a container
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

Verified (2026-08-05): `nvidia-smi` inside container shows RTX 2070, driver 595.84,
CUDA 13.2.

### Network buffer for CycloneDDS

CycloneDDS with the ARTS `cyclonedds.xml` config requests a 10 MB socket receive buffer.
The kernel default `rmem_max` (212992 bytes on this PC) is too small and causes
`rmw_create_node` to fail with "failed to increase socket receive buffer size". Required
for any ROS 2 node using CycloneDDS — affects both bare-metal and Docker (`--net=host`).

```bash
# set immediately
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# make persistent across reboots
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.d/10-cyclonedds.conf
echo "net.core.rmem_default=26214400" | sudo tee -a /etc/sysctl.d/10-cyclonedds.conf
sudo sysctl --system
```

`deploy/modules/dds.sh` now does all of the above, so the manual steps are the record of
why rather than something to run. Note the failure is not always as loud as the
`rmw_create_node` error above: when the socket request merely exceeds `rmem_max`, the
kernel caps it silently and the only symptom is dropped frames under image load.

### CPU isolation (`isolcpus`)

Physical cores 4–5 (logical 4, 5, 10, 11) isolated from the general scheduler for RT
control threads. Three kernel parameters work together: `isolcpus` removes cores from
scheduling, `nohz_full` stops the timer tick on idle isolated cores, `rcu_nocbs` offloads
RCU callbacks off those cores.

```bash
# edit GRUB config
sudo nano /etc/default/grub

# append inside the GRUB_CMDLINE_LINUX_DEFAULT quotes:
#   isolcpus=4,5,10,11 nohz_full=4,5,10,11 rcu_nocbs=4,5,10,11
# e.g.: GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=4,5,10,11 nohz_full=4,5,10,11 rcu_nocbs=4,5,10,11"

sudo update-grub
sudo reboot

# verify after reboot:
cat /sys/devices/system/cpu/isolated    # should show 4-5,10-11
```

### RT + GPU compatibility test — cyclictest (PASSED, 2026-08-05)

Install test tools:

```bash
sudo apt install -y rt-tests stress-ng glmark2
```

**Test 1 — Idle baseline (60s, no isolation, no load):**

```bash
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=60
```

**Test 2 — CPU + GPU load, no isolation (300s):**

```bash
# terminal 1: CPU stress on all cores
stress-ng --cpu $(nproc) --timeout 300

# terminal 2: GPU stress (ran on Intel iGPU by default — see note below)
glmark2 --run-forever

# terminal 3: cyclictest
sudo cyclictest --mlockall --smp --priority=80 --interval=1000 --distance=0 --duration=300
```

**Test 3 — CPU + NVIDIA GPU load, with `isolcpus` (300s):**

```bash
# terminal 1: CPU stress on non-isolated cores + GPU stress forced onto NVIDIA via PRIME
taskset -c 0-3,6-9 stress-ng --cpu 8 --timeout 300 &
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glmark2 --run-forever

# terminal 2: cyclictest on isolated cores only
sudo taskset -c 4,5,10,11 cyclictest --mlockall --priority=80 --interval=1000 --duration=300 --threads=4
```

Note: on hybrid GPU laptops, `glmark2` defaults to the Intel iGPU. The `__NV_PRIME_RENDER_OFFLOAD`
and `__GLX_VENDOR_LIBRARY_NAME` env vars force rendering onto the NVIDIA GPU, which is what we
need to test (the NVIDIA kernel module servicing GPU interrupts while RT threads run).

**Results:**

Both runs are the same machine and the same protocol. The 24.04 row set is the current
configuration; the 22.04 set is kept because the migration changed the distro, the realtime
kernel and the NVIDIA kernel-module flavour at once, and a single unexplained regression
later is much easier to place with both on the page.

| Condition | 22.04 / 5.15 avg → max | 24.04 / 6.8.1 avg → max |
|---|---|---|
| Idle, no `isolcpus` (60s) | 3 us → 678 us | 3 us → 211 us |
| CPU + GPU load, no `isolcpus` (300s) | 6–8 us → 800 us | 7 us → 410 us |
| CPU + **NVIDIA GPU** load, with `isolcpus` (300s) | **4–5 us → 190 us** | **5 us → 73 us** |

73 us worst-case = 3.6% of a 500 Hz control period (2 ms). The average is unchanged between
the two stacks; the whole improvement is in the tail, which points at the newer kernel rather
than the driver flavour.

Note that without `isolcpus` this machine misses the 200 us budget even at idle. Core
isolation is a requirement, not a tuning refinement.

**Decision 8 outcome: single-PC RT is proven.** The NVIDIA kernel module does not inject
unbounded latency on PREEMPT_RT when control threads run on isolated cores — now confirmed on
two kernels and both module flavours. Production workstation (16+ cores) will have even more
isolation headroom.

This is no longer a hand-run procedure. `./deploy/bench/rt_latency.sh` reproduces all three
conditions, derives the core map from `/sys/devices/system/cpu/isolated` rather than
hardcoding it, verifies that the discrete GPU was actually loaded (glmark2 silently falls back
to the iGPU on hybrid laptops), and writes a committed report:
[`docs/benchmarks/rt_latency_devlaptop-6c12t_20260810.md`](benchmarks/rt_latency_devlaptop-6c12t_20260810.md).
