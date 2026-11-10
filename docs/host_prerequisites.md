# Host prerequisites — the machine-dependent choices

Everything in ARTS is version-pinned except the handful of things that depend on *your*
hardware and *your* vendor accounts. Those cannot be pinned, so they are documented here
instead: what the choice is, how to make it correctly on your machine, and which value
ARTS has actually validated.

`deploy/prereqs.sh` walks through this interactively and never installs anything without your
confirmation. `arts_install.sh` runs it first. Alongside the four choices below it can also
install the parts that are not choices at all — ROS 2 Jazzy, Docker CE, and the NVIDIA
Container Toolkit — each following the vendor's current documented procedure.

```bash
./arts_install.sh --prereqs     # interactive; asks before every install
./deploy/prereqs.sh --check     # report only, never prompts
```

Four items need a decision:

| # | Item | Why it can't be pinned | Validated: production workstation | Validated: development laptop |
|---|---|---|---|---|
| 1 | NVIDIA driver branch | Depends on your GPU | 595.99.02 via `nvidia-driver-595-open` on RTX 5080 | 595.84 via `nvidia-driver-595-open` on RTX 2070 Mobile |
| 2 | PREEMPT_RT kernel | Requires an Ubuntu Pro account | 6.8.1-1062-realtime | 6.8.1-1057-realtime |
| 3 | `isolcpus` core map | Depends on your core count and topology | `2,4,6,7` on a Core Ultra 9 285K (8P+16E) | `4,5,10,11` on a 6C/12T i7-9750H |
| 4 | FLIR Spinnaker SDK | Requires a Teledyne FLIR account | 4.2.0.88 | 4.2.0.88 |

The workstation stack was validated 2026-10-26 at 19 µs worst-case RT latency:
[`docs/benchmarks/rt_latency_ultra9-285k-8p16e_20261026.md`](benchmarks/rt_latency_ultra9-285k-8p16e_20261026.md).

Validated together on 2026-08-10: `uname -v` reports `#58-Ubuntu SMP PREEMPT_RT`,
`/sys/devices/system/cpu/isolated` reports `4-5,10-11`, and both `nvidia-smi` and
`docker run --gpus all` reach the GPU under the realtime kernel. RT latency was re-measured
on this exact stack the same day — 73 µs worst case, 3.6% of a 500 Hz period, improving on
the 190 µs from the 22.04 predecessor. Full report:
[`docs/benchmarks/rt_latency_devlaptop-6c12t_20260810.md`](benchmarks/rt_latency_devlaptop-6c12t_20260810.md).

---

## 1. NVIDIA driver — pick the branch Ubuntu recommends for *your* GPU

**Do not copy a version number from this document.** The correct driver depends on your GPU,
and installing a branch that is too old for your card, or a server branch on a workstation,
causes problems that look like ARTS bugs. Ask Ubuntu what it recommends:

```bash
ubuntu-drivers devices
```

The line ending in `distro non-free recommended` is the answer for your card. On the ARTS
development laptop (RTX 2070 Mobile, Ubuntu 24.04) it reads:

```
model    : TU106M [GeForce RTX 2070 Mobile]
driver   : nvidia-driver-595-open - distro non-free recommended
driver   : nvidia-driver-595      - distro non-free
```

Install the recommended package, or its non-`-open` sibling at the same branch number:

```bash
sudo apt install nvidia-driver-595        # substitute YOUR recommended branch
sudo reboot
nvidia-smi                                # confirm the driver loaded
```

### `-open` vs proprietary, and why it matters for PREEMPT_RT

Two flavors ship at every recent branch: `nvidia-driver-<N>` (the classic proprietary kernel
module) and `nvidia-driver-<N>-open` (NVIDIA's open GPU kernel modules). They are the same
userspace driver with a different kernel module.

**Both flavours refuse to build on a PREEMPT_RT kernel.** This was verified directly on the
ARTS development machine with `nvidia-dkms-595-open`: `conftest.sh` carries a
`preempt_rt_sanity_check` that aborts with

```
The kernel you are installing for is a PREEMPT_RT kernel!
The NVIDIA driver does not support real-time kernels.
*** Failed PREEMPT_RT sanity check. Bailing out! ***
```

Choosing `-open` does **not** avoid this. The workaround is NVIDIA's own escape hatch,
`IGNORE_PREEMPT_RT_PRESENCE`, which `conftest.sh` honours in both flavours.

Overriding a vendor's explicit refusal deserves a word of justification. The check is a
support boundary, not a detected incompatibility: NVIDIA declines to support realtime
kernels, and the module compiles and runs once told to proceed. ARTS has run this
configuration on two kernels now (5.15 proprietary, 6.8.1 `-open`) with a working GPU under
load. It is nonetheless the one place in the stack where you are outside vendor support, so
treat a GPU regression after a driver bump as suspect here first.

**ARTS-validated:** `nvidia-driver-595-open` (595.84) on 6.8.1-1057-realtime with the
bypass, running the display, passing through to Docker, and measured at 73 µs worst-case RT
latency under GPU load. The predecessor configuration — the proprietary module on 5.15, also
with the bypass — gave 190 µs. Both pass; see `README.md` for the full comparison.

On Ubuntu 24.04 the recommendation for the same GPU resolves to `nvidia-driver-595-open`
instead — the same 595.84 userspace driver with NVIDIA's open kernel modules, installed as
prebuilt signed modules rather than through DKMS. That changes *how* you hit the PREEMPT_RT
problem, not whether you hit it, and it adds a second, quieter trap on top.

### The prebuilt-module trap: no NVIDIA driver on the realtime kernel

Ubuntu builds those prebuilt modules **per kernel flavour**, and it publishes them for the
generic flavour only:

```
linux-modules-nvidia-595-open-7.0.0-28-generic     ← exists
linux-modules-nvidia-595-open-6.8.1-1057-realtime  ← does not exist, for any version
```

Because the prebuilt route satisfies the driver's dependency, `dkms` is never pulled in.
So enabling the realtime kernel gives you a kernel with **no NVIDIA driver at all** — not a
build failure you can read in a log, just a missing module you discover after rebooting.
On a hybrid-graphics laptop the desktop still comes up on the integrated GPU, which makes it
easy to miss; `nvidia-smi` simply reports no devices.

Do all of this **before** rebooting into the RT kernel, while you are still on a working one.
Three steps, in order:

```bash
# 1. DKMS itself, plus the matching driver source
sudo apt install dkms nvidia-dkms-595-open

# 2. The RT bypass, as a per-module DKMS override so apt upgrades cannot undo it
#    (NOT framework.conf — see "Where the bypass goes" below)
sudo tee /etc/dkms/nvidia.conf <<'EOF'
MAKE[0]="export IGNORE_PREEMPT_RT_PRESENCE=1; ${MAKE[0]}"
EOF

# 3. Build for the RT kernel EXPLICITLY — see the note below
sudo dkms install nvidia/595.84 -k <rt-kernel>
```

Step 3 is the one that catches people. The `nvidia-dkms-*` postinst builds only for the
**currently running** kernel, so installing the package while booted on generic produces
modules for generic and silently nothing for the realtime kernel. `-k` is what targets the
other one. Verify:

```bash
dkms status                                     # expect an entry for the RT kernel
ls /lib/modules/<rt-kernel>/updates/dkms/       # expect nvidia.ko, nvidia-drm.ko, ...
```

This needs `linux-headers-<rt-kernel>`, which `pro enable realtime-kernel` installs for you.
**That makes the order matter once DKMS is present:** the RT headers carry a DKMS hook that
builds NVIDIA for the new kernel *inside* `pro enable`. Without the bypass already in place
the hook fails, the kernel packages are left half-configured (`iF`/`iU` in `dpkg -l`), the
RT kernel gets no initrd, and `/etc/default/grub.d/99-realtime.cfg` stays as `.dpkg-new`, so
GRUB is never pointed at it. `pro` reports this only as a retried, then failed, apt
operation. Write the bypass (step 2) before enabling the realtime kernel, not after.
If Secure Boot is enabled, DKMS modules must also be signed and the key enrolled via MOK at
the next boot; with Secure Boot disabled there is nothing extra to do.

Building from the generic kernel is a feature, not a compromise: the sanity check inspects
the *target* kernel's headers, so you find out whether the module compiles before the reboot
commits you to a kernel that might have no GPU.

Afterwards, consider holding the packages. The bypass survives upgrades, but a new driver
branch on a realtime kernel is a combination nobody has tested:

```bash
sudo apt-mark hold nvidia-driver-595-open nvidia-dkms-595-open
```

`deploy/prereqs.sh` does all of this: it writes the bypass before `pro enable` when DKMS is
present, detects missing modules, installs DKMS, runs the explicit `-k` build, retries with
the bypass when a build refuses with the PREEMPT_RT message, and exits non-zero listing any
half-configured package or initrd-less RT kernel rather than finishing with "Done.". It also
stops for a reboot straight after installing a driver. Carrying on in that session is what
froze the production workstation: with nvidia-container-toolkit 1.20 installed,
`nvidia-cdi-refresh.service` loads the freshly built module into the live session while
nouveau still drives the display.

Once the RT kernel boots with a working GPU, measure your own latency rather than trusting
these numbers — the result depends on your CPU, BIOS power management and IRQ routing, not
just on the software stack:

```bash
./deploy/bench/rt_latency.sh
```

### Where the bypass goes: `/etc/dkms/nvidia.conf`, not the command line or `framework.conf`

Applies to **both** driver flavours. A one-off environment variable on a single
`dkms install` does not survive: any later `apt install` or `apt upgrade` re-triggers
`dpkg --configure nvidia-dkms-<N>`, which deletes and re-extracts the DKMS source tree and
rebuilds without your flag.

The obvious persistent home, `echo 'export IGNORE_PREEMPT_RT_PRESENCE=1' >>
/etc/dkms/framework.conf`, was sourced directly by jammy's dkms 2.8, and this document recommended it
until 2026-10. **On noble's dkms 3.0.11 it is silently ignored.** dkms 3 reads
`framework.conf` through `safe_source`, which sources the file in a subshell and re-exports
only an allowlist of dkms's own variables, so the line looks applied and does nothing.
It was found on the production workstation, where `dpkg --configure -a` failed the same
sanity check with the line present.

dkms also reads per-module overrides, `/etc/dkms/<module>.conf`, after the driver's own
`dkms.conf`, and `MAKE` is on that allowlist. Because the override is sourced where the
driver's `MAKE[0]` is already visible, it can prefix the driver's build command instead of
replacing it, so the same file works for every driver version. No package owns it, so
upgrades leave it alone. Step 2 above writes it.

If a package is already wedged mid-configure from a failed build:

```bash
sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg --configure -a
```

Reboot to load the new module rather than `modprobe nvidia` — modprobing into a live session
restarts the display server and black-screens the machine.

---

## 2. PREEMPT_RT kernel — needs a (free) Ubuntu Pro account

Ubuntu ships the realtime kernel through Ubuntu Pro. A personal subscription is free for up
to five machines; sign up at <https://ubuntu.com/pro> and copy your token.

```bash
sudo pro attach <your-token>
sudo pro enable realtime-kernel
```

These are two independent operations, and the second one is what installs the kernel. A
machine can stay attached indefinitely with `realtime-kernel` still `disabled` — that is
exactly the state `pro attach` alone leaves behind. Check with `pro status`, or let
`deploy/prereqs.sh` check for you; it reads the JSON form and offers to enable the service
if the machine is already attached. Attaching an already-attached machine fails, which is
harmless but must not be allowed to skip the enable step.

Two consequences on 24.04 that are easy to miss, both confirmed on the ARTS development
machine, and both different from what the 22.04 procedure did:

**Livepatch is disabled.** `realtime-kernel` and `livepatch` are mutually exclusive, and
`pro enable` turns Livepatch off to proceed. You lose live kernel patching and will need
reboots for kernel security updates. That is the correct trade for a control host, but it
should be a decision, not a surprise.

**The realtime kernel becomes the default boot entry, not an extra one.** Enablement writes
`/etc/default/grub.d/99-realtime.cfg` containing `GRUB_FLAVOUR_ORDER="realtime"`, which sorts
the RT kernel ahead of generic in the menu, so `GRUB_DEFAULT=0` now selects it. To put
generic back in front, delete that file and re-run `sudo update-grub`.

Those two combine badly with Ubuntu's stock `GRUB_TIMEOUT_STYLE=hidden` and `GRUB_TIMEOUT=0`:
the machine will boot straight into a kernel it has never booted before, with no visible menu
to fall back from. Before the first RT boot, make the menu visible:

```
GRUB_TIMEOUT_STYLE=menu
GRUB_TIMEOUT=5
```

`deploy/prereqs.sh` detects the hidden menu and offers this edit. Then confirm:

```bash
uname -r      # ends in -realtime
uname -v      # contains PREEMPT_RT
```

The realtime kernel version tracks the Ubuntu release, so it differs from the validated one:
22.04 provides `5.15.*-realtime` (what ARTS validated), 24.04 provides `6.8.*-realtime`
(`6.8.1-1057-realtime` on the development machine). Note that this can be *older* than the
generic kernel you are running — the development machine's generic kernel is `7.0.0-28`.
Re-run cyclictest after switching releases rather than assuming the old numbers carry over.

Also note that `GRUB_CMDLINE_LINUX_DEFAULT` is global: the `isolcpus` map from section 3
applies to every entry, so the generic kernel boots with isolated cores too.

If you cannot use Ubuntu Pro, `linux-lowlatency` is a weaker fallback — it will run ARTS but
will not meet the jitter target, and RT results measured on it are not comparable.

---

## 3. `isolcpus` — depends on your core topology

The validated map isolates physical cores 4–5 (logical 4, 5, 10, 11) on a 6-core /
12-thread i7-9750H. On a different CPU the numbers are different, and copying them blindly
either isolates the wrong cores or isolates cores that do not exist.

Find your topology, then reserve two or more physical cores (both SMT siblings of each) for
the RT control threads:

```bash
lscpu --extended        # maps logical CPUs to physical cores
```

Three kernel parameters work together: `isolcpus` removes the cores from the general
scheduler, `nohz_full` stops the timer tick on them when idle, `rcu_nocbs` offloads RCU
callbacks elsewhere. Append all three to `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`:

```
isolcpus=4,5,10,11 nohz_full=4,5,10,11 rcu_nocbs=4,5,10,11
```

```bash
sudo update-grub && sudo reboot
cat /sys/devices/system/cpu/isolated    # → 4-5,10-11
```

`deploy/prereqs.sh` asks how many cores to isolate (default 2, or `ARTS_ISOLCPUS_CORES`),
computes a candidate map from sysfs, shows it, and offers to append the three parameters
for you. It backs up `/etc/default/grub` first and refuses to touch the file if `isolcpus`
is already set there. The rules, in order:

1. **The unit is an L2 cache group, not a logical CPU.** On an SMT part that is a core's
   two siblings; on an Intel hybrid part it is also each E-core cluster, whose four cores
   share one L2. Isolating part of a group leaves the RT thread competing for its cache.
2. **Performance cores only** on hybrid parts (`/sys/devices/cpu_core/cpus`).
3. **Never the group holding CPU 0**, which carries boot-time work and much of the default
   IRQ affinity.
4. **Leave the highest-boost ("favoured") cores to the scheduler**, which prefers them for
   single-threaded work, unless that leaves too few groups.
5. Of what remains, the **highest-numbered** groups.

Worked example, Core Ultra 9 285K (8 P-cores = CPUs 0–7, each with a private L2;
16 E-cores = CPUs 8–23 in four L2 clusters; CPUs 3 and 5 boost to 5.7 GHz, the rest to
5.4 GHz): 2 cores → `6,7`, 4 cores → `2,4,6,7`. The rule used before 2026-10-21 —
highest-numbered `CORE` from `lscpu -p` — picked `22,23` there: two E-cores sharing an L2
with the unisolated 20 and 21. On the i7-9750H both rules give `4,5,10,11`.

Validate with the cyclictest procedure in `README.md` and commit your numbers — the RT
latency budget is 10% of the control period (200 µs at 500 Hz).

---

## 4. Vendor SDKs behind an account wall

These cannot be fetched by a script because the vendor requires a login and EULA acceptance.
`arts_install.sh` detects them and tells you exactly what to download and where to put it;
it never downloads them for you.

| SDK | Account | Validated version | Extract to |
|---|---|---|---|
| FLIR Spinnaker | Free Teledyne account, <https://www.teledynevisionsolutions.com/products/spinnaker-sdk/> (Ubuntu 22.04 amd64 tarball; older releases under "All Versions") | 4.2.0.88 | `$ARTS_LOCAL/oem_drivers/spinnaker-4.2.0.88/` |
| Atracsys fusionTrack | Atracsys customer portal | 4.7.1 | `$ARTS_LOCAL/oem_drivers/fusiontrack_v4.7.1/` |
| Zivid | Free Zivid account | 2.14.0 | `$ARTS_LOCAL/oem_drivers/zivid-2.14.0/` |

Versions are pinned in `oem_drivers/<sdk>/VERSION`. A newer SDK usually works, but it is a
pin bump: rebuild, re-run that module's benchmark, and commit the new baseline next to the
old one under `docs/benchmarks/`.

**Spinnaker is needed only for the stereo cameras.** The mock robot, simulation and the arms
without the endoscope run without it: `ARTS_ENABLE_FLIR=0 ./arts_install.sh` leaves out the
FLIR module, `flir_ros2_ws` and (Docker) `arts-flir:jazzy`, and builds `arts:jazzy` on
`arts-base:jazzy`. `arts_ws_build` then skips the `spinnaker_camera_driver` rosdep key, because
rosdep would otherwise install `ros-jazzy-spinnaker-camera-driver`, whose deb carries its own
`libSpinnaker.so.4.4.0.99` (checked 2026-11-09).

**If Teledyne no longer offers 4.2.0.88.** On 2026-11-09 an outside tester found the old
flir.com download link dead (404; Teledyne moved the downloads to the page above) and their
account offered only a 4.4.0 release. Extract whatever release you can get into
`$ARTS_LOCAL/oem_drivers/spinnaker-<its version>/` and run
`ARTS_SPINNAKER_VERSION=<its version> ./arts_install.sh`. Every FLIR function then prints that
the SDK is not the pin; any camera number measured on that install must state the version.

---

## Known host issues — open

A running log of host-level oddities that are observed but not yet root-caused. Anything
here is a candidate explanation when something downstream fails inexplicably. Move an entry
to "resolved" with the cause when it is understood, rather than deleting it.

### Intermittent `Failed to unshare namespaces: EINVAL` under the realtime kernel

**Observed:** 2026-08-10, first session on 6.8.1-1057-realtime. A sandboxed shell command
failed to create a user namespace, then the identical command succeeded on retry. Docker was
unaffected before and after.

**Recurred 2026-08-18**, so it is not the one-off the first entry assumed. Same shape: an
`apt-cache` query in a sandboxed shell died with `Step 1-2/7 (user namespace) failed`, and the
identical query succeeded immediately when run unsandboxed. Both occurrences were read-only
commands touching nothing unusual, which argues against the failure being triggered by what
the command does. Still not root-caused; still harmless on retry.

**Why it is worth tracking:** unprivileged user-namespace creation is what container and
sandbox tooling relies on, so an intermittent failure here can surface much later as a
confusing container, build-isolation, or CI failure that looks like a Docker bug. Ubuntu
24.04 restricts unprivileged user namespaces through AppArmor
(`kernel.apparmor_restrict_unprivileged_userns`), which is a plausible interaction with the
realtime kernel's different preemption and AppArmor profile handling, but nothing is
confirmed.

**If it recurs, capture before doing anything else:**

```bash
sysctl kernel.apparmor_restrict_unprivileged_userns kernel.unprivileged_userns_clone
sudo dmesg -T | grep -iE "apparmor|userns|namespace" | tail -40
sudo journalctl -k -b --since "10 min ago" | grep -iE "apparmor|userns"
```

Do not "fix" it by globally disabling the AppArmor restriction — that is a real security
control, and a workaround applied before the cause is known would go into the published
install instructions. Establish first whether it is realtime-specific by testing the same
command under the generic kernel.
