#!/usr/bin/env bash
# deploy/prereqs.sh — the machine-dependent choices, walked through interactively.
#
# Everything else in ARTS is version-pinned. These four items cannot be, because they
# depend on the user's GPU, CPU topology, and vendor accounts:
#
#   1. NVIDIA driver branch      — whatever `ubuntu-drivers devices` recommends for YOUR GPU
#   2. PREEMPT_RT kernel         — needs a free Ubuntu Pro account
#   3. isolcpus core map         — depends on YOUR core count
#   4. Vendor SDKs               — behind account walls (FLIR, Haply, Atracsys, Zivid)
#
# This script reports the current state, prints the ARTS-validated value for reference,
# and asks before doing anything. It installs nothing without confirmation and is safe to
# re-run. Rationale for every choice: docs/host_prerequisites.md
#
# Usage:
#   ./deploy/prereqs.sh              # interactive walkthrough
#   ./deploy/prereqs.sh --check      # report only, never prompt (CI-safe)
set -uo pipefail

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"
ROS_DISTRO_ARTS="jazzy"

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# Validated on the ARTS development laptop — for comparison, NOT to copy blindly.
VALIDATED_GPU="RTX 2070 Mobile"
VALIDATED_NVIDIA="595.84 (nvidia-driver-595, proprietary)"
VALIDATED_RT_KERNEL="5.15.0-1081-realtime (Ubuntu 22.04)"
VALIDATED_ISOLCPUS="4,5,10,11 (6C/12T i7-9750H)"

bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
ok()    { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn()  { printf '  \033[33m!\033[0m %s\n' "$*"; }
info()  { printf '    %s\n' "$*"; }
head2() { printf '\n\033[1m── %s\033[0m\n' "$*"; }

# A step that was attempted and did not complete. Collected and re-printed at the
# end, and the script then exits non-zero: an install that scrolls past a failure
# and finishes with "Done." is how a broken RT kernel reached its first boot.
FAILURES=()
fail_note() {
    printf '  \033[31m✗\033[0m %s\n' "$*"
    FAILURES+=("$*")
}

# Ask a yes/no question. In --check mode always answers "no".
confirm() {
    [ "$CHECK_ONLY" = 1 ] && return 1
    local reply
    read -r -p "  → $1 [y/N] " reply
    [[ "$reply" =~ ^[Yy]$ ]]
}

# ── Ubuntu Pro state ─────────────────────────────────────────────────
# `pro status` prints a banner telling you not to parse it; the JSON form is the
# supported interface.  Attachment and per-service state are independent.

pro_is_attached() {
    command -v pro >/dev/null 2>&1 || return 1
    pro status --format json 2>/dev/null | python3 -c '
import json, sys
try:
    sys.exit(0 if json.load(sys.stdin).get("attached") else 1)
except Exception:
    sys.exit(1)
' 2>/dev/null
}

pro_service_status() {
    command -v pro >/dev/null 2>&1 || return 0
    pro status --format json 2>/dev/null | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(0)
for s in d.get("services", []):
    if s.get("name") == sys.argv[1]:
        print(s.get("status", ""))
        break
' "$1" 2>/dev/null
}

# ── GRUB safety before a first boot into a new kernel ────────────────
# On 24.04 `pro enable realtime-kernel` drops GRUB_FLAVOUR_ORDER="realtime" into
# /etc/default/grub.d/, which makes the RT kernel the DEFAULT entry rather than
# an extra one — different from 22.04. Ubuntu also ships a hidden, zero-timeout
# menu, so a kernel that fails to boot leaves no obvious way back to the old one.
grub_boot_warnings() {
    local flavour_file=/etc/default/grub.d/99-realtime.cfg
    if [ -f "$flavour_file" ] && grep -qs 'realtime' "$flavour_file"; then
        warn "$flavour_file sets GRUB_FLAVOUR_ORDER=realtime"
        info "The realtime kernel becomes the DEFAULT boot entry, not an extra one."
        info "To make generic the default again, delete that file and run update-grub."
    fi

    local timeout style
    timeout="$(sed -n 's/^GRUB_TIMEOUT=//p' /etc/default/grub 2>/dev/null | tr -d '"')"
    style="$(sed -n 's/^GRUB_TIMEOUT_STYLE=//p' /etc/default/grub 2>/dev/null | tr -d '"')"

    if [ "$style" = "hidden" ] || [ "${timeout:-0}" = "0" ]; then
        warn "the GRUB menu is hidden (style=${style:-unset}, timeout=${timeout:-unset})"
        info "You need the menu to fall back if the new kernel does not boot, or if"
        info "the NVIDIA module fails to build against it."
        if confirm "show the GRUB menu for 5 s at boot?"; then
            sudo cp /etc/default/grub "/etc/default/grub.arts.bak.$(date +%Y%m%d%H%M%S)"
            if grep -q '^GRUB_TIMEOUT_STYLE=' /etc/default/grub; then
                sudo sed -i 's/^GRUB_TIMEOUT_STYLE=.*/GRUB_TIMEOUT_STYLE=menu/' /etc/default/grub
            else
                echo 'GRUB_TIMEOUT_STYLE=menu' | sudo tee -a /etc/default/grub >/dev/null
            fi
            sudo sed -i 's/^GRUB_TIMEOUT=.*/GRUB_TIMEOUT=5/' /etc/default/grub
            sudo update-grub && ok "GRUB menu will be shown for 5 s at boot"
        fi
    fi
}

# ── NVIDIA modules for a kernel you have not booted yet ──────────────
# Ubuntu's `-open` driver installs PREBUILT signed modules, but only for the
# generic flavour (linux-modules-nvidia-<N>-open-<ver>-generic). No equivalent
# package exists for the realtime flavour, and the prebuilt route does not pull
# in DKMS — so a freshly enabled RT kernel can have NO NVIDIA driver at all, and
# you only discover it after rebooting into a GPU-less desktop.
#
# Building now, while still on a working kernel, also tests the module against
# the RT headers before the reboot commits to it.
# NVIDIA's conftest.sh aborts the build on any PREEMPT_RT kernel — the `-open`
# modules carry the same check as the proprietary ones — but it honours an
# explicit escape hatch. A one-off env var is not enough: any later apt install
# or upgrade re-runs the build from a package postinst without it.
#
# It must NOT go in /etc/dkms/framework.conf. dkms 3.x reads that file through an
# allowlist of its own variables and silently drops everything else, so the line
# looks applied and does nothing (verified on noble, dkms 3.0.11). The per-module
# override /etc/dkms/nvidia.conf is read after the driver's dkms.conf with MAKE on
# the allowlist, so it can prefix the driver's own build command — any version.
DKMS_RT_CONF=/etc/dkms/nvidia.conf

ensure_dkms_rt_bypass() {
    if grep -qs 'IGNORE_PREEMPT_RT_PRESENCE' "$DKMS_RT_CONF"; then
        ok "DKMS PREEMPT_RT bypass present in $DKMS_RT_CONF"
        return 0
    fi
    info "NVIDIA ships an escape hatch for its PREEMPT_RT refusal:"
    info "IGNORE_PREEMPT_RT_PRESENCE=1, written to $DKMS_RT_CONF so every"
    info "automatic rebuild (kernel install, driver upgrade) carries it."
    confirm "write $DKMS_RT_CONF?" || return 1
    sudo tee "$DKMS_RT_CONF" >/dev/null <<'EOF'
# Written by ARTS deploy/prereqs.sh. NVIDIA refuses to build on PREEMPT_RT kernels
# unless IGNORE_PREEMPT_RT_PRESENCE=1; framework.conf cannot carry it (dkms drops
# unlisted variables there), so it prefixes the driver's own MAKE command here.
MAKE[0]="export IGNORE_PREEMPT_RT_PRESENCE=1; ${MAKE[0]}"
EOF
    ok "bypass written to $DKMS_RT_CONF"
}

nvidia_uses_dkms() {
    dpkg-query -W -f='${db:Status-Status} ${Package}\n' 'nvidia-dkms-*' 2>/dev/null \
        | grep -q '^installed'
}

# A failed DKMS hook leaves kernel packages half-configured, and on 24.04 that also
# strands the realtime GRUB flavour file as .dpkg-new — with nothing printed by pro.
check_half_configured() {
    local broken
    broken="$(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' 2>/dev/null \
              | awk '$1 != "ii" && $1 != "rc" && $1 != "un" && $1 != "pn" && $1 != "hi" {print $2}')"
    if [ -n "$broken" ]; then
        fail_note "packages left half-configured:"
        printf '      %s\n' $broken
        info "If a DKMS build refused PREEMPT_RT, fix the bypass and then run:"
        info "    sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg --configure -a"
        return 1
    fi
    if ls /etc/default/grub.d/*.dpkg-new >/dev/null 2>&1; then
        fail_note "unapplied GRUB config: $(ls /etc/default/grub.d/*.dpkg-new | xargs)"
        return 1
    fi
    return 0
}

# Identify the registered DKMS module as "name/version", from dkms itself if it
# has a record, otherwise from the source tree apt just unpacked.
dkms_nvidia_module() {
    local m
    m="$(dkms status 2>/dev/null | awk -F, '/^nvidia/ {gsub(/ /,"",$1); print $1; exit}')"
    if [ -z "$m" ]; then
        local d
        d="$(ls -d /usr/src/nvidia-* 2>/dev/null | head -1)"
        [ -n "$d" ] && m="nvidia/$(basename "$d" | sed 's/^nvidia-//')"
    fi
    printf '%s' "$m"
}

check_nvidia_for_rt_kernels() {
    command -v nvidia-smi >/dev/null 2>&1 || return 0

    local dkms_pkg=""
    case "$nvidia_pkg" in
        nvidia-driver-*) dkms_pkg="${nvidia_pkg/driver/dkms}" ;;
    esac

    local kdir kver mod
    for kdir in /lib/modules/*realtime*; do
        [ -d "$kdir" ] || continue
        kver="$(basename "$kdir")"
        [ "$kver" = "$(uname -r)" ] && continue

        if find "$kdir" -name 'nvidia.ko*' 2>/dev/null | grep -q .; then
            ok "NVIDIA driver modules present for $kver"
            continue
        fi

        warn "no NVIDIA driver modules for $kver — nvidia-smi will fail there"
        info "Ubuntu ships prebuilt NVIDIA modules for the generic flavour only."
        info "The realtime kernel needs DKMS to build them from source."

        if [ ! -d "/usr/src/linux-headers-$kver" ]; then
            warn "kernel headers missing too"
            info "    sudo apt install linux-headers-$kver"
            continue
        fi

        # Install dkms if it is not there yet.
        if ! command -v dkms >/dev/null 2>&1; then
            if [ -z "$dkms_pkg" ]; then
                info "Install dkms and the matching nvidia-dkms-* package by hand."
                continue
            fi
            info "Fix: sudo apt install dkms $dkms_pkg"
            confirm "install dkms + $dkms_pkg now?" || continue
            sudo apt-get install -y dkms "$dkms_pkg" || continue
        fi

        # The nvidia-dkms postinst only builds for the RUNNING kernel, so the
        # non-running RT kernel has to be requested explicitly by -k.
        mod="$(dkms_nvidia_module)"
        if [ -z "$mod" ]; then
            warn "no nvidia DKMS source registered — 'sudo apt install $dkms_pkg'"
            continue
        fi

        info "Building $mod for $kver (the postinst only did the running kernel)"
        confirm "run 'sudo dkms install $mod -k $kver' now?" || continue
        sudo dkms install "$mod" -k "$kver" || true

        if find "$kdir" -name 'nvidia.ko*' 2>/dev/null | grep -q .; then
            ok "NVIDIA modules built for $kver"
            continue
        fi

        # The expected failure on an RT kernel: recover from it rather than
        # leaving the user to read a DKMS log at 2 a.m.
        local log="/var/lib/dkms/${mod%%/*}/${mod##*/}/build/make.log"
        if grep -qs 'PREEMPT_RT sanity check' "$log"; then
            warn "the build refused: NVIDIA does not support PREEMPT_RT kernels"
            if ensure_dkms_rt_bypass; then
                info "retrying the build with the bypass"
                sudo IGNORE_PREEMPT_RT_PRESENCE=1 dkms install "$mod" -k "$kver" || true
                if find "$kdir" -name 'nvidia.ko*' 2>/dev/null | grep -q .; then
                    ok "NVIDIA modules built for $kver (with the RT bypass)"
                    info "Consider 'sudo apt-mark hold $nvidia_pkg ${dkms_pkg:-}': the"
                    info "bypass survives upgrades, but a new driver branch is untested on RT."
                else
                    fail_note "NVIDIA build for $kver still failing — see $log"
                fi
            else
                fail_note "NVIDIA build for $kver refused PREEMPT_RT and the bypass was declined"
            fi
        else
            fail_note "still no NVIDIA module for $kver — see $log"
        fi
    done
}

# ── CPU isolation ────────────────────────────────────────────────────
# The unit of isolation is an L2 CACHE GROUP, not a logical CPU: every CPU that
# shares an L2 with an isolated one must be isolated too, or the RT thread still
# competes for that cache. On an SMT part the group is a core's two siblings; on
# an Intel hybrid part each E-core cluster of four shares one L2, a grouping
# `lscpu -p=CPU,CORE` does not show. Rules, in order:
#   1. Hybrid Intel parts: performance cores only (/sys/devices/cpu_core/cpus).
#   2. Never the group holding CPU 0 — boot-time work and default IRQ affinity.
#   3. Skip the highest-boost ("favoured") cores, which the scheduler prefers for
#      single-threaded work, unless that leaves too few groups.
#   4. Of what remains, the highest-numbered groups.
SYSFS_CPU="${ARTS_SYSFS_CPU:-/sys/devices/system/cpu}"
SYSFS_DEV="${ARTS_SYSFS_DEV:-/sys/devices}"

expand_cpulist() {
    local part
    for part in ${1//,/ }; do
        if [[ "$part" == *-* ]]; then seq "${part%-*}" "${part#*-}"; else echo "$part"; fi
    done
}

l2_group() {
    local idx
    for idx in "$SYSFS_CPU/cpu$1"/cache/index*; do
        if [ "$(cat "$idx/level" 2>/dev/null)" = 2 ]; then
            cat "$idx/shared_cpu_list"
            return
        fi
    done
    cat "$SYSFS_CPU/cpu$1/topology/thread_siblings_list" 2>/dev/null || echo "$1"
}

recommend_isolcpus() {
    local want="${1:-2}" c g m ok f fmax floor=""
    local -a cpus groups=() kept=()
    local -A cand=() seen=() gfreq=()

    mapfile -t cpus < <(expand_cpulist "$(cat "$SYSFS_CPU/online" 2>/dev/null)")
    [ "${#cpus[@]}" -gt 0 ] || return 1

    if [ -r "$SYSFS_DEV/cpu_core/cpus" ]; then
        for c in $(expand_cpulist "$(cat "$SYSFS_DEV/cpu_core/cpus")"); do cand[$c]=1; done
    else
        for c in "${cpus[@]}"; do cand[$c]=1; done
    fi

    for c in "${cpus[@]}"; do
        g="$(l2_group "$c")"
        [ -n "${seen[$g]:-}" ] && continue
        seen[$g]=1
        ok=1; fmax=0
        for m in $(expand_cpulist "$g"); do
            { [ -n "${cand[$m]:-}" ] && [ "$m" != 0 ]; } || ok=0
            f="$(cat "$SYSFS_CPU/cpu$m/cpufreq/cpuinfo_max_freq" 2>/dev/null || echo 0)"
            [ "$f" -gt "$fmax" ] && fmax="$f"
        done
        [ "$ok" = 1 ] || continue
        groups+=("$g"); gfreq[$g]="$fmax"
        { [ -z "$floor" ] || [ "$fmax" -lt "$floor" ]; } && floor="$fmax"
    done

    for g in "${groups[@]}"; do
        [ "${gfreq[$g]}" -le "$floor" ] && kept+=("$g")
    done
    [ "${#kept[@]}" -ge "$want" ] || kept=("${groups[@]}")

    printf '%s\n' "${kept[@]}" | sort -t, -k1,1n | tail -n "$want" \
        | while read -r g; do expand_cpulist "$g"; done | sort -n | paste -sd,
}

apply_isolcpus() {
    local map="$1" grub=/etc/default/grub
    local params="isolcpus=$map nohz_full=$map rcu_nocbs=$map"

    if [ ! -f "$grub" ]; then
        warn "$grub not found — apply the parameters by hand"
        return 1
    fi
    if grep -q 'isolcpus=' "$grub"; then
        warn "$grub already sets isolcpus — leaving it alone, edit it by hand"
        return 1
    fi

    local backup="$grub.arts.bak.$(date +%Y%m%d%H%M%S)"
    sudo cp "$grub" "$backup"
    sudo sed -i "s|^\(GRUB_CMDLINE_LINUX_DEFAULT=\"[^\"]*\)\"|\1 $params\"|" "$grub"

    if grep -q "isolcpus=$map" "$grub"; then
        info "backup: $backup"
        grep '^GRUB_CMDLINE_LINUX_DEFAULT=' "$grub" | sed 's/^/    /'
        sudo update-grub
        return 0
    fi

    warn "could not edit $grub automatically — restoring the backup"
    sudo cp "$backup" "$grub"
    return 1
}

# ── Installers ───────────────────────────────────────────────────────
# Each follows the vendor's current documented procedure and is idempotent.

install_ros2() {
    # ros2-apt-source configures the ROS repositories and keeps itself updated,
    # replacing the older manual keyring + sources.list.d recipe.
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe

    local src_ver codename
    src_ver="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
               | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"
    if [ -z "$src_ver" ]; then
        warn "could not resolve the latest ros-apt-source release — check network"
        return 1
    fi

    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${src_ver}/ros2-apt-source_${src_ver}.${codename}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update

    # desktop, not ros-base: arts_bringup.launch.py has launch_rviz:=true
    sudo apt-get install -y "ros-$ROS_DISTRO_ARTS-desktop" ros-dev-tools

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update || true
}

install_docker() {
    # Docker CE from Docker's own repository — the archive's docker.io lags and
    # splits the compose plugin differently.
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
        | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin

    # Run docker without sudo — takes effect on next login.
    if ! id -nG "$USER" | grep -qw docker; then
        sudo usermod -aG docker "$USER"
        warn "added $USER to the docker group — log out and back in for it to apply"
    fi
}

install_nvidia_container_toolkit() {
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker

    info "installed nvidia-container-toolkit $(dpkg-query -W -f='${Version}' nvidia-container-toolkit 2>/dev/null)"
    info "record this version in docs/host_prerequisites.md as the new validated pin"
}

bold "ARTS host prerequisites"
info "rationale + full procedures: docs/host_prerequisites.md"

# ─────────────────────────────────────────────────────────────────────
head2 "0. Base OS and ROS 2"

if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    if [ "${VERSION_ID:-}" = "24.04" ]; then
        ok "Ubuntu $VERSION_ID (noble) — the supported base"
    else
        warn "Ubuntu ${VERSION_ID:-unknown} — ARTS targets 24.04 (noble) with ROS 2 $ROS_DISTRO_ARTS"
    fi
fi

if [ -d "/opt/ros/$ROS_DISTRO_ARTS" ]; then
    ok "ROS 2 $ROS_DISTRO_ARTS present at /opt/ros/$ROS_DISTRO_ARTS"
else
    warn "ROS 2 $ROS_DISTRO_ARTS not installed"
    info "Needed for the bare-metal path; the Docker path does not use it."
    info "https://docs.ros.org/en/$ROS_DISTRO_ARTS/Installation/Ubuntu-Install-Debs.html"
    # ros-dev-tools pulls from noble-updates/backports; without them apt hits conflicts.
    if ! grep -q 'noble-updates' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null; then
        warn "apt Suites is missing noble-updates/noble-backports — ros-dev-tools will conflict"
        info "edit /etc/apt/sources.list.d/ubuntu.sources → 'Suites: noble noble-updates noble-backports'"
    fi
    if confirm "install ros-$ROS_DISTRO_ARTS-desktop + ros-dev-tools now?"; then
        install_ros2 && ok "ROS 2 $ROS_DISTRO_ARTS installed"
    fi
fi

# ─────────────────────────────────────────────────────────────────────
head2 "1. NVIDIA driver — branch depends on YOUR GPU"

# Status-Status, not Status-Abbrev: a held package reads "hi" rather than "ii",
# and apt-mark hold is exactly what we recommend once the RT build succeeds.
nvidia_pkg="$(dpkg-query -W -f='${db:Status-Status} ${Package}\n' 'nvidia-driver-*' 2>/dev/null \
              | awk '/^installed/ {print $2; exit}')"

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    ok "driver loaded: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
    info "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
    if [ -n "$nvidia_pkg" ]; then
                if [[ "$nvidia_pkg" == *-open ]]; then
                    info "package: $nvidia_pkg (open kernel modules — prebuilt for the"
                    info "         generic flavour only; a realtime kernel needs DKMS, step 2)"
        else
            info "package: $nvidia_pkg (proprietary kernel module — needs the RT bypass, step 2)"
        fi
    fi
    if nvidia_uses_dkms && [ ! -e "/lib/modules/$(uname -r)/build" ]; then
        warn "no headers for the running kernel $(uname -r), so DKMS skipped it"
        info "Harmless while a prebuilt module serves it, but the next driver upgrade"
        info "cannot rebuild for this kernel:  sudo apt install linux-headers-$(uname -r)"
    fi
else
    warn "no working NVIDIA driver (nvidia-smi fails or is absent)"
    if command -v ubuntu-drivers >/dev/null 2>&1; then
        recommended="$(ubuntu-drivers devices 2>/dev/null \
                       | grep 'recommended' | awk '{print $3}' | head -1)"
        if [ -n "$recommended" ]; then
            info "Ubuntu recommends for this machine: $recommended"
            info "ARTS validated: $VALIDATED_NVIDIA on a $VALIDATED_GPU"
            info "Take the recommendation — the validated number is a reference point,"
            info "not a target. See docs/host_prerequisites.md for -open vs proprietary,"
            info "which matters if you run the PREEMPT_RT kernel."
            if confirm "install $recommended now (reboot required afterwards)?"; then
                # Headers first: the nvidia-dkms postinst builds only for kernels
                # whose headers exist, and skips the rest with one quiet line.
                if [ ! -e "/lib/modules/$(uname -r)/build" ]; then
                    info "no headers for the running kernel $(uname -r) — installing them"
                    sudo apt-get install -y "linux-headers-$(uname -r)" || \
                        fail_note "linux-headers-$(uname -r) did not install"
                fi
                if sudo apt install -y "$recommended"; then
                    nvidia_pkg="$(dpkg-query -W -f='${db:Status-Status} ${Package}\n' \
                                  'nvidia-driver-*' 2>/dev/null | awk '/^installed/ {print $2; exit}')"
                    ok "installed ${nvidia_pkg:-$recommended}"
                    # Continuing in this session is what froze the workstation: any
                    # later step that rebuilds the module (a headers or kernel install)
                    # lets nvidia-cdi-refresh load it under the running nouveau display.
                    echo
                    bold "Reboot now, then re-run this script — it resumes where it stopped."
                    exit 0
                else
                    fail_note "$recommended did not install"
                fi
            fi
        else
            info "run 'ubuntu-drivers devices' and install the 'recommended' entry"
        fi
    else
        info "install ubuntu-drivers-common, then run 'ubuntu-drivers devices'"
    fi
fi

# ─────────────────────────────────────────────────────────────────────
head2 "2. PREEMPT_RT kernel — needs a free Ubuntu Pro account"

kver="$(uname -r)"
if uname -v | grep -q 'PREEMPT_RT'; then
    ok "running a PREEMPT_RT kernel: $kver"
else
    warn "running $kver — not PREEMPT_RT"
    info "ARTS validated: $VALIDATED_RT_KERNEL"
    info "The realtime kernel tracks your Ubuntu release, so it will not match the"
    info "validated version — re-run cyclictest rather than reusing old numbers."
    info ""

    # Attaching and enabling are separate operations, and a machine can sit
    # attached for years with realtime-kernel still disabled.  Check both.
    if pro_is_attached; then
        ok "Ubuntu Pro: attached"
    else
        info "Ubuntu Pro is free for up to 5 personal machines: https://ubuntu.com/pro"
        if confirm "attach this machine to Ubuntu Pro now (needs your token)?"; then
            read -r -s -p "  → Ubuntu Pro token (hidden): " pro_token
            echo
            if [ -n "$pro_token" ]; then
                sudo pro attach "$pro_token" || warn "attach failed — see the message above"
            fi
        fi
    fi

    # This is the step that actually installs the kernel.
    if pro_is_attached; then
        rt_status="$(pro_service_status realtime-kernel)"
        if [ "$rt_status" = "enabled" ]; then
            ok "realtime-kernel service enabled — reboot to use it"
        else
            warn "realtime-kernel service is ${rt_status:-disabled}"
            info "It installs as an extra GRUB entry; generic stays the default."
            info "Note: Livepatch and realtime-kernel are mutually exclusive — enabling"
            info "the RT kernel disables Canonical Livepatch."
            if confirm "enable realtime-kernel now (downloads and installs a kernel)?"; then
                # The RT headers' DKMS hook builds NVIDIA inside this command, so the
                # bypass has to exist BEFORE it runs; written afterwards, the kernel
                # packages are already half-configured and the RT kernel has no initrd.
                if nvidia_uses_dkms && ! ensure_dkms_rt_bypass; then
                    warn "without the bypass the NVIDIA build inside 'pro enable' will fail"
                fi
                if sudo pro enable realtime-kernel; then
                    ok "enabled — reboot to use it"
                else
                    fail_note "'pro enable realtime-kernel' failed (exit $?) — see its output above"
                fi
            fi
        fi
    fi

    # Whether it was just enabled or enabled earlier, check before the first boot.
    # The bypass is NOT gated on the proprietary flavour: -open refuses identically.
    if ls /boot/vmlinuz-*realtime* >/dev/null 2>&1; then
        check_nvidia_for_rt_kernels
        check_half_configured || true
        for img in /boot/vmlinuz-*realtime*; do
            [ -e "/boot/initrd.img-${img#/boot/vmlinuz-}" ] || \
                fail_note "no initrd for ${img#/boot/vmlinuz-} — it will not boot"
        done
        grub_boot_warnings
    fi
fi

# Every future kernel or driver upgrade rebuilds through DKMS, so a working RT host
# without the bypass is one apt upgrade away from the half-configured state above.
if nvidia_uses_dkms && ls /boot/vmlinuz-*realtime* >/dev/null 2>&1 \
   && ! grep -qs 'IGNORE_PREEMPT_RT_PRESENCE' "$DKMS_RT_CONF"; then
    warn "no DKMS PREEMPT_RT bypass in $DKMS_RT_CONF — the next upgrade's rebuild will refuse"
    if grep -qs 'IGNORE_PREEMPT_RT_PRESENCE' /etc/dkms/framework.conf; then
        info "/etc/dkms/framework.conf has it, but dkms 3.x ignores it there."
    fi
    ensure_dkms_rt_bypass || true
fi

# ─────────────────────────────────────────────────────────────────────
head2 "3. CPU isolation — map depends on YOUR core topology"

isolated="$(cat /sys/devices/system/cpu/isolated 2>/dev/null)"
if [ -n "$isolated" ]; then
    ok "isolated cores: $isolated"
elif grep -qs 'isolcpus=' /etc/default/grub; then
    # Configured but not yet in effect — don't offer to add it a second time.
    ok "isolcpus configured in /etc/default/grub, pending reboot"
    sed -n 's/^GRUB_CMDLINE_LINUX_DEFAULT=/    /p' /etc/default/grub
else
    warn "no isolated cores"
    info "ARTS validated: isolcpus=$VALIDATED_ISOLCPUS"

    n_isol="${ARTS_ISOLCPUS_CORES:-2}"
    if [ "$CHECK_ONLY" = 0 ]; then
        info "One isolated core per RT controller_manager is the minimum (two managers)."
        read -r -p "  → how many cores to isolate? [$n_isol] " reply
        [[ "$reply" =~ ^[1-9][0-9]*$ ]] && n_isol="$reply"
    fi
    suggested="$(recommend_isolcpus "$n_isol")"
    if [ -n "$suggested" ]; then
        info "This machine has $(nproc) logical CPUs. Recommended: $n_isol cores, chosen"
        info "as whole L2 groups (SMT siblings / E-core clusters), performance cores only,"
        info "never CPU 0, highest-boost cores left to the scheduler: $suggested"
        info "Three parameters work together — isolcpus removes the cores from the"
        info "scheduler, nohz_full stops their timer tick, rcu_nocbs moves RCU"
        info "callbacks elsewhere:"
        info "    isolcpus=$suggested nohz_full=$suggested rcu_nocbs=$suggested"
        if confirm "append these to GRUB_CMDLINE_LINUX_DEFAULT and run update-grub?"; then
            if apply_isolcpus "$suggested"; then
                ok "GRUB updated — reboot, then check /sys/devices/system/cpu/isolated"
            fi
        else
            info "Skipped. Edit /etc/default/grub yourself, then 'sudo update-grub'."
        fi
    else
        info "Could not read the topology — run 'lscpu --extended' and reserve 2+"
        info "physical cores (both SMT siblings of each) by hand."
    fi
fi

# ─────────────────────────────────────────────────────────────────────
head2 "4. Docker (optional — the container path)"

if ! command -v docker >/dev/null 2>&1; then
    warn "docker not installed"
    info "Bare metal and Docker are both first-class paths; skip if you only want bare metal."
    if confirm "install Docker CE from Docker's official repository?"; then
        install_docker && ok "docker $(docker --version | awk '{print $3}' | tr -d ,) installed"
    fi
fi

if command -v docker >/dev/null 2>&1; then
    ok "docker $(docker --version | awk '{print $3}' | tr -d ,)"

    # `docker info | grep nvidia` alone conflates three different states, and the
    # most common one right after install — a shell that predates the new docker
    # group membership — looks identical to "not registered". Check the config
    # file, which is readable without daemon access, and report them separately.
    if ! docker info >/dev/null 2>&1; then
        warn "cannot reach the Docker daemon from this shell"
        if getent group docker 2>/dev/null | grep -qw "$USER"; then
            info "$USER is in the docker group, but this session started before that"
            info "took effect — log out and back in (or reboot), then re-run."
        else
            info "Check 'systemctl status docker' and your group membership."
        fi
    fi

    if grep -qs '"nvidia"' /etc/docker/daemon.json; then
        ok "nvidia runtime registered in /etc/docker/daemon.json"
        if docker info >/dev/null 2>&1 && ! docker info 2>/dev/null | grep -q nvidia; then
            warn "registered but not active — 'sudo systemctl restart docker'"
        fi
    else
        warn "nvidia-container-toolkit not registered with Docker"
        info "Needed for --gpus all; ARTS uses it for the GPU JPEG path and sim."
        if confirm "install and register nvidia-container-toolkit?"; then
            install_nvidia_container_toolkit && ok "nvidia container runtime registered"
        fi
    fi
fi

# ─────────────────────────────────────────────────────────────────────
head2 "5. Vendor SDKs — account required, never auto-downloaded"

check_sdk() {
    local label="$1" dir="$2" url="$3"
    if [ -d "$dir" ]; then
        ok "$label → $dir"
    else
        warn "$label not mirrored"
        info "download (free account): $url"
        info "extract to: $dir"
    fi
}

if [ -f "$ARTS_ROOT/oem_drivers/spinnaker/VERSION" ]; then
    # shellcheck disable=SC1091
    ( . "$ARTS_ROOT/oem_drivers/spinnaker/VERSION"
      check_sdk "FLIR Spinnaker $VERSION" \
                "$ARTS_LOCAL/oem_drivers/spinnaker-$VERSION" \
                "$DOWNLOAD_URL (or skip cameras: ARTS_ENABLE_FLIR=0)" )
fi

echo
if [ "${#FAILURES[@]}" -gt 0 ]; then
    printf '\033[1;31m%s\033[0m\n' "Finished with ${#FAILURES[@]} failure(s):"
    printf '  \033[31m✗\033[0m %s\n' "${FAILURES[@]}"
    info "Fix these before rebooting into a new kernel; re-run once fixed."
    exit 1
fi
bold "Done."
info "Warnings (!) above are advisory — ARTS modules check their own prerequisites."
info "Re-run this script any time; it is idempotent."
