#!/usr/bin/env bash
# deploy/modules/ur.sh — the three UR arms (decision 1; arms and ports in
# arts_ros2_ws/src/arts_driver/config/ur_arms.yaml).
#
# Functions:
#   ur_host_deps      — static host address on the arm network, firewall opening for the
#                       arms' callbacks, real-time scheduling rights (host only, needs sudo)
#   ur_overlay_build  — ur_ros2_ws from ur_ros2.repos, ur_moveit_config excluded
#   ur_preflight      — read-only: host address, ping, dashboard PolyScope version per arm
#
# No OEM install: the driver is open source and pinned in ur_ros2.repos; the robot side
# (External Control URCap, host IP, port) is set on each pendant, not by this script.
#
# Docker note: the arm network is the host's; a container reaches it with --network host,
# and the drivers' callback ports must be open on the host, which is why ur_host_deps runs
# in every install path.
set -euo pipefail

ARTS_ROOT="${ARTS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ARTS_LOCAL="${ARTS_LOCAL:-$HOME/arts_local}"

# Sets SUDO (empty as root, "sudo" otherwise) and overlay_build_generic.
# shellcheck source=_common.sh
source "$ARTS_ROOT/deploy/modules/_common.sh"

UR_ARMS_YAML="$ARTS_ROOT/arts_ros2_ws/src/arts_driver/config/ur_arms.yaml"

# Prints one line per arm: name model ip polyscope script_sender_port, then "host IP BITS".
_ur_arms() {
    python3 - "$UR_ARMS_YAML" <<'EOF'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
for name, a in c['arms'].items():
    print(name, a['model'], a['ip'], a['polyscope'], a['script_sender_port'])
print('host', c['host_ip'], c['netmask_bits'])
EOF
}

# ─────────────────────────────────────────────────────────────────────
ur_host_deps() {
    echo "[ur] host_deps ..."
    local host_ip bits iface con=arts-arms
    read -r _ host_ip bits < <(_ur_arms | grep '^host ')

    # The wired port the switch is on. ARTS_UR_IFACE overrides; otherwise the only
    # ethernet device NetworkManager knows (the workstation has one).
    iface="${ARTS_UR_IFACE:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="ethernet"{print $1; exit}')}"
    [ -n "$iface" ] || { echo "[ur]   ERROR: no ethernet device found; set ARTS_UR_IFACE" >&2; exit 1; }

    # Static address, never the default route: the arm network has no gateway, and a
    # DHCP profile on the same port would leave the host without host_ip after a replug.
    if ! nmcli -t -f NAME con show | grep -qx "$con"; then
        $SUDO nmcli con add type ethernet ifname "$iface" con-name "$con" \
            ipv4.method manual ipv4.addresses "$host_ip/$bits" ipv4.never-default yes \
            ipv6.method disabled connection.autoconnect-priority 10
        echo "[ur]   NetworkManager profile $con: $iface = $host_ip/$bits (static)"
    fi

    # The arms connect BACK to the host (reverse, script sender, trajectory and script
    # command ports); an active ufw drops those and the driver waits forever for the robot.
    if command -v ufw >/dev/null && $SUDO ufw status | grep -q '^Status: active'; then
        local subnet="${host_ip%.*}.0/$bits"
        if ! $SUDO ufw status | grep -q "$subnet"; then
            $SUDO ufw allow in on "$iface" from "$subnet"
            echo "[ur]   ufw: allow in on $iface from $subnet"
        fi
    fi
    # Real-time scheduling for the controller_manager. Without it ros2_control_node logs
    # "Could not enable FIFO RT scheduling policy" and runs the 500 Hz loop at normal
    # priority (seen 2026-10-29, `ulimit -r` = 0), which makes E5's jitter a measurement of
    # the desktop, not of ARTS. The limits are the ones ros2_control and the UR driver
    # document: a realtime group with rtprio 99 and unlimited locked memory.
    local limits=/etc/security/limits.d/99-arts-realtime.conf
    if [ ! -f "$limits" ]; then
        getent group realtime >/dev/null || $SUDO groupadd -f realtime
        printf '@realtime soft rtprio 99\n@realtime hard rtprio 99\n@realtime soft memlock unlimited\n@realtime hard memlock unlimited\n' \
            | $SUDO tee "$limits" >/dev/null
        echo "[ur]   real-time limits → $limits (group realtime)"
    fi
    local who="${SUDO_USER:-$USER}"
    if [ -n "$who" ] && [ "$who" != root ] && ! id -nG "$who" | grep -qw realtime; then
        $SUDO usermod -aG realtime "$who" \
            && echo "[ur]   added $who to realtime — log out and back in for it to apply"
    fi
    echo "[ur] host_deps done."
}

# ─────────────────────────────────────────────────────────────────────
ur_overlay_build() {
    # ur_moveit_config is excluded (decision 11; the full reasoning, including why
    # --exclude sets three markers, is in middle_overlay_ros2_wss/ur_ros2_ws/build.sh).
    overlay_build_generic ur_ros2_ws ${1:+--ws "$1"} \
        --exclude Universal_Robots_ROS2_Driver/ur_moveit_config
}

# ─────────────────────────────────────────────────────────────────────
ur_preflight() {
    # Read-only. Fails if the host lacks host_ip, an arm does not answer, or an arm's
    # PolyScope differs from what ur_arms.yaml records (a pendant update is a pin change).
    local fail=0 host_ip name model ip ver port got
    read -r _ host_ip _ < <(_ur_arms | grep '^host ')
    if ip -br addr | grep -qw "$host_ip/[0-9]*"; then
        echo "[ur] host has $host_ip"
    else
        echo "[ur] FAIL host does not have $host_ip (switch unplugged, or ur.sh host_deps not run)"
        fail=1
    fi
    while read -r name model ip ver port; do
        [ "$name" = host ] && continue
        if ! ping -c 2 -W 1 "$ip" >/dev/null 2>&1; then
            echo "[ur] FAIL $name ($model, $ip): no ping reply"; fail=1; continue
        fi
        # Dashboard server, port 29999, on both CB3 and e-Series: greeting line, then reply.
        got=$( { echo PolyscopeVersion; sleep 1; } | timeout 3 nc "$ip" 29999 2>/dev/null \
               | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)
        if [ "$got" = "$ver" ]; then
            echo "[ur] ok   $name ($model, $ip): PolyScope $got, URCap port $port"
        else
            echo "[ur] FAIL $name ($model, $ip): PolyScope '${got:-no answer}', ur_arms.yaml says $ver"
            fail=1
        fi
    done < <(_ur_arms)
    return $fail
}

# ─────────────────────────────────────────────────────────────────────
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        host_deps)     ur_host_deps ;;
        overlay_build) ur_overlay_build "${2:-}" ;;
        preflight)     ur_preflight ;;
        all)           ur_host_deps; ur_overlay_build "${2:-}" ;;
        *) echo "Usage: $0 {host_deps|overlay_build [ws_path]|preflight|all}"; exit 1 ;;
    esac
fi
