#!/usr/bin/env bash
# host_setup.sh — host-only setup that containers cannot provide.
# PTP daemons, NIC/MTU tuning, udev rules, CPU isolation, kernel/GRUB notes.
#
# This is a skeleton. Concrete commands are recorded as they are proven on the
# development PC (see README.md Host setup log) and then copied here so bare-metal
# and Docker paths stay aligned via deploy/modules/*.sh.
#
set -euo pipefail

ARTS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "ARTS host_setup.sh (skeleton)"
echo "Repo: $ARTS_ROOT"
echo
echo "Already done on the development laptop (see README.md):"
echo "  - PREEMPT_RT kernel (Ubuntu Pro) as second GRUB entry"
echo "  - NVIDIA 595.84 with IGNORE_PREEMPT_RT_PRESENCE in /etc/dkms/framework.conf"
echo "  - isolcpus=4,5,10,11 nohz_full=... rcu_nocbs=..."
echo "  - Docker + nvidia-container-toolkit 1.13.5"
echo
echo "TODO: install udev rules from deploy/udev/"
echo "TODO: PTP (ptp4l / phc2sys) for the camera NIC"
echo "TODO: NIC MTU / ring-buffer tuning for GigE FLIR"
echo "TODO: generalize isolcpus map for the production workstation"
