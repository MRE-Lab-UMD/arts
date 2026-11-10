#!/bin/bash
# Copy a finished bag to external storage and verify it, so the local copy can be deleted.
#
#   export_bag.sh BAG_DIR DEST_DIR [--delete]
#
# BAG_DIR may also be a session directory holding its bags below it (BAG_DIR/bag, or a trial
# with teleop/bag and images/, up to three levels down); it is exported whole.
#
# The workstation has no room for a second internal disk, so recordings live on the system
# disk only until exported: record one session, export it, then record the next
# (docs/modules/pilot_flir_camera.md, "Storage"). Run this BETWEEN recordings, never during
# one: an external USB disk shares a controller with a camera (docs/host_usb_topology.md).
#
# Verification is a sha256 of every file on both sides, read back from the destination.
# The local copy is deleted only with --delete AND only if every hash matches.
set -euo pipefail

src=${1:?usage: export_bag.sh BAG_DIR DEST_DIR [--delete]}
dest_root=${2:?usage: export_bag.sh BAG_DIR DEST_DIR [--delete]}
delete=${3:-}
src=$(realpath "$src")
# BAG_DIR is a bag, or a session directory holding its bags below it beside the run scripts
# and analyzer output (exported whole). Every bag in it must be closed: rosbag2 writes
# metadata.yaml only when the recorder stops, so an .mcap without one is still recording.
nbags=$(find "$src" -maxdepth 3 -name metadata.yaml | wc -l)
[ "$nbags" -gt 0 ] ||
  { echo "export_bag: no metadata.yaml in $src or up to 3 levels below (no closed bag)" >&2; exit 2; }
while IFS= read -r m; do
  d=$(dirname "$m")
  [ -f "$d/metadata.yaml" ] ||
    { echo "export_bag: $d has an .mcap but no metadata.yaml (still recording?)" >&2; exit 2; }
done < <(find "$src" -maxdepth 4 -name '*.mcap')
echo "export_bag: $nbags closed bag(s) in $src"
dest="$dest_root/$(basename "$src")"
[ -e "$dest" ] && { echo "export_bag: $dest already exists" >&2; exit 2; }

need=$(du -sb "$src" | cut -f1)
free=$(df -B1 --output=avail "$dest_root" | tail -1)
if [ "$free" -lt $((need + 1000000000)) ]; then
  echo "export_bag: destination has $((free / 1000000000)) GB free, needs $((need / 1000000000)) GB + 1" >&2
  exit 2
fi

echo "export_bag: copying $((need / 1000000000)) GB to $dest"
cp -r --preserve=timestamps "$src" "$dest"
sync -f "$dest"

echo "export_bag: verifying"
(cd "$src" && find . -type f -print0 | sort -z | xargs -0 sha256sum) > /tmp/export_bag_src.$$
(cd "$dest" && find . -type f -print0 | sort -z | xargs -0 sha256sum) > /tmp/export_bag_dst.$$
if ! cmp -s /tmp/export_bag_src.$$ /tmp/export_bag_dst.$$; then
  echo "export_bag: VERIFY FAILED; local copy kept" >&2
  diff /tmp/export_bag_src.$$ /tmp/export_bag_dst.$$ >&2 || true
  rm -f /tmp/export_bag_src.$$ /tmp/export_bag_dst.$$
  exit 1
fi
cp /tmp/export_bag_src.$$ "$dest/SHA256SUMS"
rm -f /tmp/export_bag_src.$$ /tmp/export_bag_dst.$$
echo "export_bag: verified ($(wc -l < "$dest/SHA256SUMS") files); hashes in $dest/SHA256SUMS"

if [ "$delete" = "--delete" ]; then
  rm -rf "$src"
  echo "export_bag: deleted local $src"
else
  echo "export_bag: local copy kept (pass --delete to remove it after verification)"
fi
