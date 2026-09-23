#!/usr/bin/env bash
# Restore only the previously verified temporary module/profile; never enroll MOK.
set -Eeuo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
if [[ ${1:---dry-run} != --execute ]]; then
  echo 'DRY-RUN: verify existing signed YT6801, back up network state, load if absent, activate existing UUID; no profile edits, CAN or firewall changes.'
  exit 0
fi
[[ $EUID == 0 ]] || { echo 'Requires sudo in your local terminal.' >&2; exit 1; }
state=/var/lib/ranger-nav-yt6801
uuid=7d3735fd-229f-4461-bebd-421146eb06d2
pci=/sys/bus/pci/devices/0000:02:00.0
[[ $(uname -r) == 6.8.0-40-generic ]]
[[ $(cat "$pci/vendor") == 0x1f0a && $(cat "$pci/device") == 0x6801 ]]
sha256sum --check "$state/signed-module.sha256"
[[ $(modinfo -F signer "$state/yt6801.ko") == 'ranger-nav YT6801 module signing' ]]
[[ $(modinfo -F vermagic "$state/yt6801.ko") == '6.8.0-40-generic '* ]]
[[ $(modinfo -F version "$state/yt6801.ko") == 1.0.34 ]]
[[ $(nmcli -g connection.interface-name connection show uuid "$uuid") == enp2s0 ]]
[[ $(nmcli -g ipv4.addresses connection show uuid "$uuid") == 192.168.1.50/24 ]]
[[ $(nmcli -g ipv4.never-default connection show uuid "$uuid") == yes ]]
[[ -z $(nmcli -g ipv4.gateway connection show uuid "$uuid") ]]
loaded=0
[[ ! -d /sys/module/yt6801 ]] || loaded=1
if [[ -L "$pci/driver" ]]; then
 [[ $(basename "$(readlink -f "$pci/driver")") == yt6801 ]]
fi
if [[ $loaded == 1 ]]; then
 [[ $(cat /sys/module/yt6801/srcversion) == "$(modinfo -F srcversion "$state/yt6801.ko")" ]]
fi
previous=$(nmcli -t -f UUID,DEVICE connection show --active | awk -F: '$2=="enp2s0" {print $1}')
[[ -z "$previous" || "$previous" == "$uuid" ]] || { echo 'Unexpected active wired profile; stop.'; exit 1; }
umask 077
run=$(mktemp -d "$state/stage3-restore-XXXXXXXX")
exec > >(tee "$run/restore.log") 2>&1
ip -j address > "$run/address-before.json"
ip -j route show table all > "$run/routes-before.json"
ip route show default > "$run/default-before.txt"
nmcli -f all connection show uuid "$uuid" > "$run/profile-before.txt"
# Full configuration backup stays root-only, since other profiles may contain secrets.
tar -cf "$run/NetworkManager-system-connections.tar" -C /etc/NetworkManager system-connections
cat > "$run/rollback.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
# Run only after stopping the Livox driver; preserves the existing profile.
EOF
if [[ -z "$previous" ]]; then
 printf 'nmcli connection down uuid %q\n' "$uuid" >> "$run/rollback.sh"
fi
if [[ $loaded == 0 ]]; then
 echo 'rmmod yt6801' >> "$run/rollback.sh"
fi
echo "BACKUP=$run"
echo "ROLLBACK=sudo bash $run/rollback.sh"
trap 'rc=$?; echo "EXIT=$rc"; echo "Backup and rollback: $run"' EXIT
if [[ $loaded == 0 ]]; then insmod "$state/yt6801.ko"; fi
udevadm settle --timeout=15
for attempt in {1..40}; do
 if [[ -d /sys/class/net/enp2s0 ]]; then break; fi
 sleep 0.25
done
[[ $(readlink -f /sys/class/net/enp2s0/device) == "$(readlink -f "$pci")" ]]
if [[ -z "$previous" ]]; then nmcli --wait 20 connection up uuid "$uuid"; fi
ip route show default > "$run/default-after.txt"
cmp "$run/default-before.txt" "$run/default-after.txt"
ip -brief address show enp2s0
ip route get 192.168.1.101
ip neigh show dev enp2s0
echo 'Temporary network restored. No ping/scan, firmware change, persistence, MOK enrollment, or base operation.'
