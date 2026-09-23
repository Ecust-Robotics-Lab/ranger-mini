#!/usr/bin/env bash
# One-time installation for the CURRENT kernel, using the already signed module.
set -Eeuo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
root=$(cd "$(dirname "$0")/.." && pwd)
if [[ ${1:---dry-run} != --execute ]]; then
 echo 'DRY-RUN: restore existing network, back up, install existing signed module for 6.8.0-40-generic, depmod, enable this profile autoconnect; no MOK/reboot/firewall/CAN operations.';exit 0
fi
[[ $EUID == 0 ]] || { echo 'Requires sudo in your local terminal.';exit 1; }
# Restore script verifies the signed module, PCI identity, IP, no gateway, and backs up NM.
bash "$root/scripts/restore_mid360_network.sh" --execute
kernel=$(uname -r)
[[ $kernel == 6.8.0-40-generic ]]
state=/var/lib/ranger-nav-yt6801
uuid=7d3735fd-229f-4461-bebd-421146eb06d2
target=/lib/modules/$kernel/updates/ranger-nav/yt6801.ko
loadconf=/etc/modules-load.d/ranger-nav-yt6801.conf
umask 077
run=$(mktemp -d "$state/stage3-persist-XXXXXXXX")
exec > >(tee "$run/persist.log") 2>&1
trap 'rc=$?; echo "EXIT=$rc"; echo "ROLLBACK=sudo bash $run/rollback.sh"' EXIT
auto_before=$(nmcli -g connection.autoconnect connection show uuid "$uuid")
printf '%s\n' "$auto_before" > "$run/autoconnect-before.txt"
ip route show default > "$run/default-before.txt"
nmcli -f all connection show uuid "$uuid" > "$run/profile-before.txt"
tar -cf "$run/NetworkManager-system-connections.tar" -C /etc/NetworkManager system-connections
printf '#!/usr/bin/env bash\nset -euo pipefail\n' > "$run/rollback.sh"
printf 'nmcli connection modify uuid %q connection.autoconnect %q\n' "$uuid" "$auto_before" >> "$run/rollback.sh"
for item in "$target" "$loadconf"; do
 if [[ -e "$item" ]]; then
  name=$(basename "$item");cp -a "$item" "$run/$name.before"
  printf 'cp -a %q %q\n' "$run/$name.before" "$item" >> "$run/rollback.sh"
 else
  printf 'rm -f -- %q\n' "$item" >> "$run/rollback.sh"
 fi
done
printf '(umask 022; depmod -a %q)\n' "$kernel" >> "$run/rollback.sh"
echo '# Does not unload the active module or alter active connectivity.' >> "$run/rollback.sh"
echo "BACKUP=$run"
install -D -m 0644 "$state/yt6801.ko" "$target"
printf 'yt6801\n' > "$loadconf"
chmod 0644 "$loadconf"
(umask 022; depmod -a "$kernel")
[[ $(modinfo -n yt6801) == "$target" ]]
[[ $(modinfo -F signer yt6801) == 'ranger-nav YT6801 module signing' ]]
cmp "$state/yt6801.ko" "$target"
nmcli connection modify uuid "$uuid" connection.autoconnect yes
ip route show default > "$run/default-after.txt"
cmp "$run/default-before.txt" "$run/default-after.txt"
modprobe --dry-run --verbose yt6801
nmcli -g connection.id,connection.autoconnect,ipv4.addresses,ipv4.never-default connection show uuid "$uuid"
echo 'PERSISTENCE_CONFIGURED: current kernel only; actual reboot validation is pending. No reboot performed.'
