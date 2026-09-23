#!/bin/bash
# Temporary connection test. Reviewed fixed module; no base/CAN operations.
set -Eeuo pipefail
trap 'rc=$?; printf "Stopped at line %s (exit=%s).\n" "$LINENO" "$rc" >&2; exit "$rc"' ERR
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
mode=${1:---dry-run}
[[ $# -le 1 ]] || exit 2
case "$mode" in --dry-run|--apply) ;; *) echo 'Usage: bash connect_mid360.sh [--dry-run|--apply]'; exit 2;; esac
cat <<'PLAN'
Verify kernel, enrolled certificate fingerprint, signed module checksum and PCI alias.
Load only the signed YT6801 module if absent; find its PCI 0000:02:00.0 interface.
Create a dedicated NetworkManager profile with 192.168.1.50/24, no default route,
no DNS, IPv6 disabled and autoconnect disabled. Activate it and ping 192.168.1.101.
Save original state, results and exact rollback command under /var/lib/ranger-nav-yt6801.
Do not install modules, alter Wi-Fi, reboot, start ROS drivers or send CAN controls.
PLAN
[[ "$mode" == --apply ]] || { echo 'DRY-RUN: no changes made.'; exit 0; }
[[ $EUID -eq 0 ]] || { echo 'Run --apply with sudo in your own terminal.' >&2; exit 1; }
state=/var/lib/ranger-nav-yt6801
module=$state/yt6801.ko
cert=$state/MOK.der
pci=/sys/bus/pci/devices/0000:02:00.0
profile=ranger-mid360-3801
[[ $(uname -r) == 6.8.0-40-generic ]] || { echo 'Kernel changed; rebuild required.'; exit 1; }
[[ $(cat "$pci/vendor") == 0x1f0a && $(cat "$pci/device") == 0x6801 ]]
fingerprint=$(openssl x509 -inform DER -in "$cert" -noout -fingerprint -sha256)
[[ ${fingerprint#*=} == '84:B2:A7:E5:EA:31:84:56:1D:A7:B9:B2:0E:A6:A7:A7:EB:00:99:C0:D1:C5:4F:99:BA:68:05:33:CB:87:8A:98' ]]
# mokutil 0.6.0 returns 1 for an enrolled certificate (0 for not enrolled).
# Require the exact affirmative result; never accept a nonzero exit alone.
# This is a read-only MOK lookup. Kernel signature enforcement remains enabled.
if mok_result=$(LC_ALL=C mokutil --ignore-keyring --test-key "$cert" 2>&1); then
    mok_rc=0
else
    mok_rc=$?
fi
printf '%s\n' "$mok_result"
if [[ ( $mok_rc -ne 0 && $mok_rc -ne 1 ) || "$mok_result" != "$cert is already enrolled" ]]; then
    echo "Cannot confirm exact certificate enrollment (mokutil exit=$mok_rc); stop." >&2
    exit 1
fi
echo 'MOK enrollment verified; continuing module checksum validation.'
sha256sum --check "$state/signed-module.sha256"
[[ $(modinfo -F signer "$module") == 'ranger-nav YT6801 module signing' ]]
[[ $(modinfo -F vermagic "$module") == '6.8.0-40-generic '* ]]
[[ $(modinfo -F version "$module") == 1.0.34 ]]
[[ $(modinfo -F alias "$module") == 'pci:v00001F0Ad00006801sv*sd*bc*sc*i*' ]]
if nmcli -g connection.uuid connection show "$profile" >/dev/null 2>&1; then
    echo 'Project profile already exists; inspect before repeating.' >&2; exit 1
fi
if ip -4 route show table all | grep -Eq '(^| )192\.168\.1\.'; then
    echo 'Existing 192.168.1.* route; inspect before changing networking.' >&2; exit 1
fi
loaded_before=0
if [[ -d /sys/module/yt6801 ]]; then
    [[ $(cat /sys/module/yt6801/version) == 1.0.34 ]]
    [[ $(cat /sys/module/yt6801/srcversion) == "$(modinfo -F srcversion "$module")" ]]
    loaded_before=1
    echo 'Resuming with the matching YT6801 module already loaded.'
fi
if [[ -L "$pci/driver" ]]; then
    [[ $(basename "$(readlink -f "$pci/driver")") == yt6801 && $loaded_before -eq 1 ]] || {
        echo 'PCI device bound to an unexpected driver; stop.' >&2; exit 1;
    }
fi
umask 077
run=$(mktemp -d "$state/connect-XXXXXXXX")
ip -j address > "$run/address-before.json"
ip route show default > "$run/default-before.txt"
ip -j route show table all > "$run/routes-before.json"
nmcli -t -f NAME,UUID,TYPE,DEVICE connection show > "$run/connections-before.txt"
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status > "$run/devices-before.txt"
cat > "$run/rollback.sh" <<ROLLBACK
#!/bin/bash
set -euo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
if [[ -f '$run/profile-uuid.txt' ]]; then
    connection_uuid=\$(cat '$run/profile-uuid.txt')
    [[ \$connection_uuid =~ ^[0-9a-fA-F-]{36}$ ]]
    if nmcli -g connection.uuid connection show "\$connection_uuid" >/dev/null 2>&1; then
        nmcli connection delete uuid "\$connection_uuid"
    fi
fi
if [[ $loaded_before -eq 0 && -d /sys/module/yt6801 ]]; then
    rmmod yt6801
fi
echo 'Removed this test profile; pre-existing module preserved, Wi-Fi settings unchanged.'
ROLLBACK
echo "Results: $run"
echo "Rollback: sudo bash $run/rollback.sh"
trap 'rc=$?; printf "Exit=%s\n" "$rc" > "$run/exit.txt"; if (( rc != 0 )); then echo "Stopped; state preserved. Rollback: sudo bash $run/rollback.sh"; fi' EXIT
if [[ $loaded_before -eq 0 ]]; then
    insmod "$module"
fi
# The kernel first registers eth0; udev renames it asynchronously.
# Wait for udev, then require its final name and NetworkManager visibility.
udevadm settle --timeout=15
iface=
for attempt in {1..40}; do
    shopt -s nullglob
    devices=("$pci"/net/*)
    if [[ ${#devices[@]} -eq 1 ]]; then
        candidate=${devices[0]##*/}
        properties=$(udevadm info --query=property --path="${devices[0]}" 2>/dev/null) || properties=
        if [[ "$properties" == *"ID_NET_NAME=$candidate"* ]] &&
           ip link show dev "$candidate" >/dev/null 2>&1 &&
           nmcli -g GENERAL.DEVICE device show "$candidate" >/dev/null 2>&1; then
            iface=$candidate
            break
        fi
    fi
    sleep 0.25
done
[[ -n "$iface" ]] || { echo 'Final udev interface name not ready; stop.'; exit 1; }
[[ $(basename "$(readlink -f "$pci/driver")") == yt6801 ]]
echo "Final PCI Ethernet interface: $iface"
echo "$iface" > "$run/interface.txt"
ip -j address show dev "$iface" > "$run/new-interface-before.json"
previous_uuid=$(nmcli -t -f UUID,DEVICE connection show --active | awk -F: -v dev="$iface" '$2 == dev {print $1}')
if [[ $loaded_before -eq 1 && "$previous_uuid" =~ ^[0-9a-fA-F-]{36}$ ]]; then
    printf '\nnmcli --wait 20 connection up uuid %q\n' "$previous_uuid" >> "$run/rollback.sh"
fi
nmcli connection add type ethernet ifname "$iface" con-name "$profile" \
    connection.autoconnect no \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes \
    ipv4.ignore-auto-dns yes ipv6.method disabled
uuid=$(nmcli -g connection.uuid connection show "$profile")
[[ "$uuid" =~ ^[0-9a-fA-F-]{36}$ ]]
printf '%s\n' "$uuid" > "$run/profile-uuid.txt"
nmcli --wait 20 connection up uuid "$uuid"
ip route show default > "$run/default-after.txt"
cmp "$run/default-before.txt" "$run/default-after.txt" || { echo 'Default route changed; stop and inspect.'; exit 1; }
ip -brief address show dev "$iface" | tee "$run/interface-after.txt"
cat "/sys/class/net/$iface/carrier" | tee "$run/carrier.txt"
ip route get 192.168.1.101 | tee "$run/route-to-lidar.txt"
if ping -n -I "$iface" -c 5 -W 1 192.168.1.101 | tee "$run/ping.txt"; then
    ping_rc=0
else
    pipeline_rc=("${PIPESTATUS[@]}")
    ping_rc=${pipeline_rc[0]}
    # A logging failure must not become a false PASS if ping itself succeeded.
    (( ping_rc != 0 )) || ping_rc=1
fi
ip neigh show dev "$iface" | tee "$run/neighbors.txt"
if (( ping_rc == 0 )); then
    echo 'IP reachability PASS. Device serial and Livox UDP data still require verification.'
else
    echo 'No ping response: connection remains unverified; inspect link, power and actual radar IP.'
fi
echo 'Temporary test only: module is not installed for boot, profile autoconnect is off.'
exit "$ping_rc"
