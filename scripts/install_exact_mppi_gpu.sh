#!/usr/bin/env bash
# User-authorized RTX 5060 compute driver installation; no reboot/CAN/ROS.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---dry-run}"
case "$mode" in --dry-run|--execute) ;; *) exit 2;; esac
packages=(nvidia-headless-580-open=580.178.04-0ubuntu0.22.04.1 nvidia-utils-580=580.178.04-0ubuntu0.22.04.1)
plan=$(LC_ALL=C apt-get -s --no-install-recommends --no-remove install "${packages[@]}")
printf '%s\n' "$plan"
if printf '%s\n' "$plan" | rg '^Inst [^ ]+ \['; then
  echo 'Unexpected existing-package upgrade; inspect before proceeding.' >&2
  exit 2
fi
if [[ "$mode" == --dry-run ]]; then exit 0; fi
snapshot=$(mktemp -d "$root/.runtime/backups/exact_mppi_gpu_XXXXXXXX")
dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Status}\n' > "$snapshot/packages.before.tsv"
uname -a > "$snapshot/kernel.txt"
mokutil --sb-state > "$snapshot/secureboot.txt"
lsmod > "$snapshot/modules.txt"
tar -cf "$snapshot/config.tar" /etc/modprobe.d /etc/default/grub /etc/initramfs-tools /etc/apt/sources.list /etc/apt/sources.list.d 2> "$snapshot/tar.stderr"
printf '%s\n' "$plan" > "$snapshot/apt.plan.txt"
echo "Backup: $snapshot"
# Ordinary local sudo authentication. Never accept or save passwords in this script.
sudo apt-get --no-install-recommends --no-remove install "${packages[@]}" 2>&1 | tee "$snapshot/install.log"
dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Status}\n' > "$snapshot/packages.after.tsv"
python3 - "$snapshot" <<'PY'
import sys,pathlib,shlex
p=pathlib.Path(sys.argv[1])
def installed(name):
 return {r[0] for line in (p/name).read_text().splitlines() if len(r:=line.split('\t'))==3 and r[2]=='installed'}
new=sorted(installed('packages.after.tsv')-installed('packages.before.tsv'))
(p/'rollback.sh').write_text('#!/usr/bin/env bash\nset -euo pipefail\n# Review the saved package diff first. No autoremove and no reboot.\nsudo apt-get remove '+shlex.join(new)+'\n')
PY
echo 'Install completed. Secure Boot may require local MOK enrollment. No reboot was requested.'
echo "Rollback (review first): bash $snapshot/rollback.sh"
nvidia-smi || true
