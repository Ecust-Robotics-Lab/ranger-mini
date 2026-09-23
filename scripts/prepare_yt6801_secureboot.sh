#!/bin/bash
# Reviewed local preparation only: sign one pinned module and request MOK enrollment.
# Does not load/install a module, change networking, or reboot. Default: dry-run.
set -euo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin

mode=${1:---dry-run}
case "$mode" in --dry-run|--apply) ;; *) echo 'Usage: bash prepare_yt6801_secureboot.sh [--dry-run|--apply]' >&2; exit 2;; esac
[[ $# -le 1 ]] || exit 2
module=/home/lab307/ranger_nav/vendor/yt6801/source/yt6801-1.0.34/yt6801.ko
expected=aed6951c8e9656d0443c2a52b07ce0be8061a56a46d83309b9d8bebba3c83918
state=/var/lib/ranger-nav-yt6801
kernel=6.8.0-40-generic

for tool in sha256sum modinfo mokutil openssl kmodsign install ip; do
    command -v "$tool" >/dev/null || { echo "Missing tool: $tool" >&2; exit 1; }
done
[[ $(uname -r) == "$kernel" ]] || { echo 'Kernel changed; rebuild and review required.' >&2; exit 1; }
printf '%s  %s\n' "$expected" "$module" | sha256sum --check --status
[[ $(modinfo -F name "$module") == yt6801 ]]
[[ $(modinfo -F version "$module") == 1.0.34 ]]
[[ $(modinfo -F vermagic "$module") == "$kernel "* ]]
[[ $(modinfo -F alias "$module") == 'pci:v00001F0Ad00006801sv*sd*bc*sc*i*' ]]
[[ -z $(modinfo -F signer "$module") ]]
[[ ! -e "$state" && ! -L "$state" ]] || { echo "Existing $state preserved; inspect before resuming." >&2; exit 1; }
pending=$(mokutil --list-new)
[[ -z "$pending" ]] || { echo 'Existing pending MOK enrollment; resolve separately first.' >&2; exit 1; }

cat <<'PLAN'
Validated YT6801 1.0.34 for kernel 6.8.0-40-generic and PCI 1f0a:6801.
Apply will:
  1. Create root-only /var/lib/ranger-nav-yt6801 and save pre-change state.
  2. Generate a dedicated module-signing key (private key stays root-only).
  3. Copy the pinned module there and sign the copy.
  4. Run mokutil --import for the public certificate; enter a temporary
     enrollment password ONLY in this local terminal when prompted.
No module is loaded or installed, no network settings changed, no reboot.
After successful import, save your work and reboot manually. In MokManager:
Enroll MOK -> Continue -> Yes -> enter the temporary password -> Reboot.
Expected certificate CN: ranger-nav YT6801 module signing
PLAN
[[ "$mode" == --apply ]] || { echo 'DRY-RUN PASS: no changes made.'; exit 0; }
[[ $EUID -eq 0 ]] || { echo 'Apply requires sudo in your local terminal.' >&2; exit 1; }
[[ -t 0 ]] || { echo 'Apply requires an interactive local terminal.' >&2; exit 1; }
umask 077
install -d -m 0700 "$state"
ip -j address > "$state/address-before.json"
ip -j route show table all > "$state/routes-before.json"
mokutil --sb-state > "$state/secureboot-before.txt"
mokutil --list-enrolled > "$state/enrolled-before.txt"
cat > "$state/openssl.cnf" <<'CONFIG'
[req]
distinguished_name = dn
x509_extensions = module_only
prompt = no
[dn]
CN = ranger-nav YT6801 module signing
[module_only]
basicConstraints = critical,CA:FALSE
keyUsage = critical,digitalSignature
extendedKeyUsage = codeSigning,1.3.6.1.4.1.2312.16.1.2
subjectKeyIdentifier = hash
CONFIG
openssl req -new -x509 -newkey rsa:2048 -nodes -days 3650 \
    -config "$state/openssl.cnf" -outform DER \
    -keyout "$state/MOK.priv" -out "$state/MOK.der"
install -m 0600 "$module" "$state/yt6801.ko"
printf '%s  %s\n' "$expected" "$state/yt6801.ko" | sha256sum --check --status
kmodsign sha256 "$state/MOK.priv" "$state/MOK.der" "$state/yt6801.ko"
[[ $(modinfo -F signer "$state/yt6801.ko") == 'ranger-nav YT6801 module signing' ]]
sha256sum "$state/yt6801.ko" > "$state/signed-module.sha256"
openssl x509 -inform DER -in "$state/MOK.der" -noout -subject -fingerprint -sha256
echo 'Enter the temporary enrollment password at the mokutil prompts below; do not send it in chat.'
mokutil --import "$state/MOK.der"
echo 'MOK request submitted. No reboot performed; save work and reboot locally when ready.'
