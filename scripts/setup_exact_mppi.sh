#!/usr/bin/env bash
# Project-local pinned JAX/CUDA dependencies; never changes a system driver.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:---dry-run}" in
 --dry-run) echo 'DRY-RUN: create .runtime/exact_mppi_venv; install config/exact_mppi/requirements.lock.txt (GPU CUDA runtime included).'; exit 0;;
 --execute) ;;
 *) exit 2;;
esac
prefix="$root/.runtime/exact_mppi_venv"
if [[ ! -x "$prefix/bin/python3" ]]; then python3 -m venv --without-pip "$prefix"; fi
if ! "$prefix/bin/python3" -m pip --version >/dev/null 2>&1; then
 curl --fail --location --proto '=https' https://bootstrap.pypa.io/get-pip.py -o "$root/.runtime/exact_mppi_get_pip.py"
 "$prefix/bin/python3" "$root/.runtime/exact_mppi_get_pip.py"
fi
"$prefix/bin/python3" -m pip install -r "$root/config/exact_mppi/requirements.lock.txt"
JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false "$prefix/bin/python3" - <<'PY'
import jax,jax.numpy as jnp
print(jax.devices())
x=jnp.ones((64,64));(x@x).block_until_ready()
print('JAX CUDA computation PASS')
PY
