# Component provenance and licenses

`exact_mppi/mppi_jax/` is an unmodified source snapshot from
https://github.com/caseypen/EXACT-mppi at
`bdc5456938a54267408f2b15870b00ff4e91c58f`, path
`EXACT_MPPI_core/exact_mppi/mppi_jax`. Its SHA256 file inventory is in
`config/exact_mppi/sources.lock.json`. The upstream GPLv3 text is retained as
`LICENSE.exact_mppi`. No simulator, ROS wrapper, gctl, ir-sim or entire upstream
workspace is installed.

`ranger_navigation/exact_controller.py` adapts and links that GPL-3.0 optimizer,
critics, distance implementation and validator. This integration is GPL-3.0;
the package's other original files retain their existing Apache-2.0 terms.
Local use/build is not a conclusion about licensing an entire distributed robot
image. Keep the corresponding sources and license on redistribution.

JAX and jaxlib are Apache-2.0; NumPy and SciPy BSD-3-Clause; PyYAML MIT.
NVIDIA CUDA/cuDNN pip components have NVIDIA component-specific licenses in the
project virtual environment's dist-info directories. They are not relicensed by
this repository. The driver/open kernel modules use the vendor's user-space
license and MIT/GPL dual kernel license respectively. Python dependency versions
and installation commands are recorded separately from upstream source identity.

Existing Ranger ROS/SDK sources retain their original licenses and prior patches;
new V2 changes are tracked in `patches/*_exact_v2_modes.patch`. Old installations,
original maps, bags and prior failure evidence remain in place.
