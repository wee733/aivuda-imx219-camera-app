#!/usr/bin/env bash
# Offline build for Ubuntu Noble's extracted gcc-13-aarch64-linux-gnu toolchain.
# See ../README.md for the exact companion source and target SDK prerequisites.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo "Usage: $0 SYSROOT CUDA_BUFFER_SOURCE EMPTY_OUTPUT_DIR" >&2
  exit 2
fi
sys=$(realpath "$1")
companion=$(realpath "$2")
out=$(realpath "$3")
[[ -d $out && -z $(ls -A -- "$out") ]] || { echo 'Output directory must exist and be empty.' >&2; exit 2; }
src=$(cd -- "$(dirname -- "$0")" && pwd)
ros=$sys/opt/ros/lyrical
cuda=$sys/usr/local/cuda-13.2/targets/sbsa-linux
# The relocated cross-binutils use host-side BFD and libsframe from the same SDK.
export LD_LIBRARY_PATH="$sys/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cxx=("$sys/usr/bin/aarch64-linux-gnu-g++-13" "--sysroot=$sys" -O3 -DNDEBUG -fPIC -std=c++17
  -Wall -Wextra -Wpedantic "-ffile-prefix-map=$companion=/opt/ros/lyrical"
  "-ffile-prefix-map=$src=cuda_buffer_backend" "-ffile-prefix-map=$sys=/"
  "-I$companion/include" "-I$cuda/include" "-I$src/include" "-I$sys/usr/include")
for dir in "$ros"/include/*; do cxx+=("-I$dir"); done
link=(-shared -Wl,-z,defs "-L$ros/lib" "-L$cuda/lib" "-L$cuda/lib/stubs" "-L$sys/usr/lib/aarch64-linux-gnu")
objects=()
for unit in cuda_buffer cuda_buffer_impl cuda_buffer_ipc_manager cuda_memory_pool host_endpoint_manager; do
  "${cxx[@]}" -c "$companion/src/$unit.cpp" -o "$out/$unit.o"
  objects+=("$out/$unit.o")
done
# Link-only counterpart of the image's patched libcuda_buffer; retain its original
# absolute DT_NEEDED. Never install this copy or the CUDA driver's link stub.
"${cxx[@]}" "${link[@]}" -Wl,-soname,/opt/ros/lyrical/lib/libcuda_buffer.so "${objects[@]}" \
  -lrcutils -lcudart -lcuda -o "$out/libcuda_buffer.so"
"${cxx[@]}" -c "$src/src/cuda_buffer_backend_plugin.cpp" -o "$out/plugin.o"
"${cxx[@]}" "-L$out" "${link[@]}" -Wl,-soname,libcuda_buffer_backend.so -Wl,-z,nodelete \
  -Wl,-rpath,/opt/ros/lyrical/lib:/usr/local/cuda-13.2/targets/sbsa-linux/lib:/opt/ros/lyrical/lib/aarch64-linux-gnu \
  "$out/plugin.o" -lcuda_buffer_backend_msgs__rosidl_typesupport_cpp -lcuda_buffer -lcudart \
  -lclass_loader -lconsole_bridge -lrcutils -lcuda -o "$out/libcuda_buffer_backend.so"
