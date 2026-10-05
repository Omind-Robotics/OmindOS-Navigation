#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/.." && pwd)"
version="${CASADI_VERSION:-3.5.5}"
prefix="${1:-${workspace}/.local/casadi}"
source_dir="${workspace}/.local/src/casadi-${version}"
build_dir="${source_dir}/build-nav3d"

mkdir -p "$(dirname "${source_dir}")" "${prefix}"
if [[ ! -d "${source_dir}/.git" ]]; then
  git clone --depth 1 --branch "${version}" \
    https://github.com/casadi/casadi.git "${source_dir}"
fi

cmake -S "${source_dir}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${prefix}" \
  -DWITH_IPOPT=ON \
  -DWITH_PYTHON=OFF \
  -DWITH_EXAMPLES=OFF
cmake --build "${build_dir}" --parallel "$(nproc)"
cmake --install "${build_dir}"

echo "CasADi installed at ${prefix}. Re-source scripts/nav3d_env.sh before building."
