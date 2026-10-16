#!/usr/bin/env bash
# CPU-only libraries for the tray reproduction profile (Ubuntu 22.04).
# ROS and apt prerequisites are listed in dualarm_mprc/docs/TRAY_REPRODUCTION.md.
set -euo pipefail
PREFIX=${TRAY_DEPS_PREFIX:-"$HOME/.local/dualarm-tray"}
SOURCE_DIR=${TRAY_DEPS_SOURCE:-"$HOME/.cache/dualarm-tray-sources"}
JOBS=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
mkdir -p "$PREFIX" "$SOURCE_DIR"
export CMAKE_PREFIX_PATH="$PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PREFIX/lib:$PREFIX/lib64:${LD_LIBRARY_PATH:-}"
fetch() {
  local name=$1 url=$2 revision=$3
  if [[ ! -d "$SOURCE_DIR/$name/.git" ]]; then
    git clone "$url" "$SOURCE_DIR/$name"
  fi
  if [[ -n $(git -C "$SOURCE_DIR/$name" status --porcelain --untracked-files=no) ]]; then
    echo "Refusing to overwrite modified dependency: $SOURCE_DIR/$name" >&2
    exit 1
  fi
  git -C "$SOURCE_DIR/$name" fetch origin "$revision"
  git -C "$SOURCE_DIR/$name" checkout --detach FETCH_HEAD
  git -C "$SOURCE_DIR/$name" submodule update --init --recursive
}
build() {
  local name=$1
  shift
  cmake -S "$SOURCE_DIR/$name" -B "$SOURCE_DIR/$name/build-tray" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DBUILD_TESTING=OFF "$@"
  cmake --build "$SOURCE_DIR/$name/build-tray" --parallel "$JOBS"
  cmake --install "$SOURCE_DIR/$name/build-tray"
}
fetch libfranka https://github.com/frankaemika/libfranka.git 0.9.2
build libfranka -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
fetch mujoco https://github.com/google-deepmind/mujoco.git 3.2.0
build mujoco -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF
fetch osqp https://github.com/osqp/osqp.git 0dd00a578cf1c2691c5c379965d504c75bf6cfad
build osqp -DUNITTESTS=OFF -DPRINTING=ON
fetch osqp-eigen https://github.com/robotology/osqp-eigen.git 8b3d4c4be9f4c0e9deb6375ac8e89bc84606d125
build osqp-eigen
fetch coal https://github.com/coal-library/coal.git c0c8ad19e189b5c0dc12a9bd1c5ab15259c250cd
build coal -DBUILD_PYTHON_INTERFACE=OFF -DCOAL_HAS_OCTOMAP=ON
# Use %q for a safely sourceable user-chosen prefix, including spaces.
printf 'export TRAY_DEPS_PREFIX=%q\n' "$PREFIX" > "$PREFIX/env.sh"
cat >> "$PREFIX/env.sh" <<'ENV'
export CMAKE_PREFIX_PATH="$TRAY_DEPS_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$TRAY_DEPS_PREFIX/lib:$TRAY_DEPS_PREFIX/lib64:${LD_LIBRARY_PATH:-}"
export PATH="$TRAY_DEPS_PREFIX/bin:$PATH"
ENV
printf 'CPU libraries ready. Source %s/env.sh before building/running.\n' "$PREFIX"
