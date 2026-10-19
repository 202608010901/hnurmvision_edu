#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_ROOT="$REPO_ROOT/simulation"
source /opt/ros/humble/setup.bash
source "$SIM_ROOT/install_interfaces/setup.bash"
python3 "$SIM_ROOT/tools/make_training_ground.py"
cd "$SIM_ROOT/simulator"
export CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-$SIM_ROOT/simulator/target}
cargo +1.95.0 build --release --locked --no-default-features --features ros2 -j2
mkdir -p bin/lib
# Install via rename, so an already running simulator does not get overwritten in place.
cp "$CARGO_TARGET_DIR/release/daedalus" bin/daedalus.new
mv bin/daedalus.new bin/daedalus
export LD_LIBRARY_PATH="$CARGO_TARGET_DIR/release/deps:$(rustc +1.95.0 --print target-libdir):${LD_LIBRARY_PATH:-}"
ldd bin/daedalus | while read -r name arrow path rest; do
  case "$name" in
    libbevy_dylib-*.so|libstd-*.so) cp "$path" bin/lib/ ;;
  esac
done
