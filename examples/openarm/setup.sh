#!/usr/bin/env bash
# Fetches what the OpenArm example needs and drydock doesn't ship, at pinned
# versions, into ~/.cache/drydock (or $DRYDOCK_CACHE):
#
# - openarm_mujoco: OpenArm's MuJoCo models (Apache-2.0), linked here as
#   examples/openarm/openarm_mujoco for robot.yml to find;
# - openarm_can: OpenArm's own CAN library (Apache-2.0), which `hold` is
#   built on, so the arm is driven by the code that drives the real one.
#
# Then builds `hold` with g++.
#
#   examples/openarm/setup.sh
set -euo pipefail

cache=${DRYDOCK_CACHE:-$HOME/.cache/drydock}
# A commit after the 2.3.0 release: V1's masses and inertias measured, its
# finger axes fixed, and a `home` keyframe.
mujoco_commit=a3c8a57ec15d385e3c1dbd3fd090ee486b7f258c
can_tag=1.4.1
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$cache"

fetch() { # repo ref dir: a tag or a commit, one commit deep
  if [[ ! -d $3 ]]; then
    echo "fetching $1 at $2" >&2
    git init -q "$3.part"
    git -C "$3.part" fetch -q --depth 1 "https://github.com/enactic/$1.git" "$2"
    git -C "$3.part" -c advice.detachedHead=false checkout -q FETCH_HEAD
    mv "$3.part" "$3"
  fi
}
mujoco=$cache/openarm_mujoco-${mujoco_commit:0:12}
fetch openarm_mujoco "$mujoco_commit" "$mujoco"
fetch openarm_can "$can_tag" "$cache/openarm_can-$can_tag"
ln -sfn "$mujoco" "$here/openarm_mujoco"

can=$cache/openarm_can-$can_tag
echo "building hold against openarm_can $can_tag" >&2
g++ -std=c++17 -O2 -Wall -Wextra -I "$can/include" "$here/hold.cpp" \
  "$can"/src/openarm/can/socket/*.cpp "$can"/src/openarm/canbus/*.cpp "$can"/src/openarm/damiao_motor/*.cpp \
  -o "$here/hold"
echo "ready: drydock examples/openarm/robot.yml, then examples/openarm/hold" >&2
