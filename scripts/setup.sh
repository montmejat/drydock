#!/usr/bin/env bash
# Fetches MuJoCo's release for Linux x86-64, at the pinned version, into
# ~/.cache/drydock (or $DRYDOCK_CACHE): the library drydock links. Then
# `cargo build`.
#
#   scripts/setup.sh
set -euo pipefail

cache=${DRYDOCK_CACHE:-$HOME/.cache/drydock}
mujoco=3.16.0
mujoco_sha256=139577c541d43716a63992ab698cbc7b09788b1d547fb82b682d59b40ef49b9d
mkdir -p "$cache"

if [[ ! -f $cache/mujoco-$mujoco/lib/libmujoco.so ]]; then
  archive=$cache/mujoco-$mujoco-linux-x86_64.tar.gz
  echo "fetching MuJoCo $mujoco" >&2
  curl -sSLf -o "$archive" \
    "https://github.com/google-deepmind/mujoco/releases/download/$mujoco/mujoco-$mujoco-linux-x86_64.tar.gz"
  echo "$mujoco_sha256  $archive" | sha256sum -c --quiet
  tar xzf "$archive" -C "$cache"
  rm "$archive"
fi

if [[ $cache != "$HOME/.cache/drydock" ]]; then
  echo "MuJoCo in $cache/mujoco-$mujoco: now MUJOCO_DIR=$cache/mujoco-$mujoco cargo build" >&2
else
  echo "MuJoCo in $cache/mujoco-$mujoco: now cargo build" >&2
fi
