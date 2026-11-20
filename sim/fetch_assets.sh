#!/usr/bin/env bash
# Fetch the G1 MuJoCo model (meshes + MJCF) from unitree_rl_gym at a pinned commit
# into vendor/sim (gitignored). The walking policy weights are already in
# sim/assets/g1_motion_policy.npz (exported from the same commit).
set -euo pipefail
cd "$(dirname "$0")/.."
COMMIT=276801e46c5d433564f24658bac64f254b7d2d4b
DEST=vendor/sim/unitree_rl_gym
if [[ ! -d $DEST/.git ]]; then
  mkdir -p vendor/sim
  git clone -q --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_rl_gym.git "$DEST"
fi
git -C "$DEST" sparse-checkout set resources/robots/g1_description deploy/pre_train/g1 deploy/deploy_mujoco
git -C "$DEST" -c advice.detachedHead=false checkout -q "$COMMIT"
echo "unitree_rl_gym $(git -C "$DEST" rev-parse --short HEAD) ready in $DEST"
