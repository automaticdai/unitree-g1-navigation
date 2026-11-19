"""Export unitree_rl_gym's TorchScript G1 policy to NumPy weights (needs torch, once).

    python sim/tools/export_policy.py vendor/sim/unitree_rl_gym/deploy/pre_train/g1/motion.pt \
        sim/assets/g1_motion_policy.npz [--check]

--check also compares the NumPy policy with TorchScript on random observations.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_sim.policy import WalkingPolicy  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument('policy')
parser.add_argument('output')
parser.add_argument('--check', action='store_true')
args = parser.parse_args()
model = torch.jit.load(args.policy, map_location='cpu')
weights = {k: v.detach().numpy().astype(np.float64) for k, v in model.state_dict().items()
           if k.startswith(('memory.', 'actor.'))}
np.savez(args.output, **weights)
if args.check:
    policy = WalkingPolicy(weights)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        obs = rng.normal(0, 0.5, 47)
        expected = model(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)).detach().numpy().squeeze()
        worst = max(worst, float(np.abs(policy.forward(obs) - expected).max()))
    print(f'max |numpy - torch| over 200 recurrent steps: {worst:.2e}')
    assert worst < 1e-4
