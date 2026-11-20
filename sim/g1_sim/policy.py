"""Unitree's pretrained G1 walking policy (unitree_rl_gym), evaluated with NumPy.

The policy is a 1-layer LSTM (47 -> 64) followed by an ELU MLP (64 -> 32 -> 12)
that outputs leg joint offsets at 50 Hz. Weights are exported from
`deploy/pre_train/g1/motion.pt` by `sim/tools/export_policy.py`, so the
simulator needs no PyTorch. Observation layout, scales and PD gains follow
`deploy/deploy_mujoco/configs/g1.yaml` in unitree_rl_gym.
"""
import numpy as np

KPS = np.array([100, 100, 100, 150, 40, 40] * 2, float)
KDS = np.array([2, 2, 2, 4, 2, 2] * 2, float)
DEFAULT_ANGLES = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2, float)
ANG_VEL_SCALE, DOF_POS_SCALE, DOF_VEL_SCALE, ACTION_SCALE = 0.25, 1.0, 0.05, 0.25
CMD_SCALE = np.array([2.0, 2.0, 0.25])
GAIT_PERIOD = 0.8
CONTROL_DT = 0.02  # simulation_dt 0.002 x control_decimation 10


def gravity_in_body(quat_wxyz):
    """Gravity direction in the body frame, as in unitree_rl_gym's deploy_mujoco."""
    w, x, y, z = quat_wxyz
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class WalkingPolicy:
    def __init__(self, weights):
        w = dict(np.load(weights)) if isinstance(weights, str) else dict(weights)
        self.w_ih, self.w_hh = w['memory.weight_ih_l0'], w['memory.weight_hh_l0']
        self.b = w['memory.bias_ih_l0'] + w['memory.bias_hh_l0']
        self.l0_w, self.l0_b = w['actor.0.weight'], w['actor.0.bias']
        self.l2_w, self.l2_b = w['actor.2.weight'], w['actor.2.bias']
        self.hidden = self.w_hh.shape[1]
        self.reset()

    def reset(self):
        self.h = np.zeros(self.hidden)
        self.c = np.zeros(self.hidden)
        self.action = np.zeros(12)

    def forward(self, obs):
        """One LSTM + MLP step (PyTorch gate order: input, forget, cell, output)."""
        gates = self.w_ih @ obs + self.w_hh @ self.h + self.b
        i, f, g, o = np.split(gates, 4)
        self.c = _sigmoid(f) * self.c + _sigmoid(i) * np.tanh(g)
        self.h = _sigmoid(o) * np.tanh(self.c)
        x = self.l0_w @ self.h + self.l0_b
        x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU(alpha=1)
        return self.l2_w @ x + self.l2_b

    def observe(self, time, quat_wxyz, omega_body, q, dq, command):
        phase = (time % GAIT_PERIOD) / GAIT_PERIOD
        return np.concatenate([
            np.asarray(omega_body) * ANG_VEL_SCALE,
            gravity_in_body(quat_wxyz),
            np.asarray(command, float) * CMD_SCALE,
            (np.asarray(q) - DEFAULT_ANGLES) * DOF_POS_SCALE,
            np.asarray(dq) * DOF_VEL_SCALE,
            self.action,
            [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)],
        ])

    def step(self, time, quat_wxyz, omega_body, q, dq, command):
        """Return leg joint position targets for velocity command (vx, vy, wz)."""
        self.action = self.forward(self.observe(time, quat_wxyz, omega_body, q, dq, command))
        return self.action * ACTION_SCALE + DEFAULT_ANGLES


def pd_torque(target, q, dq):
    return (target - q) * KPS - dq * KDS
