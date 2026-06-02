#!/usr/bin/env python3
"""Lightweight FK + damped numerical IK for Z1 Gazebo admittance demo."""

from __future__ import annotations

import numpy as np


class Z1Kinematics:
    def __init__(self):
        self.dh_params = [
            [0.0, 0.0, 0.0585, 0.0],
            [0.0, -np.pi / 2, 0.0, -np.pi / 2],
            [0.35, 0.0, 0.0, 0.0],
            [0.218, np.pi / 2, 0.057, 0.0],
            [0.07, -np.pi / 2, 0.0, 0.0],
            [0.0492, np.pi / 2, 0.0, 0.0],
        ]
        self.joint_offsets = np.zeros(6)

    def dh_transform(self, a, alpha, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def forward_kinematics(self, q):
        theta = q + self.joint_offsets
        t = np.eye(4)
        for i in range(6):
            a, alpha, d, theta0 = self.dh_params[i]
            t = t @ self.dh_transform(a, alpha, d, theta[i] + theta0)
        pos = t[:3, 3]
        rot = t[:3, :3]
        rx = np.arctan2(rot[2, 1], rot[2, 2])
        ry = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1] ** 2 + rot[2, 2] ** 2))
        rz = np.arctan2(rot[1, 0], rot[0, 0])
        return np.array([pos[0], pos[1], pos[2], rx, ry, rz])

    def get_end_effector_pose(self, q):
        return self.forward_kinematics(q)


class InverseKinematicsSolver:
    def __init__(self, kin: Z1Kinematics, damping: float = 0.02, max_iter: int = 80, tolerance: float = 1e-3):
        self.kin = kin
        self.damping = damping
        self.max_iter = max_iter
        self.tolerance = tolerance
        self._eps = 1e-5

    def _pose_error(self, cur: np.ndarray, tgt: np.ndarray) -> np.ndarray:
        e = tgt - cur
        e[3:] = np.arctan2(np.sin(e[3:]), np.cos(e[3:]))
        return e

    def _jacobian(self, q: np.ndarray) -> np.ndarray:
        j = np.zeros((6, 6))
        f0 = self.kin.get_end_effector_pose(q)
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = self._eps
            f1 = self.kin.get_end_effector_pose(q + dq)
            j[:, i] = (f1 - f0) / self._eps
        return j

    def solve(self, target_pose: np.ndarray, q_guess: np.ndarray) -> np.ndarray:
        q = np.array(q_guess, dtype=float)
        lam = self.damping**2
        for _ in range(self.max_iter):
            cur = self.kin.get_end_effector_pose(q)
            err = self._pose_error(cur, target_pose)
            if np.linalg.norm(err) < self.tolerance:
                break
            j = self._jacobian(q)
            jtj = j.T @ j + lam * np.eye(6)
            dq = np.linalg.solve(jtj, j.T @ err)
            q += dq
            q = np.clip(q, -2.8, 2.8)
        return q
