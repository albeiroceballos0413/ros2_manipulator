"""Cinemática directa de un manipulador planar RRR (3 GDL) con Denavit-Hartenberg."""
import numpy as np


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Matriz homogénea 4x4 de un eslabón (convención DH clásica)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def forward_kinematics(q, lengths) -> np.ndarray:
    """T_0^3 = A1·A2·A3. q: ángulos [rad], lengths: [L1, L2, L3] en m."""
    T = np.eye(4)
    for qi, Li in zip(q, lengths):
        T = T @ dh_transform(a=Li, alpha=0.0, d=0.0, theta=qi)
    return T


def end_effector_pose(q, lengths):
    """Devuelve (x, y, phi) del efector final."""
    T = forward_kinematics(q, lengths)
    return T[0, 3], T[1, 3], np.arctan2(T[1, 0], T[0, 0])
