import numpy as np
from arm_kinematics.kinematics import end_effector_pose

L = [0.30, 0.25, 0.10]


def close(a, b):
    assert np.allclose(a, b, atol=1e-9), (a, b)


def test_home():
    close(end_effector_pose([0, 0, 0], L), (0.65, 0.0, 0.0))


def test_q1_90():
    close(end_effector_pose([np.pi / 2, 0, 0], L), (0.0, 0.65, np.pi / 2))


def test_q2_90():
    close(end_effector_pose([0, np.pi / 2, 0], L), (0.30, 0.35, np.pi / 2))


def test_matches_closed_form():
    q = [0.3, -0.7, 1.1]
    x = sum(Li * np.cos(sum(q[:i + 1])) for i, Li in enumerate(L))
    y = sum(Li * np.sin(sum(q[:i + 1])) for i, Li in enumerate(L))
    close(end_effector_pose(q, L), (x, y, sum(q)))
