"""Animación 2D del brazo RRR usando la cinemática directa de arm_kinematics."""
import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from arm_kinematics.kinematics import dh_transform, end_effector_pose

L = [0.30, 0.25, 0.10]          # longitudes de eslabones [m]
FPS = 30
DURATION = 8.0                  # segundos de una vuelta completa


def joint_positions(q, lengths):
    """Origen de cada frame: O0 (base), O1, O2, O3 (punta)."""
    T = np.eye(4)
    pts = [T[:2, 3].copy()]
    for qi, Li in zip(q, lengths):
        T = T @ dh_transform(Li, 0.0, 0.0, qi)
        pts.append(T[:2, 3].copy())
    return np.array(pts)


def trajectory(t):
    """Ángulos articulares de ejemplo: senoidales con distinta frecuencia."""
    w = 2 * np.pi / DURATION
    q1 = np.radians(45) + np.radians(40) * np.sin(w * t)
    q2 = np.radians(60) * np.sin(2 * w * t)
    q3 = np.radians(90) * np.sin(3 * w * t)
    return np.array([q1, q2, q3])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', help='guardar como GIF en vez de mostrar ventana')
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(6, 6))
    reach = sum(L) * 1.1
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Manipulador planar RRR — cinemática directa')
    ax.add_patch(plt.Circle((0, 0), sum(L), fill=False, ls='--', alpha=0.4))

    arm, = ax.plot([], [], '-o', lw=4, ms=8, color='tab:blue')
    trace, = ax.plot([], [], '-', lw=1, color='tab:red', alpha=0.7)
    info = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', family='monospace')
    path = []

    frames = int(DURATION * FPS)

    def update(i):
        q = trajectory(i / FPS)
        pts = joint_positions(q, L)
        x, y, phi = end_effector_pose(q, L)
        if i == 0:
            path.clear()
        path.append((x, y))
        arm.set_data(pts[:, 0], pts[:, 1])
        trace.set_data(*zip(*path))
        qd = np.degrees(q)
        info.set_text(
            f'q = ({qd[0]:6.1f}, {qd[1]:6.1f}, {qd[2]:6.1f}) deg\n'
            f'x = {x:6.3f} m   y = {y:6.3f} m\n'
            f'phi = {np.degrees(phi):6.1f} deg')
        return arm, trace, info

    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / FPS, blit=True)
    if args.save:
        anim.save(args.save, writer='pillow', fps=FPS)
        print(f'Guardado en {args.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
