import matplotlib.pyplot as plt
import numpy as np

from .constants import THETA2_OFFSET
from .kinematics import Forward_Kinematics, forward_points


def set_axes_fixed(ax, lim=700, base=(0.0, 0.0, 0.0)):
    bx, by, bz = base
    ax.set_xlim(bx - lim, bx + lim)
    ax.set_ylim(by - lim, by + lim)
    ax.set_zlim(bz, bz + lim)


def plot_robot(
    points,
    ax=None,
    title="Robot 6-Axis",
    lim=800,
    base=(0.0, 0.0, 0.0),
    trace_points=None,
    end_effector_rotation=None,
    end_effector_axis_length=90.0,
):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

    elev, azim = ax.elev, ax.azim
    ax.cla()
    xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
    ax.plot(xs, ys, zs, marker="o", linewidth=3, markersize=2)
    ax.scatter(xs, ys, zs, color="#000000", s=20, depthshade=False)

    if end_effector_rotation is not None:
        end_effector_rotation = np.array(end_effector_rotation, dtype=float).reshape(3, 3)
        ee_origin = points[-1]
        axis_colors = ["#d62728", "#2ca02c", "#1f77b4"]
        axis_labels = ["X", "Y", "Z"]
        for axis_idx in range(3):
            axis_vector = end_effector_rotation[:, axis_idx] * float(end_effector_axis_length)
            ax.quiver(
                ee_origin[0],
                ee_origin[1],
                ee_origin[2],
                axis_vector[0],
                axis_vector[1],
                axis_vector[2],
                color=axis_colors[axis_idx],
                linewidth=2,
                arrow_length_ratio=0.15,
            )
            axis_tip = ee_origin + axis_vector
            ax.text(axis_tip[0], axis_tip[1], axis_tip[2], axis_labels[axis_idx], color=axis_colors[axis_idx], fontsize=9, fontweight="bold")

    if trace_points is not None and len(trace_points) > 0:
        trace_points = np.array(trace_points, dtype=float)
        ax.plot(trace_points[:, 0], trace_points[:, 1], trace_points[:, 2], color="#d62728", linewidth=2, alpha=0.9)
        ax.scatter(trace_points[:, 0], trace_points[:, 1], trace_points[:, 2], color="#d62728", s=2, depthshade=False)

    ax.set_title(title)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    set_axes_fixed(ax, lim=lim, base=base)
    ax.view_init(elev=elev, azim=azim)
    return ax


def animate_jog(q_start, q_end, steps=80, interval=0.02, use_offset=True):
    q_start = np.array(q_start, dtype=float)
    q_end = np.array(q_end, dtype=float)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for s in range(steps + 1):
        a = s / steps
        q = (1 - a) * q_start + a * q_end
        pts, _ = forward_points(*q, use_offset=use_offset)
        plot_robot(pts, ax=ax, title=f"q(deg) = {np.round(q, 1)}")
        plt.pause(interval)
    plt.show()


def sample_trace_positions(q_start, q_end, num_samples=25):
    """Lay mau vi tri end-effector giua hai bo goc de ve trace."""
    q_start = np.array(q_start, dtype=float).flatten()
    q_end = np.array(q_end, dtype=float).flatten()
    if q_start.size != 6 or q_end.size != 6:
        raise ValueError("Expected exactly 6 joint values for trajectory sampling.")
    positions = []
    for alpha in np.linspace(0.0, 1.0, num_samples + 1)[1:]:
        q = (1.0 - alpha) * q_start + alpha * q_end
        T06 = Forward_Kinematics(q[0], q[1] + THETA2_OFFSET, q[2], q[3], q[4], q[5])
        positions.append(T06[:3, 3].copy())
    return positions
