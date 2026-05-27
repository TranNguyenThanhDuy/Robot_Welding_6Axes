import matplotlib.pyplot as plt
import numpy as np
from itertools import product

from .constants import (
    THETA2_OFFSET,
    WORKSPACE_FIXED_ANGLES,
    WORKSPACE_POINT_SIZE,
    WORKSPACE_SAMPLED_JOINTS,
    WORKSPACE_SAMPLES_PER_JOINT,
)
from .conversions import get_joint_angle_limits
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


def generate_workspace_points(
    samples_per_joint=WORKSPACE_SAMPLES_PER_JOINT,
    joint_limits=None,
    sampled_joint_indices=WORKSPACE_SAMPLED_JOINTS,
    fixed_angles=WORKSPACE_FIXED_ANGLES,
):
    samples_per_joint = int(samples_per_joint)
    if samples_per_joint < 2:
        raise ValueError("samples_per_joint must be at least 2.")

    limits = get_joint_angle_limits() if joint_limits is None else np.array(joint_limits, dtype=float)
    if limits.shape != (6, 2):
        raise ValueError("joint_limits must have shape (6, 2).")

    sampled_joint_indices = tuple(int(idx) for idx in sampled_joint_indices)
    if not sampled_joint_indices:
        raise ValueError("sampled_joint_indices must not be empty.")
    if any(idx < 0 or idx >= 6 for idx in sampled_joint_indices):
        raise ValueError("sampled_joint_indices must contain joint indices from 0 to 5.")

    fixed_angles = np.array(fixed_angles, dtype=float).flatten()
    if fixed_angles.size != 6:
        raise ValueError("fixed_angles must contain exactly 6 joint angles.")

    joint_samples = [np.linspace(limits[idx, 0], limits[idx, 1], samples_per_joint) for idx in sampled_joint_indices]
    workspace_points = np.empty((samples_per_joint ** len(sampled_joint_indices), 3), dtype=float)

    for point_idx, sampled_values in enumerate(product(*joint_samples)):
        q = fixed_angles.copy()
        for joint_idx, joint_value in zip(sampled_joint_indices, sampled_values):
            q[joint_idx] = joint_value
        T06 = Forward_Kinematics(q[0], q[1] + THETA2_OFFSET, q[2], q[3], q[4], q[5])
        workspace_points[point_idx] = T06[:3, 3]

    return workspace_points


def set_axes_equal_from_points(ax, points, padding_ratio=0.08):
    points = np.array(points, dtype=float)
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    center = (mins + maxs) / 2.0
    radius = float(np.max(maxs - mins) / 2.0)
    radius = max(radius * (1.0 + padding_ratio), 1e-6)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def plot_workspace_cloud(workspace_points, samples_per_joint=WORKSPACE_SAMPLES_PER_JOINT, point_size=WORKSPACE_POINT_SIZE):
    workspace_points = np.array(workspace_points, dtype=float)
    if workspace_points.ndim != 2 or workspace_points.shape[1] != 3:
        raise ValueError("workspace_points must have shape (N, 3).")

    points_m = workspace_points / 1000.0
    fig = plt.figure(figsize=(8, 8), facecolor="#e8e8e8")
    fig.canvas.manager.set_window_title("Robot Workspace Cloud")
    ax = fig.add_subplot(111, projection="3d", facecolor="#f1f1f1")
    ax.scatter(
        points_m[:, 0],
        points_m[:, 1],
        points_m[:, 2],
        color="#ff0000",
        s=point_size,
        alpha=0.95,
        marker=".",
        linewidths=0,
        depthshade=False,
    )
    ax.set_title("Workspace of Welding Robot", pad=14)
    ax.set_xlabel("X (m)", labelpad=8)
    ax.set_ylabel("Y (m)", labelpad=8)
    ax.set_zlabel("Z (m)", labelpad=8)
    set_axes_equal_from_points(ax, points_m, padding_ratio=0.12)
    ax.set_box_aspect((1.0, 1.0, 0.85))
    ax.view_init(elev=18, azim=-38)
    ax.grid(True, color="#d8d8d8", linewidth=0.7)
    ax.xaxis.pane.set_facecolor((0.94, 0.94, 0.94, 1.0))
    ax.yaxis.pane.set_facecolor((0.94, 0.94, 0.94, 1.0))
    ax.zaxis.pane.set_facecolor((0.94, 0.94, 0.94, 1.0))
    ax.xaxis.pane.set_edgecolor("#d0d0d0")
    ax.yaxis.pane.set_edgecolor("#d0d0d0")
    ax.zaxis.pane.set_edgecolor("#d0d0d0")
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.92)
    return fig, ax


def run_workspace_preview(samples_per_joint=WORKSPACE_SAMPLES_PER_JOINT):
    workspace_points = generate_workspace_points(samples_per_joint=samples_per_joint)
    return plot_workspace_cloud(workspace_points, samples_per_joint=samples_per_joint)
