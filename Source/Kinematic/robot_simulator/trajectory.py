import os

import numpy as np

from .constants import ENCODER_PPR, LINE_MAPPING_OUTPUT_DIR, LINE_MAPPING_SAMPLES_PER_SEGMENT, XYZ_OUTPUT_DIR
from .conversions import (
    angles_to_encoder_pulses,
    are_joint_angles_within_limits,
    convert_display_angles_to_ik_angles,
    convert_ik_angles_to_display_angles,
    encoder_pulses_to_angles,
    ensure_output_dir,
    MappingIKError,
)
from .io_utils import (
    export_mapped_pulses_to_txt,
    export_xyz_positions_to_txt,
    forward_kinematics_from_encoder_file_all,
    load_xyz_waypoints_from_txt,
)
from .kinematics import Inverse_Kinematics, forward_kinematics_from_joint_angles


def build_piecewise_segments_from_cartesian_points(points):
    point_array = np.array(points, dtype=float)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("Expected Cartesian points with shape (N, 3).")
    if len(point_array) < 2:
        raise ValueError("At least 2 Cartesian waypoints are required.")
    return [(point_array[idx].copy(), point_array[idx + 1].copy()) for idx in range(len(point_array) - 1)]


def rotation_matrix_to_quaternion(rotation_matrix):
    rotation_matrix = np.array(rotation_matrix, dtype=float).reshape(3, 3)
    trace = np.trace(rotation_matrix)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
        w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        x = 0.25 * s
        y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
        w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        y = 0.25 * s
        z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
        w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        z = 0.25 * s
    quaternion = np.array([w, x, y, z], dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def quaternion_to_rotation_matrix(quaternion):
    quaternion = np.array(quaternion, dtype=float).flatten()
    if quaternion.size != 4:
        raise ValueError("Expected quaternion with 4 values.")
    quaternion = quaternion / np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def slerp_rotation_matrices(start_rotation, end_rotation, alpha):
    alpha = float(alpha)
    start_quaternion = rotation_matrix_to_quaternion(start_rotation)
    end_quaternion = rotation_matrix_to_quaternion(end_rotation)
    dot = float(np.dot(start_quaternion, end_quaternion))
    if dot < 0.0:
        end_quaternion = -end_quaternion
        dot = -dot
    if dot > 0.9995:
        quaternion = start_quaternion + alpha * (end_quaternion - start_quaternion)
        quaternion /= np.linalg.norm(quaternion)
        return quaternion_to_rotation_matrix(quaternion)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)
    scale_start = np.sin(theta_0 - theta) / sin_theta_0
    scale_end = sin_theta / sin_theta_0
    quaternion = scale_start * start_quaternion + scale_end * end_quaternion
    quaternion /= np.linalg.norm(quaternion)
    return quaternion_to_rotation_matrix(quaternion)


def interpolate_straight_segment_points(start_point, end_point, num_samples=LINE_MAPPING_SAMPLES_PER_SEGMENT):
    if num_samples < 1:
        raise ValueError("num_samples must be at least 1.")
    start_point = np.array(start_point, dtype=float).flatten()
    end_point = np.array(end_point, dtype=float).flatten()
    if start_point.size != 3 or end_point.size != 3:
        raise ValueError("Expected 3D Cartesian points.")
    samples = []
    for alpha in np.linspace(0.0, 1.0, num_samples + 1)[1:]:
        samples.append((1.0 - alpha) * start_point + alpha * end_point)
    return np.array(samples, dtype=float)


def generate_polyline_position_samples(points, num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT):
    segments = build_piecewise_segments_from_cartesian_points(points)
    polyline = [np.array(points[0], dtype=float).copy()]
    segment_lengths = []
    waypoint_indices = [0]
    for start_point, end_point in segments:
        segment_lengths.append(float(np.linalg.norm(end_point - start_point)))
        polyline.extend(interpolate_straight_segment_points(start_point, end_point, num_samples=num_samples_per_segment))
        waypoint_indices.append(len(polyline) - 1)
    return np.array(polyline, dtype=float), np.array(segment_lengths, dtype=float), np.array(waypoint_indices, dtype=int)


def generate_polyline_position_rotation_targets(points, rotations, num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT):
    """Sinh vi tri polyline va rotation noi suy cho tung diem tu cac waypoint co dinh."""
    point_array = np.array(points, dtype=float)
    rotation_array = np.array(rotations, dtype=float)

    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("Expected Cartesian points with shape (N, 3).")
    if rotation_array.ndim != 3 or rotation_array.shape[1:] != (3, 3):
        raise ValueError("Expected rotations with shape (N, 3, 3).")
    if len(point_array) != len(rotation_array):
        raise ValueError("points and rotations must have the same number of waypoints.")

    polyline_points = [point_array[0].copy()]
    polyline_rotations = [rotation_array[0].copy()]
    segment_lengths = []
    waypoint_indices = [0]

    for idx in range(len(point_array) - 1):
        start_point = point_array[idx]
        end_point = point_array[idx + 1]
        start_rotation = rotation_array[idx]
        end_rotation = rotation_array[idx + 1]
        segment_lengths.append(float(np.linalg.norm(end_point - start_point)))

        for alpha in np.linspace(0.0, 1.0, num_samples_per_segment + 1)[1:]:
            # Position di theo doan thang noi hai waypoint.
            polyline_points.append((1.0 - alpha) * start_point + alpha * end_point)
            # Orientation doi muot theo tung doan bang SLERP de IK de bam hon.
            polyline_rotations.append(slerp_rotation_matrices(start_rotation, end_rotation, alpha))

        # Luu index cua waypoint goc trong polyline de con khoa cung no o buoc IK.
        waypoint_indices.append(len(polyline_points) - 1)

    return (
        np.array(polyline_points, dtype=float),
        np.array(polyline_rotations, dtype=float),
        np.array(segment_lengths, dtype=float),
        np.array(waypoint_indices, dtype=int),
    )


# Alias ten cu cho nhom helper noi suy waypoint.
generate_segment_samples = interpolate_straight_segment_points
generate_polyline_samples = generate_polyline_position_samples
generate_polyline_targets = generate_polyline_position_rotation_targets

def generate_segment_start_rotation_targets(rotations, num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT):
    """Sinh rotation fallback cho Line Map: diem giua giu orientation dau doan, diem cuoi giu orientation waypoint cuoi."""
    rotation_array = np.array(rotations, dtype=float)
    if rotation_array.ndim != 3 or rotation_array.shape[1:] != (3, 3):
        raise ValueError("Expected rotations with shape (N, 3, 3).")
    if len(rotation_array) < 2:
        raise ValueError("At least 2 waypoint rotations are required.")

    polyline_rotations = [rotation_array[0].copy()]
    waypoint_indices = [0]

    for idx in range(len(rotation_array) - 1):
        start_rotation = rotation_array[idx]
        end_rotation = rotation_array[idx + 1]

        for sample_idx in range(1, num_samples_per_segment + 1):
            if sample_idx == num_samples_per_segment:
                polyline_rotations.append(end_rotation.copy())
            else:
                polyline_rotations.append(start_rotation.copy())

        waypoint_indices.append(len(polyline_rotations) - 1)

    return np.array(polyline_rotations, dtype=float), np.array(waypoint_indices, dtype=int)

def cartesian_polyline_to_joint_trajectory(
    cartesian_points,
    target_rotations,
    initial_angles,
    fixed_waypoint_angles=None,
    angle_tolerance=180.0,
    debug=False,
):
    """
    Convert a Cartesian polyline to a joint trajectory using IK.
    The first point is anchored by initial_angles.
    """
    cartesian_points = np.array(cartesian_points, dtype=float)
    target_rotations = np.array(target_rotations, dtype=float)
    initial_angles = np.array(initial_angles, dtype=float).flatten()

    if cartesian_points.ndim != 2 or cartesian_points.shape[1] != 3:
        raise ValueError("Expected Cartesian points with shape (N, 3).")
    if len(cartesian_points) < 1:
        raise ValueError("Expected at least 1 Cartesian point.")
    if target_rotations.shape != (len(cartesian_points), 3, 3):
        raise ValueError("Expected target rotations with shape (N, 3, 3).")
    if initial_angles.size != 6:
        raise ValueError("Expected exactly 6 initial joint angles.")

    normalized_fixed_waypoint_angles = {}
    if fixed_waypoint_angles is not None:
        for point_idx, angles in fixed_waypoint_angles.items():
            angle_array = np.array(angles, dtype=float).flatten()
            if angle_array.size != 6:
                raise ValueError("Each fixed waypoint angle set must contain exactly 6 values.")
            if not are_joint_angles_within_limits(angle_array):
                raise ValueError(f"Fixed waypoint angles at index {point_idx} exceed joint limits.")
            normalized_fixed_waypoint_angles[int(point_idx)] = angle_array.copy()

    if not are_joint_angles_within_limits(initial_angles):
        raise ValueError("Initial joint angles exceed joint limits.")

    if 0 in normalized_fixed_waypoint_angles:
        initial_angles = normalized_fixed_waypoint_angles[0].copy()

    joint_trajectory = [initial_angles.copy()]
    current_angles = initial_angles.copy()

    for point_idx, point in enumerate(cartesian_points[1:], start=1):
        if point_idx in normalized_fixed_waypoint_angles:
            
            current_angles = normalized_fixed_waypoint_angles[point_idx].copy()
            joint_trajectory.append(current_angles.copy())
            continue


        ik_result = Inverse_Kinematics(
            point,
            target_rotations[point_idx],
            current_angles=current_angles,
            angle_tolerance=angle_tolerance,
            debug=debug,
        )

        if ik_result["status"] not in {"OK", "NO_MATCH"} or ik_result.get("solution") is None:
            raise MappingIKError(
                mode="Polyline",
                point_index=point_idx,
                point=point,
                status=ik_result.get("status"),
            )

        current_angles = np.array(ik_result["solution"], dtype=float)
        joint_trajectory.append(current_angles.copy())

    return np.array(joint_trajectory, dtype=float)

def build_line_mapping_from_encoder_file(
    file_path,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT,
    angle_tolerance=180.0,
    debug=False,
):
    """
    Build a piecewise-straight line-mapped trajectory from encoder pulse waypoints.
    The mapped trajectory preserves the input waypoints and inserts straight-line
    Cartesian samples between each consecutive pair of waypoints.
    """
    raw_results = forward_kinematics_from_encoder_file_all(
        file_path,
        ppr=ppr,
        gearbox_ratios=gearbox_ratios,
    )

    if len(raw_results) < 2:
        raise ValueError("Line mapping requires at least 2 pulse sets.")

    raw_pulses = np.array([item["pulses"] for item in raw_results], dtype=float)
    raw_angles = np.array([item["angles"] for item in raw_results], dtype=float)
    raw_angles_ik = convert_display_angles_to_ik_angles(raw_angles)
    raw_positions = np.array([item["position"] for item in raw_results], dtype=float)
    raw_rotations = np.array([item["rotation"] for item in raw_results], dtype=float)
    initial_angles = raw_angles_ik[0].copy()

    mapped_positions, mapped_rotations, segment_lengths, waypoint_indices = generate_polyline_position_rotation_targets(
        raw_positions,
        raw_rotations,
        num_samples_per_segment=num_samples_per_segment,
    )
    fixed_waypoint_angles = {
        int(polyline_idx): raw_angles_ik[waypoint_idx].copy()
        for waypoint_idx, polyline_idx in enumerate(waypoint_indices)
    }
    # IK chi tinh cho diem trung gian; waypoint that duoc khoa bang goc goc.
    # Uu tien orientation noi suy bang SLERP. Neu mot diem giua khong reachable
    # voi orientation noi suy, fallback ve orientation dau doan de uu tien duong di thang.
    try:
        mapped_angles_ik = cartesian_polyline_to_joint_trajectory(
            mapped_positions,
            mapped_rotations,
            initial_angles,
            fixed_waypoint_angles=fixed_waypoint_angles,
            angle_tolerance=angle_tolerance,
            debug=debug,
        )
        rotation_strategy = "segment_slerp"
    except MappingIKError as exc:
        if exc.mode != "Polyline":
            raise
        fallback_rotations, fallback_waypoint_indices = generate_segment_start_rotation_targets(
            raw_rotations,
            num_samples_per_segment=num_samples_per_segment,
        )
        if not np.array_equal(fallback_waypoint_indices, waypoint_indices):
            raise ValueError("Internal error: fallback waypoint indices do not match the polyline layout.") from exc
        mapped_angles_ik = cartesian_polyline_to_joint_trajectory(
            mapped_positions,
            fallback_rotations,
            initial_angles,
            fixed_waypoint_angles=fixed_waypoint_angles,
            angle_tolerance=angle_tolerance,
            debug=debug,
        )
        mapped_rotations = fallback_rotations
        rotation_strategy = "segment_start_fallback"
    mapped_angles = convert_ik_angles_to_display_angles(mapped_angles_ik)

    mapped_pulses = np.array(
        [
            angles_to_encoder_pulses(angles, ppr=ppr, gearbox_ratios=gearbox_ratios)
            for angles in mapped_angles
        ],
        dtype=int,
    )
    for waypoint_idx, polyline_idx in enumerate(waypoint_indices):

        mapped_pulses[polyline_idx] = np.rint(raw_pulses[waypoint_idx]).astype(int)
    mapped_pulses = np.rint(mapped_pulses).astype(int)
    mapped_angles_from_pulses = np.array(
        [
            encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
            for pulses in mapped_pulses
        ],
        dtype=float,
    )

    mapped_actual_positions = np.array(
        [forward_kinematics_from_joint_angles(angles)["position"] for angles in mapped_angles_from_pulses],
        dtype=float,
    )

    return {
        "raw_pulses": raw_pulses,
        "raw_angles": raw_angles,
        "raw_positions": raw_positions,
        "raw_rotations": raw_rotations,
        "mapped_positions": mapped_positions,
        "mapped_rotations": mapped_rotations,
        "mapped_angles": mapped_angles,
        "mapped_angles_from_pulses": mapped_angles_from_pulses,
        "mapped_actual_positions": mapped_actual_positions,
        "mapped_pulses": mapped_pulses,
        "waypoint_indices": waypoint_indices,
        "segment_lengths": segment_lengths,
        "num_samples_per_segment": int(num_samples_per_segment),
        "rotation_strategy": rotation_strategy,
    }


def export_line_mapping_from_encoder_file(
    source_file_path,
    mapped_pulses_output_path=None,
    mapped_angles_output_path=None,
    mapped_xyz_output_path=None,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT,
    angle_tolerance=180.0,
    debug=False,
):
    mapping = build_line_mapping_from_encoder_file(
        source_file_path,
        ppr=ppr,
        gearbox_ratios=gearbox_ratios,
        num_samples_per_segment=num_samples_per_segment,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
    base_name, ext = os.path.splitext(source_file_path)
    base_name = os.path.basename(base_name)
    if not ext:
        ext = ".txt"
    ensure_output_dir(LINE_MAPPING_OUTPUT_DIR)

    if mapped_pulses_output_path is None:
        mapped_pulses_output_path = os.path.join(LINE_MAPPING_OUTPUT_DIR, f"{base_name}_line_mapped{ext}")
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(LINE_MAPPING_OUTPUT_DIR, f"{base_name}_line_angles{ext}")
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.1f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(LINE_MAPPING_OUTPUT_DIR, f"{base_name}_line_xyz{ext}")
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping


def build_line_mapping_from_xyz_points(
    xyz_points,
    reference_angles,
    fixed_rotation=None,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT,
    angle_tolerance=180.0,
    debug=False,
):
    xyz_points = np.array(xyz_points, dtype=float)
    if xyz_points.ndim != 2 or xyz_points.shape[1] != 3:
        raise ValueError("Expected XYZ waypoints with shape (N, 3).")
    if len(xyz_points) < 2:
        raise ValueError("XYZ mapping requires at least 2 waypoints.")

    reference_angles = np.array(reference_angles, dtype=float).flatten()
    if reference_angles.size != 6:
        raise ValueError("Expected exactly 6 reference joint angles.")

    reference_pose = forward_kinematics_from_joint_angles(reference_angles)
    fixed_rotation = reference_pose["rotation"].copy() if fixed_rotation is None else np.array(fixed_rotation, dtype=float).reshape(3, 3)
    mapped_positions, _, segment_lengths, waypoint_indices = generate_polyline_position_rotation_targets(
        xyz_points,
        np.repeat(fixed_rotation[None, :, :], len(xyz_points), axis=0),
        num_samples_per_segment=num_samples_per_segment,
    )

    initial_angles_ik = convert_display_angles_to_ik_angles(reference_angles)
    first_point_ik = Inverse_Kinematics(mapped_positions[0], fixed_rotation, current_angles=initial_angles_ik, angle_tolerance=angle_tolerance, debug=debug)
    if first_point_ik["status"] not in {"OK", "NO_MATCH"} or first_point_ik.get("solution") is None:
        raise MappingIKError(
            mode="XYZ",
            point_index=0,
            point=mapped_positions[0],
            status=first_point_ik.get("status"),
            message=(
                f"XYZ: IK failed at first waypoint | status={first_point_ik.get('status')} | "
                f"XYZ=({mapped_positions[0][0]:.3f}, {mapped_positions[0][1]:.3f}, {mapped_positions[0][2]:.3f})"
            ),
        )

    fixed_waypoint_angles = {0: np.array(first_point_ik["solution"], dtype=float).copy()}
    mapped_angles_ik = cartesian_polyline_to_joint_trajectory(
        mapped_positions,
        np.repeat(fixed_rotation[None, :, :], len(mapped_positions), axis=0),
        initial_angles_ik,
        fixed_waypoint_angles=fixed_waypoint_angles,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
    mapped_angles = convert_ik_angles_to_display_angles(mapped_angles_ik)
    mapped_pulses = np.array([angles_to_encoder_pulses(angles, ppr=ppr, gearbox_ratios=gearbox_ratios) for angles in mapped_angles], dtype=int)
    mapped_angles_from_pulses = np.array(
        [encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios) for pulses in mapped_pulses],
        dtype=float,
    )
    mapped_actual_positions = np.array([forward_kinematics_from_joint_angles(angles)["position"] for angles in mapped_angles_from_pulses], dtype=float)

    return {
        "input_xyz_points": xyz_points,
        "fixed_rotation": fixed_rotation,
        "reference_angles": reference_angles,
        "mapped_positions": mapped_positions,
        "mapped_angles": mapped_angles,
        "mapped_angles_from_pulses": mapped_angles_from_pulses,
        "mapped_actual_positions": mapped_actual_positions,
        "mapped_pulses": mapped_pulses,
        "waypoint_indices": waypoint_indices,
        "segment_lengths": segment_lengths,
        "num_samples_per_segment": int(num_samples_per_segment),
    }


def export_line_mapping_from_xyz_file(
    source_file_path,
    reference_angles,
    fixed_rotation=None,
    mapped_pulses_output_path=None,
    mapped_angles_output_path=None,
    mapped_xyz_output_path=None,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT,
    angle_tolerance=180.0,
    debug=False,
):
    xyz_points = load_xyz_waypoints_from_txt(source_file_path)
    mapping = build_line_mapping_from_xyz_points(
        xyz_points,
        reference_angles=reference_angles,
        fixed_rotation=fixed_rotation,
        ppr=ppr,
        gearbox_ratios=gearbox_ratios,
        num_samples_per_segment=num_samples_per_segment,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
    base_name, ext = os.path.splitext(source_file_path)
    base_name = os.path.basename(base_name)
    if not ext:
        ext = ".txt"
    ensure_output_dir(XYZ_OUTPUT_DIR)

    if mapped_pulses_output_path is None:
        mapped_pulses_output_path = os.path.join(XYZ_OUTPUT_DIR, f"{base_name}_xyz_pulses{ext}")
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(XYZ_OUTPUT_DIR, f"{base_name}_xyz_angles{ext}")
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.1f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(XYZ_OUTPUT_DIR, f"{base_name}_xyz_actual{ext}")
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping
