import os

import numpy as np

from .constants import (
    CIRCLE_OUTPUT_DIR,
    ENCODER_PPR,
    LINE_MAPPING_LOCK_Z_TO_FIRST,
    LINE_MAPPING_OUTPUT_DIR,
    LINE_MAPPING_SAMPLES_PER_SEGMENT,
    XYZ_OUTPUT_DIR,
)
from .conversions import (
    angles_to_encoder_pulses,
    are_joint_angles_within_limits,
    convert_display_angles_to_ik_angles,
    convert_ik_angles_to_display_angles,
    encoder_pulses_to_angles,
    ensure_output_dir,
    normalize_angle,
)
from .io_utils import (
    export_mapped_pulses_to_txt,
    export_xyz_positions_to_txt,
    forward_kinematics_from_encoder_file_all,
    load_xyz_waypoints_from_txt,
)
from .kinematics import Check_T_0_6, Inverse_Kinematics, forward_kinematics_from_joint_angles


class MappingIKError(ValueError):
    def __init__(self, mode, point_index=None, point=None, status=None, message=None):
        self.mode = mode
        self.point_index = point_index
        self.point = None if point is None else np.array(point, dtype=float).flatten()
        self.status = status
        super().__init__(message or self._build_message())

    def _build_message(self):
        pieces = [f"{self.mode}: IK failed"]
        if self.point_index is not None:
            pieces.append(f"at point #{int(self.point_index) + 1}")
        if self.status:
            pieces.append(f"status={self.status}")
        if self.point is not None and self.point.size == 3:
            pieces.append(f"XYZ=({self.point[0]:.3f}, {self.point[1]:.3f}, {self.point[2]:.3f})")
        return " | ".join(pieces)


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
            polyline_points.append((1.0 - alpha) * start_point + alpha * end_point)
            polyline_rotations.append(slerp_rotation_matrices(start_rotation, end_rotation, alpha))
        waypoint_indices.append(len(polyline_points) - 1)
    return (
        np.array(polyline_points, dtype=float),
        np.array(polyline_rotations, dtype=float),
        np.array(segment_lengths, dtype=float),
        np.array(waypoint_indices, dtype=int),
    )


generate_segment_samples = interpolate_straight_segment_points
generate_polyline_samples = generate_polyline_position_samples
generate_polyline_targets = generate_polyline_position_rotation_targets


def build_relay_mapped_pulses(mapped_pulses, relay_states, waypoint_indices):
    mapped_pulses = np.rint(np.array(mapped_pulses, dtype=float)).astype(int)
    relay_states = np.array(relay_states, dtype=int).flatten()
    waypoint_indices = np.array(waypoint_indices, dtype=int).flatten()

    if mapped_pulses.ndim != 2 or mapped_pulses.shape[1] != 6:
        raise ValueError("Expected mapped pulses with shape (N, 6).")
    if relay_states.size != waypoint_indices.size:
        raise ValueError("Expected one relay state per source waypoint.")
    if waypoint_indices.size < 1:
        raise ValueError("Expected at least one waypoint index.")

    output_pulses = [mapped_pulses[0].copy()]
    output_relays = [1 if relay_states[0] != 0 else 0]

    for segment_idx in range(waypoint_indices.size - 1):
        start_idx = int(waypoint_indices[segment_idx])
        end_idx = int(waypoint_indices[segment_idx + 1])
        current_state = 1 if relay_states[segment_idx] != 0 else 0
        next_state = 1 if relay_states[segment_idx + 1] != 0 else 0

        segment_state = current_state
        if current_state == 1 and next_state == 0:
            output_pulses.append(mapped_pulses[start_idx].copy())
            output_relays.append(0)
            segment_state = 0

        for mapped_idx in range(start_idx + 1, end_idx + 1):
            output_pulses.append(mapped_pulses[mapped_idx].copy())
            output_relays.append(segment_state)

        if current_state == 0 and next_state == 1:
            output_pulses.append(mapped_pulses[end_idx].copy())
            output_relays.append(1)

    return np.array(output_pulses, dtype=int), np.array(output_relays, dtype=int)


def generate_segment_start_rotation_targets(rotations, num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT):
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
        ik_result = Inverse_Kinematics(point, target_rotations[point_idx], current_angles=current_angles, angle_tolerance=angle_tolerance, debug=debug)
        if ik_result["status"] not in {"OK", "NO_MATCH"} or ik_result.get("solution") is None:
            ik_result = Inverse_Kinematics(
                point,
                target_rotations[point_idx],
                current_angles=None,
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

        solution_angles = np.array(ik_result["solution"], dtype=float)
        if abs(solution_angles[4]) < 1.0:
            preserved_solution = solution_angles.copy()
            preserved_solution[3] = current_angles[3]
            preserved_solution[5] = normalize_angle(solution_angles[3] + solution_angles[5] - preserved_solution[3])
            if are_joint_angles_within_limits(preserved_solution) and Check_T_0_6(point, target_rotations[point_idx], *preserved_solution):
                solution_angles = preserved_solution

        current_angles = solution_angles
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
    raw_results = forward_kinematics_from_encoder_file_all(file_path, ppr=ppr, gearbox_ratios=gearbox_ratios)
    if len(raw_results) < 2:
        raise ValueError("Line mapping requires at least 2 pulse sets.")

    raw_pulses = np.array([item["pulses"] for item in raw_results], dtype=float)
    raw_relay_states = None
    if any(item.get("relay") is not None for item in raw_results):
        raw_relay_states = np.array([0 if item.get("relay") is None else item["relay"] for item in raw_results], dtype=int)
    raw_angles = np.array([item["angles"] for item in raw_results], dtype=float)
    raw_angles_ik = convert_display_angles_to_ik_angles(raw_angles)
    raw_positions = np.array([item["position"] for item in raw_results], dtype=float)
    raw_rotations = np.array([item["rotation"] for item in raw_results], dtype=float)
    initial_angles = raw_angles_ik[0].copy()
    target_positions = raw_positions.copy()
    locked_z_value = None

    if LINE_MAPPING_LOCK_Z_TO_FIRST:
        locked_z_value = float(target_positions[0, 2])
        target_positions[:, 2] = locked_z_value

    mapped_positions, mapped_rotations, segment_lengths, waypoint_indices = generate_polyline_position_rotation_targets(
        target_positions,
        raw_rotations,
        num_samples_per_segment=num_samples_per_segment,
    )
    if LINE_MAPPING_LOCK_Z_TO_FIRST:
        fixed_waypoint_angles = {0: raw_angles_ik[0].copy()}
    else:
        fixed_waypoint_angles = {int(polyline_idx): raw_angles_ik[waypoint_idx].copy() for waypoint_idx, polyline_idx in enumerate(waypoint_indices)}

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
        fallback_rotations, fallback_waypoint_indices = generate_segment_start_rotation_targets(
            raw_rotations,
            num_samples_per_segment=num_samples_per_segment,
        )
        if not np.array_equal(fallback_waypoint_indices, waypoint_indices):
            raise ValueError("Internal error: fallback waypoint indices do not match the polyline layout.") from exc
        try:
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
        except MappingIKError:
            if not LINE_MAPPING_LOCK_Z_TO_FIRST:
                raise
            fixed_start_rotations = np.repeat(raw_rotations[0][None, :, :], len(mapped_positions), axis=0)
            mapped_angles_ik = cartesian_polyline_to_joint_trajectory(
                mapped_positions,
                fixed_start_rotations,
                initial_angles,
                fixed_waypoint_angles=fixed_waypoint_angles,
                angle_tolerance=angle_tolerance,
                debug=debug,
            )
            mapped_rotations = fixed_start_rotations
            rotation_strategy = "fixed_start_fallback"

    mapped_angles = convert_ik_angles_to_display_angles(mapped_angles_ik)
    mapped_pulses = np.array([angles_to_encoder_pulses(angles, ppr=ppr, gearbox_ratios=gearbox_ratios) for angles in mapped_angles], dtype=int)
    if LINE_MAPPING_LOCK_Z_TO_FIRST:
        mapped_pulses[0] = np.rint(raw_pulses[0]).astype(int)
    else:
        for waypoint_idx, polyline_idx in enumerate(waypoint_indices):
            mapped_pulses[polyline_idx] = np.rint(raw_pulses[waypoint_idx]).astype(int)
    mapped_pulses = np.rint(mapped_pulses).astype(int)
    mapped_angles_from_pulses = np.array(
        [encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios) for pulses in mapped_pulses],
        dtype=float,
    )
    mapped_actual_positions = np.array([forward_kinematics_from_joint_angles(angles)["position"] for angles in mapped_angles_from_pulses], dtype=float)

    return {
        "raw_pulses": raw_pulses,
        "raw_relay_states": raw_relay_states,
        "raw_angles": raw_angles,
        "raw_positions": raw_positions,
        "raw_rotations": raw_rotations,
        "target_positions": target_positions,
        "locked_z_value": locked_z_value,
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
    if mapping["raw_relay_states"] is None:
        export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)
    else:
        relay_mapped_pulses, mapped_relay_states = build_relay_mapped_pulses(
            mapping["mapped_pulses"],
            mapping["raw_relay_states"],
            mapping["waypoint_indices"],
        )
        mapping["relay_mapped_pulses"] = relay_mapped_pulses
        mapping["mapped_relay_states"] = mapped_relay_states
        export_mapped_pulses_to_txt(relay_mapped_pulses, mapped_pulses_output_path, mapped_relay_states)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(LINE_MAPPING_OUTPUT_DIR, f"{base_name}_line_angles{ext}")
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(LINE_MAPPING_OUTPUT_DIR, f"{base_name}_line_xyz{ext}")
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path, decimal_places=0)

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
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(XYZ_OUTPUT_DIR, f"{base_name}_xyz_actual{ext}")
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping


def generate_circle_xy_points(center_x, center_y, z_height, radius, num_samples):
    center_x = float(center_x)
    center_y = float(center_y)
    z_height = float(z_height)
    radius = float(radius)
    num_samples = int(num_samples)
    if radius <= 0.0:
        raise ValueError("Circle radius must be greater than 0.")
    if num_samples < 8:
        raise ValueError("Circle samples must be at least 8.")

    angles = np.linspace(0.0, 2.0 * np.pi, num_samples, endpoint=False)
    circle_points = np.column_stack(
        [
            center_x + radius * np.cos(angles),
            center_y + radius * np.sin(angles),
            np.full(num_samples, z_height, dtype=float),
        ]
    )
    return np.vstack([circle_points, circle_points[0]])


def build_circle_mapping_xy(
    center_x,
    center_y,
    z_height,
    radius,
    num_samples,
    reference_angles,
    fixed_rotation=None,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    angle_tolerance=180.0,
    debug=False,
):
    circle_points = generate_circle_xy_points(center_x, center_y, z_height, radius, num_samples)
    mapping = build_line_mapping_from_xyz_points(
        circle_points,
        reference_angles=reference_angles,
        fixed_rotation=fixed_rotation,
        ppr=ppr,
        gearbox_ratios=gearbox_ratios,
        num_samples_per_segment=1,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
    mapping["circle_center"] = np.array([float(center_x), float(center_y), float(z_height)], dtype=float)
    mapping["circle_radius"] = float(radius)
    mapping["circle_samples"] = int(num_samples)
    return mapping


def _format_circle_value_for_name(value):
    formatted = f"{float(value):.3f}".rstrip("0").rstrip(".")
    return formatted.replace("-", "m").replace(".", "p")


def export_circle_mapping_xy(
    center_x,
    center_y,
    z_height,
    radius,
    num_samples,
    reference_angles,
    fixed_rotation=None,
    mapped_pulses_output_path=None,
    mapped_angles_output_path=None,
    mapped_xyz_output_path=None,
    ppr=ENCODER_PPR,
    gearbox_ratios=None,
    angle_tolerance=180.0,
    debug=False,
):
    mapping = build_circle_mapping_xy(
        center_x,
        center_y,
        z_height,
        radius,
        num_samples,
        reference_angles=reference_angles,
        fixed_rotation=fixed_rotation,
        ppr=ppr,
        gearbox_ratios=gearbox_ratios,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )

    ensure_output_dir(CIRCLE_OUTPUT_DIR)
    base_name = (
        "circle_xy"
        f"_cx_{_format_circle_value_for_name(center_x)}"
        f"_cy_{_format_circle_value_for_name(center_y)}"
        f"_z_{_format_circle_value_for_name(z_height)}"
        f"_r_{_format_circle_value_for_name(radius)}"
    )

    if mapped_pulses_output_path is None:
        mapped_pulses_output_path = os.path.join(CIRCLE_OUTPUT_DIR, f"{base_name}_circle_pulses.txt")
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(CIRCLE_OUTPUT_DIR, f"{base_name}_circle_angles.txt")
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(CIRCLE_OUTPUT_DIR, f"{base_name}_circle_actual.txt")
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping
