import os
import re

import numpy as np

from .constants import ENCODER_PPR, TEACH_OUTPUT_DIR, THETA2_OFFSET
from .conversions import encoder_pulses_to_angles, ensure_output_dir
from .kinematics import Forward_Kinematics


def load_encoder_pulses_from_txt(file_path):
    pulse_sets = load_encoder_pulse_sets_from_txt(file_path)
    return pulse_sets[0].copy()


def load_encoder_pulse_sets_from_txt(file_path):
    pulse_sets = []
    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
            if len(numbers) != 6:
                raise ValueError(f"Line {line_number} must contain exactly 6 numeric pulse values.")
            pulse_sets.append([float(value) for value in numbers])
    if not pulse_sets:
        raise ValueError("The txt file does not contain any valid pulse sets.")
    return np.array(pulse_sets, dtype=float)


def load_xyz_waypoints_from_txt(file_path):
    xyz_points = []
    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
            if len(numbers) != 3:
                raise ValueError(f"Line {line_number} must contain exactly 3 numeric XYZ values.")
            xyz_points.append([float(value) for value in numbers])
    if not xyz_points:
        raise ValueError("The txt file does not contain any valid XYZ waypoints.")
    return np.array(xyz_points, dtype=float)


def forward_kinematics_from_encoder_file(file_path, ppr=ENCODER_PPR, gearbox_ratios=None):
    pulses = load_encoder_pulses_from_txt(file_path)
    angles = encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
    T06 = Forward_Kinematics(angles[0], angles[1] + THETA2_OFFSET, angles[2], angles[3], angles[4], angles[5])
    return {
        "pulses": pulses,
        "angles": angles,
        "transform": T06,
        "position": T06[:3, 3].copy(),
        "rotation": T06[:3, :3].copy(),
    }


def forward_kinematics_from_encoder_file_all(file_path, ppr=ENCODER_PPR, gearbox_ratios=None):
    pulse_sets = load_encoder_pulse_sets_from_txt(file_path)
    results = []
    for pulses in pulse_sets:
        angles = encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
        T06 = Forward_Kinematics(angles[0], angles[1] + THETA2_OFFSET, angles[2], angles[3], angles[4], angles[5])
        results.append(
            {
                "pulses": pulses.copy(),
                "angles": angles,
                "transform": T06,
                "position": T06[:3, 3].copy(),
                "rotation": T06[:3, :3].copy(),
            }
        )
    return results


def export_angle_sets_to_txt(source_file_path, output_file_path=None, ppr=ENCODER_PPR, gearbox_ratios=None):
    pulse_sets = load_encoder_pulse_sets_from_txt(source_file_path)
    angle_sets = [encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios) for pulses in pulse_sets]

    if output_file_path is None:
        ensure_output_dir(TEACH_OUTPUT_DIR)
        base_name, ext = os.path.splitext(source_file_path)
        base_name = os.path.basename(base_name)
        if not ext:
            ext = ".txt"
        output_file_path = os.path.join(TEACH_OUTPUT_DIR, f"{base_name}_angles{ext}")

    with open(output_file_path, "w", encoding="utf-8") as file:
        for angles in angle_sets:
            file.write(" ".join(f"{angle:.6f}" for angle in angles) + "\n")

    return output_file_path, np.array(angle_sets, dtype=float)


def export_mapped_pulses_to_txt(mapped_pulses, output_file_path):
    mapped_pulses = np.rint(np.array(mapped_pulses, dtype=float)).astype(int)
    if mapped_pulses.ndim != 2 or mapped_pulses.shape[1] != 6:
        raise ValueError("Expected mapped pulses with shape (N, 6).")
    with open(output_file_path, "w", encoding="utf-8") as file:
        for pulse_set in mapped_pulses:
            file.write(" ".join(str(int(value)) for value in pulse_set) + "\n")
    return output_file_path


def export_xyz_positions_to_txt(xyz_positions, output_file_path):
    xyz_positions = np.array(xyz_positions, dtype=float)
    if xyz_positions.ndim != 2 or xyz_positions.shape[1] != 3:
        raise ValueError("Expected XYZ positions with shape (N, 3).")
    with open(output_file_path, "w", encoding="utf-8") as file:
        for xyz in xyz_positions:
            file.write(" ".join(f"{value:.3f}" for value in xyz) + "\n")
    return output_file_path
