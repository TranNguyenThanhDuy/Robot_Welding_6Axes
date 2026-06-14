import os
import re

import numpy as np

from .constants import ENCODER_PPR, TEACH_OUTPUT_DIR, THETA2_OFFSET
from .conversions import encoder_pulses_to_angles, ensure_output_dir
from .kinematics import Forward_Kinematics


def load_encoder_pulses_from_txt(file_path):
    pulse_sets = load_encoder_pulse_sets_from_txt(file_path)
    return pulse_sets[0].copy()


def load_encoder_pulse_sets_with_relay_from_txt(file_path):
    pulse_sets = []
    relay_states = []
    has_relay = False

    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
            if len(numbers) not in (6, 7):
                raise ValueError(
                    f"Line {line_number} must contain 6 pulse values"
                    " plus an optional relay state."
                )
            pulse_sets.append([float(value) for value in numbers[:6]])
            if len(numbers) == 7:
                has_relay = True
                relay_states.append(1 if float(numbers[6]) != 0.0 else 0)
            else:
                relay_states.append(0)

    if not pulse_sets:
        raise ValueError("The txt file does not contain any valid pulse sets.")

    pulses = np.array(pulse_sets, dtype=float)
    relays = np.array(relay_states, dtype=int) if has_relay else None
    return pulses, relays


def load_encoder_pulse_sets_from_txt(file_path):
    pulse_sets, _ = load_encoder_pulse_sets_with_relay_from_txt(file_path)
    return pulse_sets


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
    pulse_sets, relay_states = load_encoder_pulse_sets_with_relay_from_txt(file_path)
    results = []
    for idx, pulses in enumerate(pulse_sets):
        angles = encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
        T06 = Forward_Kinematics(angles[0], angles[1] + THETA2_OFFSET, angles[2], angles[3], angles[4], angles[5])
        results.append(
            {
                "pulses": pulses.copy(),
                "angles": angles,
                "transform": T06,
                "position": T06[:3, 3].copy(),
                "rotation": T06[:3, :3].copy(),
                "relay": None if relay_states is None else int(relay_states[idx]),
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


def export_mapped_pulses_to_txt(mapped_pulses, output_file_path, relay_states=None):
    mapped_pulses = np.rint(np.array(mapped_pulses, dtype=float)).astype(int)
    if mapped_pulses.ndim != 2 or mapped_pulses.shape[1] != 6:
        raise ValueError("Expected mapped pulses with shape (N, 6).")
    if relay_states is not None:
        relay_states = np.array(relay_states, dtype=int).flatten()
        if relay_states.size != mapped_pulses.shape[0]:
            raise ValueError("Expected one relay state per mapped pulse row.")
    with open(output_file_path, "w", encoding="utf-8") as file:
        for idx, pulse_set in enumerate(mapped_pulses):
            values = [str(int(value)) for value in pulse_set]
            if relay_states is not None:
                values.append(str(1 if int(relay_states[idx]) != 0 else 0))
            file.write(" ".join(values) + "\n")
    return output_file_path


def export_xyz_positions_to_txt(xyz_positions, output_file_path, decimal_places=3):
    xyz_positions = np.array(xyz_positions, dtype=float)
    if xyz_positions.ndim != 2 or xyz_positions.shape[1] != 3:
        raise ValueError("Expected XYZ positions with shape (N, 3).")
    if isinstance(decimal_places, (list, tuple, np.ndarray)):
        decimal_places_per_axis = [int(value) for value in decimal_places]
        if len(decimal_places_per_axis) != 3:
            raise ValueError("decimal_places must contain exactly 3 values when provided per axis.")
    else:
        decimal_places_per_axis = [int(decimal_places)] * 3
    with open(output_file_path, "w", encoding="utf-8") as file:
        for xyz in xyz_positions:
            formatted_values = []
            for value, axis_decimal_places in zip(xyz, decimal_places_per_axis):
                if axis_decimal_places <= 0:
                    formatted_values.append(str(int(round(value))))
                else:
                    formatted_values.append(f"{value:.{axis_decimal_places}f}")
            file.write(" ".join(formatted_values) + "\n")
    return output_file_path
