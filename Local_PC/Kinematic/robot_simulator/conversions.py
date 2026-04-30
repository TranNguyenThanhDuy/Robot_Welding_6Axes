import os

import numpy as np

from .constants import (
    ENCODER_PPR,
    GEAR_RATIO_J1,
    GEAR_RATIO_J2,
    GEAR_RATIO_J3,
    GEAR_RATIO_J4,
    GEAR_RATIO_J5,
    GEAR_RATIO_J6,
    JOINT_LIMITS_DEG,
    THETA2_OFFSET,
)


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_gearbox_ratios():
    return np.array(
        [
            GEAR_RATIO_J1,
            GEAR_RATIO_J2,
            GEAR_RATIO_J3,
            GEAR_RATIO_J4,
            GEAR_RATIO_J5,
            GEAR_RATIO_J6,
        ],
        dtype=float,
    )


def get_joint_angle_limits():
    return JOINT_LIMITS_DEG.copy()


def clamp_joint_angles(angles):
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")
    limits = get_joint_angle_limits()
    return np.clip(angle_array, limits[:, 0], limits[:, 1])


def are_joint_angles_within_limits(angles, atol=1e-9):
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")
    limits = get_joint_angle_limits()
    return bool(np.all(angle_array >= limits[:, 0] - atol) and np.all(angle_array <= limits[:, 1] + atol))


def ensure_output_dir(directory):
    os.makedirs(directory, exist_ok=True)
    return directory


def encoder_pulse_to_angle(pos_servo, ppr=ENCODER_PPR, gearbox_ratio=1.0):
    if ppr == 0:
        raise ValueError("ENCODER_PPR must be non-zero.")
    if gearbox_ratio == 0:
        raise ValueError("gearbox_ratio must be non-zero.")
    return float(pos_servo) * 360.0 / (float(ppr) * float(gearbox_ratio))


def encoder_pulses_to_angles(pulses, ppr=ENCODER_PPR, gearbox_ratios=None):
    pulse_array = np.array(pulses, dtype=float).flatten()
    if pulse_array.size != 6:
        raise ValueError("Expected exactly 6 encoder pulse values.")
    if gearbox_ratios is None:
        gearbox_ratios = get_gearbox_ratios()
    else:
        gearbox_ratios = np.array(gearbox_ratios, dtype=float).flatten()
    if gearbox_ratios.size != 6:
        raise ValueError("Expected exactly 6 gearbox ratios.")
    angles = [
        encoder_pulse_to_angle(pulse, ppr=ppr, gearbox_ratio=ratio)
        for pulse, ratio in zip(pulse_array, gearbox_ratios)
    ]
    return np.array(angles, dtype=float)


def convert_display_angles_to_ik_angles(angles):
    angle_array = np.array(angles, dtype=float)
    if angle_array.shape[-1] != 6:
        raise ValueError("Expected joint angles with trailing dimension size 6.")
    converted = angle_array.copy()
    converted[..., 1] = converted[..., 1] + THETA2_OFFSET
    return converted


def convert_ik_angles_to_display_angles(angles):
    angle_array = np.array(angles, dtype=float)
    if angle_array.shape[-1] != 6:
        raise ValueError("Expected joint angles with trailing dimension size 6.")
    converted = angle_array.copy()
    converted[..., 1] = converted[..., 1] - THETA2_OFFSET
    return converted


slider_angles_to_ik_angles = convert_display_angles_to_ik_angles
ik_angles_to_slider_angles = convert_ik_angles_to_display_angles


def angle_to_encoder_pulse(theta, ppr=ENCODER_PPR, gearbox_ratio=1.0):
    if ppr == 0:
        raise ValueError("ENCODER_PPR must be non-zero.")
    if gearbox_ratio == 0:
        raise ValueError("gearbox_ratio must be non-zero.")
    return int(round(float(theta) * float(ppr) * float(gearbox_ratio) / 360.0))


def angles_to_encoder_pulses(angles, ppr=ENCODER_PPR, gearbox_ratios=None):
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")
    if gearbox_ratios is None:
        gearbox_ratios = get_gearbox_ratios()
    else:
        gearbox_ratios = np.array(gearbox_ratios, dtype=float).flatten()
    if gearbox_ratios.size != 6:
        raise ValueError("Expected exactly 6 gearbox ratios.")
    pulses = [
        angle_to_encoder_pulse(angle, ppr=ppr, gearbox_ratio=ratio)
        for angle, ratio in zip(angle_array, gearbox_ratios)
    ]
    return np.array(pulses, dtype=int)
