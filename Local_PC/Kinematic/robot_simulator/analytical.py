import numpy as np
import matplotlib.pyplot as plt
import os
import re
from itertools import product
from matplotlib.widgets import Slider, Button, TextBox

# Robot DH parameters (mm)
d0 = 230
L1 = 15
L2 = 260
L3 = 66
d1 = 185
d2 = 330
# d0 = 230
# L1 = 50
# L2 = 259
# L3 = 66
# d1 = 255
# d2 = 175
# Joint angle limits (degrees)
POS = 90
NEG = -90

# Joint angle limits per axis (degrees)
JOINT_LIMITS_DEG = np.array([
    [-180.0, 180.0],  # J1
    [-135.0, 135.0],  # J2
    [-135.0, 135.0],  # J3
    [-180.0, 180.0],  # J4
    [-90.0,   90.0],  # J5
    [-180.0, 180.0],  # J6
], dtype=float)

# IK branch selection weights. Higher values make a joint prefer continuity.
# J4/J6 are weighted higher to reduce sudden wrist roll during welding paths.
IK_CONTINUITY_WEIGHTS = np.array([1.0, 1.0, 1.0, 4.0, 2.0, 6.0], dtype=float)

# Joint angles (degrees) - Current values 
theta1 = 0.0
theta2 = 0.0
theta3 = 0.0
theta4 = 0.0
theta5 = 0.0
theta6 = 0.0

# Joint angles (degrees) - Forward Kinematics values 
theta1_FK = 0.0
theta2_FK = 0.0
theta3_FK = 0.0
theta4_FK = 0.0
theta5_FK = 0.0
theta6_FK = 0.0

THETA2_OFFSET =90  # offset for joint-2 (deg)

# Encoder conversion parameters
ENCODER_PPR = 10000
GEAR_RATIO_J1 = 40
GEAR_RATIO_J2 = 81
GEAR_RATIO_J3 = 5
GEAR_RATIO_J4 = 10
GEAR_RATIO_J5 = 50
GEAR_RATIO_J6 = 1.0

# GEAR_RATIO_J1 = 1
# GEAR_RATIO_J2 = 1
# GEAR_RATIO_J3 = 1
# GEAR_RATIO_J4 = 1
# GEAR_RATIO_J5 = 1
# GEAR_RATIO_J6 = 1

# Line mapping parameters
LINE_MAPPING_SAMPLES_PER_SEGMENT = 10
CIRCLE_DEFAULT_SAMPLES = 36
CIRCLE_RADIUS = 60.0

# Workspace cloud sampling parameters
WORKSPACE_SAMPLES_PER_JOINT = 20
WORKSPACE_POINT_SIZE = 10
WORKSPACE_SAMPLED_JOINTS = (0, 1, 2)
WORKSPACE_FIXED_ANGLES = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

# Output folders
TEACH_OUTPUT_DIR = "teach_outputs"
LINE_MAPPING_OUTPUT_DIR = "line_mapping_outputs"
XYZ_OUTPUT_DIR = "xyz_outputs"
CIRCLE_OUTPUT_DIR = "circle_outputs"

def T_0_1(a1, alpha1, d1_, theta1_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta1_)), 0, np.sin(np.radians(theta1_)), a1 * np.cos(np.radians(theta1_))],
        [np.sin(np.radians(theta1_)), 0, -np.cos(np.radians(theta1_)), a1 * np.sin(np.radians(theta1_))],
        [0, 1, 0, d1_],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def T_1_2(a2, alpha2, d2_, theta2_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta2_)), -np.sin(np.radians(theta2_)), 0, a2 * np.cos(np.radians(theta2_))],
        [np.sin(np.radians(theta2_)), np.cos(np.radians(theta2_)), 0, a2 * np.sin(np.radians(theta2_))],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def T_2_3(a3, alpha3, d3_, theta3_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta3_)), 0, np.sin(np.radians(theta3_)), a3 * np.cos(np.radians(theta3_))],
        [np.sin(np.radians(theta3_)), 0, -np.cos(np.radians(theta3_)), a3 * np.sin(np.radians(theta3_))],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def T_3_4(a4, alpha4, d4_, theta4_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta4_)), 0, -np.sin(np.radians(theta4_)), 0],
        [np.sin(np.radians(theta4_)), 0, np.cos(np.radians(theta4_)), 0],
        [0, -1, 0, d4_],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def T_4_5(a5, alpha5, d5_, theta5_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta5_)), 0, np.sin(np.radians(theta5_)), 0],
        [np.sin(np.radians(theta5_)), 0, -np.cos(np.radians(theta5_)), 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def T_5_6(a6, alpha6, d6_, theta6_):
    transformationMatrix = np.array([
        [np.cos(np.radians(theta6_)), -np.sin(np.radians(theta6_)), 0, 0],
        [np.sin(np.radians(theta6_)), np.cos(np.radians(theta6_)), 0, 0],
        [0, 0, 1, d6_],
        [0, 0, 0, 1]
    ])
    return transformationMatrix

def normalize_angle(angle):
    """Chuan hoa goc ve khoang [-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def get_gearbox_ratios():
    """Tra ve gear ratio cua 6 khop J1..J6."""
    return np.array([
        GEAR_RATIO_J1,
        GEAR_RATIO_J2,
        GEAR_RATIO_J3,
        GEAR_RATIO_J4,
        GEAR_RATIO_J5,
        GEAR_RATIO_J6,
    ], dtype=float)

def get_joint_angle_limits():
    """Tra ve gioi han goc [min, max] cua 6 khop J1..J6."""
    return JOINT_LIMITS_DEG.copy()

def clamp_joint_angles(angles):
    """Ep bo goc 6 khop nam trong gioi han tung truc."""
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")
    limits = get_joint_angle_limits()
    return np.clip(angle_array, limits[:, 0], limits[:, 1])

def are_joint_angles_within_limits(angles, atol=1e-9):
    """Kiem tra bo goc 6 khop co nam trong gioi han tung truc hay khong."""
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")
    limits = get_joint_angle_limits()
    return bool(np.all(angle_array >= limits[:, 0] - atol) and np.all(angle_array <= limits[:, 1] + atol))

def ensure_output_dir(directory):
    """Tao thu muc output neu chua ton tai va tra ve duong dan."""
    os.makedirs(directory, exist_ok=True)
    return directory

class MappingIKError(ValueError):
    """Loi IK co kem thong tin ngu canh de hien thi ro hon tren UI."""

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
            pieces.append(
                f"XYZ=({self.point[0]:.3f}, {self.point[1]:.3f}, {self.point[2]:.3f})"
            )
        return ' | '.join(pieces)

def encoder_pulse_to_angle(pos_servo, ppr=ENCODER_PPR, gearbox_ratio=1.0):
    """Doi 1 gia tri xung encoder thanh goc khop (do)."""
    if ppr == 0:
        raise ValueError("ENCODER_PPR must be non-zero.")
    if gearbox_ratio == 0:
        raise ValueError("gearbox_ratio must be non-zero.")
    return float(pos_servo) * 360.0 / (float(ppr) * float(gearbox_ratio))

def encoder_pulses_to_angles(pulses, ppr=ENCODER_PPR, gearbox_ratios=None):
    """Doi 6 gia tri xung encoder thanh 6 goc khop (do)."""
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

# use for stimulate
def convert_display_angles_to_ik_angles(angles):
    """Doi goc o he hien thi/xung sang he goc noi bo cua IK/FK."""
    angle_array = np.array(angles, dtype=float)
    if angle_array.shape[-1] != 6:
        raise ValueError("Expected joint angles with trailing dimension size 6.")
    converted = angle_array.copy()
    # IK/FK uses theta2 with offset; slider/pulse angles do not include it.
    converted[..., 1] = converted[..., 1] + THETA2_OFFSET
    return converted

def convert_ik_angles_to_display_angles(angles):
    """Doi goc o he noi bo IK/FK ve he goc hien thi/xung."""
    angle_array = np.array(angles, dtype=float)
    if angle_array.shape[-1] != 6:
        raise ValueError("Expected joint angles with trailing dimension size 6.")
    converted = angle_array.copy()
    # Convert internal IK/FK angles back to display/pulse angles.
    converted[..., 1] = converted[..., 1] - THETA2_OFFSET
    return converted

# Alias ten cu de tranh gay code neu con cho nao dang goi ten helper truoc do.
slider_angles_to_ik_angles = convert_display_angles_to_ik_angles
ik_angles_to_slider_angles = convert_ik_angles_to_display_angles

def angle_to_encoder_pulse(theta, ppr=ENCODER_PPR, gearbox_ratio=1.0):
    """Doi 1 goc khop (do) thanh 1 gia tri xung encoder."""
    if ppr == 0:
        raise ValueError("ENCODER_PPR must be non-zero.")
    if gearbox_ratio == 0:
        raise ValueError("gearbox_ratio must be non-zero.")
    return int(round(float(theta) * float(ppr) * float(gearbox_ratio) / 360.0))

def angles_to_encoder_pulses(angles, ppr=ENCODER_PPR, gearbox_ratios=None):
    """Doi 6 goc khop (do) thanh 6 gia tri xung encoder."""
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

# =====================================================================
# NHOM HAM DUNG CHUNG
# - Doc file xung
# - Doi xung <-> goc
# - Tinh FK tu file xung / tu bo goc
# =====================================================================

def load_encoder_pulses_from_txt(file_path):
    """Doc bo 6 xung encoder dau tien tu file txt."""
    pulse_sets = load_encoder_pulse_sets_from_txt(file_path)
    return pulse_sets[0].copy()

def load_encoder_pulse_sets_from_txt(file_path):
    """
    Doc cac bo xung encoder tu file txt.
    - Moi dong la 1 bo xung ung voi J1..J6
    """
    pulse_sets = []

    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line:
                continue

            numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
            if len(numbers) != 6:
                raise ValueError(
                    f"Line {line_number} must contain exactly 6 numeric pulse values."
                )

            pulse_sets.append([float(value) for value in numbers])

    if not pulse_sets:
        raise ValueError("The txt file does not contain any valid pulse sets.")

    return np.array(pulse_sets, dtype=float)

def load_xyz_waypoints_from_txt(file_path):
    """
    Doc cac waypoint XYZ tu file txt.

    Dinh dang mong muon:
    - Moi dong khong rong co dung 3 gia tri so
    - Moi dong la 1 waypoint Cartesian: X Y Z
    """
    xyz_points = []

    with open(file_path, "r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()
            if not line:
                continue

            numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
            if len(numbers) != 3:
                raise ValueError(
                    f"Line {line_number} must contain exactly 3 numeric XYZ values."
                )

            xyz_points.append([float(value) for value in numbers])

    if not xyz_points:
        raise ValueError("The txt file does not contain any valid XYZ waypoints.")

    return np.array(xyz_points, dtype=float)

def forward_kinematics_from_encoder_file(file_path, ppr=ENCODER_PPR, gearbox_ratios=None):
    """
    Doc 6 xung encoder tu file txt, doi sang goc khop,
    roi tinh dong hoc thuan tu bo goc vua doi.
    """
    pulses = load_encoder_pulses_from_txt(file_path)
    angles = encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
    T06 = Forward_Kinematics(
        angles[0],
        angles[1] + THETA2_OFFSET,
        angles[2],
        angles[3],
        angles[4],
        angles[5],
    )
    return {
        "pulses": pulses,
        "angles": angles,
        "transform": T06,
        "position": T06[:3, 3].copy(),
        "rotation": T06[:3, :3].copy(),
    }

def forward_kinematics_from_encoder_file_all(file_path, ppr=ENCODER_PPR, gearbox_ratios=None):
    """Tinh FK cho tat ca bo xung tim thay trong file txt."""
    pulse_sets = load_encoder_pulse_sets_from_txt(file_path)
    results = []

    for pulses in pulse_sets:
        angles = encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
        T06 = Forward_Kinematics(
            angles[0],
            angles[1] + THETA2_OFFSET,
            angles[2],
            angles[3],
            angles[4],
            angles[5],
        )
        results.append({
            "pulses": pulses.copy(),
            "angles": angles,
            "transform": T06,
            "position": T06[:3, 3].copy(),
            "rotation": T06[:3, :3].copy(),
        })

    return results

def forward_kinematics_from_joint_angles(angles):
    """Tinh pose FK tu 1 bo 6 goc khop."""
    angle_array = np.array(angles, dtype=float).flatten()
    if angle_array.size != 6:
        raise ValueError("Expected exactly 6 joint angle values.")

    T06 = Forward_Kinematics(
        angle_array[0],
        angle_array[1] + THETA2_OFFSET,
        angle_array[2],
        angle_array[3],
        angle_array[4],
        angle_array[5],
    )
    return {
        "angles": angle_array.copy(),
        "transform": T06,
        "position": T06[:3, 3].copy(),
        "rotation": T06[:3, :3].copy(),
    }

# =====================================================================
# CHUC NANG TEACH PLAYBACK
# - Dung file xung goc
# - Doi sang goc
# - Export file goc de kiem tra / mo phong
# =====================================================================

def export_angle_sets_to_txt(source_file_path, output_file_path=None, ppr=ENCODER_PPR, gearbox_ratios=None):
    """
    Doi tat ca bo xung encoder trong file txt thanh bo goc
    va ghi ra mot file txt khac.
    Moi dong output chua 6 goc khop.
    """
    pulse_sets = load_encoder_pulse_sets_from_txt(source_file_path)
    angle_sets = [
        encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
        for pulses in pulse_sets
    ]

    if output_file_path is None:
        ensure_output_dir(TEACH_OUTPUT_DIR)
        base_name, ext = os.path.splitext(source_file_path)
        base_name = os.path.basename(base_name)
        if not ext:
            ext = ".txt"
        output_file_path = os.path.join(TEACH_OUTPUT_DIR, f"{base_name}_angles{ext}")

    with open(output_file_path, "w", encoding="utf-8") as file:
        for angles in angle_sets:
            line = " ".join(f"{angle:.6f}" for angle in angles)
            file.write(line + "\n")

    return output_file_path, np.array(angle_sets, dtype=float)

# =====================================================================
# CHUC NANG LINE MAP
# - Lay cac waypoint tu file xung
# - Noi thang tung cap waypoint lien tiep
# - Noi suy orientation theo tung doan
# - Dung IK cho cac diem trung gian
# - Export xung moi / goc moi
# =====================================================================

def build_piecewise_segments_from_cartesian_points(points):
    """Tao cac doan Cartesian lien tiep tu danh sach waypoint."""
    point_array = np.array(points, dtype=float)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError("Expected Cartesian points with shape (N, 3).")
    if len(point_array) < 2:
        raise ValueError("At least 2 Cartesian waypoints are required.")

    return [
        (point_array[idx].copy(), point_array[idx + 1].copy())
        for idx in range(len(point_array) - 1)
    ]

def rotation_matrix_to_quaternion(rotation_matrix):
    """Doi ma tran quay 3x3 thanh quaternion chuan hoa [w, x, y, z]."""
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
    """Doi quaternion chuan hoa [w, x, y, z] thanh ma tran quay 3x3."""
    quaternion = np.array(quaternion, dtype=float).flatten()
    if quaternion.size != 4:
        raise ValueError("Expected quaternion with 4 values.")
    quaternion = quaternion / np.linalg.norm(quaternion)
    w, x, y, z = quaternion

    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)

def slerp_rotation_matrices(start_rotation, end_rotation, alpha):
    """Noi suy cau giua hai ma tran quay."""
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
    """
    Generate points on one straight Cartesian segment.
    The start point is excluded; the end point is included.
    """
    if num_samples < 1:
        raise ValueError("num_samples must be at least 1.")

    start_point = np.array(start_point, dtype=float).flatten()
    end_point = np.array(end_point, dtype=float).flatten()
    if start_point.size != 3 or end_point.size != 3:
        raise ValueError("Expected 3D Cartesian points.")

    samples = []
    for alpha in np.linspace(0.0, 1.0, num_samples + 1)[1:]:
        # Linear interpolation: every generated point lies on the start -> end segment.
        point = (1.0 - alpha) * start_point + alpha * end_point
        samples.append(point)
    return np.array(samples, dtype=float)

def generate_polyline_position_samples(points, num_samples_per_segment=LINE_MAPPING_SAMPLES_PER_SEGMENT):
    """Sinh polyline Cartesian day du tu cac waypoint co dinh."""
    segments = build_piecewise_segments_from_cartesian_points(points)
    polyline = [np.array(points[0], dtype=float).copy()]
    segment_lengths = []
    waypoint_indices = [0]

    for start_point, end_point in segments:
        segment_lengths.append(float(np.linalg.norm(end_point - start_point)))
        # Join all straight segments into one waypoint polyline.
        polyline.extend(interpolate_straight_segment_points(start_point, end_point, num_samples=num_samples_per_segment))
        waypoint_indices.append(len(polyline) - 1)

    return (
        np.array(polyline, dtype=float),
        np.array(segment_lengths, dtype=float),
        np.array(waypoint_indices, dtype=int),
    )

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
            # Position follows the straight segment between two waypoints.
            polyline_points.append((1.0 - alpha) * start_point + alpha * end_point)
            # Orientation changes smoothly per segment with SLERP so IK can track it better.
            polyline_rotations.append(slerp_rotation_matrices(start_rotation, end_rotation, alpha))

        # Store original waypoint indices so IK can keep them fixed later.
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
            # If the continuity branch falls into a bad wrist state, retry this
            # target without current-angle bias before declaring IK failure.
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
            preserved_solution[5] = normalize_angle(
                solution_angles[3] + solution_angles[5] - preserved_solution[3]
            )
            if (
                are_joint_angles_within_limits(preserved_solution)
                and Check_T_0_6(point, target_rotations[point_idx], *preserved_solution)
            ):
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

    # Build a polyline from the original waypoints:
    # - Cartesian position samples on each straight segment
    # - matching rotation target for each point
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
    # Convert generated angles back to pulses for the real robot.
    mapped_pulses = np.array(
        [
            angles_to_encoder_pulses(angles, ppr=ppr, gearbox_ratios=gearbox_ratios)
            for angles in mapped_angles
        ],
        dtype=int,
    )
    for waypoint_idx, polyline_idx in enumerate(waypoint_indices):
        # Overwrite fixed waypoint pulses with original input pulses to prevent drift.
        mapped_pulses[polyline_idx] = np.rint(raw_pulses[waypoint_idx]).astype(int)
    mapped_pulses = np.rint(mapped_pulses).astype(int)
    mapped_angles_from_pulses = np.array(
        [
            encoder_pulses_to_angles(pulses, ppr=ppr, gearbox_ratios=gearbox_ratios)
            for pulses in mapped_pulses
        ],
        dtype=float,
    )
    # Run FK again from output pulses so preview matches what the robot will execute.
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

def export_mapped_pulses_to_txt(mapped_pulses, output_file_path):
    """Ghi cac bo xung encoder da map ra file txt."""
    mapped_pulses = np.rint(np.array(mapped_pulses, dtype=float)).astype(int)
    if mapped_pulses.ndim != 2 or mapped_pulses.shape[1] != 6:
        raise ValueError("Expected mapped pulses with shape (N, 6).")

    with open(output_file_path, "w", encoding="utf-8") as file:
        for pulse_set in mapped_pulses:
            file.write(" ".join(str(int(value)) for value in pulse_set) + "\n")

    return output_file_path

def export_xyz_positions_to_txt(xyz_positions, output_file_path):
    """Ghi cac toa do XYZ Cartesian ra file txt."""
    xyz_positions = np.array(xyz_positions, dtype=float)
    if xyz_positions.ndim != 2 or xyz_positions.shape[1] != 3:
        raise ValueError("Expected XYZ positions with shape (N, 3).")

    with open(output_file_path, "w", encoding="utf-8") as file:
        for xyz in xyz_positions:
            file.write(" ".join(f"{value:.3f}" for value in xyz) + "\n")

    return output_file_path

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
    """
    Build a line-mapped trajectory from encoder waypoints and export the generated
    pulses and optional angle trajectory.
    """
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
        mapped_pulses_output_path = os.path.join(
            LINE_MAPPING_OUTPUT_DIR,
            f"{base_name}_line_mapped{ext}",
        )
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(
            LINE_MAPPING_OUTPUT_DIR,
            f"{base_name}_line_angles{ext}",
        )
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    ensure_output_dir(LINE_MAPPING_OUTPUT_DIR)
    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(
            LINE_MAPPING_OUTPUT_DIR,
            f"{base_name}_line_xyz{ext}",
        )
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
    """
    Build a piecewise-straight line-mapped trajectory from Cartesian XYZ waypoints.
    Orientation is kept fixed from the provided reference pose.
    """
    xyz_points = np.array(xyz_points, dtype=float)
    if xyz_points.ndim != 2 or xyz_points.shape[1] != 3:
        raise ValueError("Expected XYZ waypoints with shape (N, 3).")
    if len(xyz_points) < 2:
        raise ValueError("XYZ mapping requires at least 2 waypoints.")

    reference_angles = np.array(reference_angles, dtype=float).flatten()
    if reference_angles.size != 6:
        raise ValueError("Expected exactly 6 reference joint angles.")

    reference_pose = forward_kinematics_from_joint_angles(reference_angles)
    if fixed_rotation is None:
        fixed_rotation = reference_pose["rotation"].copy()
    else:
        fixed_rotation = np.array(fixed_rotation, dtype=float).reshape(3, 3)

    mapped_positions, _, segment_lengths, waypoint_indices = generate_polyline_position_rotation_targets(
        xyz_points,
        np.repeat(fixed_rotation[None, :, :], len(xyz_points), axis=0),
        num_samples_per_segment=num_samples_per_segment,
    )

    initial_angles_ik = convert_display_angles_to_ik_angles(reference_angles)
    first_point_ik = Inverse_Kinematics(
        mapped_positions[0],
        fixed_rotation,
        current_angles=initial_angles_ik,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
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

    fixed_waypoint_angles = {
        0: np.array(first_point_ik["solution"], dtype=float).copy()
    }
    mapped_angles_ik = cartesian_polyline_to_joint_trajectory(
        mapped_positions,
        np.repeat(fixed_rotation[None, :, :], len(mapped_positions), axis=0),
        initial_angles_ik,
        fixed_waypoint_angles=fixed_waypoint_angles,
        angle_tolerance=angle_tolerance,
        debug=debug,
    )
    mapped_angles = convert_ik_angles_to_display_angles(mapped_angles_ik)
    mapped_pulses = np.array(
        [
            angles_to_encoder_pulses(angles, ppr=ppr, gearbox_ratios=gearbox_ratios)
            for angles in mapped_angles
        ],
        dtype=int,
    )
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
    """Build a line-mapped trajectory from XYZ waypoints and export pulses/angles/XYZ."""
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
        mapped_pulses_output_path = os.path.join(
            XYZ_OUTPUT_DIR,
            f"{base_name}_xyz_pulses{ext}",
        )
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(
            XYZ_OUTPUT_DIR,
            f"{base_name}_xyz_angles{ext}",
        )
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(
            XYZ_OUTPUT_DIR,
            f"{base_name}_xyz_actual{ext}",
        )
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping


def generate_circle_xy_points(center_x, center_y, z_height, radius, num_samples):
    """Generate one closed circular path on the XY plane at a fixed Z height."""
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
    circle_points = np.column_stack([
        center_x + radius * np.cos(angles),
        center_y + radius * np.sin(angles),
        np.full(num_samples, z_height, dtype=float),
    ])
    # Close the loop so the robot returns to the starting point.
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
    """Build a circular XY Cartesian path and convert it to pulses."""
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
    """Format numeric values into filesystem-friendly filename parts."""
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
    """Build an XY circle path and export pulses, angles, and verified XYZ."""
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
        mapped_pulses_output_path = os.path.join(
            CIRCLE_OUTPUT_DIR,
            f"{base_name}_circle_pulses.txt",
        )
    export_mapped_pulses_to_txt(mapping["mapped_pulses"], mapped_pulses_output_path)

    if mapped_angles_output_path is None:
        mapped_angles_output_path = os.path.join(
            CIRCLE_OUTPUT_DIR,
            f"{base_name}_circle_angles.txt",
        )
    with open(mapped_angles_output_path, "w", encoding="utf-8") as file:
        for angle_set in mapping["mapped_angles"]:
            file.write(" ".join(f"{angle:.6f}" for angle in angle_set) + "\n")

    if mapped_xyz_output_path is None:
        mapped_xyz_output_path = os.path.join(
            CIRCLE_OUTPUT_DIR,
            f"{base_name}_circle_actual.txt",
        )
    export_xyz_positions_to_txt(mapping["mapped_actual_positions"], mapped_xyz_output_path)

    mapping["mapped_pulses_output_path"] = mapped_pulses_output_path
    mapping["mapped_angles_output_path"] = mapped_angles_output_path
    mapping["mapped_xyz_output_path"] = mapped_xyz_output_path
    return mapping

# =====================================================================
# NHOM HAM FK / IK LOI
# - FK dung cho ve robot, tinh pose
# - IK dung trong line map va cac bai toan nghich
# =====================================================================

def Forward_Kinematics(theta1_FK_, theta2_FK_, theta3_FK_, theta4_FK_, theta5_FK_, theta6_FK_):

    T_0_1_Matrix = T_0_1(L1, POS, d0, theta1_FK_)
    T_1_2_Matrix = T_1_2(L2, 0, 0, theta2_FK_)
    T_2_3_Matrix = T_2_3(L3, POS, 0, theta3_FK_)
    T_3_4_Matrix = T_3_4(0, NEG, d1, theta4_FK_)
    T_4_5_Matrix = T_4_5(0, POS, 0, theta5_FK_)
    T_5_6_Matrix = T_5_6(0, 0, d2, theta6_FK_)

    transformationMatrix = (
        T_0_1_Matrix @ T_1_2_Matrix @ T_2_3_Matrix @
        T_3_4_Matrix @ T_4_5_Matrix @ T_5_6_Matrix
    )
    return transformationMatrix

# =====================================================================
# INVERSE KINEMATICS - Converted from C++
# =====================================================================

def R_0_3(theta1, theta2, theta3):
    """Calculate rotation matrix R_0_3 for joints 1,2,3"""
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)  # No offset - FK doesn't use offset
    T23 = T_2_3(L3, POS, 0, theta3)
    T03 = T01 @ T12 @ T23
    return T03[:3, :3]

def R_0_3_Inverse(theta1, theta2, theta3):
    """Calculate inverse rotation matrix R_0_3^-1 (transpose for rotation matrix)"""
    R03 = R_0_3(theta1, theta2, theta3)
    return R03.T

def Check_T_0_3(theta1, theta2, theta3, Wx, Wy, Wz, tolerance=5.0, debug=False):
    """Verify if theta1,2,3 produce correct wrist center position"""
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)  # No offset - FK doesn't use offset
    T23 = T_2_3(L3, POS, 0, theta3)
    T03 = T01 @ T12 @ T23

    W_calc = T03[:3, 3]
    error = np.sqrt((W_calc[0] - Wx)**2 + (W_calc[1] - Wy)**2 + (W_calc[2] - Wz)**2)
    if debug:
        print(f"  W_expected=[{Wx:.2f}, {Wy:.2f}, {Wz:.2f}], W_calc=[{W_calc[0]:.2f}, {W_calc[1]:.2f}, {W_calc[2]:.2f}], error={error:.2f}")
    return error < tolerance

def Check_T_0_4(theta1, theta2, theta3, Wx, Wy, Wz, tolerance=5.0, debug=False):
    """Verify if theta1,2,3 produce correct wrist center position at joint 4"""
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)  # No offset - FK doesn't use offset
    T23 = T_2_3(L3, POS, 0, theta3)
    T34 = T_3_4(0, NEG, d1, 0)  # theta4=0 for wrist center position
    T04 = T01 @ T12 @ T23 @ T34

    W_calc = T04[:3, 3]
    error = np.sqrt((W_calc[0] - Wx)**2 + (W_calc[1] - Wy)**2 + (W_calc[2] - Wz)**2)
    if debug:
        print(f"  W_expected=[{Wx:.2f}, {Wy:.2f}, {Wz:.2f}], W_calc=[{W_calc[0]:.2f}, {W_calc[1]:.2f}, {W_calc[2]:.2f}], error={error:.2f}")
    return error < tolerance

def Check_T_0_6(EE_pos, R_0_6, theta1, theta2, theta3, theta4, theta5, theta6, pos_tol=1.0, rot_tol=0.01):
    """Verify if all 6 joint angles produce correct end-effector pose"""
    T06 = Forward_Kinematics(theta1, theta2, theta3, theta4, theta5, theta6)  # No offset - FK doesn't use offset

    # Check position
    pos_error = np.linalg.norm(T06[:3, 3] - EE_pos)

    # Check rotation
    R_calc = T06[:3, :3]
    rot_error = np.linalg.norm(R_calc - R_0_6)

    return pos_error < pos_tol and rot_error < rot_tol

def Inverse_Kinematics(EE_pos, R_0_6, current_angles=None, angle_tolerance=5.0, debug=False):
    """
    Solve inverse kinematics for the 6-axis robot.

    Parameters:
    -----------
    EE_pos : array-like (3,) or (3,1)
        End-effector position [x, y, z] in mm.
    R_0_6 : array-like (3,3)
        Rotation matrix from base to end effector.
    current_angles : array-like (6,), optional
        Current joint angles [theta1, theta2, theta3, theta4, theta5, theta6].
        When provided, the closest IK solution is selected.
    angle_tolerance : float, optional
        Maximum per-joint angle error allowed when comparing to current_angles.
    debug : bool
        Print debug information.
    """

    # Convert inputs to numpy arrays.
    EE_pos = np.array(EE_pos).flatten()
    R_0_6 = np.array(R_0_6).reshape(3, 3)

    EEx, EEy, EEz = EE_pos

    # # Workspace check
    # Out_Of_Workspace_TH1 = EEx <= 0 and EEy >= 0  # Quadrant 2
    # Out_Of_Workspace_TH2 = EEx <= 0 and EEy <= 0  # Quadrant 3
    # Out_Of_Workspace_TH3 = EEz <= 0               # Z negative

    # if Out_Of_Workspace_TH1 or Out_Of_Workspace_TH2 or Out_Of_Workspace_TH3:
    #     if debug:
    #         print(f"OUT OF WORKSPACE! EEx={EEx:.2f}, EEy={EEy:.2f}, EEz={EEz:.2f}")
    #     return {'status': 'OOW', 'solution': None, 'num_solutions': 0}

    # Compute wrist center (W), the approximate joint-4 wrist intersection.
    # W = EE - d2 * end-effector z-axis.
    # In this DH model, joint 4 is offset from joint 3 by d1 along frame-3 Z.
    # The end effector is offset from the wrist by d2 along frame-6 Z.
    # => W_pos = EE_pos - d2 * R_0_6[:, 2] (EE Z axis).
    W_pos = EE_pos - d2 * R_0_6[:, 2]
    Wx, Wy, Wz = W_pos

    if debug:
        print(f"EE Position: [{EEx:.2f}, {EEy:.2f}, {EEz:.2f}] mm")
        print(f"Wrist Center: [{Wx:.2f}, {Wy:.2f}, {Wz:.2f}] mm")

    # ==================== Compute Theta1 ====================
    theta1_IK = np.degrees(np.arctan2(Wy, Wx))

    if debug:
        print(f"\n[STEP 1] theta1 = {theta1_IK:.2f} deg")

    # ==================== Compute Theta3 ====================
    r_xy = np.sqrt(Wx**2 + Wy**2) - L1 # BA'
    r_z = Wz - d0 #A'W
    D = np.sqrt(r_xy**2 + r_z**2) # BW

    TS_Cos_Gamma = -(L2**2 + L3**2 + d1**2) + (r_xy**2 + r_z**2)
    MS_Cos_Gamma = 2 * L2 * np.sqrt(L3**2 + d1**2)

    Cos_Gamma = TS_Cos_Gamma / MS_Cos_Gamma

    # Check workspace reachability.
    if abs(Cos_Gamma) > 1:
        if debug:
            print(f"OUT OF WORKSPACE! Cos_Gamma = {Cos_Gamma:.3f} (|value| > 1)")
        return {'status': 'OOW', 'solution': None, 'num_solutions': 0}

    Sin_Gamma = np.sqrt(1 - Cos_Gamma**2)

    Gamma_1 = np.degrees(np.arctan2(Sin_Gamma, Cos_Gamma))
    Gamma_2 = np.degrees(np.arctan2(-Sin_Gamma, Cos_Gamma))

    Beta = np.degrees(np.arctan2(d1, L3))

    theta3_IK_1 = Gamma_1 + Beta
    theta3_IK_2 = Gamma_2 + Beta

    if debug:
        print(f"\n[STEP 2] theta3 solutions:")
        print(f"  theta3_IK_1 = {theta3_IK_1:.2f} deg")
        print(f"  theta3_IK_2 = {theta3_IK_2:.2f} deg")

    # ==================== Compute Theta2 ====================
    TS_Cos_Phi = r_xy**2 + r_z**2 + L2**2 - (L3**2 + d1**2)
    MS_Cos_Phi = 2 * D * L2

    Cos_Phi = TS_Cos_Phi / MS_Cos_Phi
    Sin_Phi = np.sqrt(1 - Cos_Phi**2)

    Phi_1 = np.degrees(np.arctan2(Sin_Phi, Cos_Phi))
    Phi_2 = np.degrees(np.arctan2(-Sin_Phi, Cos_Phi))

    Sigma = np.degrees(np.arctan2(r_z, r_xy))

    theta2_IK_1 = Sigma - Phi_1
    theta2_IK_2 = Sigma - Phi_2

    # Normalize to [-180, 180].
    theta2_IK_1 = normalize_angle(theta2_IK_1)
    theta2_IK_2 = normalize_angle(theta2_IK_2)

    if debug:
        print(f"\n[STEP 3] theta2 solutions:")
        print(f"  theta2_IK_1 = {theta2_IK_1:.2f} deg")
        print(f"  theta2_IK_2 = {theta2_IK_2:.2f} deg")

    # ==================== Filter Theta1,2,3 candidates ====================
    # 4 candidate sets: (theta1, theta2_1, theta3_1), (theta1, theta2_1, theta3_2),
    #              (theta1, theta2_2, theta3_1), (theta1, theta2_2, theta3_2)

    valid_solutions_123 = []  # Store all valid joint 1-3 candidates.

    # Verify position only here; joint limits are checked later.
    candidates = [
        (theta1_IK, theta2_IK_1, theta3_IK_1),
        (theta1_IK, theta2_IK_1, theta3_IK_2),
        (theta1_IK, theta2_IK_2, theta3_IK_1),
        (theta1_IK, theta2_IK_2, theta3_IK_2)
    ]

    if debug:
        print(f"\n[STEP 4] Check 4 candidate sets (theta1, theta2, theta3):")

    for idx, (t1, t2, t3) in enumerate(candidates, 1):
        if debug:
            print(f"  Candidate {idx}: theta1={t1:.2f} deg, theta2={t2:.2f} deg, theta3={t3:.2f} deg", end="")
        # Verify using T_0_4 wrist center only.
        if Check_T_0_4(t1, t2, t3, Wx, Wy, Wz, tolerance=5.0, debug=False):
            valid_solutions_123.append((t1, t2, t3))
            if debug:
                print(" -> OK VALID")
        elif debug:
            print(" -> FAIL")

    if len(valid_solutions_123) == 0:
        if debug:
            print("\n[FAIL] No valid solution for joints 1-3")
        return {'status': 'FAIL', 'solution': None, 'all_solutions': [], 'num_solutions': 0}

    if debug:
        print(f"\n[STEP 5] Found {len(valid_solutions_123)} valid solutions for joints 1-3")

    # ==================== Compute Theta4,5,6 for all valid 1-3 candidates ====================
    all_complete_solutions = []

    for sol_idx, (theta1_IK, theta2_IK, theta3_IK) in enumerate(valid_solutions_123, 1):
        if debug:
            print(f"\n[STEP 6.{sol_idx}] Compute theta4,5,6 for solution {sol_idx}: [{theta1_IK:.2f} deg, {theta2_IK:.2f} deg, {theta3_IK:.2f} deg]")

        R_0_3_inv = R_0_3_Inverse(theta1_IK, theta2_IK, theta3_IK)
        R_3_6 = R_0_3_inv @ R_0_6

        r13 = R_3_6[0, 2]
        r23 = R_3_6[1, 2]
        r33 = R_3_6[2, 2]
        r31 = R_3_6[2, 0]
        r32 = R_3_6[2, 1]

        # Compute theta5 with two candidate signs.
        Cos_Theta5 = r33
        Sin_Theta5 = np.sqrt(max(0, 1 - Cos_Theta5**2))

        theta5_IK_1 = np.degrees(np.arctan2(Sin_Theta5, Cos_Theta5))
        theta5_IK_2 = np.degrees(np.arctan2(-Sin_Theta5, Cos_Theta5))

        if debug:
            print(f"  theta5_IK_1 = {theta5_IK_1:.2f} deg, theta5_IK_2 = {theta5_IK_2:.2f} deg")

        # Check singularity when theta5 is near 0.
        is_singularity = abs(Sin_Theta5) < 0.001

        if is_singularity and current_angles is not None:
            # Singularity: theta4 + theta6 = const
            # Keep theta4 from current_angles and solve theta6.
            theta4_IK = current_angles[3]
            # Compute theta6 from R_3_6 based on theta4.
            # If theta5 is 0, theta4 + theta6 is derived from atan2(r32, r31).
            sum_46 = np.degrees(np.arctan2(r32, r31))
            theta6_IK = sum_46 - theta4_IK
            # Normalize theta6
            if theta6_IK >= 90:
                theta6_IK -= 180
            elif theta6_IK <= -90:
                theta6_IK += 180
            if debug:
                print(f"  [SINGULARITY] theta5 ~= 0, keeping theta4={theta4_IK:.2f} deg, theta6={theta6_IK:.2f} deg")
        else:
            # Normal case: compute theta4 and theta6 from R_3_6.
            theta4_IK = np.degrees(np.arctan2(r23, r13))
            theta6_IK = np.degrees(np.arctan2(r32, -r31))

            # Normalize theta4 and theta6 to [-90, 90].
            if theta4_IK >= 90:
                theta4_IK -= 180
            elif theta4_IK <= -90:
                theta4_IK += 180

            if theta6_IK >= 90:
                theta6_IK -= 180
            elif theta6_IK <= -90:
                theta6_IK += 180

            if debug:
                print(f"  theta4 = {theta4_IK:.2f} deg, theta6 = {theta6_IK:.2f} deg")

        # Try both theta5 candidates. Also add equivalent wrist branches so
        # continuity weighting can avoid sudden J4/J6 flips near wrist singularity.
        for idx_theta5, theta5_IK in enumerate([theta5_IK_1, theta5_IK_2], 1):
            if debug:
                print(f"  Try theta5_{idx_theta5} = {theta5_IK:.2f} deg -> ", end="")

            candidate_solutions = [
                [theta1_IK, theta2_IK, theta3_IK, theta4_IK, theta5_IK, theta6_IK],
                [
                    theta1_IK,
                    theta2_IK,
                    theta3_IK,
                    normalize_angle(theta4_IK + 180.0),
                    -theta5_IK,
                    normalize_angle(theta6_IK - 180.0),
                ],
                [
                    theta1_IK,
                    theta2_IK,
                    theta3_IK,
                    normalize_angle(theta4_IK - 180.0),
                    -theta5_IK,
                    normalize_angle(theta6_IK + 180.0),
                ],
            ]

            if current_angles is not None and abs(theta5_IK) < 1.0:
                preserved_theta4 = float(current_angles[3])
                preserved_theta6 = normalize_angle(theta4_IK + theta6_IK - preserved_theta4)
                candidate_solutions.append([
                    theta1_IK,
                    theta2_IK,
                    theta3_IK,
                    preserved_theta4,
                    theta5_IK,
                    preserved_theta6,
                ])

            valid_count = 0
            seen_keys = set()
            for solution in candidate_solutions:
                key = tuple(round(float(value), 6) for value in solution)
                if key in seen_keys:
                    continue
                seen_keys.add(key)

                if not are_joint_angles_within_limits(solution):
                    continue
                if not Check_T_0_6(EE_pos, R_0_6, *solution):
                    continue

                all_complete_solutions.append(solution)
                valid_count += 1

            if debug:
                if valid_count:
                    print(f"OK VALID ({valid_count} wrist branches)")
                else:
                    print("FAIL (no valid wrist branch)")

    if len(all_complete_solutions) == 0:
        if debug:
            print("\n[FAIL] No complete valid IK solution")
        return {'status': 'FAIL', 'solution': None, 'all_solutions': [], 'num_solutions': 0}

    if debug:
        print(f"\n[STEP 7] Found {len(all_complete_solutions)} complete solutions")

    # ==================== Select best solution ====================
    if current_angles is not None:
        # Select the solution closest to the current joint state.
        current_angles = np.array(current_angles).flatten()
        min_error = float('inf')
        best_solution = all_complete_solutions[0]

        if debug:
            print(f"\n[STEP 8] Chon nghiem gan nhat voi current_angles = {[f'{x:.2f}' for x in current_angles]}")

        for idx, sol in enumerate(all_complete_solutions, 1):
            # Compute weighted continuity error with angle wrapping.
            error = 0.0
            errors_per_joint = []
            weighted_errors_per_joint = []
            for i in range(6):
                diff = abs(sol[i] - current_angles[i])
                # Handle angular wraparound so 180 and -180 are treated as adjacent.
                if diff > 180:
                    diff = 360 - diff
                weighted_diff = IK_CONTINUITY_WEIGHTS[i] * diff
                errors_per_joint.append(diff)
                weighted_errors_per_joint.append(weighted_diff)
                error += weighted_diff

            if debug:
                print(
                    f"  Solution {idx}: weighted_error={error:.2f} "
                    f"(raw per joint: {[f'{e:.2f}' for e in errors_per_joint]}, "
                    f"weighted: {[f'{e:.2f}' for e in weighted_errors_per_joint]})"
                )

            if error < min_error:
                min_error = error
                best_solution = sol

        # Compute max raw joint error for the tolerance check.
        max_individual_error = 0
        for i in range(6):
            diff = abs(best_solution[i] - current_angles[i])
            if diff > 180:
                diff = 360 - diff
            max_individual_error = max(max_individual_error, diff)

        if debug:
            print(f"\n[RESULT] Chon nghiem voi error={min_error:.2f} deg, max_error={max_individual_error:.2f} deg")
            print(f"  Best Solution: {[f'{x:.2f}' for x in best_solution]}")

        # Report NO_MATCH if the closest solution is still too far away.
        if max_individual_error > angle_tolerance:
            if debug:
                print(f"WARNING: No solution matches expected angles within tolerance ({angle_tolerance:.1f} deg)")
                print(f"  Expected: {[f'{x:.2f}' for x in current_angles]}")
                print(f"  Best IK:  {[f'{x:.2f}' for x in best_solution]}")
                print(f"  Error:    {[f'{abs(best_solution[i] - current_angles[i]):.2f}' for i in range(6)]}")
            return {
                'status': 'NO_MATCH', 
                'solution': best_solution,
                'all_solutions': all_complete_solutions,
                'num_solutions': len(all_complete_solutions),
                'error': max_individual_error
            }
    else:
        # If no reference angle is provided, use the first solution.
        best_solution = all_complete_solutions[0]
        if debug:
            print(f"\n[RESULT] No current_angles provided, use the first solution: {[f'{x:.2f}' for x in best_solution]}")

    # Clean up -0.0 artifacts
    best_solution = [0.0 if abs(x) < 1e-10 else x for x in best_solution]
    all_complete_solutions = [[0.0 if abs(x) < 1e-10 else x for x in sol] for sol in all_complete_solutions]

    return {
        'status': 'OK', 
        'solution': best_solution, 
        'all_solutions': all_complete_solutions,
        'num_solutions': len(all_complete_solutions)
    }

# Return joint positions (p0..p6) and cumulative transforms (T0i)
def forward_points(theta1_, theta2_, theta3_, theta4_, theta5_, theta6_, use_offset=True):

    if use_offset:
        theta2_ = theta2_ + THETA2_OFFSET

    T01 = T_0_1(L1, POS, d0, theta1_)
    T12 = T_1_2(L2, 0, 0, theta2_)
    T23 = T_2_3(L3, POS, 0, theta3_)
    T34 = T_3_4(0, NEG, d1, theta4_)
    T45 = T_4_5(0, POS, 0, theta5_)
    T56 = T_5_6(0, 0, d2, theta6_)

    T = np.eye(4)
    pts = [T[:3, 3].copy()]  # p0

    Ts = []
    for Ti in [T01, T12, T23, T34, T45, T56]:
        T = T @ Ti
        Ts.append(T.copy())          # T0i
        pts.append(T[:3, 3].copy())  # pi

    return np.array(pts), Ts

# Plot robot skeleton from joint points (p0..p6)
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

    # save view before clearing
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
            ax.text(
                axis_tip[0],
                axis_tip[1],
                axis_tip[2],
                axis_labels[axis_idx],
                color=axis_colors[axis_idx],
                fontsize=9,
                fontweight="bold",
            )

    if trace_points is not None and len(trace_points) > 0:
        trace_points = np.array(trace_points, dtype=float)
        ax.plot(
            trace_points[:, 0],
            trace_points[:, 1],
            trace_points[:, 2],
            color="#d62728",
            linewidth=2,
            alpha=0.9,
        )
        # Red trace points
        ax.scatter(
            trace_points[:, 0],
            trace_points[:, 1],
            trace_points[:, 2],
            color="#d62728",
            s=2,
            depthshade=False,
        )

    ax.set_title(title)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    # fixed axes (do NOT follow robot)
    set_axes_fixed(ax, lim=lim, base=base)

    # restore view
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

# =====================================================================
# WORKSPACE CLOUD SIMULATION
# =====================================================================

def generate_workspace_points(
    samples_per_joint=WORKSPACE_SAMPLES_PER_JOINT,
    joint_limits=None,
    sampled_joint_indices=WORKSPACE_SAMPLED_JOINTS,
    fixed_angles=WORKSPACE_FIXED_ANGLES,
):
    """Sample main positioning joints and return reachable end-effector points."""
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

    joint_samples = [
        np.linspace(limits[idx, 0], limits[idx, 1], samples_per_joint)
        for idx in sampled_joint_indices
    ]
    total_points = samples_per_joint ** len(sampled_joint_indices)
    workspace_points = np.empty((total_points, 3), dtype=float)

    for point_idx, sampled_values in enumerate(product(*joint_samples)):
        q = fixed_angles.copy()
        for joint_idx, joint_value in zip(sampled_joint_indices, sampled_values):
            q[joint_idx] = joint_value
        T06 = Forward_Kinematics(q[0], q[1] + THETA2_OFFSET, q[2], q[3], q[4], q[5])
        workspace_points[point_idx] = T06[:3, 3]

    return workspace_points


def set_axes_equal_from_points(ax, points, padding_ratio=0.08):
    """Set equal 3D axes around a point cloud."""
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
    """Plot sampled workspace as an end-effector point cloud."""
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

    ax.set_title(f"Workspace of Welding Robot", pad=14)
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
    """Generate and show the sampled workspace cloud."""
    workspace_points = generate_workspace_points(samples_per_joint=samples_per_joint)
    return plot_workspace_cloud(workspace_points, samples_per_joint=samples_per_joint)

# =====================================================================
# UI CHINH
# - Mode Teach Playback
# - Mode Line Map
# =====================================================================

def run_slider_ui():
    from tkinter import Tk, filedialog, messagebox

    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("6-Axis Robot Simulator")

    robot_panel = [0.02, 0.16, 0.50, 0.78]
    controls_panel = [0.56, 0.08, 0.40, 0.84]

    ax_robot = fig.add_axes(robot_panel, projection="3d")
    lim = 700
    base = (0.0, 0.0, 0.0)

    ax_header = fig.add_axes([controls_panel[0], 0.88, controls_panel[2], 0.04])
    ax_header.axis("off")
    ax_header.text(0.0, 0.5, "Joint Controls", fontsize=11, fontweight="bold", va="center")

    mode_button_w = 0.075
    mode_button_gap = 0.012
    mode_row_y = 0.84

    ax_mode_teach = fig.add_axes([controls_panel[0], mode_row_y, mode_button_w, 0.028])
    btn_mode_teach = Button(ax_mode_teach, "Teach")
    ax_mode_line = fig.add_axes([controls_panel[0] + mode_button_w + mode_button_gap, mode_row_y, mode_button_w, 0.028])
    btn_mode_line = Button(ax_mode_line, "Line Map")
    ax_mode_xyz = fig.add_axes([controls_panel[0] + 2 * (mode_button_w + mode_button_gap), mode_row_y, mode_button_w, 0.028])
    btn_mode_xyz = Button(ax_mode_xyz, "XYZ")
    ax_mode_circle = fig.add_axes([controls_panel[0] + 3 * (mode_button_w + mode_button_gap), mode_row_y, mode_button_w, 0.028])
    btn_mode_circle = Button(ax_mode_circle, "Circle")

    ax_file_label = fig.add_axes([controls_panel[0], 0.792, 0.18, 0.024])
    ax_file_label.axis("off")
    txt_file_label = ax_file_label.text(0.0, 0.5, "Trajectory File (.txt)", fontsize=9, fontweight="bold", va="center")

    ax_primary = fig.add_axes([controls_panel[0] + 4 * (mode_button_w + mode_button_gap), mode_row_y, 0.055, 0.028])
    btn_primary = Button(ax_primary, "Run", hovercolor="#dddddd")

    path_box_w = 0.23
    path_button_w = 0.04
    path_gap = 0.008
    path_h = 0.028
    path_y = 0.758
    ax_file_path = fig.add_axes([controls_panel[0], path_y, path_box_w, path_h])
    txt_file_path = TextBox(ax_file_path, "", initial="")
    txt_file_path.text_disp.set_clip_on(True)
    txt_file_path.text_disp.set_clip_box(ax_file_path.bbox)
    ax_file_button = fig.add_axes([controls_panel[0] + path_box_w + path_gap, path_y, path_button_w, path_h])
    btn_file = Button(ax_file_button, "...")

    circle_param_y_top = 0.722
    circle_param_h = 0.022
    circle_param_axes = []
    circle_param_boxes = {}

    circle_label_w = 0.032
    circle_value_w = 0.055
    circle_col_gap = 0.012
    circle_group_w = circle_label_w + circle_value_w
    circle_col_x = [
        controls_panel[0],
        controls_panel[0] + circle_group_w + circle_col_gap,
        controls_panel[0] + 2 * (circle_group_w + circle_col_gap),
        controls_panel[0] + 3 * (circle_group_w + circle_col_gap),
    ]

    def add_circle_param(x_label, y_pos, label, key, initial, fontsize=8.0):
        ax_label = fig.add_axes([x_label, y_pos, circle_label_w, circle_param_h])
        ax_label.axis("off")
        ax_label.text(1.0, 0.5, label, fontsize=fontsize, fontweight="bold", va="center", ha="right")
        circle_param_axes.append(ax_label)

        ax_value = fig.add_axes([x_label + circle_label_w + 0.006, y_pos, circle_value_w, circle_param_h])
        textbox = TextBox(ax_value, "", initial=initial, textalignment="center")
        textbox.text_disp.set_fontsize(8.0)
        circle_param_axes.append(ax_value)
        circle_param_boxes[key] = textbox

    def add_circle_param_row(y_pos, specs):
        for x_label, label, key, initial in specs:
            add_circle_param(x_label, y_pos, label, key, initial)

    circle_row_specs = [
        (circle_col_x[0], "Tam X", "center_x", "450"),
        (circle_col_x[1], "Tam Y", "center_y", "0"),
        (circle_col_x[2], "Z", "z_height", "620"),
        (circle_col_x[3], "R", "radius", "80"),
    ]

    add_circle_param_row(circle_param_y_top, circle_row_specs)

    ax_text = fig.add_axes([controls_panel[0], 0.645, controls_panel[2], 0.065])
    ax_text.axis("off")
    txt_ee = ax_text.text(
        0.0,
        1.0,
        "Position: X=0.0 mm   Y=0.0 mm   Z=0.0 mm\nOrientation: R=0.0 deg   P=0.0 deg   Y=0.0 deg",
        fontsize=8.5,
        fontweight="bold",
        va="top",
    )

    names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    sliders = []
    textboxes = []
    joint_limits = get_joint_angle_limits()
    file_state = {
        "mode": "teach",
        "path": "",
        "is_running": False,
        "current_q": np.zeros(6, dtype=float),
        "trace_points": [],
    }

    slider_x = controls_panel[0] + 0.03
    slider_y_start = 0.56
    row_gap = 0.075
    slider_w = 0.18
    slider_h = 0.035
    textbox_w = 0.13
    textbox_offset = 0.03

    for i, name in enumerate(names):
        slider_y = slider_y_start - i * row_gap

        ax_s = fig.add_axes([slider_x, slider_y, slider_w, slider_h])
        s = Slider(
            ax_s,
            f"{name} (deg)",
            joint_limits[i, 0],
            joint_limits[i, 1],
            valinit=0.0,
            valstep=0.1,
        )
        s.valtext.set_visible(False)
        sliders.append(s)

        ax_t = fig.add_axes([slider_x + slider_w + textbox_offset, slider_y, textbox_w, slider_h])
        t = TextBox(ax_t, "", initial="0.0", textalignment="center")
        textboxes.append(t)

    ax_reset = fig.add_axes([controls_panel[0] + 0.10, 0.055, 0.15, 0.03])
    btn_reset = Button(ax_reset, "Reset", hovercolor="#dddddd")
    ax_workspace = fig.add_axes([controls_panel[0] + 0.10, 0.015, 0.15, 0.03])
    btn_workspace = Button(ax_workspace, "Workspace", hovercolor="#dddddd")

    for button in [
        btn_mode_teach,
        btn_mode_line,
        btn_mode_xyz,
        btn_mode_circle,
        btn_primary,
        btn_file,
        btn_reset,
        btn_workspace,
    ]:
        button.label.set_fontsize(8.5)

    def choose_txt_file():
        root = Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        file_path = filedialog.askopenfilename(
            title="Select TXT file",
            filetypes=[("Text files", "*.txt")],
        )
        root.destroy()
        return file_path

    def select_txt_file(_event):
        file_path = choose_txt_file()
        if file_path:
            file_state["path"] = file_path
            txt_file_path.set_val(file_path)

    def on_file_path_submit(text):
        file_state["path"] = text.strip()

    def show_warning(message):
        root = Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        messagebox.showwarning("Warning", message, parent=root)
        root.destroy()

    def show_info(message):
        root = Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        messagebox.showinfo("Info", message, parent=root)
        root.destroy()

    def format_mapping_error(exc, mode_name):
        if isinstance(exc, MappingIKError):
            lines = [f"{mode_name} failed during IK."]
            if exc.point_index is not None:
                lines.append(f"Point index: {exc.point_index} (point #{exc.point_index + 1})")
            if exc.point is not None and exc.point.size == 3:
                lines.append(f"XYZ target: {exc.point[0]:.3f}, {exc.point[1]:.3f}, {exc.point[2]:.3f}")
            if exc.status is not None:
                lines.append(f"IK status: {exc.status}")
            if exc.status == "OOW":
                lines.append("Hint: target point or tool orientation is outside reachable workspace.")
            elif exc.status == "FAIL":
                lines.append("Hint: position may be reachable, but current orientation or joint limits make IK invalid.")
            elif exc.status == "NO_MATCH":
                lines.append("Hint: IK found a pose, but it was too far from the previous joint state.")
            return "\n".join(lines)
        return str(exc)

    def set_mode(mode):
        file_state["mode"] = mode
        show_circle_params = False
        ax_file_label.set_visible(not show_circle_params)
        ax_file_path.set_visible(not show_circle_params)
        ax_file_button.set_visible(not show_circle_params)
        for axis in circle_param_axes:
            axis.set_visible(show_circle_params)

        if mode == "teach":
            txt_file_label.set_text("Trajectory File (.txt)")
            btn_primary.label.set_text("Run")
            btn_mode_teach.color = "#d9ead3"
            btn_mode_line.color = "#f0f0f0"
            btn_mode_xyz.color = "#f0f0f0"
            btn_mode_circle.color = "#f0f0f0"
        elif mode == "line":
            txt_file_label.set_text("Waypoint File (.txt)")
            btn_primary.label.set_text("Map")
            btn_mode_teach.color = "#f0f0f0"
            btn_mode_line.color = "#d9ead3"
            btn_mode_xyz.color = "#f0f0f0"
            btn_mode_circle.color = "#f0f0f0"
        elif mode == "xyz":
            txt_file_label.set_text("XYZ File (.txt)")
            btn_primary.label.set_text("XYZ")
            btn_mode_teach.color = "#f0f0f0"
            btn_mode_line.color = "#f0f0f0"
            btn_mode_xyz.color = "#d9ead3"
            btn_mode_circle.color = "#f0f0f0"
        else:
            txt_file_label.set_text("Circle Pose File (.txt)")
            btn_primary.label.set_text("Run")
            btn_mode_teach.color = "#f0f0f0"
            btn_mode_line.color = "#f0f0f0"
            btn_mode_xyz.color = "#f0f0f0"
            btn_mode_circle.color = "#d9ead3"
        fig.canvas.draw_idle()

    def render_robot(q, sync_textboxes=True):
        q = clamp_joint_angles(q)
        file_state["current_q"] = q.copy()

        if sync_textboxes:
            for angle, textbox in zip(q, textboxes):
                textbox.set_val(f"{angle:.1f}")

        pts, _ = forward_points(*q, use_offset=True)
        T06 = Forward_Kinematics(q[0], q[1] + THETA2_OFFSET, q[2], q[3], q[4], q[5])
        plot_robot(
            pts,
            ax=ax_robot,
            title="Robot 6-Axis",
            lim=lim,
            base=base,
            trace_points=file_state["trace_points"],
            end_effector_rotation=T06[:3, :3],
        )
        p = T06[:3, 3]
        R = T06[:3, :3]

        pitch = np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2)))
        yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))

        txt_ee.set_text(
            f"Position: X={p[0]:.1f} mm   Y={p[1]:.1f} mm   Z={p[2]:.1f} mm\n"
            f"Orientation: R={roll:.1f} deg   P={pitch:.1f} deg   Y={yaw:.1f} deg"
        )

        fig.canvas.draw_idle()

    def update(_=None):
        if file_state["is_running"]:
            return
        file_state["trace_points"] = []
        q = [s.val for s in sliders]
        render_robot(q, sync_textboxes=True)

    def run_encoder_file(_event):
        file_path = file_state["path"].strip() or txt_file_path.text.strip()
        if not file_path:
            show_warning("Please choose a .txt file first.")
            return

        if not os.path.isfile(file_path):
            show_warning("File not found.")
            return

        try:
            exported_file_path, _ = export_angle_sets_to_txt(file_path)
            results = forward_kinematics_from_encoder_file_all(file_path)
        except Exception as exc:
            show_warning(format_mapping_error(exc, "Teach"))
            return

        file_state["path"] = file_path
        file_state["is_running"] = True
        file_state["trace_points"] = []

        try:
            for idx, item in enumerate(results):
                target_q = np.array(item["angles"], dtype=float)
                if idx == 0:
                    file_state["trace_points"] = [item["position"].copy()]
                else:
                    file_state["trace_points"].extend(
                        sample_trace_positions(file_state["current_q"], target_q, num_samples=25)
                    )
                render_robot(item["angles"], sync_textboxes=False)
                plt.pause(0.8)
        finally:
            file_state["is_running"] = False

        show_info(f"Run completed.\nConverted angle file saved to:\n{exported_file_path}")

    def run_line_mapping(_event):
        file_path = file_state["path"].strip() or txt_file_path.text.strip()
        if not file_path:
            show_warning("Please choose a .txt file first.")
            return

        if not os.path.isfile(file_path):
            show_warning("File not found.")
            return

        try:
            mapping = export_line_mapping_from_encoder_file(file_path)
        except Exception as exc:
            show_warning(format_mapping_error(exc, "Line Map"))
            return

        file_state["path"] = file_path
        file_state["is_running"] = True
        file_state["trace_points"] = []

        try:
            for idx, angle_set in enumerate(mapping["mapped_angles_from_pulses"]):
                if idx == 0:
                    file_state["trace_points"] = [mapping["mapped_actual_positions"][0].copy()]
                else:
                    file_state["trace_points"].append(mapping["mapped_actual_positions"][idx].copy())
                render_robot(angle_set, sync_textboxes=False)
                plt.pause(0.08)
        finally:
            file_state["is_running"] = False

        show_info(
            "Line mapping completed.\n"
            f"Mapped pulses saved to:\n{mapping['mapped_pulses_output_path']}\n\n"
            f"Mapped angles saved to:\n{mapping['mapped_angles_output_path']}\n\n"
            f"Mapped XYZ saved to:\n{mapping['mapped_xyz_output_path']}"
        )

    def run_xyz_mapping(_event):
        file_path = file_state["path"].strip() or txt_file_path.text.strip()
        if not file_path:
            show_warning("Please choose a .txt file first.")
            return

        if not os.path.isfile(file_path):
            show_warning("File not found.")
            return

        try:
            mapping = export_line_mapping_from_xyz_file(
                file_path,
                reference_angles=file_state["current_q"],
            )
        except Exception as exc:
            show_warning(format_mapping_error(exc, "XYZ"))
            return

        file_state["path"] = file_path
        file_state["is_running"] = True
        file_state["trace_points"] = []

        try:
            for idx, angle_set in enumerate(mapping["mapped_angles_from_pulses"]):
                if idx == 0:
                    file_state["trace_points"] = [mapping["mapped_actual_positions"][0].copy()]
                else:
                    file_state["trace_points"].append(mapping["mapped_actual_positions"][idx].copy())
                render_robot(angle_set, sync_textboxes=False)
                plt.pause(0.08)
        finally:
            file_state["is_running"] = False

        show_info(
            "XYZ mapping completed.\n"
            f"Mapped pulses saved to:\n{mapping['mapped_pulses_output_path']}\n\n"
            f"Mapped angles saved to:\n{mapping['mapped_angles_output_path']}\n\n"
            f"Mapped XYZ saved to:\n{mapping['mapped_xyz_output_path']}"
        )

    def parse_circle_parameters():
        try:
            center_x = float(circle_param_boxes["center_x"].text.strip())
            center_y = float(circle_param_boxes["center_y"].text.strip())
            z_height = float(circle_param_boxes["z_height"].text.strip())
            radius = float(circle_param_boxes["radius"].text.strip())
        except ValueError as exc:
            raise ValueError("Circle parameters must be numeric.") from exc

        num_samples = int(CIRCLE_DEFAULT_SAMPLES)
        if radius <= 0.0:
            raise ValueError("Circle radius must be greater than 0.")
        if num_samples < 8:
            raise ValueError("Circle samples must be at least 8.")

        return center_x, center_y, z_height, radius, num_samples

    def run_circle_mapping(_event):
        file_path = file_state["path"].strip() or txt_file_path.text.strip()
        if not file_path:
            show_warning("Please choose a circle pose .txt file first.")
            return

        if not os.path.isfile(file_path):
            show_warning("File not found.")
            return

        try:
            reference_pulses = load_encoder_pulses_from_txt(file_path)
            reference_angles = encoder_pulses_to_angles(reference_pulses)
            reference_pose = forward_kinematics_from_joint_angles(reference_angles)
            center_x, center_y, z_height = reference_pose["position"]
            mapping = export_circle_mapping_xy(
                center_x,
                center_y,
                z_height,
                CIRCLE_RADIUS,
                CIRCLE_DEFAULT_SAMPLES,
                reference_angles=reference_angles,
                fixed_rotation=reference_pose["rotation"],
            )
        except Exception as exc:
            show_warning(format_mapping_error(exc, "Circle"))
            return

        file_state["is_running"] = True
        file_state["trace_points"] = []

        try:
            for idx, angle_set in enumerate(mapping["mapped_angles_from_pulses"]):
                if idx == 0:
                    file_state["trace_points"] = [mapping["mapped_actual_positions"][0].copy()]
                else:
                    file_state["trace_points"].append(mapping["mapped_actual_positions"][idx].copy())
                render_robot(angle_set, sync_textboxes=False)
                plt.pause(0.08)
        finally:
            file_state["is_running"] = False

        show_info(
            "Circle mapping completed.\n"
            f"Mapped pulses saved to:\n{mapping['mapped_pulses_output_path']}\n\n"
            f"Mapped angles saved to:\n{mapping['mapped_angles_output_path']}\n\n"
            f"Mapped XYZ saved to:\n{mapping['mapped_xyz_output_path']}"
        )

    def on_primary_action(event):
        if file_state["mode"] == "teach":
            run_encoder_file(event)
        elif file_state["mode"] == "line":
            run_line_mapping(event)
        elif file_state["mode"] == "xyz":
            run_xyz_mapping(event)
        else:
            run_circle_mapping(event)

    for s in sliders:
        s.on_changed(update)

    def make_textbox_handler(idx):
        def handler(text):
            try:
                val = float(text)
                val = np.clip(val, joint_limits[idx, 0], joint_limits[idx, 1])
                sliders[idx].set_val(val)
            except ValueError:
                textboxes[idx].set_val(f"{sliders[idx].val:.1f}")
        return handler

    for i, t in enumerate(textboxes):
        t.on_submit(make_textbox_handler(i))

    def on_reset(_event):
        for s in sliders:
            s.reset()
        update()

    def on_workspace(_event):
        try:
            run_workspace_preview(samples_per_joint=WORKSPACE_SAMPLES_PER_JOINT)
            plt.show(block=False)
        except Exception as exc:
            show_warning(str(exc))

    btn_file.on_clicked(select_txt_file)
    btn_primary.on_clicked(on_primary_action)
    btn_reset.on_clicked(on_reset)
    btn_workspace.on_clicked(on_workspace)
    btn_mode_teach.on_clicked(lambda _event: set_mode("teach"))
    btn_mode_line.on_clicked(lambda _event: set_mode("line"))
    btn_mode_xyz.on_clicked(lambda _event: set_mode("xyz"))
    btn_mode_circle.on_clicked(lambda _event: set_mode("circle"))
    txt_file_path.on_submit(on_file_path_submit)

    set_mode("teach")
    update()
    plt.show()

def test_inverse_kinematics():
    """Test IK with multiple cases."""

    def run_test(name, q_input):
        """Helper function to run one test case"""
        print(f"\n--- {name} ---")
        T = Forward_Kinematics(*q_input)
        p = T[:3, 3]
        R = T[:3, :3]
        print(f"Input angles (deg): {[round(float(x), 1) for x in q_input]}")

        result = Inverse_Kinematics(p, R, current_angles=q_input, angle_tolerance=2.0, debug=False)

        if result['status'] in ['OK', 'SUCCESS']:
            solution = result['solution']
            print("IK succeeded.")
            print(f"Recovered IK angles (deg): {[round(float(x), 1) for x in solution]}")
            errors = [abs(normalize_angle(solution[i] - q_input[i])) for i in range(6)]
            max_err = max(errors)
            status = "PASS" if max_err < 1.0 else "FAIL"
            print(f"Per-joint error (deg): {[round(float(e), 3) for e in errors]}")
            print(f"Result: {result['status']} ({result.get('num_solutions', 0)} solutions) -> {status}")
        else:
            print(f"Result: {result['status']}, {result.get('num_solutions', 0)} solutions")
        return result

    print("=" * 70)
    print("TEST INVERSE KINEMATICS")
    print("=" * 70)

    run_test("Test 1: Reset Position", [0, 90, 0, 0, 0, 0])
    run_test("Test 2: Custom Angles", [30, 30, -30, 20, 35, -15])
    run_test("Test 3: Near Limits", [60, -60, 120, -70, 80, 70])
    run_test("Test 4: Rotated EE", [45, 45, -60, 30, -45, 45])
    run_test("Test 5: Negative Angles", [-45, -30, -60, -25, -40, -50])
    run_test("Test 6: Singularity (theta5=0)", [0, 30, -60, 15, 0, 30])
    run_test("Test 7: Random Config", [72, -45, 90, 45, -72, -35])
    run_test("Test 8: Vertical Reach", [0, 60, -30, 0, 0, 0])
    run_test("Test 9: Large Angles", [120, 100, -100, 70, 70, 70])
    run_test("Test 10: Mixed Signs", [-60, 45, -90, 30, -45, 60])

    print("\n" + "=" * 70)
    print("TEST COMPLETED")
    print("=" * 70)

if __name__ == "__main__":
    # Select test mode or UI mode.
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "test":
        test_inverse_kinematics()
    elif len(sys.argv) > 1 and sys.argv[1] == "workspace":
        samples = int(sys.argv[2]) if len(sys.argv) > 2 else WORKSPACE_SAMPLES_PER_JOINT
        run_workspace_preview(samples_per_joint=samples)
        plt.show()
    else:
        run_slider_ui()



