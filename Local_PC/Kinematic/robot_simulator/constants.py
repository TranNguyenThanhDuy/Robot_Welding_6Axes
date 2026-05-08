import numpy as np


# Robot DH parameters (mm)
d0 = 230
L1 = 50
L2 = 259
L3 = 66
d1 = 255
d2 = 175

# Joint angle limits (degrees)
POS = 90
NEG = -90

JOINT_LIMITS_DEG = np.array(
    [
        [-180.0, 180.0],
        [-135.0, 135.0],
        [-135.0, 135.0],
        [-180.0, 180.0],
        [-90.0, 90.0],
        [-180.0, 180.0],
    ],
    dtype=float,
)

IK_CONTINUITY_WEIGHTS = np.array([1.0, 1.0, 1.0, 4.0, 2.0, 6.0], dtype=float)

THETA2_OFFSET = 90

ENCODER_PPR = 10000
GEAR_RATIO_J1 = 40
GEAR_RATIO_J2 = 75
GEAR_RATIO_J3 = 5
GEAR_RATIO_J4 = 10
GEAR_RATIO_J5 = 50
GEAR_RATIO_J6 = 1.0

LINE_MAPPING_SAMPLES_PER_SEGMENT = 10

WORKSPACE_SAMPLES_PER_JOINT = 20
WORKSPACE_POINT_SIZE = 10
WORKSPACE_SAMPLED_JOINTS = (0, 1, 2)
WORKSPACE_FIXED_ANGLES = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

TEACH_OUTPUT_DIR = "teach_outputs"
LINE_MAPPING_OUTPUT_DIR = "line_mapping_outputs"
XYZ_OUTPUT_DIR = "xyz_outputs"
