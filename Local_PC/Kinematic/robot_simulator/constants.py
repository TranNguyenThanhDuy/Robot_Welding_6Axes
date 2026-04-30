import numpy as np


# Robot DH parameters (mm)
d0 = 230
L1 = 50
L2 = 259
L3 = 66
d1 = 180
d2 = 140

# Joint angle limits (degrees)
POS = 90
NEG = -90

JOINT_LIMITS_DEG = np.array(
    [
        [-180.0, 180.0],
        [-100.0, 100.0],
        [-135.0, 135.0],
        [-180.0, 180.0],
        [-90.0, 90.0],
        [-180.0, 180.0],
    ],
    dtype=float,
)

THETA2_OFFSET = 90

ENCODER_PPR = 10000
GEAR_RATIO_J1 = 40
GEAR_RATIO_J2 = 75
GEAR_RATIO_J3 = 5
GEAR_RATIO_J4 = 10
GEAR_RATIO_J5 = 50
GEAR_RATIO_J6 = 1.0

LINE_MAPPING_SAMPLES_PER_SEGMENT = 10

TEACH_OUTPUT_DIR = "teach_outputs"
LINE_MAPPING_OUTPUT_DIR = "line_mapping_outputs"
XYZ_OUTPUT_DIR = "xyz_outputs"
