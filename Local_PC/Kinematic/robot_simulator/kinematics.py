import numpy as np

from .constants import IK_CONTINUITY_WEIGHTS, L1, L2, L3, NEG, POS, THETA2_OFFSET, d0, d1, d2
from .conversions import are_joint_angles_within_limits, normalize_angle


def T_0_1(a1, alpha1, d1_, theta1_):
    return np.array(
        [
            [np.cos(np.radians(theta1_)), 0, np.sin(np.radians(theta1_)), a1 * np.cos(np.radians(theta1_))],
            [np.sin(np.radians(theta1_)), 0, -np.cos(np.radians(theta1_)), a1 * np.sin(np.radians(theta1_))],
            [0, 1, 0, d1_],
            [0, 0, 0, 1],
        ]
    )


def T_1_2(a2, alpha2, d2_, theta2_):
    return np.array(
        [
            [np.cos(np.radians(theta2_)), -np.sin(np.radians(theta2_)), 0, a2 * np.cos(np.radians(theta2_))],
            [np.sin(np.radians(theta2_)), np.cos(np.radians(theta2_)), 0, a2 * np.sin(np.radians(theta2_))],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def T_2_3(a3, alpha3, d3_, theta3_):
    return np.array(
        [
            [np.cos(np.radians(theta3_)), 0, np.sin(np.radians(theta3_)), a3 * np.cos(np.radians(theta3_))],
            [np.sin(np.radians(theta3_)), 0, -np.cos(np.radians(theta3_)), a3 * np.sin(np.radians(theta3_))],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ]
    )


def T_3_4(a4, alpha4, d4_, theta4_):
    return np.array(
        [
            [np.cos(np.radians(theta4_)), 0, -np.sin(np.radians(theta4_)), 0],
            [np.sin(np.radians(theta4_)), 0, np.cos(np.radians(theta4_)), 0],
            [0, -1, 0, d4_],
            [0, 0, 0, 1],
        ]
    )


def T_4_5(a5, alpha5, d5_, theta5_):
    return np.array(
        [
            [np.cos(np.radians(theta5_)), 0, np.sin(np.radians(theta5_)), 0],
            [np.sin(np.radians(theta5_)), 0, -np.cos(np.radians(theta5_)), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ]
    )


def T_5_6(a6, alpha6, d6_, theta6_):
    return np.array(
        [
            [np.cos(np.radians(theta6_)), -np.sin(np.radians(theta6_)), 0, 0],
            [np.sin(np.radians(theta6_)), np.cos(np.radians(theta6_)), 0, 0],
            [0, 0, 1, d6_],
            [0, 0, 0, 1],
        ]
    )


def Forward_Kinematics(theta1_FK_, theta2_FK_, theta3_FK_, theta4_FK_, theta5_FK_, theta6_FK_):
    T_0_1_Matrix = T_0_1(L1, POS, d0, theta1_FK_)
    T_1_2_Matrix = T_1_2(L2, 0, 0, theta2_FK_)
    T_2_3_Matrix = T_2_3(L3, POS, 0, theta3_FK_)
    T_3_4_Matrix = T_3_4(0, NEG, d1, theta4_FK_)
    T_4_5_Matrix = T_4_5(0, POS, 0, theta5_FK_)
    T_5_6_Matrix = T_5_6(0, 0, d2, theta6_FK_)
    return T_0_1_Matrix @ T_1_2_Matrix @ T_2_3_Matrix @ T_3_4_Matrix @ T_4_5_Matrix @ T_5_6_Matrix


def R_0_3(theta1, theta2, theta3):
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)
    T23 = T_2_3(L3, POS, 0, theta3)
    return (T01 @ T12 @ T23)[:3, :3]


def R_0_3_Inverse(theta1, theta2, theta3):
    return R_0_3(theta1, theta2, theta3).T


def Check_T_0_3(theta1, theta2, theta3, Wx, Wy, Wz, tolerance=5.0, debug=False):
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)
    T23 = T_2_3(L3, POS, 0, theta3)
    W_calc = (T01 @ T12 @ T23)[:3, 3]
    error = np.sqrt((W_calc[0] - Wx) ** 2 + (W_calc[1] - Wy) ** 2 + (W_calc[2] - Wz) ** 2)
    if debug:
        print(f"  W_expected=[{Wx:.2f}, {Wy:.2f}, {Wz:.2f}], W_calc=[{W_calc[0]:.2f}, {W_calc[1]:.2f}, {W_calc[2]:.2f}], error={error:.2f}")
    return error < tolerance


def Check_T_0_4(theta1, theta2, theta3, Wx, Wy, Wz, tolerance=5.0, debug=False):
    T01 = T_0_1(L1, POS, d0, theta1)
    T12 = T_1_2(L2, 0, 0, theta2)
    T23 = T_2_3(L3, POS, 0, theta3)
    T34 = T_3_4(0, NEG, d1, 0)
    W_calc = (T01 @ T12 @ T23 @ T34)[:3, 3]
    error = np.sqrt((W_calc[0] - Wx) ** 2 + (W_calc[1] - Wy) ** 2 + (W_calc[2] - Wz) ** 2)
    if debug:
        print(f"  W_expected=[{Wx:.2f}, {Wy:.2f}, {Wz:.2f}], W_calc=[{W_calc[0]:.2f}, {W_calc[1]:.2f}, {W_calc[2]:.2f}], error={error:.2f}")
    return error < tolerance


def Check_T_0_6(EE_pos, R_0_6, theta1, theta2, theta3, theta4, theta5, theta6, pos_tol=1.0, rot_tol=0.01):
    T06 = Forward_Kinematics(theta1, theta2, theta3, theta4, theta5, theta6)
    pos_error = np.linalg.norm(T06[:3, 3] - EE_pos)
    rot_error = np.linalg.norm(T06[:3, :3] - R_0_6)
    return pos_error < pos_tol and rot_error < rot_tol


def Inverse_Kinematics(EE_pos, R_0_6, current_angles=None, angle_tolerance=5.0, debug=False):
    EE_pos = np.array(EE_pos).flatten()
    R_0_6 = np.array(R_0_6).reshape(3, 3)
    W_pos = EE_pos - d2 * R_0_6[:, 2]
    Wx, Wy, Wz = W_pos

    theta1_IK = np.degrees(np.arctan2(Wy, Wx))
    r_xy = np.sqrt(Wx ** 2 + Wy ** 2) - L1
    r_z = Wz - d0
    D = np.sqrt(r_xy ** 2 + r_z ** 2)

    TS_Cos_Gamma = -(L2 ** 2 + L3 ** 2 + d1 ** 2) + (r_xy ** 2 + r_z ** 2)
    MS_Cos_Gamma = 2 * L2 * np.sqrt(L3 ** 2 + d1 ** 2)
    Cos_Gamma = TS_Cos_Gamma / MS_Cos_Gamma
    if abs(Cos_Gamma) > 1:
        return {"status": "OOW", "solution": None, "num_solutions": 0}

    Sin_Gamma = np.sqrt(1 - Cos_Gamma ** 2)
    Gamma_1 = np.degrees(np.arctan2(Sin_Gamma, Cos_Gamma))
    Gamma_2 = np.degrees(np.arctan2(-Sin_Gamma, Cos_Gamma))
    Beta = np.degrees(np.arctan2(d1, L3))
    theta3_IK_1 = Gamma_1 + Beta
    theta3_IK_2 = Gamma_2 + Beta

    TS_Cos_Phi = r_xy ** 2 + r_z ** 2 + L2 ** 2 - (L3 ** 2 + d1 ** 2)
    MS_Cos_Phi = 2 * D * L2
    Cos_Phi = TS_Cos_Phi / MS_Cos_Phi
    Sin_Phi = np.sqrt(1 - Cos_Phi ** 2)
    Phi_1 = np.degrees(np.arctan2(Sin_Phi, Cos_Phi))
    Phi_2 = np.degrees(np.arctan2(-Sin_Phi, Cos_Phi))
    Sigma = np.degrees(np.arctan2(r_z, r_xy))
    theta2_IK_1 = normalize_angle(Sigma - Phi_1)
    theta2_IK_2 = normalize_angle(Sigma - Phi_2)

    valid_solutions_123 = []
    candidates = [
        (theta1_IK, theta2_IK_1, theta3_IK_1),
        (theta1_IK, theta2_IK_1, theta3_IK_2),
        (theta1_IK, theta2_IK_2, theta3_IK_1),
        (theta1_IK, theta2_IK_2, theta3_IK_2),
    ]
    for t1, t2, t3 in candidates:
        if Check_T_0_4(t1, t2, t3, Wx, Wy, Wz, tolerance=5.0, debug=False):
            valid_solutions_123.append((t1, t2, t3))
    if not valid_solutions_123:
        return {"status": "FAIL", "solution": None, "all_solutions": [], "num_solutions": 0}

    all_complete_solutions = []
    for theta1_IK, theta2_IK, theta3_IK in valid_solutions_123:
        R_3_6 = R_0_3_Inverse(theta1_IK, theta2_IK, theta3_IK) @ R_0_6
        r13, r23, r33, r31, r32 = R_3_6[0, 2], R_3_6[1, 2], R_3_6[2, 2], R_3_6[2, 0], R_3_6[2, 1]
        Cos_Theta5 = r33
        Sin_Theta5 = np.sqrt(max(0, 1 - Cos_Theta5 ** 2))
        theta5_candidates = [
            np.degrees(np.arctan2(Sin_Theta5, Cos_Theta5)),
            np.degrees(np.arctan2(-Sin_Theta5, Cos_Theta5)),
        ]
        is_singularity = abs(Sin_Theta5) < 0.001

        if is_singularity and current_angles is not None:
            theta4_IK = current_angles[3]
            theta6_IK = np.degrees(np.arctan2(r32, r31)) - theta4_IK
            if theta6_IK >= 90:
                theta6_IK -= 180
            elif theta6_IK <= -90:
                theta6_IK += 180
        else:
            theta4_IK = np.degrees(np.arctan2(r23, r13))
            theta6_IK = np.degrees(np.arctan2(r32, -r31))
            if theta4_IK >= 90:
                theta4_IK -= 180
            elif theta4_IK <= -90:
                theta4_IK += 180
            if theta6_IK >= 90:
                theta6_IK -= 180
            elif theta6_IK <= -90:
                theta6_IK += 180

        for theta5_IK in theta5_candidates:
            candidate_solutions = []
            for theta4_offset in (0.0, 180.0, -180.0):
                for theta5_candidate in (theta5_IK, -theta5_IK):
                    for theta6_offset in (0.0, 180.0, -180.0):
                        candidate_solutions.append(
                            [
                                theta1_IK,
                                theta2_IK,
                                theta3_IK,
                                normalize_angle(theta4_IK + theta4_offset),
                                theta5_candidate,
                                normalize_angle(theta6_IK + theta6_offset),
                            ]
                        )

            if current_angles is not None and abs(theta5_IK) < 1.0:
                preserved_theta4 = float(current_angles[3])
                preserved_theta6 = normalize_angle(theta4_IK + theta6_IK - preserved_theta4)
                candidate_solutions.append(
                    [
                        theta1_IK,
                        theta2_IK,
                        theta3_IK,
                        preserved_theta4,
                        theta5_IK,
                        preserved_theta6,
                    ]
                )

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

    if not all_complete_solutions:
        return {"status": "FAIL", "solution": None, "all_solutions": [], "num_solutions": 0}

    if current_angles is not None:
        current_angles = np.array(current_angles).flatten()
        min_error = float("inf")
        best_solution = all_complete_solutions[0]
        for sol in all_complete_solutions:
            error = 0
            for i in range(6):
                diff = abs(sol[i] - current_angles[i])
                if diff > 180:
                    diff = 360 - diff
                error += IK_CONTINUITY_WEIGHTS[i] * diff
            if error < min_error:
                min_error = error
                best_solution = sol

        max_individual_error = 0
        for i in range(6):
            diff = abs(best_solution[i] - current_angles[i])
            if diff > 180:
                diff = 360 - diff
            max_individual_error = max(max_individual_error, diff)

        if max_individual_error > angle_tolerance:
            return {
                "status": "NO_MATCH",
                "solution": best_solution,
                "all_solutions": all_complete_solutions,
                "num_solutions": len(all_complete_solutions),
                "error": max_individual_error,
            }
    else:
        best_solution = all_complete_solutions[0]

    best_solution = [0.0 if abs(x) < 1e-10 else x for x in best_solution]
    all_complete_solutions = [[0.0 if abs(x) < 1e-10 else x for x in sol] for sol in all_complete_solutions]
    return {
        "status": "OK",
        "solution": best_solution,
        "all_solutions": all_complete_solutions,
        "num_solutions": len(all_complete_solutions),
    }


def forward_points(theta1_, theta2_, theta3_, theta4_, theta5_, theta6_, use_offset=True):
    if use_offset:
        theta2_ = theta2_ + THETA2_OFFSET

    transforms = [
        T_0_1(L1, POS, d0, theta1_),
        T_1_2(L2, 0, 0, theta2_),
        T_2_3(L3, POS, 0, theta3_),
        T_3_4(0, NEG, d1, theta4_),
        T_4_5(0, POS, 0, theta5_),
        T_5_6(0, 0, d2, theta6_),
    ]

    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    Ts = []
    for Ti in transforms:
        T = T @ Ti
        Ts.append(T.copy())
        pts.append(T[:3, 3].copy())
    return np.array(pts), Ts


def forward_kinematics_from_joint_angles(angles):
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
