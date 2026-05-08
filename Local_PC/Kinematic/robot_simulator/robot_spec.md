# Robot Motion Feature Specification

## 1. Scope
This document describes the current behavior of `analytical.py`.

The simulator currently supports these modes:

1. `Teach`
2. `Line Map`
3. `XYZ`
4. `Workspace` preview

The purpose of this spec is to keep the implementation and documentation aligned.

---

## 2. Shared Robot Parameters

### 2.1 Kinematic Model
The robot model uses DH-style transformation matrices and the following link parameters in millimeters:

```txt
d0 = 230
L1 = 50
L2 = 259
L3 = 66
d1 = 255
d2 = 175
```

`THETA2_OFFSET = 90 deg` is applied internally when running FK/IK from display or pulse angles.

### 2.2 Encoder Conversion
Encoder conversion uses:

```txt
angle_deg = pulse * 360 / (ENCODER_PPR * gear_ratio)
pulse = angle_deg * ENCODER_PPR * gear_ratio / 360
```

Current parameters:

```txt
ENCODER_PPR = 10000
GEAR_RATIO_J1 = 40
GEAR_RATIO_J2 = 75
GEAR_RATIO_J3 = 5
GEAR_RATIO_J4 = 10
GEAR_RATIO_J5 = 50
GEAR_RATIO_J6 = 1.0
```

These values affect:

- Teach Playback
- Encoder-based Line Map
- XYZ pulse export
- FK verification after pulse rounding

### 2.3 Joint Limits
Current joint limits:

```txt
J1: -180 to 180 deg
J2: -135 to 135 deg
J3: -135 to 135 deg
J4: -180 to 180 deg
J5:  -90 to  90 deg
J6: -180 to 180 deg
```

These limits are used by:

- Manual sliders
- Textbox input
- IK solution filtering
- Fixed waypoint validation

### 2.4 Interpolation Density
Cartesian line interpolation uses:

```txt
LINE_MAPPING_SAMPLES_PER_SEGMENT = 10
```

For each segment between two consecutive waypoints, the code creates `10` samples including the segment end point. The segment start point is already present from the previous waypoint.

### 2.5 IK Continuity And Wrist Stabilization
The IK solver can produce multiple valid joint solutions for the same target pose. The code selects the solution closest to the previous joint state using weighted continuity:

```txt
IK_CONTINUITY_WEIGHTS = [1, 1, 1, 4, 2, 6]
```

This gives higher priority to keeping `J4` and especially `J6` continuous. The goal is to reduce sudden wrist rotation during welding paths.

When `J5` is near a wrist singularity:

```txt
abs(J5) < 1 deg
```

additional handling attempts to preserve the previous `J4` and compensate with `J6`, but only if FK verification confirms the pose is still valid.

This behavior applies to modes that use IK:

- `Line Map`
- `XYZ`

It does not apply to `Teach`, because Teach does not use IK.

---

## 3. Teach Mode

### 3.1 Purpose
`Teach` replays recorded encoder pulse data.

### 3.2 Input Format
Each non-empty line must contain exactly six pulse values:

```txt
J1 J2 J3 J4 J5 J6
```

### 3.3 Processing Flow
For each line:

1. Read six encoder pulse values.
2. Convert pulses to joint angles.
3. Run Forward Kinematics.
4. Update the robot preview.
5. Update end-effector position and orientation text.

Teach does not remap the trajectory and does not use IK.

### 3.4 Output
Teach exports converted joint angles to:

```txt
teach_outputs/<source>_angles.txt
```

---

## 4. Line Map Mode From Encoder Pulses

### 4.1 Purpose
`Line Map` uses encoder pulse rows as source waypoints, converts them to Cartesian waypoints, then creates a new straight-line Cartesian trajectory between consecutive waypoints.

### 4.2 Input Format
Each non-empty line must contain exactly six pulse values:

```txt
J1 J2 J3 J4 J5 J6
```

Each row is treated as a source waypoint, not as a joint-space trajectory that must be replayed directly.

### 4.3 Processing Flow
1. Read all encoder pulse waypoints.
2. Convert pulses to joint angles.
3. Run FK to get Cartesian waypoint positions and rotations.
4. Generate straight Cartesian segments between consecutive waypoints.
5. Interpolate intermediate XYZ points on each segment.
6. Interpolate orientation by SLERP between waypoint rotations.
7. Lock original waypoints so IK cannot move them.
8. Run IK only for intermediate points.
9. Convert generated angles to encoder pulses.
10. Overwrite original waypoint pulse rows back into the output to prevent endpoint drift.
11. Convert output pulses back to angles.
12. Run FK again to produce verified actual XYZ positions.
13. Preview the robot using the verified actual path.
14. Export pulse, angle, and XYZ files.

### 4.4 Orientation Strategy
The default orientation strategy is:

```txt
segment_slerp
```

If IK fails with SLERP orientation, the code falls back to:

```txt
segment_start_fallback
```

In fallback mode, intermediate points keep the segment-start orientation and the segment end keeps the original end-waypoint orientation.

The actual strategy is stored in:

```txt
rotation_strategy
```

### 4.5 Output
Line Map exports:

```txt
line_mapping_outputs/<source>_line_mapped.txt
line_mapping_outputs/<source>_line_angles.txt
line_mapping_outputs/<source>_line_xyz.txt
```

`_line_xyz.txt` is generated from FK verification after output pulse rounding.

---

## 5. XYZ Mode

### 5.1 Purpose
`XYZ` creates a Cartesian line trajectory directly from input XYZ waypoints.

### 5.2 Input Format
Each non-empty line must contain exactly three numeric values:

```txt
X Y Z
```

### 5.3 Processing Flow
1. Read all XYZ waypoints.
2. Generate straight Cartesian segments between consecutive waypoints.
3. Interpolate intermediate XYZ points on each segment.
4. Get the current robot pose from the current slider/joint state.
5. Use the current end-effector rotation as a fixed orientation for the whole XYZ path.
6. Run IK for the generated Cartesian points.
7. Convert generated angles to encoder pulses.
8. Convert output pulses back to angles.
9. Run FK again to produce verified actual XYZ positions.
10. Preview the robot using the verified actual path.
11. Export pulse, angle, and XYZ files.

### 5.4 Orientation Rule
The XYZ input file currently contains position only. It does not include RPY or a rotation matrix.

Therefore, XYZ mode uses a fixed orientation taken from the robot pose at the moment the mode is started.

This means the starting robot posture is important. If the welding tool is tilted too much before running XYZ mode, the entire generated path keeps that same tilted orientation.

### 5.5 Output
XYZ exports:

```txt
xyz_outputs/<source>_xyz_pulses.txt
xyz_outputs/<source>_xyz_angles.txt
xyz_outputs/<source>_xyz_actual.txt
```

`_xyz_actual.txt` is generated from FK verification after output pulse rounding.

---

## 6. Actual Path Verification

For generated modes (`Line Map` and `XYZ`), the code distinguishes:

```txt
desired path = ideal Cartesian interpolation target
actual path  = FK result after angle -> pulse -> angle conversion
```

The UI preview uses the verified actual path:

```txt
mapped_actual_positions
```

This is closer to what the real robot receives because pulse rounding has already been applied.

---

## 7. UI Behavior

The UI uses mode buttons, not real tab widgets.

Current mode buttons:

```txt
Teach
Line Map
XYZ
```

Common UI elements:

- File path textbox
- Browse button
- Main action button: `Run`, `Map`, or `XYZ`
- Manual sliders for `J1..J6`
- Textboxes for `J1..J6`
- End-effector position display
- End-effector RPY display
- Local end-effector axis display
- Warning popup
- Workspace preview button

The local end-effector axes are displayed as:

```txt
X axis: red
Y axis: green
Z axis: blue
```

---

## 8. Workspace Preview

The code includes a workspace cloud preview.

Current parameters:

```txt
WORKSPACE_SAMPLES_PER_JOINT = 20
WORKSPACE_POINT_SIZE = 10
WORKSPACE_SAMPLED_JOINTS = (0, 1, 2)
WORKSPACE_FIXED_ANGLES = [0, 0, 0, 0, 0, 0]
```

The workspace preview currently samples only the main positioning joints:

```txt
J1, J2, J3
```

`J4, J5, J6` are held fixed. This produces a cleaner positioning workspace cloud, but it is not the complete 6D TCP workspace.

It can be opened from the UI using the `Workspace` button, or from the command line:

```powershell
python analytical.py workspace
python analytical.py workspace 20
```

---

## 9. Error Handling

The code uses `MappingIKError` for clearer IK failure reporting.

The UI formats IK errors with:

- Mode name
- Point index
- XYZ target
- IK status
- Suggested hint

Typical statuses:

```txt
OOW      = target is outside workspace
FAIL     = no complete valid IK solution
NO_MATCH = IK found a solution but it is far from the current joint state
```

---

## 10. Current Limitations

Current limitations:

- The UI uses mode buttons instead of true tab widgets.
- XYZ input does not support explicit `X Y Z R P Y` yet.
- There is no separate motion-smoothness report file.
- There is no automatic warning threshold for Cartesian deviation between desired path and verified actual path.
- Workspace preview is a positioning workspace, not a full orientation-aware welding workspace.
- Real robot accuracy still depends on DH parameters, home offsets, gear ratios, motor direction, TCP/tool length, controller timing, and acceleration settings.

---

## 11. Mode Summary

### Teach
```txt
Original encoder pulses -> joint angles -> FK -> replay original motion
```

### Line Map
```txt
Encoder pulse waypoints -> FK Cartesian waypoints -> straight-line interpolation -> IK -> new pulses
```

### XYZ
```txt
XYZ waypoints -> straight-line interpolation -> fixed-orientation IK -> new pulses
```

### Workspace
```txt
Sample J1-J3 within joint limits -> FK -> point cloud preview
```
