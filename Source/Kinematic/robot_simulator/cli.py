import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider, TextBox

from .constants import THETA2_OFFSET
from .conversions import MappingIKError, clamp_joint_angles, get_joint_angle_limits, normalize_angle
from .io_utils import export_angle_sets_to_txt, forward_kinematics_from_encoder_file_all
from .kinematics import Forward_Kinematics, Inverse_Kinematics, forward_points
from .trajectory import export_line_mapping_from_encoder_file, export_line_mapping_from_xyz_file
from .visualization import plot_robot, sample_trace_positions


def run_slider_ui():
    from tkinter import Tk, filedialog, messagebox

    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title("6-Axis Robot Simulator")
    robot_panel = [0.02, 0.16, 0.52, 0.78]
    controls_panel = [0.62, 0.08, 0.35, 0.84]
    ax_robot = fig.add_axes(robot_panel, projection="3d")
    lim = 700
    base = (0.0, 0.0, 0.0)

    ax_header = fig.add_axes([controls_panel[0], 0.88, controls_panel[2], 0.05])
    ax_header.axis("off")
    ax_header.text(0.0, 0.5, "Joint Controls", fontsize=13, fontweight="bold", va="center")

    ax_mode_teach = fig.add_axes([controls_panel[0], 0.84, 0.10, 0.034])
    btn_mode_teach = Button(ax_mode_teach, "Teach")
    ax_mode_line = fig.add_axes([controls_panel[0] + 0.12, 0.84, 0.10, 0.034])
    btn_mode_line = Button(ax_mode_line, "Line Map")
    ax_mode_xyz = fig.add_axes([controls_panel[0] + 0.24, 0.84, 0.10, 0.034])
    btn_mode_xyz = Button(ax_mode_xyz, "XYZ")

    ax_file_label = fig.add_axes([controls_panel[0], 0.80, 0.22, 0.03])
    ax_file_label.axis("off")
    txt_file_label = ax_file_label.text(0.0, 0.5, "Trajectory File (.txt)", fontsize=10, fontweight="bold", va="center")
    ax_primary = fig.add_axes([controls_panel[0] + 0.25, 0.802, 0.06, 0.028])
    btn_primary = Button(ax_primary, "Run", hovercolor="#dddddd")

    path_box_w = 0.26
    path_button_w = 0.05
    path_gap = 0.01
    path_h = 0.032
    path_y = 0.765
    ax_file_path = fig.add_axes([controls_panel[0], path_y, path_box_w, path_h])
    txt_file_path = TextBox(ax_file_path, "", initial="")
    txt_file_path.text_disp.set_clip_on(True)
    txt_file_path.text_disp.set_clip_box(ax_file_path.bbox)
    ax_file_button = fig.add_axes([controls_panel[0] + path_box_w + path_gap, path_y, path_button_w, path_h])
    btn_file = Button(ax_file_button, "...")

    ax_text = fig.add_axes([controls_panel[0], 0.68, controls_panel[2], 0.08])
    ax_text.axis("off")
    txt_ee = ax_text.text(
        0.0,
        1.0,
        "Position: X=0.0 mm   Y=0.0 mm   Z=0.0 mm\nOrientation: R=0.0 deg   P=0.0 deg   Y=0.0 deg",
        fontsize=10,
        fontweight="bold",
        va="top",
    )

    names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    sliders = []
    textboxes = []
    joint_limits = get_joint_angle_limits()
    file_state = {"mode": "teach", "path": "", "is_running": False, "current_q": np.zeros(6, dtype=float), "trace_points": []}

    slider_x = controls_panel[0] + 0.03
    slider_y_start = 0.60
    row_gap = 0.075
    slider_w = 0.18
    slider_h = 0.035
    textbox_w = 0.13
    textbox_offset = 0.03

    for i, name in enumerate(names):
        slider_y = slider_y_start - i * row_gap
        ax_s = fig.add_axes([slider_x, slider_y, slider_w, slider_h])
        s = Slider(ax_s, f"{name} (deg)", joint_limits[i, 0], joint_limits[i, 1], valinit=0.0, valstep=0.1)
        s.valtext.set_visible(False)
        sliders.append(s)
        ax_t = fig.add_axes([slider_x + slider_w + textbox_offset, slider_y, textbox_w, slider_h])
        textboxes.append(TextBox(ax_t, "", initial="0.0", textalignment="center"))

    ax_reset = fig.add_axes([controls_panel[0] + 0.09, 0.12, 0.16, 0.045])
    btn_reset = Button(ax_reset, "Reset", hovercolor="#dddddd")

    def choose_txt_file():
        root = Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        file_path = filedialog.askopenfilename(title="Select TXT file", filetypes=[("Text files", "*.txt")])
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
        if mode == "teach":
            txt_file_label.set_text("Trajectory File (.txt)")
            btn_primary.label.set_text("Run")
            btn_mode_teach.color = "#d9ead3"
            btn_mode_line.color = "#f0f0f0"
            btn_mode_xyz.color = "#f0f0f0"
        elif mode == "line":
            txt_file_label.set_text("Waypoint File (.txt)")
            btn_primary.label.set_text("Map")
            btn_mode_teach.color = "#f0f0f0"
            btn_mode_line.color = "#d9ead3"
            btn_mode_xyz.color = "#f0f0f0"
        else:
            txt_file_label.set_text("XYZ File (.txt)")
            btn_primary.label.set_text("XYZ")
            btn_mode_teach.color = "#f0f0f0"
            btn_mode_line.color = "#f0f0f0"
            btn_mode_xyz.color = "#d9ead3"
        fig.canvas.draw_idle()

    def render_robot(q, sync_textboxes=True):
        q = clamp_joint_angles(q)
        file_state["current_q"] = q.copy()
        if sync_textboxes:
            for angle, textbox in zip(q, textboxes):
                textbox.set_val(f"{angle:.1f}")
        pts, _ = forward_points(*q, use_offset=True)
        T06 = Forward_Kinematics(q[0], q[1] + THETA2_OFFSET, q[2], q[3], q[4], q[5])
        plot_robot(pts, ax=ax_robot, title="Robot 6-Axis", lim=lim, base=base, trace_points=file_state["trace_points"], end_effector_rotation=T06[:3, :3])
        p = T06[:3, 3]
        R = T06[:3, :3]
        pitch = np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)))
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
        render_robot([s.val for s in sliders], sync_textboxes=True)

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
                    file_state["trace_points"].extend(sample_trace_positions(file_state["current_q"], target_q, num_samples=25))
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
            mapping = export_line_mapping_from_xyz_file(file_path, reference_angles=file_state["current_q"])
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

    def on_primary_action(event):
        if file_state["mode"] == "teach":
            run_encoder_file(event)
        elif file_state["mode"] == "line":
            run_line_mapping(event)
        else:
            run_xyz_mapping(event)

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

    btn_file.on_clicked(select_txt_file)
    btn_primary.on_clicked(on_primary_action)
    btn_reset.on_clicked(on_reset)
    btn_mode_teach.on_clicked(lambda _event: set_mode("teach"))
    btn_mode_line.on_clicked(lambda _event: set_mode("line"))
    btn_mode_xyz.on_clicked(lambda _event: set_mode("xyz"))
    txt_file_path.on_submit(on_file_path_submit)

    set_mode("teach")
    update()
    plt.show()


def test_inverse_kinematics():
    def run_test(name, q_input):
        print(f"\n--- {name} ---")
        T = Forward_Kinematics(*q_input)
        p = T[:3, 3]
        R = T[:3, :3]
        print(f"Input joint angles (deg): {[round(float(x), 1) for x in q_input]}")
        result = Inverse_Kinematics(p, R, current_angles=q_input, angle_tolerance=2.0, debug=False)
        if result["status"] in ["OK", "SUCCESS"]:
            solution = result["solution"]
            print("IK solved.")
            print(f"Recovered joint angles (deg): {[round(float(x), 1) for x in solution]}")
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


def run_line_mapping_cli(file_path):
    if not file_path:
        print("Missing line-map input file path.")
        return 1
    if not os.path.isfile(file_path):
        print(f"Input file not found: {file_path}")
        return 1

    try:
        mapping = export_line_mapping_from_encoder_file(file_path)
    except Exception as exc:
        print(f"Line mapping failed: {exc}")
        return 1

    print("Line mapping completed.")
    print(f"Mapped pulses: {mapping['mapped_pulses_output_path']}")
    print(f"Mapped angles: {mapping['mapped_angles_output_path']}")
    print(f"Mapped XYZ: {mapping['mapped_xyz_output_path']}")
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] == "test":
        test_inverse_kinematics()
        return 0
    if argv and argv[0] == "line-map":
        return run_line_mapping_cli(argv[1] if len(argv) > 1 else "")

    run_slider_ui()
    return 0
