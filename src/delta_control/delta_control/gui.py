#!/usr/bin/env python3

import csv
import math
import queue
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from delta_control.kinematics import DeltaIKError, DeltaKinematics2
from delta_control.workspace import calculate_workspace_surface


class DeltaControlNode(Node):
    def __init__(self):
        super().__init__("delta_control_gui")
        self.trajectory_pub = self.create_publisher(
            Float64MultiArray,
            "/kinematic2/joint_trajectory_ros",
            10,
        )
        self.joint_ref_pub = self.create_publisher(
            Float64MultiArray,
            "/kinematic2/joint_ref_ros",
            10,
        )
        self.feedback_xyz = None
        self.feedback_theta = None
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/feedback_xyz",
            self._on_xyz,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/delta_robot/feedback_theta",
            self._on_theta,
            10,
        )

    def _on_xyz(self, msg):
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            self.feedback_xyz = tuple(float(v) for v in msg.data[:3])

    def _on_theta(self, msg):
        if len(msg.data) >= 3 and all(math.isfinite(v) for v in msg.data[:3]):
            self.feedback_theta = tuple(float(v) for v in msg.data[:3])

    def publish_trajectory(self, samples, trajectory_id):
        data = [float(trajectory_id), float(len(samples))]
        for sample in samples:
            data.extend(
                [
                    float(sample["t"]),
                    *[float(v) for v in sample["q"]],
                    *[float(v) for v in sample["q_dot"]],
                    *[float(v) for v in sample["q_ddot"]],
                ]
            )
        msg = Float64MultiArray()
        msg.data = data
        self.trajectory_pub.publish(msg)

    def publish_joint_reference(self, q, q_dot=(0.0, 0.0, 0.0)):
        msg = Float64MultiArray()
        msg.data = [*[float(v) for v in q], *[float(v) for v in q_dot]]
        self.joint_ref_pub.publish(msg)


class DeltaControlApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.solver = DeltaKinematics2()
        self.trajectory_id = 0
        self.last_samples = []
        self.jog_after_id = None
        self.jog_delay_id = None
        self.held_jog_direction = None
        self.jog_stream_active = False
        self.jog_stream_point = None
        self.jog_stream_q = None
        self.jog_stream_model_q = None
        self.jog_stream_period = 0.05
        self.workspace_running = False
        self.workspace_results = queue.Queue()
        self.workspace_divisions = tk.IntVar(value=100)
        self.last_target = (
            self.solver.g.home_x,
            self.solver.g.home_y,
            self.solver.g.home_z,
        )

        self.root.title("Delta 4DOF Control")
        self.root.geometry("1120x760")
        self.root.minsize(980, 660)

        self.step_mm = tk.DoubleVar(value=5.0)
        self.jog_time = tk.DoubleVar(value=0.6)
        self.x_mm = tk.DoubleVar(value=0.0)
        self.y_mm = tk.DoubleVar(value=0.0)
        self.z_mm = tk.DoubleVar(value=-361.867)
        self.duration = tk.DoubleVar(value=3.0)
        self.waypoints = tk.IntVar(value=61)
        self.status = tk.StringVar(value="San sang")
        self.feedback = tk.StringVar(value="Feedback: dang cho Gazebo...")
        self.summary = tk.StringVar(value="Chua co quy dao")

        self._configure_style()
        self._build_ui()
        self.root.bind("<ButtonRelease-1>", self._stop_hold_jog, add="+")
        self.root.after(20, self._spin_ros)
        self.root.after(100, self._refresh_feedback)

    def _configure_style(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("DejaVu Sans", 17, "bold"))
        style.configure("Section.TLabelframe.Label", font=("DejaVu Sans", 11, "bold"))
        style.configure("Jog.TButton", font=("DejaVu Sans", 12, "bold"), padding=10)
        style.configure("Primary.TButton", font=("DejaVu Sans", 10, "bold"), padding=8)

    def _build_ui(self):
        shell = ttk.Frame(self.root, padding=14)
        shell.pack(fill="both", expand=True)

        header = ttk.Frame(shell)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="Delta 4DOF Control", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.status).pack(side="right")

        panels = ttk.Panedwindow(shell, orient="horizontal")
        panels.pack(fill="x")

        jog_panel = ttk.LabelFrame(
            panels,
            text="Bang 1 - Dieu khien bang mui ten",
            style="Section.TLabelframe",
            padding=12,
        )
        target_panel = ttk.LabelFrame(
            panels,
            text="Bang 2 - Toa do va quy hoach quy dao",
            style="Section.TLabelframe",
            padding=12,
        )
        panels.add(jog_panel, weight=1)
        panels.add(target_panel, weight=1)

        self._build_jog_panel(jog_panel)
        self._build_target_panel(target_panel)

        feedback_frame = ttk.Frame(shell)
        feedback_frame.pack(fill="x", pady=(10, 6))
        ttk.Label(feedback_frame, textvariable=self.feedback, foreground="#135d9c").pack(
            side="left"
        )
        ttk.Label(feedback_frame, textvariable=self.summary).pack(side="right")

        table_frame = ttk.LabelFrame(
            shell,
            text="Quy dao da lap",
            style="Section.TLabelframe",
            padding=6,
        )
        table_frame.pack(fill="both", expand=True)

        columns = ("i", "t", "x", "y", "z", "q1", "q2", "q3")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=13)
        labels = {
            "i": "#",
            "t": "t (s)",
            "x": "X (mm)",
            "y": "Y (mm)",
            "z": "Z (mm)",
            "q1": "q1 (deg)",
            "q2": "q2 (deg)",
            "q3": "q3 (deg)",
        }
        widths = {"i": 45, "t": 75, "x": 90, "y": 90, "z": 90, "q1": 90, "q2": 90, "q3": 90}
        for column in columns:
            self.table.heading(column, text=labels[column])
            self.table.column(column, width=widths[column], anchor="center", stretch=True)

        scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _build_jog_panel(self, parent):
        settings = ttk.Frame(parent)
        settings.pack(fill="x", pady=(0, 10))
        ttk.Label(settings, text="Buoc (mm)").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(
            settings,
            from_=0.1,
            to=100.0,
            increment=0.5,
            textvariable=self.step_mm,
            width=9,
        ).grid(row=0, column=1, padx=(6, 16))
        ttk.Label(settings, text="Thoi gian (s)").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(
            settings,
            from_=0.1,
            to=10.0,
            increment=0.1,
            textvariable=self.jog_time,
            width=9,
        ).grid(row=0, column=3, padx=6)

        pad = ttk.Frame(parent)
        pad.pack(pady=4)
        self._create_hold_jog_button(pad, "\u2191  Y+", (0, 1, 0)).grid(
            row=0, column=1, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "\u2190  X-", (-1, 0, 0)).grid(
            row=1, column=0, padx=5, pady=5, sticky="ew"
        )
        ttk.Button(
            pad, text="HOME", style="Primary.TButton", command=self._go_home
        ).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self._create_hold_jog_button(pad, "X+  \u2192", (1, 0, 0)).grid(
            row=1, column=2, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "\u2193  Y-", (0, -1, 0)).grid(
            row=2, column=1, padx=5, pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "Z \u2191", (0, 0, 1)).grid(
            row=0, column=3, padx=(18, 5), pady=5, sticky="ew"
        )
        self._create_hold_jog_button(pad, "Z \u2193", (0, 0, -1)).grid(
            row=2, column=3, padx=(18, 5), pady=5, sticky="ew"
        )

        ttk.Label(
            parent,
            text="Bam mot lan de di mot buoc. Nhan giu de tiep tuc jog, tha chuot de dung.",
            wraplength=430,
        ).pack(anchor="w", pady=(10, 0))

    def _create_hold_jog_button(self, parent, text, direction):
        button = ttk.Button(parent, text=text, style="Jog.TButton")
        button.bind(
            "<ButtonPress-1>",
            lambda _event, value=direction: self._start_hold_jog(value),
        )
        return button

    def _build_target_panel(self, parent):
        grid = ttk.Frame(parent)
        grid.pack(fill="x")

        fields = (
            ("X (mm)", self.x_mm),
            ("Y (mm)", self.y_mm),
            ("Z (mm)", self.z_mm),
            ("Thoi gian T (s)", self.duration),
            ("So waypoint N", self.waypoints),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(grid, text=label).grid(row=row, column=0, padx=4, pady=5, sticky="w")
            ttk.Entry(grid, textvariable=variable, width=18).grid(
                row=row, column=1, padx=4, pady=5, sticky="ew"
            )
        grid.columnconfigure(1, weight=1)

        buttons = ttk.Frame(parent)
        buttons.pack(fill="x", pady=(12, 0))
        ttk.Button(
            buttons,
            text="Lap quy dao",
            command=self._plan_from_fields,
        ).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(
            buttons,
            text="Chay quy dao",
            style="Primary.TButton",
            command=self._execute_from_fields,
        ).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(
            buttons,
            text="Xuat CSV",
            command=self._export_csv,
        ).pack(side="left", fill="x", expand=True, padx=(4, 0))

        self.workspace_button = ttk.Button(
            parent,
            text="RUN WORKSPACE",
            style="Primary.TButton",
            command=self._run_workspace,
        )
        self.workspace_button.pack(fill="x", pady=(10, 0))
        workspace_settings = ttk.Frame(parent)
        workspace_settings.pack(fill="x", pady=(6, 0))
        ttk.Label(workspace_settings, text="Chia workspace N").pack(side="left")
        ttk.Spinbox(
            workspace_settings,
            from_=5,
            to=24,
            increment=1,
            textvariable=self.workspace_divisions,
            width=7,
        ).pack(side="right")

    def _current_point(self):
        if self.node.feedback_xyz is not None:
            return self.node.feedback_xyz
        return self.last_target

    def _target_from_fields(self):
        return (
            float(self.x_mm.get()) / 1000.0,
            float(self.y_mm.get()) / 1000.0,
            float(self.z_mm.get()) / 1000.0,
        )

    def _plan(self, target, duration, count):
        start = self._current_point()
        actual_q = self.node.feedback_theta
        samples = self.solver.generate_joint_trajectory(
            p0=start,
            p1=target,
            duration=float(duration),
            n_waypoints=int(count),
            preferred_start=actual_q,
        )
        if actual_q is not None:
            offsets = tuple(actual_q[i] - samples[0]["q"][i] for i in range(3))
            for sample in samples:
                tau = sample["tau"]
                tau2 = tau * tau
                tau3 = tau2 * tau
                tau4 = tau3 * tau
                tau5 = tau4 * tau
                blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
                blend_dot = (
                    30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
                ) / float(duration)
                sample["q"] = tuple(
                    sample["q"][i] + (1.0 - blend) * offsets[i]
                    for i in range(3)
                )
                sample["q_dot"] = tuple(
                    sample["q_dot"][i] - blend_dot * offsets[i]
                    for i in range(3)
                )

            dt = float(duration) / float(len(samples) - 1)
            for k, sample in enumerate(samples):
                left = max(0, k - 1)
                right = min(len(samples) - 1, k + 1)
                span = (right - left) * dt
                sample["q_ddot"] = tuple(
                    (samples[right]["q_dot"][i] - samples[left]["q_dot"][i])
                    / span
                    for i in range(3)
                )
        self.last_samples = samples
        self.last_target = target
        self._show_samples(samples)

        distance = math.dist(start, target)
        max_qd = max(abs(v) for sample in samples for v in sample["q_dot"])
        max_qdd = max(abs(v) for sample in samples for v in sample["q_ddot"])
        self.summary.set(
            f"Quang duong: {distance * 1000.0:.2f} mm | "
            f"v TB: {distance / float(duration) * 1000.0:.2f} mm/s"
        )
        self.status.set(
            f"Da lap N={len(samples)}, max |qd|={max_qd:.3f} rad/s, "
            f"max |qdd|={max_qdd:.3f} rad/s2"
        )
        return samples

    def _show_samples(self, samples):
        for item in self.table.get_children():
            self.table.delete(item)
        for sample in samples:
            p = sample["p"]
            q_deg = [math.degrees(value) for value in sample["q"]]
            self.table.insert(
                "",
                "end",
                values=(
                    sample["index"],
                    f"{sample['t']:.3f}",
                    f"{p[0] * 1000.0:.2f}",
                    f"{p[1] * 1000.0:.2f}",
                    f"{p[2] * 1000.0:.2f}",
                    f"{q_deg[0]:.2f}",
                    f"{q_deg[1]:.2f}",
                    f"{q_deg[2]:.2f}",
                ),
            )

    def _plan_from_fields(self):
        try:
            self._plan(
                self._target_from_fields(),
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong lap duoc quy dao", str(error))

    def _execute_from_fields(self):
        try:
            samples = self._plan(
                self._target_from_fields(),
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong chay duoc quy dao", str(error))

    def _publish(self, samples):
        self.trajectory_id += 1
        self.node.publish_trajectory(samples, self.trajectory_id)
        self.status.set(f"Da gui trajectory #{self.trajectory_id} den Gazebo")

    def _start_hold_jog(self, direction):
        self._cancel_hold_jog()
        self.held_jog_direction = direction
        self.jog_delay_id = self.root.after(250, self._begin_hold_jog)

    def _begin_hold_jog(self):
        self.jog_delay_id = None
        if self.held_jog_direction is None:
            return
        try:
            point = self._current_point()
            actual_q = self.node.feedback_theta
            model_q = self.solver.inverse_kinematics(
                *point,
                preferred=actual_q,
            )
            q = actual_q if actual_q is not None else model_q
            self.jog_stream_point = point
            self.jog_stream_q = q
            self.jog_stream_model_q = model_q
            self.jog_stream_active = True
            self._stream_hold_jog()
        except (ValueError, DeltaIKError) as error:
            self._cancel_hold_jog()
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _stream_hold_jog(self):
        self.jog_after_id = None
        if not self.jog_stream_active or self.held_jog_direction is None:
            return
        try:
            duration = float(self.jog_time.get())
            step = float(self.step_mm.get()) / 1000.0
            if duration <= 0.0 or step <= 0.0:
                raise ValueError("Buoc va thoi gian jog phai lon hon 0.")

            direction = self.held_jog_direction
            speed = step / duration
            p_dot = tuple(axis * speed for axis in direction)
            point = self.jog_stream_point
            target = tuple(
                point[i] + p_dot[i] * self.jog_stream_period for i in range(3)
            )
            model_q = self.solver.inverse_kinematics(
                *target,
                preferred=self.jog_stream_model_q,
            )
            q = tuple(
                self.jog_stream_q[i]
                + self.solver.normalize_angle(
                    model_q[i] - self.jog_stream_model_q[i]
                )
                for i in range(3)
            )
            q_dot = tuple(
                (q[i] - self.jog_stream_q[i]) / self.jog_stream_period
                for i in range(3)
            )
            self.node.publish_joint_reference(q, q_dot)

            self.jog_stream_point = target
            self.jog_stream_q = q
            self.jog_stream_model_q = model_q
            self.last_target = target
            self.x_mm.set(round(target[0] * 1000.0, 3))
            self.y_mm.set(round(target[1] * 1000.0, 3))
            self.z_mm.set(round(target[2] * 1000.0, 3))
            self.status.set(f"Jog lien tuc: {speed * 1000.0:.2f} mm/s")
            self.jog_after_id = self.root.after(
                int(self.jog_stream_period * 1000.0),
                self._stream_hold_jog,
            )
        except (ValueError, DeltaIKError, ZeroDivisionError) as error:
            self._cancel_hold_jog(send_stop=True)
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _stop_hold_jog(self, _event=None):
        direction = self.held_jog_direction
        was_click = self.jog_delay_id is not None and not self.jog_stream_active
        if was_click:
            self.root.after_cancel(self.jog_delay_id)
            self.jog_delay_id = None

        if self.jog_stream_active:
            self._cancel_hold_jog(send_stop=True)
        else:
            self._cancel_hold_jog()
            if was_click and direction is not None:
                self._jog(*direction)

    def _cancel_hold_jog(self, send_stop=False):
        if send_stop and self.jog_stream_q is not None:
            self.node.publish_joint_reference(self.jog_stream_q)
        self.held_jog_direction = None
        self.jog_stream_active = False
        if self.jog_delay_id is not None:
            self.root.after_cancel(self.jog_delay_id)
            self.jog_delay_id = None
        if self.jog_after_id is not None:
            self.root.after_cancel(self.jog_after_id)
            self.jog_after_id = None
        self.jog_stream_point = None
        self.jog_stream_q = None
        self.jog_stream_model_q = None

    def _jog(self, dx, dy, dz):
        try:
            start = self._current_point()
            step = float(self.step_mm.get()) / 1000.0
            target = (
                start[0] + dx * step,
                start[1] + dy * step,
                start[2] + dz * step,
            )
            self.x_mm.set(round(target[0] * 1000.0, 3))
            self.y_mm.set(round(target[1] * 1000.0, 3))
            self.z_mm.set(round(target[2] * 1000.0, 3))
            samples = self._plan(target, float(self.jog_time.get()), 31)
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            self._cancel_hold_jog()
            messagebox.showerror("Jog ngoai workspace", str(error))

    def _go_home(self):
        try:
            target = (
                self.solver.g.home_x,
                self.solver.g.home_y,
                self.solver.g.home_z,
            )
            self.x_mm.set(target[0] * 1000.0)
            self.y_mm.set(target[1] * 1000.0)
            self.z_mm.set(target[2] * 1000.0)
            samples = self._plan(
                target,
                float(self.duration.get()),
                int(self.waypoints.get()),
            )
            samples[-1]["q"] = (0.0, 0.0, 0.0)
            samples[-1]["q_dot"] = (0.0, 0.0, 0.0)
            samples[-1]["q_ddot"] = (0.0, 0.0, 0.0)
            self._show_samples(samples)
            self._publish(samples)
        except (ValueError, DeltaIKError) as error:
            messagebox.showerror("Khong ve duoc HOME", str(error))

    def _run_workspace(self):
        if self.workspace_running:
            return
        self.workspace_running = True
        self.workspace_button.state(["disabled"])
        try:
            divisions = int(self.workspace_divisions.get())
        except (ValueError, tk.TclError):
            divisions = 12
        self.status.set(f"Dang chia workspace {divisions} x {divisions} x {divisions}...")
        threading.Thread(
            target=self._calculate_workspace,
            args=(divisions,),
            daemon=True,
        ).start()
        self.root.after(50, self._poll_workspace)

    def _calculate_workspace(self, divisions):
        try:
            mesh = calculate_workspace_surface(
                self.solver,
                scan_divisions=divisions,
            )
            self.workspace_results.put(("ok", mesh))
        except Exception as error:
            self.workspace_results.put(("error", str(error)))

    def _poll_workspace(self):
        try:
            result, value = self.workspace_results.get_nowait()
        except queue.Empty:
            if self.workspace_running:
                self.root.after(50, self._poll_workspace)
            return

        if result == "ok":
            self._show_workspace(value)
        else:
            self._workspace_failed(value)

    def _workspace_failed(self, error):
        self.workspace_running = False
        self.workspace_button.state(["!disabled"])
        self.status.set("Tinh workspace that bai")
        messagebox.showerror("Khong ve duoc workspace", error)

    def _show_workspace(self, mesh):
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        window = tk.Toplevel(self.root)
        window.title("Delta 4DOF Workspace")
        window.geometry("940x760")

        figure = Figure(figsize=(9, 7), dpi=100)
        axis = figure.add_subplot(111, projection="3d")
        axis.scatter(
            mesh.volume_x,
            mesh.volume_y,
            mesh.volume_z,
            c=mesh.volume_z,
            cmap="viridis",
            s=7,
            alpha=0.055,
            linewidths=0,
            depthshade=False,
            label="The tich workspace",
        )
        points = axis.scatter(
            mesh.x,
            mesh.y,
            mesh.z,
            c=mesh.z,
            cmap="viridis",
            s=2.5,
            alpha=0.34,
            linewidths=0,
            depthshade=False,
            label="Bien workspace",
        )
        path_mm = mesh.scan_path * 1000.0
        axis.plot(
            path_mm[:, 0],
            path_mm[:, 1],
            path_mm[:, 2],
            color="#d62728",
            linewidth=1.4,
            label="Quy dao Gazebo",
        )
        axis.scatter(
            [self.solver.g.home_x * 1000.0],
            [self.solver.g.home_y * 1000.0],
            [self.solver.g.home_z * 1000.0],
            color="#d62728",
            s=65,
            label="HOME (q1=q2=q3=0)",
        )
        axis.set_xlabel("X (mm)")
        axis.set_ylabel("Y (mm)")
        axis.set_zlabel("Z (mm)")
        axis.set_title(
            f"The tich workspace lien thong tu HOME - luoi {mesh.grid_step_mm:.0f} mm"
        )
        axis.set_box_aspect((1.0, 1.0, 0.55))
        axis.view_init(elev=24, azim=-55)
        axis.legend(loc="upper right")
        figure.colorbar(points, ax=axis, shrink=0.72, pad=0.08, label="Z (mm)")

        bounds = mesh.bounds_mm
        figure.text(
            0.02,
            0.02,
            (
                f"Bien hien thi: X [{bounds[0]:.0f}, {bounds[1]:.0f}] mm | "
                f"Y [{bounds[2]:.0f}, {bounds[3]:.0f}] mm | "
                f"Z [{bounds[4]:.0f}, {bounds[5]:.0f}] mm\n"
                "Bien phuong trinh (0.01 mm): "
                "X +/-333.65 | Y +/-315.12 | Z [-482.84, -199.10] mm"
            ),
            fontsize=9,
        )
        figure.tight_layout(rect=(0, 0.07, 1, 1))

        canvas = FigureCanvasTkAgg(figure, master=window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

        try:
            duration = self._execute_workspace_scan(mesh.scan_path)
        except (ValueError, DeltaIKError) as error:
            self._workspace_failed(str(error))
            return

        self.status.set(f"Dang quet workspace trong {duration:.1f} s")
        self.root.after(
            max(1, int(duration * 1000.0)),
            self._workspace_finished,
        )

    def _workspace_finished(self):
        self.workspace_running = False
        self.workspace_button.state(["!disabled"])
        self.status.set("Da quet xong workspace va ve HOME")

    def _execute_workspace_scan(self, scan_path):
        start = self._current_point()
        actual_q = self.node.feedback_theta
        home = tuple(float(v) for v in scan_path[0])
        transition = self.solver.generate_joint_trajectory(
            p0=start,
            p1=home,
            duration=3.0,
            n_waypoints=61,
            preferred_start=actual_q,
        )
        if actual_q is not None:
            offsets = tuple(
                actual_q[i] - transition[0]["q"][i] for i in range(3)
            )
            for sample in transition:
                tau = sample["tau"]
                blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                sample["q"] = tuple(
                    sample["q"][i] + (1.0 - blend) * offsets[i]
                    for i in range(3)
                )

        samples = list(transition)
        samples[-1]["q"] = (0.0, 0.0, 0.0)
        time_value = samples[-1]["t"]
        previous_p = home
        previous_q = samples[-1]["q"]
        speed = 0.06
        max_spacing = 0.002

        for endpoint in scan_path[1:]:
            endpoint = tuple(float(v) for v in endpoint)
            distance = math.dist(previous_p, endpoint)
            segment_count = max(1, int(math.ceil(distance / max_spacing)))
            segment_time = max(distance / speed, 0.05)
            for step_index in range(1, segment_count + 1):
                ratio = step_index / segment_count
                p = tuple(
                    previous_p[i] + ratio * (endpoint[i] - previous_p[i])
                    for i in range(3)
                )
                q = self.solver.inverse_kinematics(
                    *p,
                    preferred=previous_q,
                )
                time_value += segment_time / segment_count
                samples.append(
                    {
                        "index": len(samples),
                        "t": time_value,
                        "tau": 0.0,
                        "p": p,
                        "p_dot": (0.0, 0.0, 0.0),
                        "p_ddot": (0.0, 0.0, 0.0),
                        "q": q,
                        "q_dot": (0.0, 0.0, 0.0),
                        "q_ddot": (0.0, 0.0, 0.0),
                    }
                )
                previous_q = q
            previous_p = endpoint

        for index, sample in enumerate(samples):
            left = max(0, index - 1)
            right = min(len(samples) - 1, index + 1)
            dt = samples[right]["t"] - samples[left]["t"]
            if dt > 1e-9:
                sample["q_dot"] = tuple(
                    (samples[right]["q"][i] - samples[left]["q"][i]) / dt
                    for i in range(3)
                )
        for index, sample in enumerate(samples):
            left = max(0, index - 1)
            right = min(len(samples) - 1, index + 1)
            dt = samples[right]["t"] - samples[left]["t"]
            if dt > 1e-9:
                sample["q_ddot"] = tuple(
                    (
                        samples[right]["q_dot"][i]
                        - samples[left]["q_dot"][i]
                    )
                    / dt
                    for i in range(3)
                )

        samples[0]["q_dot"] = (0.0, 0.0, 0.0)
        samples[0]["q_ddot"] = (0.0, 0.0, 0.0)
        samples[-1]["q"] = (0.0, 0.0, 0.0)
        samples[-1]["q_dot"] = (0.0, 0.0, 0.0)
        samples[-1]["q_ddot"] = (0.0, 0.0, 0.0)
        self.last_samples = samples
        self.last_target = home
        self._publish(samples)
        return time_value

    def _export_csv(self):
        if not self.last_samples:
            messagebox.showwarning("Chua co du lieu", "Hay lap quy dao truoc.")
            return
        output_dir = Path.home() / "Downloads"
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"delta_trajectory_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["index", "t", "x", "y", "z", "q1", "q2", "q3", "qd1", "qd2", "qd3"]
            )
            for sample in self.last_samples:
                writer.writerow(
                    [
                        sample["index"],
                        sample["t"],
                        *sample["p"],
                        *sample["q"],
                        *sample["q_dot"],
                    ]
                )
        self.status.set(f"Da xuat CSV: {path}")

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(20, self._spin_ros)

    def _refresh_feedback(self):
        xyz = self.node.feedback_xyz
        theta = self.node.feedback_theta
        if xyz is not None:
            text = (
                f"Feedback XYZ: {xyz[0] * 1000.0:.2f}, "
                f"{xyz[1] * 1000.0:.2f}, {xyz[2] * 1000.0:.2f} mm"
            )
            if theta is not None:
                text += " | q: " + ", ".join(f"{math.degrees(v):.2f}" for v in theta) + " deg"
            self.feedback.set(text)
        self.root.after(100, self._refresh_feedback)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaControlNode()
    root = tk.Tk()
    app = DeltaControlApp(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
