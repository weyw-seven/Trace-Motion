"""N1 virtual-map editor.

The editor intentionally works on Map-local millimetres.  Photo backgrounds
are reserved as a future MapBackground layer (N1B); this release focuses on
the hand-drawn map, Home placement, landmarks and persistence.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from uuid import uuid4

from .demo.map_model import (
    PATH_KIND_FREEHAND,
    PATH_KIND_POLYLINE,
    MapDocument,
    MapHome,
    MapLandmark,
    MapModelError,
    MapObstacle,
    MapPath,
)
from .demo.map_view import MapViewTransform
from .demo.map_planner import (
    NavigationMode,
    NavigationPlanner,
    NavigationPlanningError,
    PathDirection,
    PenMode,
    PlannerConfig,
    VehicleProfile,
    _world_obstacles,
)
from .demo.hardware_preflight import HardwarePreflight, preflight_m6_trajectory
from .demo.navigation_simulator import SimulationState, SimulatedNavigationRunner
from .demo.protocol import Pose
from .demo.robot_link import (
    RobotLinkError,
    RobotLinkState,
    SerialRobotLink,
    TcpRobotLink,
    crc32_hex,
    list_serial_ports,
)
from .geometry import Arc, CubicBezier, Line, Point2D
from .preview import sample_geometry
from .toolpath import Motion, PenDown, PenState, PenUp
from .demo.toolpath_review import ToolpathReview, review_trj2


class MapEditorApp:
    def __init__(self, root: tk.Tk, *, document: MapDocument | None = None) -> None:
        self.root = root
        self.root.title("Virtual Map Editor · N2/N3")
        self.root.geometry("1280x820")
        self.root.minsize(980, 650)
        self.document = document or MapDocument.new()
        self._map_file_path: Path | None = None
        self._robot_transport_kind = "serial"
        self._selected_kind: str | None = None
        self._selected_id: str | None = None
        self._selected_path_ids: list[str] = []
        self._active_points: list[Point2D] = []
        self._panning = False
        self._pan_start: tuple[float, float] | None = None
        self._home_dragging = False
        self._undo: list[MapDocument] = []
        self._redo: list[MapDocument] = []
        self._target_map: Point2D | None = None
        self._plan = None
        self._planner = NavigationPlanner()
        self._runner = SimulatedNavigationRunner(Pose(0.0, 0.0, 0.0))
        self._robot_link: SerialRobotLink | None = None
        self._robot_panel: tk.Toplevel | None = None
        self._robot_action_queue: queue.Queue[tuple[str, bool, object]] = queue.Queue()
        self._robot_action_busy = False
        self._toolpath_panel: tk.Toplevel | None = None
        self._toolpath_preview_canvas: tk.Canvas | None = None
        self._toolpath_review: ToolpathReview | None = None
        self._uploaded_real_receipt = None
        self._uploaded_real_payload_crc: str | None = None
        self._uploaded_real_start_pose: Pose | None = None
        self._uploaded_real_reset_crc: str | None = None
        self._pending_real_upload: tuple[str, Pose, HardwarePreflight] | None = None
        self._pending_real_reset_crc: str | None = None

        self.tool_var = tk.StringVar(value="select")
        self.name_var = tk.StringVar(value=self.document.name)
        self.width_var = tk.DoubleVar(value=self.document.width_mm)
        self.height_var = tk.DoubleVar(value=self.document.height_mm)
        self.status_var = tk.StringVar(value="N2 ready · Map-local millimetres · photo background reserved for N1B")
        self.coord_var = tk.StringVar(value="Map (—, —) mm   WORLD (—, —) mm")
        self.selection_var = tk.StringVar(value="No selection")
        self.object_name_var = tk.StringVar(value="")
        self.nav_mode_var = tk.StringVar(value="click")
        self.pen_mode_var = tk.StringVar(value="move")
        self.direction_var = tk.StringVar(value="auto")
        self.nav_pose_var = tk.StringVar(value="Pose (0.0, 0.0, 0.0°) · IDLE")
        self.nav_job_var = tk.StringVar(value="No N2 job planned")
        self.sequence_var = tk.StringVar(value="No paths queued")
        self.robot_summary_var = tk.StringVar(value="Real robot: DISCONNECTED")
        self.robot_port_var = tk.StringVar(value="")
        self.robot_baud_var = tk.StringVar(value="115200")
        self.robot_host_var = tk.StringVar(value="")
        self.robot_tcp_port_var = tk.StringVar(value="5000")
        self.robot_status_var = tk.StringVar(value="DISCONNECTED")
        self.robot_pose_var = tk.StringVar(value="Pose —")
        self.robot_job_var = tk.StringVar(value="Job —")
        self.robot_action_status_var = tk.StringVar(value="Real robot panel ready")
        self.robot_diagnostic_var = tk.StringVar(value="No ESP32 diagnostic lines received")
        self.robot_preflight_var = tk.StringVar(value="Preflight — plan a job and connect an M6 robot")
        self.robot_angle_var = tk.DoubleVar(value=90.0)
        self.robot_speed_var = tk.DoubleVar(value=45.0)
        self.vehicle_enabled_var = tk.BooleanVar(value=False)
        self.vehicle_front_var = tk.DoubleVar(value=70.0)
        self.vehicle_rear_var = tk.DoubleVar(value=70.0)
        self.vehicle_left_var = tk.DoubleVar(value=60.0)
        self.vehicle_right_var = tk.DoubleVar(value=60.0)
        self.vehicle_safety_margin_var = tk.DoubleVar(value=5.0)
        self.vehicle_localization_margin_var = tk.DoubleVar(value=0.0)
        self.vehicle_summary_var = tk.StringVar(value="Footprint disabled · legacy point-robot planning")
        planner_config = self._planner.config
        self.travel_speed_var = tk.DoubleVar(value=planner_config.travel_speed_mm_s)
        self.draw_speed_var = tk.DoubleVar(value=planner_config.draw_speed_mm_s)
        self.acceleration_var = tk.DoubleVar(value=planner_config.acceleration_mm_s2)
        self.fit_tolerance_var = tk.DoubleVar(value=planner_config.geometry_tolerance_mm)
        self.arc_fit_var = tk.BooleanVar(value=planner_config.enable_arc_fitting)
        self.min_arc_points_var = tk.IntVar(value=planner_config.min_arc_points)
        self.min_arc_sweep_var = tk.DoubleVar(value=planner_config.min_arc_sweep_deg)
        self.min_arc_radius_var = tk.DoubleVar(value=planner_config.min_arc_radius_mm)
        self.smoothing_iterations_var = tk.IntVar(value=planner_config.smoothing_iterations)
        self.smoothing_strength_var = tk.DoubleVar(value=planner_config.smoothing_strength)
        self.simplify_tolerance_var = tk.DoubleVar(value=planner_config.simplify_tolerance_mm)
        self.circle_warning_limit_var = tk.IntVar(value=800)
        self.short_circle_warning_var = tk.DoubleVar(value=5.0)
        self.toolpath_review_var = tk.StringVar(value="No planned TRJ2 payload")
        self.toolpath_warning_var = tk.StringVar(value="")

        self._build_layout()
        self._bind_events()
        # A document supplied by the launcher may already contain saved UI
        # preferences.  Apply them only after all Tk variables exist.
        self._apply_saved_settings(self.document.settings)
        self._redraw()
        self.root.after(50, self._poll_simulation)
        self.root.after(100, self._poll_robot_link)
        self.root.protocol("WM_DELETE_WINDOW", self._on_window_close)

    def _build_layout(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # The N2 controls are taller than a typical laptop window.  Keep the
        # whole control panel in its own scrollable column so the map canvas
        # remains available at the right and the existing control order does
        # not need to be split across several toolbars.
        sidebar = ttk.Frame(self.root)
        sidebar.grid(row=0, column=0, sticky="ns")
        sidebar.columnconfigure(0, weight=1)
        sidebar.rowconfigure(0, weight=1)
        self.sidebar_canvas = tk.Canvas(
            sidebar,
            width=315,
            background="#f8fafc",
            highlightthickness=0,
        )
        self.sidebar_canvas.grid(row=0, column=0, sticky="ns")
        sidebar_scrollbar = ttk.Scrollbar(
            sidebar,
            orient="vertical",
            command=self.sidebar_canvas.yview,
        )
        sidebar_scrollbar.grid(row=0, column=1, sticky="ns")
        self.sidebar_canvas.configure(yscrollcommand=sidebar_scrollbar.set)

        controls = ttk.Frame(self.sidebar_canvas, padding=10)
        sidebar_window = self.sidebar_canvas.create_window(
            (0, 0),
            window=controls,
            anchor="nw",
        )
        controls.bind(
            "<Configure>",
            lambda _event: self.sidebar_canvas.configure(
                scrollregion=self.sidebar_canvas.bbox("all")
            ),
        )
        self.sidebar_canvas.bind(
            "<Configure>",
            lambda event: self.sidebar_canvas.itemconfigure(
                sidebar_window,
                width=event.width,
            ),
        )
        self.sidebar_canvas.bind("<Enter>", self._on_sidebar_enter)
        self.sidebar_canvas.bind("<Leave>", self._on_sidebar_leave)
        controls.bind("<Enter>", self._on_sidebar_enter)
        controls.bind("<Leave>", self._on_sidebar_leave)
        self._sidebar_wheel_active = False
        self.root.bind_all("<MouseWheel>", self._on_sidebar_mousewheel, add="+")

        canvas_frame = ttk.Frame(self.root, padding=(0, 10, 10, 10))
        canvas_frame.grid(row=0, column=1, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        ttk.Label(controls, text="Virtual Map · N2", font=("Segoe UI", 16, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )
        ttk.Button(controls, text="New", command=self.new_map).grid(row=1, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(controls, text="Open…", command=self.open_map).grid(row=1, column=1, sticky="ew")
        ttk.Button(controls, text="Save", command=self.save_map).grid(row=2, column=0, sticky="ew", padx=(0, 4), pady=4)
        ttk.Button(controls, text="Save as…", command=self.save_as).grid(row=2, column=1, sticky="ew", pady=4)

        row = 4
        ttk.Label(controls, text="Map settings", font=("Segoe UI", 10, "bold")).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        row = self._entry_row(controls, row, "Name", self.name_var)
        row = self._spin_row(controls, row, "Width mm", self.width_var, 10.0, 100000.0, 10.0)
        row = self._spin_row(controls, row, "Height mm", self.height_var, 10.0, 100000.0, 10.0)
        ttk.Button(controls, text="Apply map size", command=self.apply_map_settings).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(3, 10)
        )
        row += 1

        ttk.Label(controls, text="Tools", font=("Segoe UI", 10, "bold")).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        for value, label in (
            ("select", "Select / inspect"),
            ("draw", "Freehand path"),
            ("polyline", "Polyline path"),
            ("home", "Place / drag Home"),
            ("landmark", "Add landmark"),
            ("obstacle", "Draw obstacle polygon"),
            ("sequence", "Queue path sequence"),
            ("target", "Click navigation target"),
        ):
            ttk.Radiobutton(controls, text=label, value=value, variable=self.tool_var).grid(
                row=row, column=0, columnspan=2, sticky="w", pady=1
            )
            row += 1
        ttk.Button(controls, text="Delete selected", command=self.delete_selected).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(5, 3)
        )
        row += 1
        ttk.Button(controls, text="Cancel current drawing", command=self._cancel_active_drawing).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(0, 3)
        )
        row += 1
        ttk.Button(controls, text="Undo", command=self.undo).grid(row=row, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(controls, text="Redo", command=self.redo).grid(row=row, column=1, sticky="ew")
        row += 1

        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=12)
        row += 1
        ttk.Label(controls, text="Selected object", font=("Segoe UI", 10, "bold")).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        ttk.Label(controls, textvariable=self.selection_var, wraplength=250).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(3, 12)
        )
        ttk.Label(controls, text="Name").grid(row=row + 1, column=0, sticky="w", pady=2)
        self.object_name_entry = ttk.Entry(controls, textvariable=self.object_name_var, width=18)
        self.object_name_entry.grid(row=row + 1, column=1, sticky="e", pady=2)
        ttk.Button(controls, text="Apply name", command=self.apply_object_name).grid(
            row=row + 2, column=0, columnspan=2, sticky="ew", pady=(2, 10)
        )
        ttk.Label(controls, text="Path planning mode").grid(
            row=row + 3, column=0, columnspan=2, sticky="w", pady=(0, 2)
        )
        ttk.Button(
            controls,
            text="Use smooth freehand (rounds corners)",
            command=lambda: self.set_selected_path_kind(PATH_KIND_FREEHAND),
        ).grid(row=row + 4, column=0, columnspan=2, sticky="ew", pady=1)
        ttk.Button(
            controls,
            text="Use exact polyline",
            command=lambda: self.set_selected_path_kind(PATH_KIND_POLYLINE),
        ).grid(row=row + 5, column=0, columnspan=2, sticky="ew", pady=(1, 8))
        ttk.Label(controls, text="N1B photo background is reserved\nand will be added as a separate layer.", foreground="#666", wraplength=250).grid(
            row=row + 6, column=0, columnspan=2, sticky="w", pady=(0, 12)
        )
        n2_row = row + 7
        navigation = ttk.LabelFrame(controls, text="N2 Navigation / simulator", padding=6)
        navigation.grid(row=n2_row, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        ttk.Radiobutton(navigation, text="Click to go", value="click", variable=self.nav_mode_var).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(navigation, text="Selected path", value="path", variable=self.nav_mode_var).grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(navigation, text="Return Home", value="home", variable=self.nav_mode_var).grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Label(navigation, text="Pen").grid(row=3, column=0, sticky="w")
        ttk.Radiobutton(navigation, text="Up", value="move", variable=self.pen_mode_var).grid(row=3, column=1, sticky="w")
        ttk.Radiobutton(navigation, text="Draw", value="draw", variable=self.pen_mode_var).grid(row=4, column=1, sticky="w")
        ttk.Label(navigation, text="Direction").grid(row=5, column=0, sticky="w")
        ttk.Radiobutton(navigation, text="Auto", value="auto", variable=self.direction_var).grid(row=5, column=1, sticky="w")
        ttk.Radiobutton(navigation, text="Forward", value="forward", variable=self.direction_var).grid(row=6, column=1, sticky="w")
        ttk.Radiobutton(navigation, text="Reverse", value="reverse", variable=self.direction_var).grid(row=7, column=1, sticky="w")
        ttk.Button(navigation, text="Plan job", command=self.plan_navigation).grid(row=8, column=0, sticky="ew", pady=(5, 2))
        ttk.Button(navigation, text="Run simulation", command=self.run_simulation).grid(row=8, column=1, sticky="ew", pady=(5, 2))
        ttk.Button(navigation, text="Stop", command=self.stop_simulation).grid(row=9, column=0, sticky="ew")
        ttk.Button(navigation, text="E-Stop", command=self.estop_simulation).grid(row=9, column=1, sticky="ew")
        ttk.Button(navigation, text="Clear E-Stop", command=self.clear_estop).grid(row=10, column=0, sticky="ew")
        ttk.Button(navigation, text="Reset pose", command=self.reset_simulator).grid(row=10, column=1, sticky="ew")
        ttk.Label(navigation, text="Queued path order").grid(row=11, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Label(navigation, textvariable=self.sequence_var, wraplength=240).grid(row=12, column=0, columnspan=2, sticky="w")
        ttk.Button(navigation, text="Clear sequence", command=self._clear_path_sequence).grid(row=13, column=0, sticky="ew", pady=(2, 0))
        ttk.Button(navigation, text="Undo last", command=self._undo_path_sequence).grid(row=13, column=1, sticky="ew", pady=(2, 0))
        ttk.Label(navigation, textvariable=self.nav_pose_var, wraplength=240).grid(row=14, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Label(navigation, textvariable=self.nav_job_var, wraplength=240).grid(row=15, column=0, columnspan=2, sticky="w")
        robot_menu = tk.Menu(navigation, tearoff=False)
        robot_menu.add_command(label="Open real robot panel", command=self.open_robot_panel)
        robot_menu.add_command(label="Refresh serial ports", command=self.refresh_robot_ports)
        robot_menu.add_separator()
        robot_menu.add_command(label="Disconnect real robot", command=self.disconnect_robot)
        ttk.Menubutton(navigation, text="Real robot ▾", menu=robot_menu).grid(
            row=16, column=0, columnspan=2, sticky="ew", pady=(6, 1)
        )
        ttk.Label(navigation, textvariable=self.robot_summary_var, wraplength=240, foreground="#7c2d12").grid(
            row=17, column=0, columnspan=2, sticky="w"
        )
        ttk.Button(
            navigation,
            text="Toolpath parameters / preview…",
            command=self.open_toolpath_panel,
        ).grid(row=18, column=0, columnspan=2, sticky="ew", pady=(6, 1))
        ttk.Label(controls, textvariable=self.status_var, foreground="#555", wraplength=250).grid(
            row=n2_row + 1, column=0, columnspan=2, sticky="w"
        )

        self.canvas = tk.Canvas(canvas_frame, background="#fafafa", highlightthickness=1, highlightbackground="#cbd5e1")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        ttk.Label(canvas_frame, textvariable=self.coord_var, anchor="w").grid(row=1, column=0, sticky="ew", pady=(6, 0))

    @staticmethod
    def _entry_row(parent: ttk.Frame, row: int, label: str, variable: tk.Variable) -> int:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=variable, width=14).grid(row=row, column=1, sticky="e", pady=2)
        return row + 1

    @staticmethod
    def _spin_row(parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: float, maximum: float, increment: float) -> int:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Spinbox(parent, textvariable=variable, from_=minimum, to=maximum, increment=increment, width=14).grid(
            row=row, column=1, sticky="e", pady=2
        )
        return row + 1

    def _bind_events(self) -> None:
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_left_press)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_release)
        self.canvas.bind("<Double-Button-1>", self._on_double_click)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<ButtonPress-2>", self._on_pan_press)
        self.canvas.bind("<B2-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-2>", self._on_pan_release)
        self.canvas.bind("<MouseWheel>", self._on_wheel)
        self.root.bind("<Delete>", lambda _event: self.delete_selected())
        self.root.bind("<Escape>", lambda _event: self._cancel_active_drawing())
        self.root.bind("<Control-z>", lambda _event: self.undo())
        self.root.bind("<Control-y>", lambda _event: self.redo())
        self.object_name_entry.bind("<Return>", lambda _event: self.apply_object_name())

    def open_robot_panel(self) -> None:
        """Open the independent real-robot connection, vehicle and control panel."""

        if self._robot_panel is not None and self._robot_panel.winfo_exists():
            self._robot_panel.deiconify()
            self._robot_panel.lift()
            self.refresh_robot_ports()
            return
        panel = tk.Toplevel(self.root)
        panel.title("Real Robot · N3 Link")
        panel.geometry("500x680")
        panel.minsize(450, 600)
        panel.transient(self.root)
        self._robot_panel = panel
        panel.protocol("WM_DELETE_WINDOW", panel.withdraw)

        body = ttk.Frame(panel, padding=12)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="Real robot connection", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        ttk.Label(
            body,
            text="Connection, vehicle footprint and live controls are kept together here.\nN3 validates the link before any real motion.",
            foreground="#555",
        ).pack(anchor="w", pady=(3, 12))

        notebook = ttk.Notebook(body)
        notebook.pack(fill="both", expand=True)
        connection_tab = ttk.Frame(notebook, padding=8)
        vehicle_tab = ttk.Frame(notebook, padding=8)
        live_tab = ttk.Frame(notebook, padding=8)
        notebook.add(connection_tab, text="Connection")
        notebook.add(vehicle_tab, text="Vehicle & safety")
        notebook.add(live_tab, text="Live control")

        connection = ttk.LabelFrame(connection_tab, text="Transport connection", padding=8)
        connection.pack(fill="x", pady=(0, 8))
        self._robot_transport_notebook = ttk.Notebook(connection)
        self._robot_transport_notebook.pack(fill="x")
        serial_tab = ttk.Frame(self._robot_transport_notebook, padding=6)
        wifi_tab = ttk.Frame(self._robot_transport_notebook, padding=6)
        self._robot_transport_notebook.add(serial_tab, text="USB Serial")
        self._robot_transport_notebook.add(wifi_tab, text="Wi-Fi TCP")
        self._robot_transport_notebook.select(1 if self._robot_transport_kind == "wifi" else 0)

        ttk.Label(serial_tab, text="COM port").grid(row=0, column=0, sticky="w", pady=2)
        self._robot_port_combo = ttk.Combobox(serial_tab, textvariable=self.robot_port_var, state="readonly", width=20)
        self._robot_port_combo.grid(row=0, column=1, sticky="ew", pady=2)
        ttk.Button(serial_tab, text="Refresh", command=self.refresh_robot_ports).grid(row=0, column=2, padx=(5, 0))
        ttk.Label(serial_tab, text="Baud").grid(row=1, column=0, sticky="w", pady=2)
        ttk.Combobox(serial_tab, textvariable=self.robot_baud_var, values=("115200", "230400", "460800"), state="readonly", width=20).grid(row=1, column=1, sticky="ew", pady=2)
        serial_tab.columnconfigure(1, weight=1)

        ttk.Label(wifi_tab, text="Robot IP / host").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Entry(wifi_tab, textvariable=self.robot_host_var, width=22).grid(row=0, column=1, sticky="ew", pady=2)
        ttk.Label(wifi_tab, text="TCP port").grid(row=1, column=0, sticky="w", pady=2)
        ttk.Spinbox(wifi_tab, textvariable=self.robot_tcp_port_var, from_=1, to=65535, increment=1, width=10).grid(row=1, column=1, sticky="w", pady=2)
        ttk.Label(wifi_tab, text="ESP32 must join the same Wi-Fi network.", foreground="#555").grid(row=2, column=0, columnspan=2, sticky="w", pady=(3, 0))
        wifi_tab.columnconfigure(1, weight=1)

        button_row = ttk.Frame(connection)
        button_row.pack(fill="x", pady=(6, 0))
        ttk.Button(button_row, text="Connect", command=self.connect_robot).pack(side="left", fill="x", expand=True)
        ttk.Button(button_row, text="Disconnect", command=self.disconnect_robot).pack(side="left", fill="x", expand=True, padx=(5, 0))

        ttk.Label(
            connection_tab,
            text="Wi-Fi mode expects the ESP32 TCP service and PC to be on the same network.\nUSB mode uses the selected COM port.",
            foreground="#555",
            wraplength=420,
        ).pack(anchor="w", pady=(4, 0))
        ttk.Label(
            connection_tab,
            textvariable=self.robot_diagnostic_var,
            foreground="#64748b",
            wraplength=420,
        ).pack(anchor="w", pady=(8, 0))

        profile = self._planner.config.vehicle_profile
        self._set_vehicle_profile_vars(profile)
        vehicle_tab.columnconfigure(1, weight=1)
        ttk.Label(
            vehicle_tab,
            text="All dimensions are millimetres from the odometry reference point.\nThe planner uses the enclosing circle, so clearance remains safe during rotation.",
            foreground="#555",
            wraplength=420,
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        ttk.Checkbutton(
            vehicle_tab,
            text="Enable vehicle footprint in obstacle planning",
            variable=self.vehicle_enabled_var,
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 8))
        vehicle_row = 2
        for label, variable in (
            ("Front (+X) mm", self.vehicle_front_var),
            ("Rear (-X) mm", self.vehicle_rear_var),
            ("Left (+Y) mm", self.vehicle_left_var),
            ("Right (-Y) mm", self.vehicle_right_var),
            ("Safety margin mm", self.vehicle_safety_margin_var),
            ("Localization margin mm", self.vehicle_localization_margin_var),
        ):
            ttk.Label(vehicle_tab, text=label).grid(row=vehicle_row, column=0, sticky="w", pady=3)
            ttk.Spinbox(vehicle_tab, textvariable=variable, from_=0.0, to=10000.0, increment=1.0, width=14).grid(
                row=vehicle_row, column=1, sticky="ew", pady=3
            )
            vehicle_row += 1
        ttk.Separator(vehicle_tab).grid(row=vehicle_row, column=0, columnspan=3, sticky="ew", pady=8)
        vehicle_row += 1
        ttk.Label(vehicle_tab, textvariable=self.vehicle_summary_var, wraplength=420).grid(
            row=vehicle_row, column=0, columnspan=3, sticky="w", pady=(0, 8)
        )
        vehicle_row += 1
        ttk.Button(vehicle_tab, text="Apply vehicle profile and replan", command=self.apply_vehicle_profile).grid(
            row=vehicle_row, column=0, columnspan=3, sticky="ew", pady=2
        )
        vehicle_row += 1
        ttk.Button(vehicle_tab, text="Reset vehicle defaults", command=self.reset_vehicle_profile).grid(
            row=vehicle_row, column=0, columnspan=3, sticky="ew", pady=2
        )
        vehicle_row += 1
        ttk.Label(
            vehicle_tab,
            text="Changing the profile invalidates the current plan. Plan job again before upload.",
            foreground="#92400e",
            wraplength=420,
        ).grid(row=vehicle_row, column=0, columnspan=3, sticky="w", pady=(8, 0))

        state_frame = ttk.LabelFrame(live_tab, text="Telemetry / state", padding=8)
        state_frame.pack(fill="x", pady=(0, 8))
        ttk.Label(state_frame, text="Link").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(state_frame, textvariable=self.robot_status_var).grid(row=0, column=1, sticky="w", pady=2)
        ttk.Label(state_frame, textvariable=self.robot_pose_var, wraplength=300).grid(row=1, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Label(state_frame, textvariable=self.robot_job_var, wraplength=300).grid(row=2, column=0, columnspan=2, sticky="w", pady=2)
        ttk.Label(state_frame, textvariable=self.robot_preflight_var, wraplength=420, foreground="#555").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(4, 0)
        )

        job_frame = ttk.LabelFrame(live_tab, text="Trajectory job", padding=8)
        job_frame.pack(fill="x", pady=(0, 8))
        ttk.Button(job_frame, text="Upload planned job", command=self.upload_real_job).grid(row=0, column=0, sticky="ew", pady=2)
        ttk.Button(job_frame, text="Run trajectory", command=self.run_real_job).grid(row=0, column=1, sticky="ew", padx=(5, 0), pady=2)
        ttk.Button(job_frame, text="STOP", command=self.stop_real_robot).grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Button(job_frame, text="E-STOP", command=self.estop_real_robot).grid(row=1, column=1, sticky="ew", padx=(5, 0), pady=2)
        ttk.Button(job_frame, text="Clear E-Stop / re-arm", command=self.clear_real_estop).grid(row=2, column=0, sticky="ew", pady=2)
        ttk.Button(job_frame, text="Reset pose to planned start", command=self.reset_real_pose).grid(row=2, column=1, sticky="ew", padx=(5, 0), pady=2)
        ttk.Button(job_frame, text="Preflight planned job", command=self.preflight_real_job).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(5, 0)
        )
        job_frame.columnconfigure(0, weight=1)
        job_frame.columnconfigure(1, weight=1)

        rotate_frame = ttk.LabelFrame(live_tab, text="Relative rotation", padding=8)
        rotate_frame.pack(fill="x", pady=(0, 8))
        ttk.Label(rotate_frame, text="Angle °").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(rotate_frame, textvariable=self.robot_angle_var, from_=-360.0, to=360.0, increment=5.0, width=10).grid(row=0, column=1, sticky="ew")
        ttk.Label(rotate_frame, text="Speed °/s").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(rotate_frame, textvariable=self.robot_speed_var, from_=1.0, to=360.0, increment=5.0, width=10).grid(row=1, column=1, sticky="ew")
        ttk.Button(rotate_frame, text="Rotate", command=self.rotate_real_robot).grid(row=0, column=2, rowspan=2, padx=(6, 0), sticky="ns")
        rotate_frame.columnconfigure(1, weight=1)

        ttk.Label(body, textvariable=self.robot_action_status_var, wraplength=440, foreground="#555").pack(anchor="w", pady=(8, 0))
        self._refresh_vehicle_profile_summary()
        self.refresh_robot_ports()

    def _set_vehicle_profile_vars(self, profile: VehicleProfile) -> None:
        self.vehicle_enabled_var.set(profile.enabled)
        self.vehicle_front_var.set(profile.front_mm)
        self.vehicle_rear_var.set(profile.rear_mm)
        self.vehicle_left_var.set(profile.left_mm)
        self.vehicle_right_var.set(profile.right_mm)
        self.vehicle_safety_margin_var.set(profile.safety_margin_mm)
        self.vehicle_localization_margin_var.set(profile.localization_margin_mm)

    def _vehicle_profile_from_vars(self) -> VehicleProfile:
        return VehicleProfile(
            enabled=bool(self.vehicle_enabled_var.get()),
            front_mm=float(self.vehicle_front_var.get()),
            rear_mm=float(self.vehicle_rear_var.get()),
            left_mm=float(self.vehicle_left_var.get()),
            right_mm=float(self.vehicle_right_var.get()),
            safety_margin_mm=float(self.vehicle_safety_margin_var.get()),
            localization_margin_mm=float(self.vehicle_localization_margin_var.get()),
        )

    def _refresh_vehicle_profile_summary(self) -> None:
        try:
            profile = self._vehicle_profile_from_vars()
        except (TypeError, ValueError, tk.TclError, NavigationPlanningError):
            self.vehicle_summary_var.set("Invalid vehicle profile values")
            return
        if not profile.enabled:
            self.vehicle_summary_var.set("Footprint disabled · legacy point-robot planning")
            return
        self.vehicle_summary_var.set(
            f"Effective clearance radius: {profile.effective_radius_mm:.1f} mm\n"
            f"Effective diameter: {profile.effective_diameter_mm:.1f} mm"
        )

    def apply_vehicle_profile(self) -> None:
        try:
            profile = self._vehicle_profile_from_vars()
            self._planner = NavigationPlanner(replace(self._planner.config, vehicle_profile=profile))
            self._plan = None
            self._toolpath_review = None
            self.nav_job_var.set("No N2 job planned · vehicle profile changed")
            self.toolpath_review_var.set("No planned TRJ2 payload · click Plan job or Preview current plan")
            self.toolpath_warning_var.set("")
            self._refresh_vehicle_profile_summary()
            self.status_var.set(
                f"Vehicle profile applied · effective radius {profile.effective_radius_mm:.1f} mm; plan again"
                if profile.enabled
                else "Vehicle footprint disabled; plan again"
            )
            self._redraw()
            self._draw_toolpath_preview()
        except (TypeError, ValueError, tk.TclError, NavigationPlanningError) as exc:
            self._refresh_vehicle_profile_summary()
            self.status_var.set(f"Invalid vehicle profile: {exc}")
            messagebox.showerror("Vehicle & safety", str(exc), parent=self._robot_panel or self.root)

    def reset_vehicle_profile(self) -> None:
        self._set_vehicle_profile_vars(VehicleProfile())
        self._refresh_vehicle_profile_summary()
        self.status_var.set("Vehicle profile reset; click Apply vehicle profile and replan")

    def refresh_robot_ports(self) -> None:
        ports = list_serial_ports()
        combo = getattr(self, "_robot_port_combo", None)
        if combo is not None and combo.winfo_exists():
            combo.configure(values=ports)
        if ports and self.robot_port_var.get() not in ports:
            self.robot_port_var.set(ports[0])
        if not ports and not self.robot_port_var.get():
            self.robot_port_var.set("")
        message = f"Serial ports: {', '.join(ports) if ports else 'none detected'}"
        self.robot_action_status_var.set(message)
        self.status_var.set(message)

    def connect_robot(self) -> None:
        if self._robot_link is not None and self._robot_link.connected:
            self.robot_action_status_var.set("Real robot link is already connected")
            self.status_var.set("Real robot link is already connected")
            return
        notebook = getattr(self, "_robot_transport_notebook", None)
        selected_tab = notebook.index(notebook.select()) if notebook is not None else 0
        self.robot_diagnostic_var.set("Waiting for ESP32 N3 HELLO…")
        try:
            if selected_tab == 1:
                host = self.robot_host_var.get().strip()
                if not host:
                    raise RobotLinkError("Enter the ESP32 IP address or host name")
                self._robot_link = TcpRobotLink(
                    host,
                    int(self.robot_tcp_port_var.get()),
                )
                self._robot_transport_kind = "wifi"
            else:
                port = self.robot_port_var.get().strip()
                if not port:
                    raise RobotLinkError("Select a serial port in the Real robot panel")
                self._robot_link = SerialRobotLink(port, baudrate=int(self.robot_baud_var.get()))
                self._robot_transport_kind = "serial"
            self._robot_link.connect()
            transport_name = "Wi-Fi TCP" if selected_tab == 1 else "USB serial"
            message = f"{transport_name} opened; waiting for ESP32 HELLO"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
        except (RobotLinkError, ValueError) as exc:
            if self._robot_link is not None:
                self._robot_link.close()
            self._robot_link = None
            self.robot_action_status_var.set(str(exc))
            self.status_var.set(str(exc))

    def disconnect_robot(self) -> None:
        if self._robot_link is not None:
            self._robot_link.close()
        self._robot_link = None
        self.robot_status_var.set(RobotLinkState.DISCONNECTED.value)
        self.robot_summary_var.set("Real robot: DISCONNECTED")
        self.robot_pose_var.set("Pose —")
        self.robot_job_var.set("Job —")
        self.robot_action_status_var.set("Real robot disconnected")
        self.status_var.set("Real robot disconnected")

    def _start_robot_action(self, label: str, action) -> bool:
        link = self._robot_link
        if link is None or not link.connected:
            self.robot_action_status_var.set("Connect the real robot first")
            self.status_var.set("Connect the real robot first")
            return False
        if self._robot_action_busy:
            self.robot_action_status_var.set("A real-robot action is already in progress")
            self.status_var.set("A real-robot action is already in progress")
            return False
        self._robot_action_busy = True
        self.robot_action_status_var.set(f"Real robot: {label}…")
        self.status_var.set(f"Real robot: {label}…")

        def worker() -> None:
            try:
                result = action(link)
            except Exception as exc:  # surfaced on the Tk thread below
                self._robot_action_queue.put((label, False, exc))
            else:
                self._robot_action_queue.put((label, True, result))

        threading.Thread(target=worker, name="robot-link-action", daemon=True).start()
        return True

    def upload_real_job(self) -> None:
        checked = self._preflight_planned_real_job(show_errors=True)
        if checked is None:
            return
        payload, start_pose, preflight = checked
        if preflight.warnings:
            detail = "\n".join(preflight.warnings)
            if not messagebox.askokcancel(
                "Trajectory preflight warnings",
                f"{preflight.summary()}\n\n{detail}\n\nUpload anyway?",
                parent=self._robot_panel or self.root,
            ):
                return
        payload_crc = crc32_hex(payload)
        self._pending_real_upload = (payload_crc, start_pose, preflight)
        if not self._start_robot_action(
            "upload",
            lambda link: link.upload(payload, job_id=f"map-{uuid4().hex[:10]}"),
        ):
            self._pending_real_upload = None

    def run_real_job(self) -> None:
        checked = self._preflight_planned_real_job(show_errors=True)
        if checked is None:
            return
        payload, _start_pose, _preflight = checked
        payload_crc = crc32_hex(payload)
        receipt = self._uploaded_real_receipt
        if receipt is None or self._uploaded_real_payload_crc != payload_crc:
            message = "Upload this exact planned job before running it"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
            return
        if self._uploaded_real_reset_crc != payload_crc:
            message = "Place the robot at the planned start, then click Reset pose to planned start"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
            return
        self._start_robot_action(
            "run",
            lambda link: link.run(job_id=receipt.job_id, crc32=receipt.crc32, reset_odometry=False),
        )

    def stop_real_robot(self) -> None:
        self._start_robot_action("stop", lambda link: link.stop())

    def estop_real_robot(self) -> None:
        self._start_robot_action("emergency stop", lambda link: link.estop())

    def clear_real_estop(self) -> None:
        self._start_robot_action("clear emergency stop", lambda link: link.clear_estop())

    def reset_real_pose(self) -> None:
        checked = self._preflight_planned_real_job(show_errors=True)
        if checked is None:
            return
        payload, start_pose, _preflight = checked
        payload_crc = crc32_hex(payload)
        if self._uploaded_real_receipt is None or self._uploaded_real_payload_crc != payload_crc:
            message = "Upload this exact planned job before resetting its start pose"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
            return
        self._pending_real_reset_crc = payload_crc
        if not self._start_robot_action(
            "reset planned start",
            lambda link: link.reset_pose(start_pose),
        ):
            self._pending_real_reset_crc = None

    def preflight_real_job(self) -> None:
        self._preflight_planned_real_job(show_errors=True)

    def _preflight_planned_real_job(
        self,
        *,
        show_errors: bool,
    ) -> tuple[bytes, Pose, HardwarePreflight] | None:
        if self._plan is None:
            self.plan_navigation()
        if self._plan is None:
            return None
        link = self._robot_link
        if link is None or not link.connected:
            message = "Connect the M6 robot and wait for READY before real-job preflight"
            self.robot_preflight_var.set(message)
            self.status_var.set(message)
            return None
        try:
            self._plan.ensure_current(self.document)
            payload = self._plan.trj2_bytes()
            result = preflight_m6_trajectory(payload, hello=link.hello)
        except (NavigationPlanningError, ValueError) as exc:
            self.robot_preflight_var.set(str(exc))
            self.status_var.set(str(exc))
            return None

        detail = result.summary()
        if result.errors:
            message = f"Preflight blocked: {'; '.join(result.errors)}"
            self.robot_preflight_var.set(message)
            self.status_var.set(message)
            if show_errors:
                messagebox.showerror("M6 trajectory preflight", f"{detail}\n\n" + "\n".join(result.errors), parent=self._robot_panel or self.root)
            return None
        if result.warnings:
            self.robot_preflight_var.set(f"Preflight warning · {detail} · {'; '.join(result.warnings)}")
        else:
            self.robot_preflight_var.set(f"Preflight ready · {detail}")
        return payload, self._plan.start_pose, result

    def rotate_real_robot(self) -> None:
        link = self._robot_link
        if link is None or not link.connected:
            self.robot_action_status_var.set("Connect the real robot first")
            self.status_var.set("Connect the real robot first")
            return
        if not bool((link.hello or {}).get("rotate_enabled")):
            message = (
                "ROTATE_REL is disabled in the connected motion-circle-pen firmware. "
                "It is a separate guarded motion-rotate diagnostic profile."
            )
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
            messagebox.showinfo("Rotate unavailable", message, parent=self._robot_panel or self.root)
            return
        self._start_robot_action(
            "rotate",
            lambda link: link.rotate_relative(float(self.robot_angle_var.get()), float(self.robot_speed_var.get())),
        )

    def _handle_robot_event(self, event: dict[str, object]) -> None:
        kind = event.get("type")
        if kind == "POSE":
            self.robot_pose_var.set(
                f"Pose ({float(event['x_mm']):.1f}, {float(event['y_mm']):.1f}, {float(event['yaw_deg']):.1f}°) · seq {event['seq']}"
            )
        elif kind == "STATUS":
            current_job = event.get("current_job") or "—"
            runner_error = event.get("runner_error") or "NONE"
            self.robot_job_var.set(
                f"Job {current_job} · Runner {event.get('runner')} · Tracker {event.get('tracker')} · "
                f"Pen {event.get('pen')} · Odom {'Ready' if event.get('odom_ready') else 'Not ready'} · "
                f"Runner error {runner_error}"
            )
        elif kind == "HELLO":
            message = f"ESP32 HELLO · firmware {event.get('firmware', 'unknown')}"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)
            self.robot_diagnostic_var.set("N3 protocol handshake complete")
        elif kind == "LOG":
            message = str(event.get("message", "")).strip()
            if message:
                self.robot_diagnostic_var.set(f"ESP32 diagnostic: {message[:360]}")
        elif kind == "ERROR":
            message = f"ESP32 {event.get('code', 'ERROR')}: {event.get('message', '')}"
            self.robot_action_status_var.set(message)
            self.status_var.set(message)

    def _poll_robot_link(self) -> None:
        link = self._robot_link
        if link is not None:
            for event in link.poll_events():
                self._handle_robot_event(event)
            self.robot_status_var.set(link.state.value)
            self.robot_summary_var.set(f"Real robot: {link.state.value}")
            if link.last_pose is not None and self.robot_pose_var.get() == "Pose —":
                pose = link.last_pose
                self.robot_pose_var.set(f"Pose ({pose.x_mm:.1f}, {pose.y_mm:.1f}, {pose.yaw_deg:.1f}°)")
        try:
            while True:
                label, success, result = self._robot_action_queue.get_nowait()
                self._robot_action_busy = False
                if success:
                    if label == "upload" and self._pending_real_upload is not None:
                        payload_crc, start_pose, preflight = self._pending_real_upload
                        self._uploaded_real_receipt = result
                        self._uploaded_real_payload_crc = payload_crc
                        self._uploaded_real_start_pose = start_pose
                        self._uploaded_real_reset_crc = None
                        self.robot_preflight_var.set(
                            f"Uploaded {preflight.summary()} · place robot at planned start, then reset pose"
                        )
                    elif label == "reset planned start" and self._pending_real_reset_crc is not None:
                        self._uploaded_real_reset_crc = self._pending_real_reset_crc
                        pose = self._uploaded_real_start_pose
                        if pose is not None:
                            self.robot_preflight_var.set(
                                f"Start reset to ({pose.x_mm:.1f}, {pose.y_mm:.1f}, {pose.yaw_deg:.1f}°) · ready to run"
                            )
                    message = f"Real robot {label} complete"
                else:
                    message = f"Real robot {label} failed: {result}"
                if label == "upload":
                    self._pending_real_upload = None
                if label == "reset planned start":
                    self._pending_real_reset_crc = None
                self.robot_action_status_var.set(message)
                self.status_var.set(message)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_robot_link)

    def _on_window_close(self) -> None:
        if self._robot_link is not None:
            self._robot_link.close()
        self.root.destroy()

    def open_toolpath_panel(self) -> None:
        """Open the independent fitting controls and exact TRJ2 review."""

        if self._toolpath_panel is not None and self._toolpath_panel.winfo_exists():
            self._toolpath_panel.deiconify()
            self._toolpath_panel.lift()
            self._refresh_toolpath_review()
            return

        panel = tk.Toplevel(self.root)
        panel.title("Toolpath parameters · TRJ2 review")
        panel.geometry("980x720")
        panel.minsize(820, 600)
        panel.transient(self.root)
        self._toolpath_panel = panel
        panel.protocol("WM_DELETE_WINDOW", panel.withdraw)

        body = ttk.Frame(panel, padding=12)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)
        ttk.Label(body, text="Toolpath parameters / preview", font=("Segoe UI", 14, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        ttk.Label(
            body,
            text="These values affect the next navigation plan. The review counts the exact TRJ2 records that will be uploaded.",
            foreground="#555",
            wraplength=900,
        ).grid(row=0, column=1, sticky="w", padx=(12, 0))

        controls = ttk.LabelFrame(body, text="Motion and freehand fitting", padding=8)
        controls.grid(row=1, column=0, sticky="nsw", pady=(10, 0))
        row = 0
        ttk.Label(controls, text="M6 motion", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 4)
        )
        row += 1
        self._toolpath_parameter_row(controls, row, "Travel speed mm/s", self.travel_speed_var, 30.0, 250.0, 5.0)
        row += 1
        self._toolpath_parameter_row(controls, row, "Draw speed mm/s", self.draw_speed_var, 30.0, 250.0, 5.0)
        row += 1
        self._toolpath_parameter_row(controls, row, "Acceleration mm/s²", self.acceleration_var, 50.0, 1000.0, 50.0)
        row += 1
        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1
        ttk.Label(controls, text="Freehand fitting", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 4)
        )
        row += 1
        ttk.Checkbutton(controls, text="Enable LINE + CIRCLE fitting", variable=self.arc_fit_var).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(0, 4)
        )
        row += 1
        self._toolpath_parameter_row(controls, row, "Geometry tolerance mm", self.fit_tolerance_var, 0.05, 100.0, 0.1)
        row += 1
        self._toolpath_parameter_row(controls, row, "Simplify tolerance mm", self.simplify_tolerance_var, 0.0, 100.0, 0.1)
        row += 1
        self._toolpath_parameter_row(controls, row, "Smoothing iterations", self.smoothing_iterations_var, 0, 20, 1)
        row += 1
        self._toolpath_parameter_row(controls, row, "Smoothing strength", self.smoothing_strength_var, 0.0, 1.0, 0.05)
        row += 1
        self._toolpath_parameter_row(controls, row, "Minimum arc points", self.min_arc_points_var, 3, 200, 1)
        row += 1
        self._toolpath_parameter_row(controls, row, "Minimum arc sweep °", self.min_arc_sweep_var, 0.1, 180.0, 1.0)
        row += 1
        self._toolpath_parameter_row(controls, row, "Minimum arc radius mm", self.min_arc_radius_var, 0.1, 10000.0, 0.5)
        row += 1
        ttk.Separator(controls).grid(row=row, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1
        ttk.Label(controls, text="Soft review warnings", font=("Segoe UI", 10, "bold")).grid(
            row=row, column=0, columnspan=2, sticky="w"
        )
        row += 1
        self._toolpath_parameter_row(controls, row, "Warn above CIRCLE count", self.circle_warning_limit_var, 0, 100000, 10)
        row += 1
        self._toolpath_parameter_row(controls, row, "Warn below arc length mm", self.short_circle_warning_var, 0.0, 10000.0, 0.5)
        row += 1
        ttk.Button(controls, text="Apply and replan", command=self.apply_toolpath_parameters).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=(10, 2)
        )
        row += 1
        ttk.Button(controls, text="Reset fitting defaults", command=self.reset_toolpath_parameters).grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=2
        )
        ttk.Button(controls, text="Preview current plan", command=self.preview_toolpath).grid(
            row=row + 1, column=0, columnspan=2, sticky="ew", pady=2
        )

        review_frame = ttk.LabelFrame(body, text="Exact TRJ2 preflight", padding=8)
        review_frame.grid(row=1, column=1, sticky="nsew", padx=(12, 0), pady=(10, 0))
        review_frame.columnconfigure(0, weight=1)
        review_frame.rowconfigure(1, weight=1)
        ttk.Label(review_frame, textvariable=self.toolpath_review_var, justify="left", wraplength=520).grid(
            row=0, column=0, sticky="nw", pady=(0, 6)
        )
        ttk.Label(
            review_frame,
            textvariable=self.toolpath_warning_var,
            foreground="#b45309",
            justify="left",
            wraplength=520,
        ).grid(row=0, column=1, sticky="nw", padx=(12, 0), pady=(0, 6))
        self._toolpath_preview_canvas = tk.Canvas(
            review_frame,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#cbd5e1",
            width=650,
            height=480,
        )
        self._toolpath_preview_canvas.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self._toolpath_preview_canvas.bind("<Configure>", lambda _event: self._draw_toolpath_preview())
        self._refresh_toolpath_review()

    @staticmethod
    def _toolpath_parameter_row(parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: float, maximum: float, increment: float) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Spinbox(parent, textvariable=variable, from_=minimum, to=maximum, increment=increment, width=12).grid(
            row=row, column=1, sticky="e", pady=2
        )

    def reset_toolpath_parameters(self) -> None:
        config = PlannerConfig()
        self.travel_speed_var.set(config.travel_speed_mm_s)
        self.draw_speed_var.set(config.draw_speed_mm_s)
        self.acceleration_var.set(config.acceleration_mm_s2)
        self.fit_tolerance_var.set(config.geometry_tolerance_mm)
        self.arc_fit_var.set(config.enable_arc_fitting)
        self.min_arc_points_var.set(config.min_arc_points)
        self.min_arc_sweep_var.set(config.min_arc_sweep_deg)
        self.min_arc_radius_var.set(config.min_arc_radius_mm)
        self.smoothing_iterations_var.set(config.smoothing_iterations)
        self.smoothing_strength_var.set(config.smoothing_strength)
        self.simplify_tolerance_var.set(config.simplify_tolerance_mm)
        self.status_var.set("Fitting controls reset; click Apply and replan")

    def apply_toolpath_parameters(self) -> None:
        try:
            current = self._planner.config
            config = replace(
                current,
                travel_speed_mm_s=float(self.travel_speed_var.get()),
                draw_speed_mm_s=float(self.draw_speed_var.get()),
                acceleration_mm_s2=float(self.acceleration_var.get()),
                geometry_tolerance_mm=float(self.fit_tolerance_var.get()),
                enable_arc_fitting=bool(self.arc_fit_var.get()),
                min_arc_points=int(self.min_arc_points_var.get()),
                min_arc_sweep_deg=float(self.min_arc_sweep_var.get()),
                min_arc_radius_mm=float(self.min_arc_radius_var.get()),
                smoothing_iterations=int(self.smoothing_iterations_var.get()),
                smoothing_strength=float(self.smoothing_strength_var.get()),
                simplify_tolerance_mm=float(self.simplify_tolerance_var.get()),
            )
            self._planner = NavigationPlanner(config)
            self._plan = None
            self._toolpath_review = None
            self.nav_job_var.set("No N2 job planned · parameters changed")
            self.toolpath_review_var.set("No planned TRJ2 payload · click Plan job or Preview current plan")
            self.toolpath_warning_var.set("")
            self.status_var.set("Toolpath fitting parameters applied; plan again to inspect the new geometry")
            self._redraw()
            self._draw_toolpath_preview()
        except (TypeError, ValueError, NavigationPlanningError) as exc:
            self.status_var.set(f"Invalid toolpath parameters: {exc}")
            messagebox.showerror("Toolpath parameters", str(exc), parent=self._toolpath_panel or self.root)

    def preview_toolpath(self) -> None:
        if self._plan is None:
            self.plan_navigation()
        if self._plan is None:
            return
        self._refresh_toolpath_review()
        self.status_var.set("Exact TRJ2 toolpath preview refreshed")

    def _refresh_toolpath_review(self) -> None:
        if self._plan is None:
            self._toolpath_review = None
            if self._toolpath_panel is not None and self._toolpath_panel.winfo_exists():
                self.toolpath_review_var.set("No planned TRJ2 payload · click Plan job or Preview current plan")
                self.toolpath_warning_var.set("")
            self._draw_toolpath_preview()
            return
        try:
            self._plan.ensure_current(self.document)
            review = review_trj2(self._plan.trj2_bytes())
        except (NavigationPlanningError, ValueError) as exc:
            self._toolpath_review = None
            self.toolpath_review_var.set(f"TRJ2 review failed: {exc}")
            self.toolpath_warning_var.set("")
            self._draw_toolpath_preview()
            return
        self._toolpath_review = review
        minimum = "—" if review.min_circle_length_mm is None else f"{review.min_circle_length_mm:.2f} mm"
        maximum = "—" if review.max_circle_length_mm is None else f"{review.max_circle_length_mm:.2f} mm"
        self.toolpath_review_var.set(
            f"TRJ2 size: {review.payload_size} B\n"
            f"Records: {review.record_count}\n"
            f"LINE: {review.line_count}   CIRCLE: {review.circle_count}\n"
            f"CUBIC: {review.cubic_count}\n"
            f"PEN UP: {review.pen_up_count}   PEN DOWN: {review.pen_down_count}\n"
            f"WAIT: {review.wait_count}\n"
            f"Vehicle clearance: {self._planner.config.vehicle_profile.effective_radius_mm:.1f} mm\n"
            f"Circle arc length min/max: {minimum} / {maximum}"
        )
        warnings = review.warning_messages(
            max_circles=int(self.circle_warning_limit_var.get()),
            min_circle_length_mm=float(self.short_circle_warning_var.get()),
        )
        self.toolpath_warning_var.set("⚠ " + "\n⚠ ".join(warnings) if warnings else "No soft preflight warnings")
        self._draw_toolpath_preview()

    def _draw_toolpath_preview(self) -> None:
        canvas = self._toolpath_preview_canvas
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        plan = self._plan
        if plan is None:
            canvas.create_text(20, 20, anchor="nw", text="Plan a path to preview the exact TRJ2 toolpath", fill="#64748b")
            return
        samples: list[tuple[Point2D, ...]] = []
        for record in plan.toolpath.records:
            if isinstance(record, Motion):
                samples.append(tuple(sample_geometry(record.geometry, sample_step_mm=5.0)))
        all_points = [point for group in samples for point in group]
        if not all_points:
            canvas.create_text(20, 20, anchor="nw", text="The plan has no motion records", fill="#64748b")
            return
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        margin = 36.0
        min_x = min(point.x_mm for point in all_points)
        max_x = max(point.x_mm for point in all_points)
        min_y = min(point.y_mm for point in all_points)
        max_y = max(point.y_mm for point in all_points)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 2.0 * margin) / span_x, (height - 2.0 * margin) / span_y)

        def pixel(point: Point2D) -> tuple[float, float]:
            return (
                margin + (point.x_mm - min_x) * scale,
                height - margin - (point.y_mm - min_y) * scale,
            )

        canvas.create_text(margin, 10, anchor="nw", text="+Y / left", fill="#475569", font=("Segoe UI", 9))
        canvas.create_text(width - margin, height - 16, anchor="se", text="+X / forward", fill="#475569", font=("Segoe UI", 9))
        state = PenState.UP
        for record in plan.toolpath.records:
            if isinstance(record, PenUp):
                state = PenState.UP
                continue
            if isinstance(record, PenDown):
                state = PenState.DOWN
                continue
            if not isinstance(record, Motion):
                continue
            points = [pixel(point) for point in sample_geometry(record.geometry, sample_step_mm=4.0)]
            flat = [coordinate for point in points for coordinate in point]
            if len(flat) >= 4:
                canvas.create_line(
                    *flat,
                    fill="#2563eb" if state is PenState.DOWN else "#f59e0b",
                    width=3 if state is PenState.DOWN else 2,
                    dash=() if state is PenState.DOWN else (6, 4),
                )
        canvas.create_text(
            margin,
            height - 8,
            anchor="sw",
            text="blue = Pen Down   orange dashed = Pen Up",
            fill="#334155",
            font=("Segoe UI", 9),
        )

    def _on_sidebar_enter(self, _event: tk.Event) -> None:
        self._sidebar_wheel_active = True

    def _on_sidebar_leave(self, _event: tk.Event) -> None:
        self._sidebar_wheel_active = False

    def _on_sidebar_mousewheel(self, event: tk.Event):
        """Scroll only when the pointer is over the left control column."""

        if not hasattr(self, "sidebar_canvas"):
            return
        x = self.sidebar_canvas.winfo_pointerx()
        y = self.sidebar_canvas.winfo_pointery()
        left = self.sidebar_canvas.winfo_rootx()
        top = self.sidebar_canvas.winfo_rooty()
        right = left + self.sidebar_canvas.winfo_width()
        bottom = top + self.sidebar_canvas.winfo_height()
        if not (left <= x <= right and top <= y <= bottom):
            return
        delta = getattr(event, "delta", 0)
        if delta:
            units = -int(delta / 120)
            if units == 0:
                units = -1 if delta > 0 else 1
        elif getattr(event, "num", None) == 4:
            units = -1
        elif getattr(event, "num", None) == 5:
            units = 1
        else:
            return
        self.sidebar_canvas.yview_scroll(units, "units")
        return "break"

    @staticmethod
    def _saved_setting_value(source: dict, name: str, default: object) -> object:
        """Read one primitive setting while tolerating hand-edited files."""

        value = source.get(name, default)
        try:
            if isinstance(default, bool):
                return value if isinstance(value, bool) else default
            if isinstance(default, int):
                if isinstance(value, bool):
                    return default
                return int(value)
            if isinstance(default, float):
                number = float(value)
                return number if math.isfinite(number) else default
        except (TypeError, ValueError, OverflowError):
            return default
        return value

    def _current_transport_kind(self) -> str:
        notebook = getattr(self, "_robot_transport_notebook", None)
        if notebook is not None and notebook.winfo_exists():
            try:
                return "wifi" if notebook.index(notebook.select()) == 1 else "serial"
            except tk.TclError:
                pass
        return self._robot_transport_kind

    def _settings_snapshot(self) -> dict[str, object]:
        """Build the serializable UI preferences for the current map."""

        config = self._planner.config
        profile = config.vehicle_profile
        planner_fields = (
            "travel_speed_mm_s",
            "draw_speed_mm_s",
            "acceleration_mm_s2",
            "geometry_tolerance_mm",
            "enable_arc_fitting",
            "min_arc_points",
            "min_arc_sweep_deg",
            "min_arc_radius_mm",
            "smoothing_iterations",
            "smoothing_strength",
            "simplify_tolerance_mm",
            "continuity_tolerance_mm",
            "obstacle_clearance_mm",
            "obstacle_sample_step_mm",
        )
        vehicle_fields = (
            "enabled",
            "front_mm",
            "rear_mm",
            "left_mm",
            "right_mm",
            "safety_margin_mm",
            "localization_margin_mm",
        )
        target = None
        if self._target_map is not None:
            target = [self._target_map.x_mm, self._target_map.y_mm]
        selected_ids = [
            path_id
            for path_id in self._selected_path_ids
            if any(path.path_id == path_id for path in self.document.paths)
        ]
        try:
            baudrate = str(self.robot_baud_var.get()).strip() or "115200"
        except (tk.TclError, AttributeError):
            baudrate = "115200"
        try:
            tcp_port = str(self.robot_tcp_port_var.get()).strip() or "5000"
        except (tk.TclError, AttributeError):
            tcp_port = "5000"
        return {
            "schema": 1,
            "planner": {name: getattr(config, name) for name in planner_fields},
            "vehicle": {name: getattr(profile, name) for name in vehicle_fields},
            "navigation": {
                "mode": self.nav_mode_var.get(),
                "pen_mode": self.pen_mode_var.get(),
                "direction": self.direction_var.get(),
                "selected_path_ids": selected_ids,
                "target_map": target,
            },
            # This is only the last-used endpoint.  Connection state, live
            # pose and E-Stop state are deliberately never persisted.
            "transport": {
                "type": self._current_transport_kind(),
                "port": str(self.robot_port_var.get()).strip(),
                "baudrate": baudrate,
                "host": str(self.robot_host_var.get()).strip(),
                "tcp_port": tcp_port,
            },
        }

    def _apply_saved_settings(self, settings: dict | None) -> None:
        """Restore saved preferences without opening a real-robot link."""

        if not isinstance(settings, dict):
            settings = {}
        default = PlannerConfig()
        planner_raw = settings.get("planner")
        planner_raw = planner_raw if isinstance(planner_raw, dict) else {}
        planner_fields = (
            "travel_speed_mm_s",
            "draw_speed_mm_s",
            "acceleration_mm_s2",
            "geometry_tolerance_mm",
            "enable_arc_fitting",
            "min_arc_points",
            "min_arc_sweep_deg",
            "min_arc_radius_mm",
            "smoothing_iterations",
            "smoothing_strength",
            "simplify_tolerance_mm",
            "continuity_tolerance_mm",
            "obstacle_clearance_mm",
            "obstacle_sample_step_mm",
        )
        planner_values = {
            name: self._saved_setting_value(planner_raw, name, getattr(default, name))
            for name in planner_fields
        }
        # Older map files used zero as an implicit/default acceleration.  The
        # real N3 runner requires an explicit positive value, so migrate that
        # legacy sentinel on load.  The migrated value is persisted next time
        # the map is saved.
        try:
            if float(planner_values["acceleration_mm_s2"]) == 0.0:
                planner_values["acceleration_mm_s2"] = default.acceleration_mm_s2
        except (TypeError, ValueError):
            pass
        default_profile = default.vehicle_profile
        vehicle_raw = settings.get("vehicle")
        vehicle_raw = vehicle_raw if isinstance(vehicle_raw, dict) else {}
        vehicle_fields = (
            "enabled",
            "front_mm",
            "rear_mm",
            "left_mm",
            "right_mm",
            "safety_margin_mm",
            "localization_margin_mm",
        )
        vehicle_values = {
            name: self._saved_setting_value(vehicle_raw, name, getattr(default_profile, name))
            for name in vehicle_fields
        }
        try:
            profile = VehicleProfile(**vehicle_values)
            config = PlannerConfig(**planner_values, vehicle_profile=profile)
        except (NavigationPlanningError, TypeError, ValueError):
            # A malformed hand-edited settings block must not prevent the map
            # itself from opening.
            profile = default_profile
            config = default
        self._planner = NavigationPlanner(config)
        self.travel_speed_var.set(config.travel_speed_mm_s)
        self.draw_speed_var.set(config.draw_speed_mm_s)
        self.acceleration_var.set(config.acceleration_mm_s2)
        self.fit_tolerance_var.set(config.geometry_tolerance_mm)
        self.arc_fit_var.set(config.enable_arc_fitting)
        self.min_arc_points_var.set(config.min_arc_points)
        self.min_arc_sweep_var.set(config.min_arc_sweep_deg)
        self.min_arc_radius_var.set(config.min_arc_radius_mm)
        self.smoothing_iterations_var.set(config.smoothing_iterations)
        self.smoothing_strength_var.set(config.smoothing_strength)
        self.simplify_tolerance_var.set(config.simplify_tolerance_mm)
        self._set_vehicle_profile_vars(profile)

        navigation_raw = settings.get("navigation")
        navigation_raw = navigation_raw if isinstance(navigation_raw, dict) else {}
        mode = navigation_raw.get("mode", "click")
        pen_mode = navigation_raw.get("pen_mode", "move")
        direction = navigation_raw.get("direction", "auto")
        self.nav_mode_var.set(mode if mode in {"click", "path", "home"} else "click")
        self.pen_mode_var.set(pen_mode if pen_mode in {"move", "draw"} else "move")
        self.direction_var.set(direction if direction in {"auto", "forward", "reverse"} else "auto")
        raw_ids = navigation_raw.get("selected_path_ids", [])
        if not isinstance(raw_ids, list):
            raw_ids = []
        valid_ids = {path.path_id for path in self.document.paths}
        self._selected_path_ids = [path_id for path_id in raw_ids if isinstance(path_id, str) and path_id in valid_ids]
        self._target_map = None
        raw_target = navigation_raw.get("target_map")
        if isinstance(raw_target, (list, tuple)) and len(raw_target) == 2:
            try:
                x_mm = float(raw_target[0])
                y_mm = float(raw_target[1])
                if math.isfinite(x_mm) and math.isfinite(y_mm):
                    self._target_map = Point2D(x_mm, y_mm)
            except (TypeError, ValueError):
                pass

        transport_raw = settings.get("transport")
        transport_raw = transport_raw if isinstance(transport_raw, dict) else {}
        transport_kind = transport_raw.get("type", "serial")
        self._robot_transport_kind = "wifi" if transport_kind == "wifi" else "serial"
        self.robot_port_var.set(str(transport_raw.get("port", "")))
        self.robot_baud_var.set(str(transport_raw.get("baudrate", "115200")))
        self.robot_host_var.set(str(transport_raw.get("host", "")))
        self.robot_tcp_port_var.set(str(transport_raw.get("tcp_port", "5000")))
        notebook = getattr(self, "_robot_transport_notebook", None)
        if notebook is not None and notebook.winfo_exists():
            try:
                notebook.select(1 if self._robot_transport_kind == "wifi" else 0)
            except tk.TclError:
                pass
        self._plan = None
        self._toolpath_review = None
        self.nav_job_var.set("No N2 job planned")
        self.toolpath_review_var.set("No planned TRJ2 payload")
        self.toolpath_warning_var.set("")
        self._refresh_vehicle_profile_summary()
        self._update_path_sequence_display()
        if hasattr(self, "canvas"):
            self._redraw()

    def _sync_current_settings(self) -> None:
        """Attach current UI preferences to the in-memory document before save."""

        settings = self._settings_snapshot()
        if self.document.settings != settings:
            # Settings are metadata, so this update intentionally does not add
            # an undo entry or invalidate an already compiled geometry plan.
            self.document = self.document._replace(settings=settings)

    def _view(self) -> MapViewTransform:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        base = MapViewTransform.fit_workspace(self.document.width_mm, self.document.height_mm, width, height)
        zoom = self.document.view.zoom
        scale = base.pixels_per_mm * zoom
        return MapViewTransform(
            base.origin_u_px + self.document.view.pan_u_px,
            base.origin_v_px + self.document.view.pan_v_px,
            scale,
        )

    def _replace_document(self, document: MapDocument, *, record_undo: bool = True) -> None:
        if record_undo and document != self.document:
            self._undo.append(self.document)
            self._redo.clear()
        self.document = document
        # A plan contains translated WORLD geometry and must be rebuilt after
        # any map edit (including moving Home).
        self._plan = None
        self._uploaded_real_receipt = None
        self._uploaded_real_payload_crc = None
        self._uploaded_real_start_pose = None
        self._uploaded_real_reset_crc = None
        self.robot_preflight_var.set("Preflight — map changed; plan and upload again")
        self.nav_job_var.set("No N2 job planned")
        self.name_var.set(document.name)
        self.width_var.set(document.width_mm)
        self.height_var.set(document.height_mm)
        self._selected_path_ids = [
            path_id for path_id in self._selected_path_ids
            if any(path.path_id == path_id for path in document.paths)
        ]
        self._update_path_sequence_display()
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.delete("all")
        transform = self._view()
        self._draw_grid(transform)
        self._draw_obstacles(transform)
        self._draw_paths(transform)
        self._draw_home(transform)
        self._draw_landmarks(transform)
        self._draw_navigation(transform)
        if self._active_points:
            pixels = [transform.map_to_pixel(point) for point in self._active_points]
            if len(pixels) >= 2:
                self.canvas.create_line(*[coordinate for point in pixels for coordinate in point], fill="#16a34a", dash=(5, 3), width=2)
            for u, v in pixels:
                self.canvas.create_oval(u - 3, v - 3, u + 3, v + 3, fill="#16a34a", outline="")

    def _draw_grid(self, transform: MapViewTransform) -> None:
        self.canvas.create_rectangle(
            *transform.map_to_pixel(Point2D(0.0, self.document.height_mm)),
            *transform.map_to_pixel(Point2D(self.document.width_mm, 0.0)),
            outline="#64748b",
            width=2,
        )
        step = 50.0
        if self.document.width_mm > 2500 or self.document.height_mm > 2500:
            step = 250.0
        elif self.document.width_mm < 500 and self.document.height_mm < 500:
            step = 25.0
        x = 0.0
        while x <= self.document.width_mm + 1.0e-9:
            u0, v0 = transform.map_to_pixel(Point2D(x, 0.0))
            u1, v1 = transform.map_to_pixel(Point2D(x, self.document.height_mm))
            self.canvas.create_line(u0, v0, u1, v1, fill="#e2e8f0")
            x += step
        y = 0.0
        while y <= self.document.height_mm + 1.0e-9:
            u0, v0 = transform.map_to_pixel(Point2D(0.0, y))
            u1, v1 = transform.map_to_pixel(Point2D(self.document.width_mm, y))
            self.canvas.create_line(u0, v0, u1, v1, fill="#e2e8f0")
            y += step

    def _draw_paths(self, transform: MapViewTransform) -> None:
        for path in self.document.paths:
            pixels = [transform.map_to_pixel(point) for point in path.points_map_mm]
            flat = [coordinate for point in pixels for coordinate in point]
            selected = self._selected_kind == "path" and path.path_id == self._selected_id
            queued = path.path_id in self._selected_path_ids
            path_color = "#d97706" if queued else ("#2563eb" if not selected else "#dc2626")
            if len(flat) >= 4:
                self.canvas.create_line(*flat, fill=path_color, width=3 if (selected or queued) else 2, smooth=False)
                if path.closed:
                    self.canvas.create_line(*flat[-2:], *flat[:2], fill=path_color, width=2)
            for index, (u, v) in enumerate(pixels):
                radius = 4 if selected else 2
                self.canvas.create_oval(u - radius, v - radius, u + radius, v + radius, fill="#dc2626" if index == 0 else path_color, outline="")
            if pixels:
                self.canvas.create_text(pixels[0][0] + 8, pixels[0][1] - 8, text=path.name, anchor="sw", fill="#1e3a8a")
                if queued:
                    number = self._selected_path_ids.index(path.path_id) + 1
                    mid = pixels[len(pixels) // 2]
                    self.canvas.create_oval(mid[0] - 10, mid[1] - 10, mid[0] + 10, mid[1] + 10, fill="#f59e0b", outline="#92400e", width=2)
                    self.canvas.create_text(mid[0], mid[1], text=str(number), fill="#451a03", font=("Segoe UI", 9, "bold"))

    def _draw_obstacles(self, transform: MapViewTransform) -> None:
        for obstacle in self.document.obstacles:
            if not obstacle.enabled:
                continue
            pixels = [transform.map_to_pixel(point) for point in obstacle.points_map_mm]
            flat = [coordinate for point in pixels for coordinate in point]
            selected = self._selected_kind == "obstacle" and obstacle.obstacle_id == self._selected_id
            if len(flat) >= 6:
                self.canvas.create_polygon(
                    *flat,
                    fill="#fecaca" if not selected else "#fca5a5",
                    outline="#dc2626" if selected else "#ef4444",
                    width=3 if selected else 2,
                )
            if pixels:
                self.canvas.create_text(
                    pixels[0][0] + 8,
                    pixels[0][1] + 8,
                    text=obstacle.name,
                    anchor="nw",
                    fill="#991b1b",
                )
        profile = self._planner.config.vehicle_profile
        if profile.enabled:
            for polygon in _world_obstacles(self.document, self._planner.config):
                map_points = [self.document.map_point(point) for point in polygon]
                pixels = [transform.map_to_pixel(point) for point in map_points]
                flat = [coordinate for point in pixels for coordinate in point]
                if len(flat) >= 6:
                    self.canvas.create_polygon(
                        *flat,
                        fill="",
                        outline="#7c3aed",
                        dash=(7, 4),
                        width=2,
                    )

    def _draw_home(self, transform: MapViewTransform) -> None:
        point = Point2D(self.document.home.map_x_mm, self.document.home.map_y_mm)
        u, v = transform.map_to_pixel(point)
        self.canvas.create_line(u - 10, v, u + 10, v, fill="#dc2626", width=2)
        self.canvas.create_line(u, v - 10, u, v + 10, fill="#dc2626", width=2)
        heading = Point2D(point.x_mm + 35.0, point.y_mm)
        hu, hv = transform.map_to_pixel(heading)
        self.canvas.create_line(u, v, hu, hv, fill="#f59e0b", width=3, arrow=tk.LAST)
        self.canvas.create_text(u + 12, v + 12, text="Home (0,0)", anchor="nw", fill="#991b1b")

    def _draw_landmarks(self, transform: MapViewTransform) -> None:
        for item in self.document.landmarks:
            u, v = transform.map_to_pixel(item.point_map)
            selected = self._selected_kind == "landmark" and item.landmark_id == self._selected_id
            radius = 8 if selected else 5
            self.canvas.create_oval(
                u - radius,
                v - radius,
                u + radius,
                v + radius,
                outline="#dc2626" if selected else "#7c3aed",
                width=3 if selected else 2,
            )
            self.canvas.create_text(u + 8, v, text=item.name, anchor="w", fill="#6b21a8")

    def _draw_pen_event_marker(
        self,
        transform: MapViewTransform,
        world_point: Point2D,
        state: PenState,
        event_index: int,
    ) -> None:
        """Draw an explicit PEN UP/DOWN command on the planned Toolpath."""

        map_point = self.document.map_point(world_point)
        u, v = transform.map_to_pixel(map_point)
        down = state is PenState.DOWN
        color = "#16a34a" if down else "#dc2626"
        if down:
            marker = (u, v - 8, u - 7, v + 5, u + 7, v + 5)
        else:
            marker = (u, v + 8, u - 7, v - 5, u + 7, v - 5)
        self.canvas.create_polygon(*marker, fill=color, outline="#172554", width=1)
        label = "PEN DOWN" if down else "PEN UP"
        label_x = u + 10
        label_y = v - 14 if down else v + 14
        self.canvas.create_text(
            label_x,
            label_y,
            text=f"{label}  #{event_index}",
            anchor="sw" if down else "nw",
            fill=color,
            font=("Segoe UI", 9, "bold"),
        )

    def _draw_navigation(self, transform: MapViewTransform) -> None:
        """Draw target, planned WORLD motions/events, and the simulator trace."""

        if self._target_map is not None:
            u, v = transform.map_to_pixel(self._target_map)
            self.canvas.create_oval(u - 7, v - 7, u + 7, v + 7, outline="#16a34a", width=2)
            self.canvas.create_line(u - 12, v, u + 12, v, fill="#16a34a", width=2)
            self.canvas.create_line(u, v - 12, u, v + 12, fill="#16a34a", width=2)
            self.canvas.create_text(u + 10, v - 10, text="Target", anchor="sw", fill="#166534")

        if self._plan is not None:
            if self._plan.target_world is not None and (
                self._plan.requested_target_world is None
                or self._plan.target_world.distance_to(self._plan.requested_target_world) > 1.0e-6
            ):
                effective_map = self.document.map_point(self._plan.target_world)
                eu, ev = transform.map_to_pixel(effective_map)
                self.canvas.create_oval(eu - 8, ev - 8, eu + 8, ev + 8, outline="#7c3aed", width=2)
                self.canvas.create_text(eu + 10, ev + 10, text="Effective target", anchor="nw", fill="#6b21a8")
            state = PenState.UP
            current_world = self._plan.start_point
            event_index = 0
            for record in self._plan.toolpath.records:
                if isinstance(record, PenUp):
                    state = PenState.UP
                    if self._plan.pen_mode is PenMode.DRAW:
                        event_index += 1
                        self._draw_pen_event_marker(transform, current_world, state, event_index)
                    continue
                if isinstance(record, PenDown):
                    state = PenState.DOWN
                    if self._plan.pen_mode is PenMode.DRAW:
                        event_index += 1
                        self._draw_pen_event_marker(transform, current_world, state, event_index)
                    continue
                if not isinstance(record, Motion):
                    continue
                points = [self.document.map_point(point) for point in sample_geometry(record.geometry, sample_step_mm=5.0)]
                pixels = [transform.map_to_pixel(point) for point in points]
                flat = [coordinate for point in pixels for coordinate in point]
                if len(flat) >= 4:
                    self.canvas.create_line(
                        *flat,
                        fill="#0ea5e9" if state is PenState.DOWN else "#f59e0b",
                        width=3 if state is PenState.DOWN else 2,
                        dash=() if state is PenState.DOWN else (5, 3),
                    )
                current_world = record.end_point()

        trace = self._runner.trace
        if len(trace) >= 2:
            pixels = [transform.map_to_pixel(self.document.map_point(Point2D(pose.x_mm, pose.y_mm))) for pose in trace]
            self.canvas.create_line(*[coordinate for point in pixels for coordinate in point], fill="#0f766e", width=2)

        pose = self._runner.pose
        robot_map = self.document.map_point(Point2D(pose.x_mm, pose.y_mm))
        u, v = transform.map_to_pixel(robot_map)
        self.canvas.create_oval(u - 6, v - 6, u + 6, v + 6, fill="#0f766e", outline="#064e3b", width=2)
        heading_length = 25.0
        yaw_rad = math.radians(pose.yaw_deg)
        heading = Point2D(robot_map.x_mm + heading_length * math.cos(yaw_rad), robot_map.y_mm + heading_length * math.sin(yaw_rad))
        hu, hv = transform.map_to_pixel(heading)
        self.canvas.create_line(u, v, hu, hv, fill="#064e3b", width=3, arrow=tk.LAST)

    def _set_coord_status(self, point: Point2D) -> None:
        world = self.document.world_point(point)
        self.coord_var.set(f"Map ({point.x_mm:.1f}, {point.y_mm:.1f}) mm   WORLD ({world.x_mm:.1f}, {world.y_mm:.1f}) mm")

    def _point_at_event(self, event: tk.Event) -> Point2D:
        point = self._view().pixel_to_map(event.x, event.y)
        self._set_coord_status(point)
        return point

    def _on_motion(self, event: tk.Event) -> None:
        self._set_coord_status(self._view().pixel_to_map(event.x, event.y))

    def _on_left_press(self, event: tk.Event) -> None:
        point = self._point_at_event(event)
        tool = self.tool_var.get()
        if tool == "draw":
            self._active_points = [point]
        elif tool == "polyline":
            self._active_points.append(point)
            self._redraw()
        elif tool == "obstacle":
            self._active_points.append(point)
            self._redraw()
        elif tool == "home":
            self._home_dragging = True
            self._replace_document(self.document.with_home(MapHome(point.x_mm, point.y_mm, self.document.home.yaw_deg, self.document.home.tag_id)))
        elif tool == "landmark":
            landmark = MapLandmark(f"landmark-{uuid4().hex[:8]}", f"L{len(self.document.landmarks) + 1}", point.x_mm, point.y_mm)
            self._replace_document(self.document.with_landmark(landmark))
            self._selected_kind = "landmark"
            self._selected_id = landmark.landmark_id
            self.selection_var.set(f"Landmark: {landmark.name}")
            self.object_name_var.set(landmark.name)
        elif tool == "target":
            self._target_map = point
            self.nav_mode_var.set("click")
            self._plan = None
            self.nav_job_var.set(f"Target set: ({point.x_mm:.1f}, {point.y_mm:.1f}) map mm")
            self.status_var.set("Navigation target set; press Plan job")
            self._redraw()
        elif tool == "sequence":
            selected = self._nearest_object(point)
            if selected is None or selected[0] != "path":
                self.status_var.set("Click a path to add it to the navigation sequence")
            else:
                self._queue_path_selection(selected[1])
            self._redraw()
        elif tool == "select":
            selected = self._nearest_object(point)
            if selected is None:
                self._clear_selection()
            else:
                self._selected_kind, self._selected_id, _distance = selected
                if self._selected_kind == "path":
                    path = next(item for item in self.document.paths if item.path_id == self._selected_id)
                    self.selection_var.set(f"Path: {path.name}")
                    self.object_name_var.set(path.name)
                elif self._selected_kind == "landmark":
                    landmark = next(item for item in self.document.landmarks if item.landmark_id == self._selected_id)
                    self.selection_var.set(f"Landmark: {landmark.name}")
                    self.object_name_var.set(landmark.name)
                else:
                    obstacle = next(item for item in self.document.obstacles if item.obstacle_id == self._selected_id)
                    self.selection_var.set(f"Obstacle: {obstacle.name}")
                    self.object_name_var.set(obstacle.name)
            self._redraw()

    def _update_path_sequence_display(self) -> None:
        if not self._selected_path_ids:
            self.sequence_var.set("No paths queued")
            return
        labels = []
        for index, path_id in enumerate(self._selected_path_ids, start=1):
            path = next((item for item in self.document.paths if item.path_id == path_id), None)
            labels.append(f"{index}. {path.name if path else path_id}")
        self.sequence_var.set("  ".join(labels))

    def _queue_path_selection(self, path_id: str) -> None:
        if path_id in self._selected_path_ids:
            number = self._selected_path_ids.index(path_id) + 1
            self.status_var.set(f"Path is already queued as #{number}")
            return
        self._selected_path_ids.append(path_id)
        path = next(item for item in self.document.paths if item.path_id == path_id)
        self._selected_kind = "path"
        self._selected_id = path_id
        self.selection_var.set(f"Path #{len(self._selected_path_ids)}: {path.name}")
        self.object_name_var.set(path.name)
        self.nav_mode_var.set("path")
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self._update_path_sequence_display()
        self.status_var.set(f"Queued path #{len(self._selected_path_ids)}: {path.name}")

    def _clear_path_sequence(self) -> None:
        self._selected_path_ids.clear()
        self._update_path_sequence_display()
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self.status_var.set("Path sequence cleared")
        self._redraw()

    def _undo_path_sequence(self) -> None:
        if not self._selected_path_ids:
            return
        removed = self._selected_path_ids.pop()
        self._update_path_sequence_display()
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self.status_var.set(f"Removed path from sequence: {removed}")
        self._redraw()

    def _on_left_drag(self, event: tk.Event) -> None:
        point = self._point_at_event(event)
        if self.tool_var.get() == "draw" and self._active_points:
            if self._active_points[-1].distance_to(point) >= 2.0 / self._view().pixels_per_mm:
                self._active_points.append(point)
                self._redraw()
        elif self.tool_var.get() == "home" and self._home_dragging:
            self._replace_document(
                self.document.with_home(MapHome(point.x_mm, point.y_mm, self.document.home.yaw_deg, self.document.home.tag_id)),
                record_undo=False,
            )
        elif self._panning and self._pan_start is not None:
            dx = event.x - self._pan_start[0]
            dy = event.y - self._pan_start[1]
            self._pan_start = (event.x, event.y)
            view = self.document.view
            self._replace_document(self.document._replace(view=type(view)(view.pan_u_px + dx, view.pan_v_px + dy, view.zoom)), record_undo=False)

    def _on_left_release(self, _event: tk.Event) -> None:
        if self.tool_var.get() == "draw":
            self._finish_active_path()
        self._home_dragging = False

    def _on_double_click(self, _event: tk.Event) -> None:
        if self.tool_var.get() == "polyline":
            self._finish_active_path()
        elif self.tool_var.get() == "obstacle":
            self._finish_active_obstacle()

    def _finish_active_path(self) -> None:
        if len(self._active_points) >= 2:
            path = MapPath(
                tuple(self._active_points),
                path_id=f"path-{uuid4().hex[:8]}",
                name=f"Path {len(self.document.paths) + 1}",
                path_kind=(
                    PATH_KIND_POLYLINE
                    if self.tool_var.get() == "polyline"
                    else PATH_KIND_FREEHAND
                ),
            )
            self._replace_document(self.document.with_path(path))
            self._selected_kind = "path"
            self._selected_id = path.path_id
            self.selection_var.set(f"Path: {path.name}")
            self.object_name_var.set(path.name)
        self._active_points = []
        self._redraw()

    def _finish_active_obstacle(self) -> None:
        if len(self._active_points) < 3:
            self.status_var.set("An obstacle needs at least three points")
            self._active_points = []
            self._redraw()
            return
        try:
            obstacle = MapObstacle(
                tuple(self._active_points),
                obstacle_id=f"obstacle-{uuid4().hex[:8]}",
                name=f"Obstacle {len(self.document.obstacles) + 1}",
            )
        except MapModelError as exc:
            # Do not leave an invalid polygon in the transient drawing buffer.
            # Otherwise Delete has no object ID to remove and the red error
            # sketch appears to be stuck on the canvas.
            self._active_points = []
            self.status_var.set(str(exc))
            messagebox.showerror("Virtual Map", str(exc), parent=self.root)
            self._redraw()
            return
        self._replace_document(self.document.with_obstacle(obstacle))
        self._selected_kind = "obstacle"
        self._selected_id = obstacle.obstacle_id
        self.selection_var.set(f"Obstacle: {obstacle.name}")
        self.object_name_var.set(obstacle.name)
        self._active_points = []
        self.status_var.set("Obstacle added; its interior is treated as forbidden")
        self._redraw()

    @staticmethod
    def _distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
        dx = end.x_mm - start.x_mm
        dy = end.y_mm - start.y_mm
        length_sq = dx * dx + dy * dy
        if length_sq <= 1.0e-12:
            return point.distance_to(start)
        t = ((point.x_mm - start.x_mm) * dx + (point.y_mm - start.y_mm) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        projection = Point2D(start.x_mm + t * dx, start.y_mm + t * dy)
        return point.distance_to(projection)

    @staticmethod
    def _point_inside_polygon(point: Point2D, polygon: tuple[Point2D, ...]) -> bool:
        inside = False
        previous = polygon[-1]
        for current in polygon:
            crosses = (current.y_mm > point.y_mm) != (previous.y_mm > point.y_mm)
            if crosses:
                x_at_y = (
                    (previous.x_mm - current.x_mm)
                    * (point.y_mm - current.y_mm)
                    / (previous.y_mm - current.y_mm)
                    + current.x_mm
                )
                if point.x_mm < x_at_y:
                    inside = not inside
            previous = current
        return inside

    def _nearest_object(self, point: Point2D) -> tuple[str, str, float] | None:
        best: tuple[float, str, str] | None = None
        tolerance = 12.0 / self._view().pixels_per_mm
        for landmark in self.document.landmarks:
            distance = point.distance_to(landmark.point_map)
            if distance <= tolerance and (best is None or distance < best[0]):
                best = (distance, "landmark", landmark.landmark_id)
        for obstacle in self.document.obstacles:
            if not obstacle.enabled:
                continue
            segments = list(zip(obstacle.points_map_mm, obstacle.points_map_mm[1:] + obstacle.points_map_mm[:1]))
            distance = 0.0 if self._point_inside_polygon(point, obstacle.points_map_mm) else min(
                (self._distance_to_segment(point, start, end) for start, end in segments),
                default=float("inf"),
            )
            if distance <= tolerance and (best is None or distance < best[0]):
                best = (distance, "obstacle", obstacle.obstacle_id)
        for path in self.document.paths:
            segments = list(zip(path.points_map_mm, path.points_map_mm[1:]))
            if path.closed:
                segments.append((path.points_map_mm[-1], path.points_map_mm[0]))
            distance = min(
                (self._distance_to_segment(point, start, end) for start, end in segments),
                default=float("inf"),
            )
            if distance <= tolerance and (best is None or distance < best[0]):
                best = (distance, "path", path.path_id)
        return (best[1], best[2], best[0]) if best else None

    def _on_pan_press(self, event: tk.Event) -> None:
        self._panning = True
        self._pan_start = (event.x, event.y)

    def _on_pan_drag(self, event: tk.Event) -> None:
        if not self._panning or self._pan_start is None:
            return
        dx = event.x - self._pan_start[0]
        dy = event.y - self._pan_start[1]
        self._pan_start = (event.x, event.y)
        view = self.document.view
        self._replace_document(self.document._replace(view=type(view)(view.pan_u_px + dx, view.pan_v_px + dy, view.zoom)), record_undo=False)

    def _on_pan_release(self, _event: tk.Event) -> None:
        self._panning = False
        self._pan_start = None

    def _on_wheel(self, event: tk.Event) -> None:
        factor = 1.1 if event.delta > 0 else 1.0 / 1.1
        view = self.document.view
        zoom = min(8.0, max(0.2, view.zoom * factor))
        self._replace_document(self.document._replace(view=type(view)(view.pan_u_px, view.pan_v_px, zoom)), record_undo=False)

    def apply_map_settings(self) -> None:
        try:
            document = self.document._replace(
                name=self.name_var.get().strip() or "untitled",
                width_mm=float(self.width_var.get()),
                height_mm=float(self.height_var.get()),
            )
            self._replace_document(document)
            self.status_var.set("Map settings applied")
        except (TypeError, ValueError, MapModelError) as exc:
            messagebox.showerror("Virtual Map", str(exc), parent=self.root)

    def new_map(self) -> None:
        self._replace_document(MapDocument.new())
        self._map_file_path = None
        self._apply_saved_settings({})
        self._clear_selection()
        self.status_var.set("New map")

    def open_map(self) -> None:
        selected = filedialog.askopenfilename(filetypes=[("Virtual map", "*.vmap.json"), ("JSON", "*.json")])
        if not selected:
            return
        try:
            document = MapDocument.load(selected)
            self._replace_document(document)
            self._apply_saved_settings(document.settings)
            self._map_file_path = Path(selected)
            self._clear_selection()
            self.status_var.set(f"Opened {selected}")
        except MapModelError as exc:
            messagebox.showerror("Virtual Map", str(exc), parent=self.root)

    def save_map(self) -> None:
        if self._map_file_path is None:
            self.save_as()
            return
        self._save_document_to_path(self._map_file_path)

    def _save_document_to_path(self, path: str | Path) -> bool:
        try:
            self._sync_current_settings()
            destination = self.document.save(path)
        except (MapModelError, OSError, TypeError, ValueError) as exc:
            messagebox.showerror("Save virtual map", str(exc), parent=self.root)
            return False
        self._map_file_path = destination
        self.status_var.set(f"Saved {destination}")
        return True

    def save_as(self) -> None:
        selected = filedialog.asksaveasfilename(
            defaultextension=".vmap.json",
            filetypes=[("Virtual map", "*.vmap.json"), ("JSON", "*.json")],
        )
        if selected:
            self._save_document_to_path(selected)

    def _cancel_active_drawing(self) -> None:
        """Discard the in-progress path/polygon without changing the map."""

        if not self._active_points:
            return
        self._active_points = []
        self.status_var.set("Current drawing canceled")
        self._redraw()

    def delete_selected(self) -> None:
        # A path/polyline/obstacle is only added to MapDocument after it is
        # completed.  Delete therefore doubles as the cancel action while a
        # drawing is still being edited, including after a validation error.
        if self._active_points:
            self._cancel_active_drawing()
            return
        if self._selected_kind is None or self._selected_id is None:
            return
        if self._selected_kind == "path":
            self._selected_path_ids = [item for item in self._selected_path_ids if item != self._selected_id]
            document = self.document.without_path(self._selected_id)
        elif self._selected_kind == "landmark":
            document = self.document.without_landmark(self._selected_id)
        elif self._selected_kind == "obstacle":
            document = self.document.without_obstacle(self._selected_id)
        else:
            return
        self._replace_document(document)
        self._update_path_sequence_display()
        self._clear_selection()

    def _clear_selection(self) -> None:
        self._selected_kind = None
        self._selected_id = None
        self.selection_var.set("No selection")
        self.object_name_var.set("")
        self._redraw()

    def plan_navigation(self) -> None:
        """Compile the currently selected N2 mode into a reviewable plan."""

        try:
            current_pose = self._runner.pose
            mode = self.nav_mode_var.get()
            if mode == "click":
                if self._target_map is None:
                    raise NavigationPlanningError("choose the Click navigation target tool and click the map first")
                plan = self._planner.plan_click_to_go(self.document, current_pose, self._target_map)
            elif mode == "path":
                if not self._selected_path_ids and (self._selected_kind != "path" or self._selected_id is None):
                    raise NavigationPlanningError("select or queue at least one path before planning Selected path")
                pen_mode = PenMode.DRAW if self.pen_mode_var.get() == "draw" else PenMode.MOVE_ONLY
                direction = PathDirection(self.direction_var.get().upper())
                path_ids = tuple(self._selected_path_ids) or (self._selected_id,)
                plan = self._planner.plan_path_sequence(
                    self.document,
                    current_pose,
                    path_ids,
                    pen_mode=pen_mode,
                    direction=direction,
                )
            elif mode == "home":
                plan = self._planner.plan_return_home(self.document, current_pose)
            else:
                raise NavigationPlanningError(f"unknown navigation mode: {mode}")
            self._plan = plan
            self._uploaded_real_receipt = None
            self._uploaded_real_payload_crc = None
            self._uploaded_real_start_pose = None
            self._uploaded_real_reset_crc = None
            self.robot_preflight_var.set("Preflight — new plan; upload and reset planned start")
            end = plan.end_point
            endpoint = "—" if end is None else f"({end.x_mm:.1f}, {end.y_mm:.1f})"
            payload = plan.trj2_bytes()
            review = review_trj2(payload)
            self.nav_job_var.set(
                f"Planned {plan.mode.value} · {plan.toolpath.motion_count} motions · "
                f"records {review.record_count} (L{review.line_count}/C{review.circle_count}) · "
                f"{len(payload)} B · CRC {crc32_hex(payload)} · end {endpoint} WORLD"
            )
            self.status_var.set("N2/N3 plan ready; inspect cyan/orange route before simulation or upload")
            self._refresh_toolpath_review()
            self._redraw()
        except (NavigationPlanningError, ValueError) as exc:
            self.status_var.set(str(exc))
            messagebox.showerror("N2 navigation", str(exc), parent=self.root)

    def run_simulation(self) -> None:
        if self._plan is None:
            self.plan_navigation()
        if self._plan is None:
            return
        try:
            self._plan.ensure_current(self.document)
            self._runner.start(self._plan)
            self.status_var.set("Simulation running; teal trace is actual simulated pose")
            self._redraw()
        except (NavigationPlanningError, ValueError) as exc:
            self.status_var.set(str(exc))
            messagebox.showerror("N2 simulation", str(exc), parent=self.root)

    def stop_simulation(self) -> None:
        self._runner.stop()
        self.status_var.set("Simulation stopped")
        self._redraw()

    def estop_simulation(self) -> None:
        self._runner.emergency_stop()
        self.status_var.set("Emergency stop active; clear it before running")
        self._redraw()

    def clear_estop(self) -> None:
        self._runner.clear_emergency_stop()
        self.status_var.set("Emergency stop cleared")
        self._redraw()

    def reset_simulator(self) -> None:
        self._runner.reset(Pose(0.0, 0.0, 0.0))
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self.status_var.set("Simulator pose reset to WORLD origin")
        self._redraw()

    def _poll_simulation(self) -> None:
        if self._runner.state is SimulationState.RUNNING:
            self._runner.tick(0.05)
            if self._runner.state is SimulationState.FINISHED:
                self.status_var.set("Simulation finished")
            self._redraw()
        pose = self._runner.pose
        self.nav_pose_var.set(
            f"Pose ({pose.x_mm:.1f}, {pose.y_mm:.1f}, {pose.yaw_deg:.1f}°) · {self._runner.state.value}"
        )
        self.root.after(50, self._poll_simulation)

    def apply_object_name(self) -> None:
        if self._selected_kind is None or self._selected_id is None:
            self.status_var.set("Select a path or landmark before naming it")
            return
        name = self.object_name_var.get().strip()
        if not name:
            self.status_var.set("Name cannot be empty")
            return
        if self._selected_kind == "path":
            current = next((item for item in self.document.paths if item.path_id == self._selected_id), None)
            if current is None:
                self._clear_selection()
                return
            replacement = MapPath(
                current.points_map_mm,
                path_id=current.path_id,
                name=name,
                closed=current.closed,
                drawable=current.drawable,
                path_kind=current.path_kind,
            )
            self._replace_document(self.document.with_path(replacement))
            self.selection_var.set(f"Path: {name}")
        elif self._selected_kind == "landmark":
            current = next((item for item in self.document.landmarks if item.landmark_id == self._selected_id), None)
            if current is None:
                self._clear_selection()
                return
            replacement = MapLandmark(current.landmark_id, name, current.map_x_mm, current.map_y_mm)
            self._replace_document(self.document.with_landmark(replacement))
            self.selection_var.set(f"Landmark: {name}")
        elif self._selected_kind == "obstacle":
            current = next((item for item in self.document.obstacles if item.obstacle_id == self._selected_id), None)
            if current is None:
                self._clear_selection()
                return
            replacement = MapObstacle(
                current.points_map_mm,
                obstacle_id=current.obstacle_id,
                name=name,
                enabled=current.enabled,
            )
            self._replace_document(self.document.with_obstacle(replacement))
            self.selection_var.set(f"Obstacle: {name}")
        self.status_var.set(f"Renamed to {name}")

    def set_selected_path_kind(self, path_kind: str) -> None:
        """Change how a selected path is compiled for navigation."""

        if self._selected_kind != "path" or self._selected_id is None:
            self.status_var.set("Select a path before changing its planning mode")
            return
        current = next(
            (item for item in self.document.paths if item.path_id == self._selected_id),
            None,
        )
        if current is None:
            self._clear_selection()
            return
        if path_kind not in (PATH_KIND_FREEHAND, PATH_KIND_POLYLINE):
            self.status_var.set(f"Unknown path planning mode: {path_kind}")
            return
        replacement = MapPath(
            current.points_map_mm,
            path_id=current.path_id,
            name=current.name,
            closed=current.closed,
            drawable=current.drawable,
            path_kind=path_kind,
        )
        self._replace_document(self.document.with_path(replacement))
        if path_kind == PATH_KIND_POLYLINE:
            self.status_var.set("Path mode set to exact polyline; original vertices and straight segments are preserved")
        else:
            self.status_var.set("Path mode set to smooth freehand; smoothing may move vertices and round sharp corners")

    def undo(self) -> None:
        if not self._undo:
            return
        self._redo.append(self.document)
        self.document = self._undo.pop()
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self.name_var.set(self.document.name)
        self.width_var.set(self.document.width_mm)
        self.height_var.set(self.document.height_mm)
        self._apply_saved_settings(self.document.settings)
        self._redraw()

    def redo(self) -> None:
        if not self._redo:
            return
        self._undo.append(self.document)
        self.document = self._redo.pop()
        self._plan = None
        self.nav_job_var.set("No N2 job planned")
        self.name_var.set(self.document.name)
        self.width_var.set(self.document.width_mm)
        self.height_var.set(self.document.height_mm)
        self._apply_saved_settings(self.document.settings)
        self._redraw()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m pc_trajectory.navigation_ui")
    parser.add_argument("--map", type=Path, help="open a .vmap.json file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    document = MapDocument.load(args.map) if args.map else None
    root = tk.Tk()
    app = MapEditorApp(root, document=document)
    if args.map:
        app._map_file_path = args.map
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["MapEditorApp", "main"]
