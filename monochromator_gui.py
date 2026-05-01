import sys
import traceback

try:
    from PySide6.QtCore import QEvent, QThread, QTimer, Qt, Signal
    from PySide6.QtGui import QKeySequence, QShortcut
    from PySide6.QtWidgets import (
        QAbstractSpinBox,
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QInputDialog,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QPushButton,
        QScrollArea,
        QSpinBox,
        QSplitter,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    print("PySide6 is required for the desktop GUI. Install it with: pip install PySide6")
    raise

import controller_gui_backend as backend_module


class BackendTaskThread(QThread):
    succeeded = Signal(object)
    failed = Signal(str)

    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self._fn = fn
        self._args = args
        self._kwargs = kwargs

    def run(self):
        try:
            result = self._fn(*self._args, **self._kwargs)
            self.succeeded.emit(result)
        except BaseException:
            self.failed.emit(traceback.format_exc())


class MonochromatorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.backend = None
        self._busy = False
        self._task_thread = None
        self._active_task_kind = None
        self._closing = False
        self._shutdown_complete = False
        self._jog_hold_direction = 0
        self._pending_jog_speed_ms = None

        self.setWindowTitle("Monochromator Control")
        self.resize(1200, 860)

        self._build_ui()
        self._refresh_ports(preserve_selection=False)
        self._set_backend_ready(False)
        self._set_busy(False)

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(750)
        self.status_timer.timeout.connect(self._refresh_status_if_idle)
        self.status_timer.start()
        QApplication.instance().installEventFilter(self)

        self.quit_shortcut = QShortcut(QKeySequence.StandardKey.Quit, self)
        self.quit_shortcut.activated.connect(self.close)
        self.close_shortcut = QShortcut(QKeySequence.StandardKey.Close, self)
        self.close_shortcut.activated.connect(self.close)

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_status_group())
        self.workspace_tabs = QTabWidget()
        self.workspace_tabs.addTab(self._wrap_workspace_page(self._build_control_tab()), "Control")
        self.workspace_tabs.addTab(self._wrap_workspace_page(self._build_calibration_group()), "Calibration")
        self.workspace_tabs.addTab(self._wrap_workspace_page(self._build_scan_group()), "Scan")

        self.workspace_splitter = QSplitter(Qt.Vertical)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.addWidget(self.workspace_tabs)
        self.workspace_splitter.addWidget(self._build_log_group())
        self.workspace_splitter.setStretchFactor(0, 4)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes([700, 180])
        root.addWidget(self.workspace_splitter, 1)

        self.setCentralWidget(central)

    def _wrap_workspace_page(self, widget):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        return scroll

    def _build_control_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self._build_jog_group(), 1)
        layout.addWidget(self._build_home_group(), 1)
        layout.addWidget(self._build_move_group(), 2)
        return tab

    def _build_connection_group(self):
        box = QGroupBox("Connection")
        layout = QHBoxLayout(box)

        layout.addWidget(QLabel("Port"))

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.port_combo.setMinimumContentsLength(28)
        layout.addWidget(self.port_combo, 1)

        self.port_refresh_button = QPushButton("Refresh Ports")
        self.port_refresh_button.clicked.connect(self._refresh_ports)
        layout.addWidget(self.port_refresh_button)

        self.init_button = QPushButton("Initialize Controller")
        self.init_button.clicked.connect(self._initialize_backend)
        layout.addWidget(self.init_button)

        self.refresh_button = QPushButton("Refresh Status")
        self.refresh_button.clicked.connect(self._refresh_status_manual)
        layout.addWidget(self.refresh_button)

        self.cancel_button = QPushButton("Cancel Operation")
        self.cancel_button.clicked.connect(self._cancel_current_operation)
        layout.addWidget(self.cancel_button)

        self.close_button = QPushButton("Close GUI")
        self.close_button.clicked.connect(self.close)
        layout.addWidget(self.close_button)

        self.connection_label = QLabel("Backend not initialized.")
        self.connection_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(self.connection_label, 1)
        return box

    def _build_status_group(self):
        box = QGroupBox("Status")
        layout = QGridLayout(box)

        self.status_value_labels = {}
        fields = [
            ("step_count", "Step Count"),
            ("theta_deg", "Angle"),
            ("estimated_wavelength_nm", "Estimated λ"),
            ("s1", "Sensor S1"),
            ("s2", "Sensor S2"),
            ("jog_speed_ms", "Jog Speed"),
            ("optical_offset_steps", "Optical Offset"),
            ("home_window_open", "Home Window"),
            ("debug_enabled", "Debug"),
        ]

        for row, (key, title) in enumerate(fields):
            layout.addWidget(QLabel(f"{title}:"), row // 3, (row % 3) * 2)
            value = QLabel("--")
            self.status_value_labels[key] = value
            layout.addWidget(value, row // 3, (row % 3) * 2 + 1)

        return box

    def _build_jog_group(self):
        box = QGroupBox("Jog")
        layout = QVBoxLayout(box)

        form = QFormLayout()
        self.jog_steps_spin = QSpinBox()
        self.jog_steps_spin.setRange(1, 10000)
        self.jog_steps_spin.setValue(1)
        form.addRow("Steps per jog", self.jog_steps_spin)

        self.jog_speed_spin = QSpinBox()
        self.jog_speed_spin.setRange(1, 1000)
        self.jog_speed_spin.setValue(1)
        form.addRow("Jog speed (ms)", self.jog_speed_spin)
        layout.addLayout(form)

        speed_row = QHBoxLayout()
        self.apply_speed_button = QPushButton("Apply Speed")
        self.apply_speed_button.clicked.connect(self._apply_jog_speed)
        speed_row.addWidget(self.apply_speed_button)
        layout.addLayout(speed_row)

        self.arrow_jog_checkbox = QCheckBox("Arrow-key jog armed")
        self.arrow_jog_checkbox.setChecked(False)
        self.arrow_jog_checkbox.toggled.connect(self._on_arrow_jog_toggled)
        layout.addWidget(self.arrow_jog_checkbox)

        jog_row = QHBoxLayout()
        self.ccw_button = QPushButton("CCW Step")
        self.ccw_button.clicked.connect(self._jog_ccw)
        jog_row.addWidget(self.ccw_button)

        self.cw_button = QPushButton("CW Step")
        self.cw_button.clicked.connect(self._jog_cw)
        jog_row.addWidget(self.cw_button)
        layout.addLayout(jog_row)

        self.jog_hint_label = QLabel(
            "Hold UP for forward, DOWN for reverse. Press Q to quit jog mode. "
            "RIGHT arrow speeds up by 1 ms/step. LEFT arrow slows down by 1 ms/step."
        )
        self.jog_hint_label.setWordWrap(True)
        layout.addWidget(self.jog_hint_label)
        return box

    def _build_home_group(self):
        box = QGroupBox("Home")
        layout = QVBoxLayout(box)

        self.disk_home_button = QPushButton("Go Disk Home")
        self.disk_home_button.clicked.connect(lambda: self._run_task("Go Disk Home", self.backend.go_disk_home))
        layout.addWidget(self.disk_home_button)

        self.reset_disk_home_button = QPushButton("Reset Disk Home")
        self.reset_disk_home_button.clicked.connect(self._reset_disk_home)
        layout.addWidget(self.reset_disk_home_button)

        self.optical_home_button = QPushButton("Go Optical Home")
        self.optical_home_button.clicked.connect(lambda: self._run_task("Go Optical Home", self.backend.go_optical_home))
        layout.addWidget(self.optical_home_button)

        self.reset_optical_home_button = QPushButton("Reset Optical Home")
        self.reset_optical_home_button.clicked.connect(self._reset_optical_home)
        layout.addWidget(self.reset_optical_home_button)

        hint = QLabel(
            "Reset actions save the current position as the new reference. "
            "For best results: rehome, then move to the target point first, then save it here."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return box

    def _build_move_group(self):
        box = QGroupBox("Positioning")
        layout = QVBoxLayout(box)

        rel_form = QHBoxLayout()
        self.relative_steps_spin = QSpinBox()
        self.relative_steps_spin.setRange(-5_000_000, 5_000_000)
        self.relative_steps_spin.setValue(100)
        rel_form.addWidget(QLabel("Relative steps"))
        rel_form.addWidget(self.relative_steps_spin)
        self.relative_move_button = QPushButton("Move Relative")
        self.relative_move_button.clicked.connect(
            lambda: self._run_task("Move Relative", self.backend.move_relative_steps, self.relative_steps_spin.value())
        )
        rel_form.addWidget(self.relative_move_button)
        layout.addLayout(rel_form)

        abs_form = QHBoxLayout()
        self.absolute_steps_spin = QSpinBox()
        self.absolute_steps_spin.setRange(-5_000_000, 5_000_000)
        self.absolute_steps_spin.setValue(0)
        abs_form.addWidget(QLabel("Absolute steps"))
        abs_form.addWidget(self.absolute_steps_spin)
        self.absolute_move_button = QPushButton("Move Absolute")
        self.absolute_move_button.clicked.connect(
            lambda: self._run_task("Move Absolute", self.backend.move_absolute_steps, self.absolute_steps_spin.value())
        )
        abs_form.addWidget(self.absolute_move_button)
        layout.addLayout(abs_form)

        angle_form = QHBoxLayout()
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-360.0, 360.0)
        self.angle_spin.setDecimals(4)
        self.angle_spin.setSingleStep(0.05)
        angle_form.addWidget(QLabel("Angle θ (deg)"))
        angle_form.addWidget(self.angle_spin)
        self.angle_mode_combo = QComboBox()
        self.angle_mode_combo.addItem("From Home", "from_home_ccw")
        self.angle_mode_combo.addItem("Shortest Path", "shortest")
        angle_form.addWidget(self.angle_mode_combo)
        self.angle_move_button = QPushButton("Move to Angle")
        self.angle_move_button.clicked.connect(
            lambda: self._run_task(
                "Move to Angle",
                self.backend.move_to_angle,
                self.angle_spin.value(),
                self.angle_mode_combo.currentData(),
            )
        )
        angle_form.addWidget(self.angle_move_button)
        layout.addLayout(angle_form)

        wl_form = QHBoxLayout()
        self.wavelength_spin = QDoubleSpinBox()
        self.wavelength_spin.setRange(0.0, 5000.0)
        self.wavelength_spin.setDecimals(4)
        self.wavelength_spin.setSingleStep(1.0)
        wl_form.addWidget(QLabel("Wavelength λ (nm)"))
        wl_form.addWidget(self.wavelength_spin)
        self.order_spin = QSpinBox()
        self.order_spin.setRange(-10, 10)
        self.order_spin.setValue(1)
        wl_form.addWidget(QLabel("m"))
        wl_form.addWidget(self.order_spin)
        self.wavelength_mode_combo = QComboBox()
        self.wavelength_mode_combo.addItem("From Home", "from_home_ccw")
        self.wavelength_mode_combo.addItem("Shortest Path", "shortest")
        wl_form.addWidget(self.wavelength_mode_combo)
        self.wavelength_move_button = QPushButton("Move to λ")
        self.wavelength_move_button.clicked.connect(
            lambda: self._run_task(
                "Move to Wavelength",
                self.backend.move_to_wavelength,
                self.wavelength_spin.value(),
                self.order_spin.value(),
                self.wavelength_mode_combo.currentData(),
            )
        )
        wl_form.addWidget(self.wavelength_move_button)
        layout.addLayout(wl_form)

        return box

    def _build_scan_group(self):
        box = QGroupBox("Scan")
        layout = QVBoxLayout(box)

        common = QGridLayout()
        self.scan_mode_combo = QComboBox()
        self.scan_mode_combo.addItem("From Home", "from_home_ccw")
        self.scan_mode_combo.addItem("Shortest Path", "shortest")
        common.addWidget(QLabel("Approach"), 0, 0)
        common.addWidget(self.scan_mode_combo, 0, 1)

        self.scan_repeats_spin = QSpinBox()
        self.scan_repeats_spin.setRange(1, 1000)
        self.scan_repeats_spin.setValue(1)
        common.addWidget(QLabel("Repeats"), 0, 2)
        common.addWidget(self.scan_repeats_spin, 0, 3)

        self.scan_pingpong = QCheckBox("PingPong")
        common.addWidget(self.scan_pingpong, 1, 0)

        self.scan_rehome = QCheckBox("Rehome per repeat")
        common.addWidget(self.scan_rehome, 1, 1)

        self.scan_dwell_spin = QDoubleSpinBox()
        self.scan_dwell_spin.setRange(0.0, 3600.0)
        self.scan_dwell_spin.setDecimals(3)
        self.scan_dwell_spin.setSingleStep(0.05)
        common.addWidget(QLabel("Dwell (s)"), 1, 2)
        common.addWidget(self.scan_dwell_spin, 1, 3)

        layout.addLayout(common)

        self.scan_tabs = QTabWidget()
        self.scan_tabs.addTab(self._build_wavelength_scan_tab(), "Wavelength")
        self.scan_tabs.addTab(self._build_angle_scan_tab(), "Angle")
        layout.addWidget(self.scan_tabs)
        return box

    def _build_calibration_group(self):
        box = QGroupBox("Calibration")
        layout = QVBoxLayout(box)
        self.calibration_tabs = QTabWidget()
        self.calibration_tabs.addTab(self._build_s2_calibration_tab(), "S2 / Position")
        self.calibration_tabs.addTab(self._build_s1_calibration_tab(), "S1 / Motor Disk")
        self.calibration_tabs.addTab(self._build_wavelength_calibration_tab(), "Wavelength")
        self.calibration_tabs.addTab(self._build_zwo_calibration_tab(), "ZWO (Experimental)")
        layout.addWidget(self.calibration_tabs)
        return box

    def _build_s2_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        controls = QHBoxLayout()
        self.s2_approach_combo = QComboBox()
        self.s2_approach_combo.addItem("Approach + / CW", +1)
        self.s2_approach_combo.addItem("Approach - / CCW", -1)
        controls.addWidget(QLabel("Approach direction"))
        controls.addWidget(self.s2_approach_combo)

        self.s2_calibrate_button = QPushButton("Calibrate S2 360°")
        self.s2_calibrate_button.clicked.connect(self._start_s2_calibration)
        controls.addWidget(self.s2_calibrate_button)

        self.s2_status_button = QPushButton("Show S2 Status + Integrity")
        self.s2_status_button.clicked.connect(self._refresh_s2_status_view)
        controls.addWidget(self.s2_status_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.s2_info_output = QPlainTextEdit()
        self.s2_info_output.setReadOnly(True)
        self.s2_info_output.setFocusPolicy(Qt.NoFocus)
        self.s2_info_output.setPlainText("Controller not initialized.")
        layout.addWidget(self.s2_info_output)
        return tab

    def _build_s1_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        controls = QHBoxLayout()
        self.s1_calibrate_button = QPushButton("Calibrate S1 360°")
        self.s1_calibrate_button.clicked.connect(self._start_s1_calibration)
        controls.addWidget(self.s1_calibrate_button)

        self.s1_status_button = QPushButton("Show S1 Status")
        self.s1_status_button.clicked.connect(self._refresh_s1_status_view)
        controls.addWidget(self.s1_status_button)
        controls.addStretch(1)
        layout.addLayout(controls)

        hint = QLabel(
            "This follows the CLI small-disk routine: it finds the S1 OPEN window, then measures one "
            "full OPEN -> BLOCKED -> OPEN cycle in the forward/CW direction. That motion advances the "
            "stage by about one motor revolution and does not auto-rehome afterward."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.s1_info_output = QPlainTextEdit()
        self.s1_info_output.setReadOnly(True)
        self.s1_info_output.setFocusPolicy(Qt.NoFocus)
        self.s1_info_output.setPlainText("Controller not initialized.")
        layout.addWidget(self.s1_info_output)
        return tab

    def _build_wavelength_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        ref_form = QGridLayout()
        self.wl_ref_lambda_spin = QDoubleSpinBox()
        self.wl_ref_lambda_spin.setRange(0.0, 5000.0)
        self.wl_ref_lambda_spin.setDecimals(4)
        self.wl_ref_lambda_spin.setValue(500.0)
        ref_form.addWidget(QLabel("λ (nm)"), 0, 0)
        ref_form.addWidget(self.wl_ref_lambda_spin, 0, 1)

        self.wl_ref_order_spin = QSpinBox()
        self.wl_ref_order_spin.setRange(-20, 20)
        self.wl_ref_order_spin.setValue(1)
        ref_form.addWidget(QLabel("Order m"), 0, 2)
        ref_form.addWidget(self.wl_ref_order_spin, 0, 3)

        self.wl_add_current_ref_button = QPushButton("Add Ref Using Current θ")
        self.wl_add_current_ref_button.clicked.connect(self._add_wavelength_ref_current)
        ref_form.addWidget(self.wl_add_current_ref_button, 0, 4)

        self.wl_ref_theta_spin = QDoubleSpinBox()
        self.wl_ref_theta_spin.setRange(-360.0, 360.0)
        self.wl_ref_theta_spin.setDecimals(6)
        self.wl_ref_theta_spin.setValue(0.0)
        ref_form.addWidget(QLabel("Entered θ (deg)"), 1, 0)
        ref_form.addWidget(self.wl_ref_theta_spin, 1, 1)

        self.wl_add_entered_ref_button = QPushButton("Add Ref Using Entered θ")
        self.wl_add_entered_ref_button.clicked.connect(self._add_wavelength_ref_entered)
        ref_form.addWidget(self.wl_add_entered_ref_button, 1, 4)
        layout.addLayout(ref_form)

        fit_row = QHBoxLayout()
        self.wl_fit_affine_button = QPushButton("Fit Affine Littrow")
        self.wl_fit_affine_button.clicked.connect(lambda: self._fit_wavelength_model("affine", "Fit affine Littrow model"))
        fit_row.addWidget(self.wl_fit_affine_button)

        self.wl_fit_origin_button = QPushButton("Fit Affine (a=0)")
        self.wl_fit_origin_button.clicked.connect(lambda: self._fit_wavelength_model("origin", "Fit affine Littrow model (a=0)"))
        fit_row.addWidget(self.wl_fit_origin_button)

        self.wl_fit_sin_offset_button = QPushButton("Fit A·sin(θ + θ0)")
        self.wl_fit_sin_offset_button.clicked.connect(lambda: self._fit_wavelength_model("sin_offset", "Fit A·sin(θ + θ0) model"))
        fit_row.addWidget(self.wl_fit_sin_offset_button)

        self.wl_fit_sin_cos_button = QPushButton("Fit A·sinθ + B·cosθ")
        self.wl_fit_sin_cos_button.clicked.connect(lambda: self._fit_wavelength_model("sin_cos", "Fit A·sinθ + B·cosθ model"))
        fit_row.addWidget(self.wl_fit_sin_cos_button)
        layout.addLayout(fit_row)

        manage_row = QHBoxLayout()
        self.wl_show_status_button = QPushButton("Show Refs + Model")
        self.wl_show_status_button.clicked.connect(self._refresh_wavelength_status_view)
        manage_row.addWidget(self.wl_show_status_button)

        self.wl_restore_backup_button = QPushButton("Use Backup Model")
        self.wl_restore_backup_button.clicked.connect(self._restore_wavelength_backup)
        manage_row.addWidget(self.wl_restore_backup_button)

        self.wl_clear_button = QPushButton("Clear ALL Refs + Model")
        self.wl_clear_button.clicked.connect(self._clear_wavelength_calibration)
        manage_row.addWidget(self.wl_clear_button)
        manage_row.addStretch(1)
        layout.addLayout(manage_row)

        self.wl_info_output = QPlainTextEdit()
        self.wl_info_output.setReadOnly(True)
        self.wl_info_output.setFocusPolicy(Qt.NoFocus)
        self.wl_info_output.setPlainText("Controller not initialized.")
        layout.addWidget(self.wl_info_output)
        return tab

    def _build_zwo_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        note = QLabel(
            "Experimental. This runs a live line scan with a ZWO camera, adds a wavelength reference, "
            "and can optionally fit/save a model immediately after the scan."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        form = QGridLayout()
        self.zwo_lambda_spin = QDoubleSpinBox()
        self.zwo_lambda_spin.setRange(0.0, 5000.0)
        self.zwo_lambda_spin.setDecimals(4)
        self.zwo_lambda_spin.setValue(500.0)
        form.addWidget(QLabel("Known λ (nm)"), 0, 0)
        form.addWidget(self.zwo_lambda_spin, 0, 1)

        self.zwo_order_spin = QSpinBox()
        self.zwo_order_spin.setRange(-20, 20)
        self.zwo_order_spin.setValue(1)
        form.addWidget(QLabel("Order m"), 0, 2)
        form.addWidget(self.zwo_order_spin, 0, 3)

        self.zwo_premove_check = QCheckBox("Pre-move using current model")
        form.addWidget(self.zwo_premove_check, 0, 4)

        self.zwo_mode_combo = QComboBox()
        self.zwo_mode_combo.addItem("From Home", "from_home_ccw")
        self.zwo_mode_combo.addItem("Shortest Path", "shortest")
        form.addWidget(QLabel("Approach mode"), 1, 0)
        form.addWidget(self.zwo_mode_combo, 1, 1)

        self.zwo_half_width_spin = QSpinBox()
        self.zwo_half_width_spin.setRange(1, 5_000_000)
        self.zwo_half_width_spin.setValue(200)
        form.addWidget(QLabel("Half-width (steps)"), 1, 2)
        form.addWidget(self.zwo_half_width_spin, 1, 3)

        self.zwo_step_size_spin = QSpinBox()
        self.zwo_step_size_spin.setRange(1, 5_000_000)
        self.zwo_step_size_spin.setValue(5)
        form.addWidget(QLabel("Step size (steps)"), 1, 4)
        form.addWidget(self.zwo_step_size_spin, 1, 5)

        self.zwo_settle_spin = QDoubleSpinBox()
        self.zwo_settle_spin.setRange(0.0, 60.0)
        self.zwo_settle_spin.setDecimals(3)
        self.zwo_settle_spin.setValue(0.05)
        form.addWidget(QLabel("Settle (s)"), 2, 0)
        form.addWidget(self.zwo_settle_spin, 2, 1)

        self.zwo_avg_frames_spin = QSpinBox()
        self.zwo_avg_frames_spin.setRange(1, 1000)
        self.zwo_avg_frames_spin.setValue(3)
        form.addWidget(QLabel("Frames / point"), 2, 2)
        form.addWidget(self.zwo_avg_frames_spin, 2, 3)

        self.zwo_camera_index_spin = QSpinBox()
        self.zwo_camera_index_spin.setRange(0, 32)
        self.zwo_camera_index_spin.setValue(0)
        form.addWidget(QLabel("Camera index"), 2, 4)
        form.addWidget(self.zwo_camera_index_spin, 2, 5)

        self.zwo_exposure_spin = QDoubleSpinBox()
        self.zwo_exposure_spin.setRange(0.01, 60_000.0)
        self.zwo_exposure_spin.setDecimals(3)
        self.zwo_exposure_spin.setValue(10.0)
        form.addWidget(QLabel("Exposure (ms)"), 3, 0)
        form.addWidget(self.zwo_exposure_spin, 3, 1)

        self.zwo_gain_spin = QSpinBox()
        self.zwo_gain_spin.setRange(0, 1000)
        self.zwo_gain_spin.setValue(0)
        form.addWidget(QLabel("Gain"), 3, 2)
        form.addWidget(self.zwo_gain_spin, 3, 3)

        self.zwo_move_to_peak_check = QCheckBox("Move to fitted peak after scan")
        self.zwo_move_to_peak_check.setChecked(True)
        form.addWidget(self.zwo_move_to_peak_check, 3, 4)

        self.zwo_fit_mode_combo = QComboBox()
        self.zwo_fit_mode_combo.addItem("Skip refit", "skip")
        self.zwo_fit_mode_combo.addItem("Affine Littrow", "affine")
        self.zwo_fit_mode_combo.addItem("Affine Littrow (a=0)", "origin")
        self.zwo_fit_mode_combo.addItem("A·sin(θ + θ0)", "sin_offset")
        self.zwo_fit_mode_combo.addItem("A·sinθ + B·cosθ", "sin_cos")
        form.addWidget(QLabel("Refit after scan"), 4, 0)
        form.addWidget(self.zwo_fit_mode_combo, 4, 1)

        self.zwo_roi_edit = QLineEdit()
        self.zwo_roi_edit.setPlaceholderText("x,y,w,h  (optional)")
        form.addWidget(QLabel("ROI"), 4, 2)
        form.addWidget(self.zwo_roi_edit, 4, 3)

        self.zwo_lib_path_edit = QLineEdit()
        self.zwo_lib_path_edit.setPlaceholderText("ASICamera2 SDK path (optional)")
        form.addWidget(QLabel("SDK path"), 4, 4)
        form.addWidget(self.zwo_lib_path_edit, 4, 5)
        layout.addLayout(form)

        action_row = QHBoxLayout()
        self.zwo_run_button = QPushButton("Run ZWO-Assisted Reference Scan")
        self.zwo_run_button.clicked.connect(self._run_zwo_reference_capture)
        action_row.addWidget(self.zwo_run_button)
        action_row.addStretch(1)
        layout.addLayout(action_row)

        self.zwo_info_output = QPlainTextEdit()
        self.zwo_info_output.setReadOnly(True)
        self.zwo_info_output.setFocusPolicy(Qt.NoFocus)
        self.zwo_info_output.setPlainText("Controller not initialized.")
        layout.addWidget(self.zwo_info_output)
        return tab

    def _build_wavelength_scan_tab(self):
        tab = QWidget()
        layout = QGridLayout(tab)

        self.scan_lam_start = QDoubleSpinBox()
        self.scan_lam_start.setRange(0.0, 5000.0)
        self.scan_lam_start.setValue(500.0)
        layout.addWidget(QLabel("λ1 (nm)"), 0, 0)
        layout.addWidget(self.scan_lam_start, 0, 1)

        self.scan_lam_end = QDoubleSpinBox()
        self.scan_lam_end.setRange(0.0, 5000.0)
        self.scan_lam_end.setValue(700.0)
        layout.addWidget(QLabel("λ2 (nm)"), 0, 2)
        layout.addWidget(self.scan_lam_end, 0, 3)

        self.scan_lam_order = QSpinBox()
        self.scan_lam_order.setRange(-10, 10)
        self.scan_lam_order.setValue(1)
        layout.addWidget(QLabel("Order m"), 1, 0)
        layout.addWidget(self.scan_lam_order, 1, 1)

        self.scan_lam_step = QDoubleSpinBox()
        self.scan_lam_step.setRange(0.0001, 5000.0)
        self.scan_lam_step.setDecimals(4)
        self.scan_lam_step.setValue(1.0)
        layout.addWidget(QLabel("Step size (nm)"), 1, 2)
        layout.addWidget(self.scan_lam_step, 1, 3)

        self.scan_lam_margin = QDoubleSpinBox()
        self.scan_lam_margin.setRange(0.0, 5000.0)
        self.scan_lam_margin.setDecimals(4)
        self.scan_lam_margin.setValue(0.0)
        layout.addWidget(QLabel("Margin (nm)"), 2, 0)
        layout.addWidget(self.scan_lam_margin, 2, 1)

        self.start_wavelength_scan_button = QPushButton("Start Wavelength Scan")
        self.start_wavelength_scan_button.clicked.connect(self._start_wavelength_scan)
        layout.addWidget(self.start_wavelength_scan_button, 2, 3)
        return tab

    def _build_angle_scan_tab(self):
        tab = QWidget()
        layout = QGridLayout(tab)

        self.scan_theta_start = QDoubleSpinBox()
        self.scan_theta_start.setRange(-360.0, 360.0)
        self.scan_theta_start.setDecimals(4)
        self.scan_theta_start.setValue(0.0)
        layout.addWidget(QLabel("θ1 (deg)"), 0, 0)
        layout.addWidget(self.scan_theta_start, 0, 1)

        self.scan_theta_end = QDoubleSpinBox()
        self.scan_theta_end.setRange(-360.0, 360.0)
        self.scan_theta_end.setDecimals(4)
        self.scan_theta_end.setValue(10.0)
        layout.addWidget(QLabel("θ2 (deg)"), 0, 2)
        layout.addWidget(self.scan_theta_end, 0, 3)

        self.scan_theta_step = QDoubleSpinBox()
        self.scan_theta_step.setRange(0.0001, 360.0)
        self.scan_theta_step.setDecimals(4)
        self.scan_theta_step.setValue(0.05)
        layout.addWidget(QLabel("Step size (deg)"), 1, 0)
        layout.addWidget(self.scan_theta_step, 1, 1)

        self.scan_theta_margin = QDoubleSpinBox()
        self.scan_theta_margin.setRange(0.0, 360.0)
        self.scan_theta_margin.setDecimals(4)
        self.scan_theta_margin.setValue(0.0)
        layout.addWidget(QLabel("Margin (deg)"), 1, 2)
        layout.addWidget(self.scan_theta_margin, 1, 3)

        self.start_angle_scan_button = QPushButton("Start Angle Scan")
        self.start_angle_scan_button.clicked.connect(self._start_angle_scan)
        layout.addWidget(self.start_angle_scan_button, 2, 3)
        return tab

    def _build_log_group(self):
        box = QGroupBox("Activity")
        layout = QVBoxLayout(box)
        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self.log_output)
        return box

    def _set_backend_ready(self, ready):
        self._refresh_control_states()

    def _set_busy(self, busy, message=None):
        self._busy = bool(busy)
        self._refresh_control_states()
        if message:
            self.connection_label.setText(message)

    def _log(self, message):
        self.log_output.appendPlainText(message)

    def _arrow_jog_status_text(self):
        return "Arrow jog armed. Hold UP/DOWN to move. LEFT/RIGHT change speed. Press Q to quit jog mode."

    def _refresh_control_states(self):
        ready = self.backend is not None
        armed = bool(ready and self.arrow_jog_checkbox.isChecked() and not self._closing and not self._shutdown_complete)
        idle = bool(ready and not self._busy and not self._closing and not armed)
        connect_idle = bool(not self._busy and not self._closing and not armed)
        arrow_toggle_enabled = bool(ready and not self._closing and not self._shutdown_complete and (not self._busy or armed))

        self.init_button.setEnabled(connect_idle)
        self.port_combo.setEnabled(connect_idle)
        self.port_refresh_button.setEnabled(connect_idle)
        self.refresh_button.setEnabled(bool(ready and not self._busy and not self._closing and not armed))
        self.cancel_button.setEnabled(bool(ready and not self._closing and not armed))
        self.close_button.setEnabled(bool(not self._shutdown_complete and not self._closing))
        self.arrow_jog_checkbox.setEnabled(arrow_toggle_enabled)
        tabs_accessible = bool(not self._closing and not armed)
        self.workspace_tabs.setTabEnabled(0, bool(not self._closing))
        self.workspace_tabs.setTabEnabled(1, tabs_accessible)
        self.workspace_tabs.setTabEnabled(2, tabs_accessible)
        if armed and self.workspace_tabs.currentIndex() != 0:
            self.workspace_tabs.setCurrentIndex(0)
        self.calibration_tabs.setEnabled(tabs_accessible)
        self.scan_tabs.setEnabled(tabs_accessible)

        for widget in [
            self.jog_steps_spin,
            self.jog_speed_spin,
            self.apply_speed_button,
            self.ccw_button,
            self.cw_button,
            self.disk_home_button,
            self.reset_disk_home_button,
            self.optical_home_button,
            self.reset_optical_home_button,
            self.relative_steps_spin,
            self.relative_move_button,
            self.absolute_steps_spin,
            self.absolute_move_button,
            self.angle_spin,
            self.angle_mode_combo,
            self.angle_move_button,
            self.wavelength_spin,
            self.order_spin,
            self.wavelength_mode_combo,
            self.wavelength_move_button,
            self.s1_calibrate_button,
            self.s1_status_button,
            self.s2_approach_combo,
            self.s2_calibrate_button,
            self.s2_status_button,
            self.wl_ref_lambda_spin,
            self.wl_ref_order_spin,
            self.wl_add_current_ref_button,
            self.wl_ref_theta_spin,
            self.wl_add_entered_ref_button,
            self.wl_fit_affine_button,
            self.wl_fit_origin_button,
            self.wl_fit_sin_offset_button,
            self.wl_fit_sin_cos_button,
            self.wl_show_status_button,
            self.wl_restore_backup_button,
            self.wl_clear_button,
            self.zwo_lambda_spin,
            self.zwo_order_spin,
            self.zwo_premove_check,
            self.zwo_mode_combo,
            self.zwo_half_width_spin,
            self.zwo_step_size_spin,
            self.zwo_settle_spin,
            self.zwo_avg_frames_spin,
            self.zwo_camera_index_spin,
            self.zwo_exposure_spin,
            self.zwo_gain_spin,
            self.zwo_move_to_peak_check,
            self.zwo_fit_mode_combo,
            self.zwo_roi_edit,
            self.zwo_lib_path_edit,
            self.zwo_run_button,
            self.scan_mode_combo,
            self.scan_repeats_spin,
            self.scan_pingpong,
            self.scan_rehome,
            self.scan_dwell_spin,
            self.scan_lam_start,
            self.scan_lam_end,
            self.scan_lam_order,
            self.scan_lam_step,
            self.scan_lam_margin,
            self.start_wavelength_scan_button,
            self.scan_theta_start,
            self.scan_theta_end,
            self.scan_theta_step,
            self.scan_theta_margin,
            self.start_angle_scan_button,
        ]:
            widget.setEnabled(idle)

    def _widget_or_child_has_focus(self, widget):
        focus_widget = QApplication.focusWidget()
        if focus_widget is None:
            return False
        return focus_widget is widget or widget.isAncestorOf(focus_widget)

    def _arrow_jog_blocked_by_focus(self):
        focus_widget = QApplication.focusWidget()
        while focus_widget is not None:
            if isinstance(focus_widget, (QAbstractSpinBox, QComboBox, QLineEdit)):
                return True
            focus_widget = focus_widget.parentWidget()
        return False

    def _selected_port(self):
        data = self.port_combo.currentData()
        if isinstance(data, str) and data.strip():
            return data.strip()

        text = self.port_combo.currentText().strip()
        if not text or text.lower().startswith("auto-detect"):
            return None
        return text

    def _refresh_ports(self, preserve_selection=True):
        current_data = self.port_combo.currentData()
        current_text = self.port_combo.currentText().strip()
        restore_manual_text = False

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItem("Auto-detect (scan available ports)", None)

        ports = backend_module.list_available_serial_ports()
        for port_info in ports:
            self.port_combo.addItem(port_info["label"], port_info["device"])

        selected_index = 0
        if preserve_selection:
            for idx in range(self.port_combo.count()):
                if current_data and self.port_combo.itemData(idx) == current_data:
                    selected_index = idx
                    break
            else:
                if current_text and not current_text.lower().startswith("auto-detect"):
                    restore_manual_text = True
        elif ports:
            selected_index = 1

        self.port_combo.setCurrentIndex(selected_index)
        if restore_manual_text:
            self.port_combo.setEditText(current_text)
        self.port_combo.blockSignals(False)

        if ports:
            self._log(f"[GUI] Found {len(ports)} serial port(s)")
        else:
            self._log("[GUI] No serial ports found; you can still type a port manually")

    def _validate_nonzero_order(self, order_value, context):
        if int(order_value) == 0:
            QMessageBox.warning(self, context, "Diffraction order m must be nonzero.")
            return False
        return True

    def _format_s2_status(self, status):
        steps_per_rev = status.get("s2_steps_per_rev")
        steps_per_deg = status.get("steps_per_deg")
        integrity_ok = status.get("integrity_ok")
        integrity_error = status.get("integrity_error_steps")
        lines = [
            f"S2_STEPS_PER_REV: {steps_per_rev if steps_per_rev is not None else '--'}",
            f"STEPS_PER_DEG: {f'{steps_per_deg:.6f}' if steps_per_deg is not None else '--'}",
            f"Integrity: {'OK' if integrity_ok else 'DRIFT'}",
            f"Integrity error (steps): {integrity_error if integrity_error is not None else '--'}",
            f"Current step_count: {status.get('step_count', '--')}",
            f"S1: {status.get('s1', '--')}",
            f"S2: {status.get('s2', '--')}",
        ]
        return "\n".join(lines)

    def _format_s1_status(self, status):
        lines = [
            f"S1_STEPS_PER_REV: {status.get('s1_steps_per_rev', '--')}",
            f"S1-based estimated step_count: {status.get('s1_estimated_step_count', '--')}",
            f"Integrity: {'OK' if status.get('integrity_ok') else 'DRIFT'}",
            f"Integrity error (steps): {status.get('integrity_error_steps', '--')}",
            f"rev_count: {status.get('rev_count', '--')}",
            f"steps_since_rev: {status.get('steps_since_rev', '--')}",
            f"steps_till_rev: {status.get('steps_till_rev', '--')}",
            f"pre_rev_steps: {status.get('pre_rev_steps', '--')}",
            f"post_rev_steps: {status.get('post_rev_steps', '--')}",
            f"EDGE_HOLDOFF_STEPS: {status.get('edge_holdoff_steps', '--')}",
            f"Current step_count: {status.get('step_count', '--')}",
            f"S1: {status.get('s1', '--')}",
            f"S2: {status.get('s2', '--')}",
        ]
        return "\n".join(lines)

    def _format_wavelength_status(self, status):
        refs = status.get("refs", [])
        lines = [
            f"References: {status.get('refs_count', 0)}",
            f"Backup available: {'Yes' if status.get('backup_available') else 'No'}",
        ]

        if status.get("uses_ideal_fallback"):
            lines.append("Model: (none) — ideal Littrow fallback (affine Littrow, a=0, b=1)")
        else:
            lines.append(f"Model: {status.get('model_summary') or '(unknown)'}")

        rms = status.get("residual_rms_nm")
        mx = status.get("residual_max_nm")
        if rms is not None and mx is not None:
            lines.append(f"Residuals on λ/m: RMS={rms:.6f} nm, max={mx:.6f} nm")

        lines.append("")
        lines.append("Reference points:")
        if not refs:
            lines.append("  (none)")
        else:
            for idx, ref in enumerate(refs, start=1):
                lines.append(
                    f"  {idx}: θ={float(ref['theta_deg']):.6f}°, λ={float(ref['lambda_nm']):.4f} nm, m={int(ref['order_m'])}"
                )
        return "\n".join(lines)

    def _format_zwo_result(self, payload):
        if not isinstance(payload, dict):
            return "No ZWO result available."
        lines = [
            f"Points scanned: {payload.get('points_scanned', '--')}",
            f"Peak step: {payload.get('peak_step', '--')}",
            f"Peak θ: {payload.get('peak_theta_deg', '--')}",
            f"Peak metric: {payload.get('peak_metric', '--')}",
            f"Peak on scan edge: {'Yes' if payload.get('peak_on_edge') else 'No'}",
            f"Reference count after scan: {payload.get('refs_count', '--')}",
        ]
        if payload.get("model_summary"):
            lines.append(f"Saved model: {payload['model_summary']}")
        if payload.get("residual_rms_nm") is not None and payload.get("residual_max_nm") is not None:
            lines.append(
                f"Residuals on λ/m: RMS={payload['residual_rms_nm']:.6f} nm, max={payload['residual_max_nm']:.6f} nm"
            )
        return "\n".join(lines)

    def _refresh_s2_status_view(self, silent=False):
        if self.backend is None:
            self.s2_info_output.setPlainText("Controller not initialized.")
            return
        try:
            status = self.backend.get_s2_calibration_status()
            self.s2_info_output.setPlainText(self._format_s2_status(status))
        except BaseException:
            error_text = traceback.format_exc()
            self.s2_info_output.setPlainText(error_text)
            if not silent:
                self._log("[GUI] S2 calibration status refresh failed")
                self._log(error_text)

    def _refresh_s1_status_view(self, silent=False):
        if self.backend is None:
            self.s1_info_output.setPlainText("Controller not initialized.")
            return
        try:
            status = self.backend.get_s1_calibration_status()
            self.s1_info_output.setPlainText(self._format_s1_status(status))
        except BaseException:
            error_text = traceback.format_exc()
            self.s1_info_output.setPlainText(error_text)
            if not silent:
                self._log("[GUI] S1 calibration status refresh failed")
                self._log(error_text)

    def _refresh_wavelength_status_view(self, silent=False):
        if self.backend is None:
            self.wl_info_output.setPlainText("Controller not initialized.")
            return
        try:
            status = self.backend.get_wavelength_calibration_status()
            self.wl_info_output.setPlainText(self._format_wavelength_status(status))
        except BaseException:
            error_text = traceback.format_exc()
            self.wl_info_output.setPlainText(error_text)
            if not silent:
                self._log("[GUI] Wavelength calibration status refresh failed")
                self._log(error_text)

    def _refresh_calibration_views(self, silent=False):
        self._refresh_s2_status_view(silent=silent)
        self._refresh_s1_status_view(silent=silent)
        self._refresh_wavelength_status_view(silent=silent)

    def _on_arrow_jog_toggled(self, checked):
        if checked and self.backend is None:
            self.arrow_jog_checkbox.blockSignals(True)
            self.arrow_jog_checkbox.setChecked(False)
            self.arrow_jog_checkbox.blockSignals(False)
            self._log("[GUI] Initialize the controller before arming arrow jog")
            return

        if checked and self._busy:
            self.arrow_jog_checkbox.blockSignals(True)
            self.arrow_jog_checkbox.setChecked(False)
            self.arrow_jog_checkbox.blockSignals(False)
            self._log("[GUI] Wait for the current operation to finish before arming arrow jog")
            return

        self._jog_hold_direction = 0
        self._pending_jog_speed_ms = None
        self._refresh_control_states()

        if checked:
            self.arrow_jog_checkbox.setFocus(Qt.OtherFocusReason)
            self.connection_label.setText(self._arrow_jog_status_text())
            self._log(
                "[GUI] Arrow jog armed. Hold UP for forward, DOWN for reverse. "
                "Press Q to quit jog mode. RIGHT arrow speeds up by 1 ms/step. "
                "LEFT arrow slows down by 1 ms/step."
            )
            return

        if self.backend is not None and not self._busy:
            ready_port = "unknown port"
            try:
                ready_port = self.backend.status().get("port") or "unknown port"
            except BaseException:
                pass
            self.connection_label.setText(f"Controller ready on {ready_port}.")
        elif self.backend is not None:
            self.connection_label.setText("Exiting arrow jog...")
        else:
            self.connection_label.setText("Backend not initialized.")
        self._log("[GUI] Arrow jog disarmed")

    def _initialize_backend(self):
        if self._busy:
            return
        selected_port = self._selected_port()
        init_target = selected_port or "auto-detect"
        self._set_busy(True, f"Initializing backend on {init_target}...")
        self._log(f"[GUI] Initializing controller backend on {init_target}")
        try:
            if self.backend is not None and selected_port:
                current_status = self.backend.status()
                current_port = current_status.get("port")
                if current_port and current_port != selected_port:
                    self._log(f"[GUI] Reconnecting controller from {current_port} to {selected_port}")
                    self.backend.shutdown()
                    self.backend = None
            self.backend = backend_module.MonochromatorGUIBackend()
            snapshot = self.backend.initialize(selected_port)
            ready_port = snapshot.get("port") or init_target
            self._set_busy(False, f"Controller ready on {ready_port}.")
            self._set_backend_ready(True)
            self._update_status(snapshot)
            self._refresh_calibration_views(silent=True)
            self._log("[GUI] Backend initialized")
        except BaseException:
            error_text = traceback.format_exc()
            self.backend = None
            self._set_busy(False, "Initialization failed.")
            self._set_backend_ready(False)
            self._log("[GUI] Backend initialization failed")
            self._log(error_text)
            QMessageBox.critical(self, "Backend initialization failed", error_text)

    def _run_task(self, label, fn, *args, task_kind="default", log_message=True, **kwargs):
        if self.backend is None:
            QMessageBox.warning(self, "Controller not ready", "Initialize the controller backend first.")
            return
        if self._busy or self._closing:
            return
        self._set_busy(True, f"{label}...")
        self._active_task_kind = task_kind
        if log_message:
            self._log(f"[GUI] {label}")
        self._task_thread = BackendTaskThread(fn, *args, **kwargs)
        self._task_thread.succeeded.connect(self._task_succeeded)
        self._task_thread.failed.connect(self._task_failed)
        self._task_thread.finished.connect(self._task_finished)
        self._task_thread.start()

    def _task_succeeded(self, result):
        task_kind = self._active_task_kind
        ready_message = self._arrow_jog_status_text() if self.arrow_jog_checkbox.isChecked() else "Controller ready."
        self._set_busy(False, "Closing controller..." if self._closing else ready_message)
        self._update_status(result)
        if task_kind in {"continuous_jog", "continuous_jog_speed"}:
            QTimer.singleShot(0, self._continue_continuous_jog)
            return
        if task_kind in {"s1_calibrate", "s2_calibrate", "wl_ref", "wl_fit", "wl_backup", "wl_clear", "zwo_calibration"}:
            self._refresh_calibration_views(silent=True)
        if task_kind == "zwo_calibration":
            self.zwo_info_output.setPlainText(self._format_zwo_result(result.get("_result")))
        if self._closing:
            self._log("[GUI] Operation finished during shutdown")
            return
        self._log("[GUI] Operation complete")

    def _task_failed(self, error_text):
        task_kind = self._active_task_kind
        fail_message = self._arrow_jog_status_text() if self.arrow_jog_checkbox.isChecked() else "Operation failed."
        self._set_busy(False, "Closing controller..." if self._closing else fail_message)
        if task_kind in {"continuous_jog", "continuous_jog_speed"}:
            self._jog_hold_direction = 0
            self._pending_jog_speed_ms = None
            if self._closing:
                return
            if "Operation cancelled" in error_text:
                self._log("[GUI] Jog cancelled")
                return
        self._log("[GUI] Operation failed")
        self._log(error_text)
        if self._closing:
            return
        QMessageBox.critical(self, "Operation failed", error_text)

    def _task_finished(self):
        thread = self.sender()
        if thread is self._task_thread:
            self._task_thread = None
            self._active_task_kind = None
        if thread is not None:
            thread.deleteLater()
        if self._closing and not self._shutdown_complete and self._task_thread is None:
            self._finish_shutdown_after_task()

    def _refresh_status_manual(self):
        if self.backend is None or self._busy:
            return
        try:
            status = self.backend.status()
            self._update_status(status)
            ready_port = status.get("port") or "unknown port"
            self.connection_label.setText(f"Controller ready on {ready_port}.")
        except BaseException:
            error_text = traceback.format_exc()
            self._log("[GUI] Status refresh failed")
            self._log(error_text)

    def _refresh_status_if_idle(self):
        if self.backend is not None and not self._busy and not self.arrow_jog_checkbox.isChecked():
            self._refresh_status_manual()

    def _apply_jog_speed(self):
        new_ms = self.jog_speed_spin.value()
        if self._busy and self._active_task_kind not in {"continuous_jog", "continuous_jog_speed"}:
            return
        if self._jog_hold_direction != 0 or self._active_task_kind in {"continuous_jog", "continuous_jog_speed"}:
            self._pending_jog_speed_ms = new_ms
            return
        self._run_task("Set jog speed", self.backend.set_jog_speed_ms, new_ms)

    def _adjust_jog_speed(self, delta_ms):
        new_ms = max(1, min(1000, self.jog_speed_spin.value() + int(delta_ms)))
        if self._busy and self._active_task_kind not in {"continuous_jog", "continuous_jog_speed"}:
            return
        self.jog_speed_spin.setValue(new_ms)
        if self._jog_hold_direction != 0 or self._active_task_kind in {"continuous_jog", "continuous_jog_speed"}:
            self._pending_jog_speed_ms = new_ms
            return
        self._run_task("Set jog speed", self.backend.set_jog_speed_ms, new_ms)

    def _jog_cw(self):
        self._run_task("Jog CW", self.backend.jog_step, +1, self.jog_steps_spin.value())

    def _jog_ccw(self):
        self._run_task("Jog CCW", self.backend.jog_step, -1, self.jog_steps_spin.value())

    def _start_continuous_jog(self, direction):
        if self.backend is None or self._closing:
            return
        self._jog_hold_direction = 1 if int(direction) > 0 else -1
        if not self._busy:
            self._continue_continuous_jog()

    def _stop_continuous_jog(self, direction=None):
        if direction is None or self._jog_hold_direction == direction:
            self._jog_hold_direction = 0

    def _continue_continuous_jog(self):
        if self.backend is None or self._closing or self._busy:
            return

        if self._pending_jog_speed_ms is not None:
            pending_ms = self._pending_jog_speed_ms
            self._pending_jog_speed_ms = None
            self._run_task(
                "Set jog speed",
                self.backend.set_jog_speed_ms,
                pending_ms,
                task_kind="continuous_jog_speed",
                log_message=False,
            )
            return

        if self._jog_hold_direction == 0 or not self.arrow_jog_checkbox.isChecked():
            return

        label = "Jog CW" if self._jog_hold_direction > 0 else "Jog CCW"
        self._run_task(
            label,
            self.backend.jog_step,
            self._jog_hold_direction,
            1,
            task_kind="continuous_jog",
            log_message=False,
        )

    def _cancel_current_operation(self):
        if self.backend is None:
            return
        self._jog_hold_direction = 0
        self.backend.cancel()
        self._log("[GUI] Cancel requested")

    def _shutdown_backend(self):
        self.status_timer.stop()
        app = QApplication.instance()
        if app is not None:
            try:
                app.removeEventFilter(self)
            except Exception:
                pass

        if self.backend is None:
            return

        self._log("[GUI] Shutting down controller backend")
        try:
            self.backend.shutdown()
            self._log("[GUI] Backend shutdown complete")
        except BaseException:
            error_text = traceback.format_exc()
            self._log("[GUI] Backend shutdown failed")
            self._log(error_text)
        finally:
            self.backend = None
            self._set_backend_ready(False)

    def _finish_shutdown_after_task(self):
        if self._shutdown_complete:
            return
        self._shutdown_backend()
        self._shutdown_complete = True
        QTimer.singleShot(0, self.close)

    def eventFilter(self, obj, event):
        if self.arrow_jog_checkbox.isChecked() and self.backend is not None:
            if event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
                key = event.key()
                if key == Qt.Key_Up:
                    self._stop_continuous_jog(+1)
                    return True
                if key == Qt.Key_Down:
                    self._stop_continuous_jog(-1)
                    return True
            if self._arrow_jog_blocked_by_focus():
                return super().eventFilter(obj, event)
            if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
                key = event.key()
                modifiers = event.modifiers()
                if key == Qt.Key_Escape:
                    self.arrow_jog_checkbox.setChecked(False)
                    return True
                if key == Qt.Key_Q and modifiers == Qt.NoModifier:
                    self.arrow_jog_checkbox.setChecked(False)
                    return True
                if key == Qt.Key_Up:
                    if self._busy:
                        return True
                    self._start_continuous_jog(+1)
                    return True
                if key == Qt.Key_Down:
                    if self._busy:
                        return True
                    self._start_continuous_jog(-1)
                    return True
                if key == Qt.Key_Right:
                    self._adjust_jog_speed(-1)
                    return True
                if key == Qt.Key_Left:
                    self._adjust_jog_speed(+1)
                    return True
        return super().eventFilter(obj, event)

    def _scan_common_kwargs(self):
        return {
            "mode": self.scan_mode_combo.currentData(),
            "repeats": self.scan_repeats_spin.value(),
            "dwell_s": self.scan_dwell_spin.value(),
            "rehome_each_repeat": self.scan_rehome.isChecked(),
            "pingpong": self.scan_pingpong.isChecked(),
        }

    def _start_wavelength_scan(self):
        kwargs = self._scan_common_kwargs()
        kwargs["margin_nm"] = self.scan_lam_margin.value()
        self._run_task(
            "Scan wavelength",
            self.backend.scan_wavelength,
            self.scan_lam_start.value(),
            self.scan_lam_end.value(),
            self.scan_lam_order.value(),
            self.scan_lam_step.value(),
            **kwargs,
        )

    def _start_angle_scan(self):
        kwargs = self._scan_common_kwargs()
        kwargs["margin_deg"] = self.scan_theta_margin.value()
        self._run_task(
            "Scan angle",
            self.backend.scan_angle,
            self.scan_theta_start.value(),
            self.scan_theta_end.value(),
            self.scan_theta_step.value(),
            **kwargs,
        )

    def _start_s2_calibration(self):
        if self.backend is None or self._busy or self._closing:
            return
        self._run_task(
            "Calibrate S2 360°",
            self.backend.calibrate_s2,
            self.s2_approach_combo.currentData(),
            task_kind="s2_calibrate",
        )

    def _start_s1_calibration(self):
        if self.backend is None or self._busy or self._closing:
            return
        self._run_task(
            "Calibrate S1 360°",
            self.backend.calibrate_s1,
            task_kind="s1_calibrate",
        )

    def _add_wavelength_ref_current(self):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._validate_nonzero_order(self.wl_ref_order_spin.value(), "Add wavelength reference"):
            return
        self._run_task(
            "Add wavelength ref (current θ)",
            self.backend.add_wavelength_ref_current_angle,
            self.wl_ref_lambda_spin.value(),
            self.wl_ref_order_spin.value(),
            task_kind="wl_ref",
        )

    def _add_wavelength_ref_entered(self):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._validate_nonzero_order(self.wl_ref_order_spin.value(), "Add wavelength reference"):
            return
        self._run_task(
            "Add wavelength ref (entered θ)",
            self.backend.add_wavelength_ref_entered_angle,
            self.wl_ref_theta_spin.value(),
            self.wl_ref_lambda_spin.value(),
            self.wl_ref_order_spin.value(),
            task_kind="wl_ref",
        )

    def _fit_wavelength_model(self, fit_kind, label):
        if self.backend is None or self._busy or self._closing:
            return
        self._run_task(label, self.backend.fit_wavelength_model, fit_kind, task_kind="wl_fit")

    def _restore_wavelength_backup(self):
        if self.backend is None or self._busy or self._closing:
            return
        self._run_task("Restore wavelength backup", self.backend.restore_wavelength_backup, task_kind="wl_backup")

    def _clear_wavelength_calibration(self):
        if self.backend is None or self._busy or self._closing:
            return
        reply = QMessageBox.question(
            self,
            "Clear Wavelength Calibration",
            "This will clear ALL wavelength references and the saved wavelength model.\n\nContinue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        if not self._confirm_reset_phrase("Clear Wavelength Calibration", "CLEAR WAVELENGTH CAL"):
            return
        self._run_task("Clear wavelength calibration", self.backend.clear_wavelength_calibration, task_kind="wl_clear")

    def _run_zwo_reference_capture(self):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._validate_nonzero_order(self.zwo_order_spin.value(), "ZWO-assisted wavelength reference"):
            return
        self.zwo_info_output.setPlainText("Running experimental ZWO-assisted reference capture...")
        self._run_task(
            "ZWO-assisted wavelength reference",
            self.backend.zwo_assisted_reference_capture_gui,
            task_kind="zwo_calibration",
            lambda_nm=self.zwo_lambda_spin.value(),
            order_m=self.zwo_order_spin.value(),
            premove=self.zwo_premove_check.isChecked(),
            mode=self.zwo_mode_combo.currentData(),
            half_width_steps=self.zwo_half_width_spin.value(),
            step_size_steps=self.zwo_step_size_spin.value(),
            settle_s=self.zwo_settle_spin.value(),
            average_frames=self.zwo_avg_frames_spin.value(),
            camera_index=self.zwo_camera_index_spin.value(),
            exposure_ms=self.zwo_exposure_spin.value(),
            gain=self.zwo_gain_spin.value(),
            lib_path=self.zwo_lib_path_edit.text().strip() or None,
            roi=self.zwo_roi_edit.text().strip() or None,
            move_to_peak=self.zwo_move_to_peak_check.isChecked(),
            fit_mode=self.zwo_fit_mode_combo.currentData(),
        )

    def _confirm_reset_phrase(self, title, phrase):
        typed_text, ok = QInputDialog.getText(
            self,
            title,
            f"Type exactly:\n{phrase}",
        )
        if not ok:
            self._log(f"[GUI] {title} cancelled")
            return False
        if typed_text.strip() != phrase:
            QMessageBox.warning(
                self,
                title,
                "Confirmation text did not match. Reset cancelled.",
            )
            self._log(f"[GUI] {title} cancelled: confirmation text mismatch")
            return False
        return True

    def _reset_disk_home(self):
        if self.backend is None or self._busy or self._closing:
            return

        reply = QMessageBox.question(
            self,
            "Reset Disk Home",
            (
                "This will save the current position as the new Disk Home and reset step count to 0.\n\n"
                "Recommended workflow:\n"
                "1. Move to the true disk-home position first.\n"
                "2. Make sure S2 is centered/open as intended.\n"
                "3. Then save the current position.\n\n"
                "Continue?"
            ),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        if not self._confirm_reset_phrase("Reset Disk Home", "RESET DISK HOME"):
            return

        self._run_task("Reset Disk Home", self.backend.reset_disk_home_here)

    def _reset_optical_home(self):
        if self.backend is None or self._busy or self._closing:
            return

        reply = QMessageBox.question(
            self,
            "Reset Optical Home",
            (
                "This will save the current position as the new Optical Home offset.\n\n"
                "Recommended workflow:\n"
                "1. Go to Disk Home.\n"
                "2. Jog to the desired optical-home position.\n"
                "3. Then save the current position.\n\n"
                "Continue?"
            ),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        if not self._confirm_reset_phrase("Reset Optical Home", "RESET OPTICAL HOME"):
            return

        self._run_task("Reset Optical Home", self.backend.reset_optical_home_here)

    def _update_status(self, status):
        if not isinstance(status, dict):
            return

        def set_text(key, value):
            self.status_value_labels[key].setText(value)

        set_text("step_count", str(status.get("step_count", "--")))
        theta = status.get("theta_deg")
        set_text("theta_deg", "--" if theta is None else f"{theta:.6f} deg")
        est_wl = status.get("estimated_wavelength_nm")
        set_text("estimated_wavelength_nm", "--" if est_wl is None else f"{est_wl:.3f} nm")
        set_text("s1", str(status.get("s1", "--")))
        set_text("s2", str(status.get("s2", "--")))
        jog_speed_ms = status.get("jog_speed_ms")
        set_text("jog_speed_ms", "--" if jog_speed_ms is None else f"{jog_speed_ms} ms/step")
        set_text("optical_offset_steps", str(status.get("optical_offset_steps", "--")))
        set_text("home_window_open", "Yes" if status.get("home_window_open") else "No")
        set_text("debug_enabled", "On" if status.get("debug_enabled") else "Off")

        if jog_speed_ms is not None and not self._widget_or_child_has_focus(self.jog_speed_spin):
            self.jog_speed_spin.setValue(int(jog_speed_ms))

    def closeEvent(self, event):
        if self._shutdown_complete:
            event.accept()
            super().closeEvent(event)
            return

        self._jog_hold_direction = 0
        self._pending_jog_speed_ms = None
        self._closing = True
        self._set_busy(self._busy, "Closing controller...")

        if self.backend is not None and self._task_thread is not None and self._task_thread.isRunning():
            self._log("[GUI] Close requested; cancelling active operation")
            self.backend.cancel()
            event.ignore()
            return

        self._shutdown_backend()
        self._shutdown_complete = True
        event.accept()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MonochromatorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
