import sys
import traceback

try:
    from PySide6.QtCore import QEvent, QThread, QTimer, Qt, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPlainTextEdit,
        QPushButton,
        QSpinBox,
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
        self._closing = False
        self._shutdown_complete = False

        self.setWindowTitle("Monochromator Control")
        self.resize(1200, 860)

        self._build_ui()
        self._set_backend_ready(False)
        self._set_busy(False)

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(750)
        self.status_timer.timeout.connect(self._refresh_status_if_idle)
        self.status_timer.start()
        QApplication.instance().installEventFilter(self)

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_status_group())

        top_row = QHBoxLayout()
        top_row.addWidget(self._build_jog_group(), 1)
        top_row.addWidget(self._build_home_group(), 1)
        top_row.addWidget(self._build_move_group(), 2)
        root.addLayout(top_row)

        root.addWidget(self._build_scan_group())
        root.addWidget(self._build_log_group(), 1)

        self.setCentralWidget(central)

    def _build_connection_group(self):
        box = QGroupBox("Connection")
        layout = QHBoxLayout(box)

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
        layout.addWidget(self.arrow_jog_checkbox)

        jog_row = QHBoxLayout()
        self.ccw_button = QPushButton("CCW Step")
        self.ccw_button.clicked.connect(self._jog_ccw)
        jog_row.addWidget(self.ccw_button)

        self.cw_button = QPushButton("CW Step")
        self.cw_button.clicked.connect(self._jog_cw)
        jog_row.addWidget(self.cw_button)
        layout.addLayout(jog_row)

        hint = QLabel("Use exact step jogs for manual positioning. CW is positive. When armed, Up/Down jog and Left/Right change speed.")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return box

    def _build_home_group(self):
        box = QGroupBox("Home")
        layout = QVBoxLayout(box)

        self.disk_home_button = QPushButton("Go Disk Home")
        self.disk_home_button.clicked.connect(lambda: self._run_task("Go Disk Home", self.backend.go_disk_home))
        layout.addWidget(self.disk_home_button)

        self.optical_home_button = QPushButton("Go Optical Home")
        self.optical_home_button.clicked.connect(lambda: self._run_task("Go Optical Home", self.backend.go_optical_home))
        layout.addWidget(self.optical_home_button)

        hint = QLabel("CLI-only calibration flows remain in controller.py. This GUI is the new control surface.")
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

        tabs = QTabWidget()
        tabs.addTab(self._build_wavelength_scan_tab(), "Wavelength")
        tabs.addTab(self._build_angle_scan_tab(), "Angle")
        layout.addWidget(tabs)
        return box

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

        start_button = QPushButton("Start Wavelength Scan")
        start_button.clicked.connect(self._start_wavelength_scan)
        layout.addWidget(start_button, 2, 3)
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

        start_button = QPushButton("Start Angle Scan")
        start_button.clicked.connect(self._start_angle_scan)
        layout.addWidget(start_button, 2, 3)
        return tab

    def _build_log_group(self):
        box = QGroupBox("Activity")
        layout = QVBoxLayout(box)
        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)
        return box

    def _set_backend_ready(self, ready):
        controls_ready = bool(ready) and not self._busy and not self._closing
        self.refresh_button.setEnabled(bool(ready) and not self._closing)
        self.cancel_button.setEnabled(bool(ready) and not self._closing)
        for widget in [
            self.apply_speed_button,
            self.ccw_button,
            self.cw_button,
            self.disk_home_button,
            self.optical_home_button,
            self.relative_move_button,
            self.absolute_move_button,
            self.angle_move_button,
            self.wavelength_move_button,
        ]:
            widget.setEnabled(controls_ready)

    def _set_busy(self, busy, message=None):
        self._busy = bool(busy)
        self.init_button.setEnabled(not busy and not self._closing)
        self.close_button.setEnabled(not self._shutdown_complete)
        self._set_backend_ready(self.backend is not None)
        if message:
            self.connection_label.setText(message)

    def _log(self, message):
        self.log_output.appendPlainText(message)

    def _initialize_backend(self):
        if self._busy:
            return
        self._set_busy(True, "Initializing backend...")
        self._log("[GUI] Initializing controller backend")
        try:
            self.backend = backend_module.MonochromatorGUIBackend()
            snapshot = self.backend.initialize()
            self._set_busy(False, "Controller ready.")
            self._set_backend_ready(True)
            self._update_status(snapshot)
            self._log("[GUI] Backend initialized")
        except BaseException:
            error_text = traceback.format_exc()
            self.backend = None
            self._set_busy(False, "Initialization failed.")
            self._set_backend_ready(False)
            self._log("[GUI] Backend initialization failed")
            self._log(error_text)
            QMessageBox.critical(self, "Backend initialization failed", error_text)

    def _run_task(self, label, fn, *args, **kwargs):
        if self.backend is None:
            QMessageBox.warning(self, "Controller not ready", "Initialize the controller backend first.")
            return
        if self._busy or self._closing:
            return
        self._set_busy(True, f"{label}...")
        self._log(f"[GUI] {label}")
        self._task_thread = BackendTaskThread(fn, *args, **kwargs)
        self._task_thread.succeeded.connect(self._task_succeeded)
        self._task_thread.failed.connect(self._task_failed)
        self._task_thread.finished.connect(self._task_finished)
        self._task_thread.start()

    def _task_succeeded(self, result):
        self._set_busy(False, "Closing controller..." if self._closing else "Controller ready.")
        self._update_status(result)
        if self._closing:
            self._log("[GUI] Operation finished during shutdown")
            return
        self._log("[GUI] Operation complete")

    def _task_failed(self, error_text):
        self._set_busy(False, "Closing controller..." if self._closing else "Operation failed.")
        self._log("[GUI] Operation failed")
        self._log(error_text)
        if self._closing:
            return
        QMessageBox.critical(self, "Operation failed", error_text)

    def _task_finished(self):
        thread = self.sender()
        if thread is self._task_thread:
            self._task_thread = None
        if thread is not None:
            thread.deleteLater()
        if self._closing and not self._shutdown_complete and self._task_thread is None:
            self._finish_shutdown_after_task()

    def _refresh_status_manual(self):
        if self.backend is None or self._busy:
            return
        try:
            self._update_status(self.backend.status())
            self.connection_label.setText("Controller ready.")
        except BaseException:
            error_text = traceback.format_exc()
            self._log("[GUI] Status refresh failed")
            self._log(error_text)

    def _refresh_status_if_idle(self):
        if self.backend is not None and not self._busy:
            self._refresh_status_manual()

    def _apply_jog_speed(self):
        self._run_task("Set jog speed", self.backend.set_jog_speed_ms, self.jog_speed_spin.value())

    def _adjust_jog_speed(self, delta_ms):
        new_ms = max(1, min(1000, self.jog_speed_spin.value() + int(delta_ms)))
        self.jog_speed_spin.setValue(new_ms)
        self._run_task("Set jog speed", self.backend.set_jog_speed_ms, new_ms)

    def _jog_cw(self):
        self._run_task("Jog CW", self.backend.jog_step, +1, self.jog_steps_spin.value())

    def _jog_ccw(self):
        self._run_task("Jog CCW", self.backend.jog_step, -1, self.jog_steps_spin.value())

    def _cancel_current_operation(self):
        if self.backend is None:
            return
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
        if (
            event.type() == QEvent.KeyPress
            and self.arrow_jog_checkbox.isChecked()
            and not event.isAutoRepeat()
            and self.backend is not None
            and not self._busy
        ):
            key = event.key()
            if key == Qt.Key_Up:
                self._jog_cw()
                return True
            if key == Qt.Key_Down:
                self._jog_ccw()
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

        if jog_speed_ms is not None:
            self.jog_speed_spin.setValue(int(jog_speed_ms))

    def closeEvent(self, event):
        if self._shutdown_complete:
            event.accept()
            super().closeEvent(event)
            return

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
