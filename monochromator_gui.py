import os
import sys
import time
import traceback

try:
    import numpy as np
except ImportError:
    np = None

try:
    from PySide6.QtCore import QEvent, QPoint, QPointF, QRect, QRectF, QThread, QTimer, Qt, Signal
    from PySide6.QtGui import QColor, QImage, QKeySequence, QPainter, QPen, QPixmap, QPolygonF, QShortcut
    from PySide6.QtWidgets import (
        QAbstractSpinBox,
        QApplication,
        QCheckBox,
        QComboBox,
        QDialog,
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


class ZWOLivePreviewThread(QThread):
    frame_ready = Signal(object)
    failed = Signal(str)

    def __init__(self, backend, fps=30.0):
        super().__init__()
        self._backend = backend
        self._fps = max(0.5, float(fps))
        self._running = True

    def stop(self):
        self._running = False
        self.requestInterruption()

    def run(self):
        frame_interval_s = 1.0 / self._fps
        try:
            while self._running and not self.isInterruptionRequested():
                frame_started = time.monotonic()
                payload = self._backend.capture_zwo_live_frame()
                if not self._running or self.isInterruptionRequested():
                    break
                self.frame_ready.emit(payload)
                remaining_s = frame_interval_s - (time.monotonic() - frame_started)
                if remaining_s > 0:
                    self.msleep(max(1, int(round(remaining_s * 1000.0))))
        except BaseException:
            if self._running and not self.isInterruptionRequested():
                self.failed.emit(traceback.format_exc())


class ArrowJogDialog(QDialog):
    start_direction = Signal(int)
    stop_direction = Signal(int)
    speed_delta = Signal(int)
    exit_requested = Signal()
    close_gui_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Arrow Jog Mode")
        self.setModal(True)
        self.setWindowModality(Qt.ApplicationModal)
        self.setMinimumWidth(520)

        layout = QVBoxLayout(self)
        message = QLabel(
            "Arrow jog is active.\n\n"
            "Hold UP for forward.\n"
            "Hold DOWN for reverse.\n"
            "RIGHT speeds up by 1 ms/step.\n"
            "LEFT slows down by 1 ms/step.\n\n"
            "Press Q or Esc to exit jog mode."
        )
        message.setWordWrap(True)
        layout.addWidget(message)

        button_row = QHBoxLayout()
        self.exit_button = QPushButton("Exit Jog")
        self.exit_button.clicked.connect(self.exit_requested.emit)
        button_row.addWidget(self.exit_button)

        self.close_gui_button = QPushButton("Close GUI")
        self.close_gui_button.clicked.connect(self.close_gui_requested.emit)
        button_row.addWidget(self.close_gui_button)
        layout.addLayout(button_row)

    def keyPressEvent(self, event):
        key = event.key()
        modifiers = event.modifiers()
        if event.isAutoRepeat():
            event.accept()
            return
        if key == Qt.Key_Escape or (key == Qt.Key_Q and modifiers == Qt.NoModifier):
            self.exit_requested.emit()
            event.accept()
            return
        if key == Qt.Key_Up:
            self.start_direction.emit(+1)
            event.accept()
            return
        if key == Qt.Key_Down:
            self.start_direction.emit(-1)
            event.accept()
            return
        if key == Qt.Key_Right:
            self.speed_delta.emit(-1)
            event.accept()
            return
        if key == Qt.Key_Left:
            self.speed_delta.emit(+1)
            event.accept()
            return
        event.accept()

    def keyReleaseEvent(self, event):
        key = event.key()
        if event.isAutoRepeat():
            event.accept()
            return
        if key == Qt.Key_Up:
            self.stop_direction.emit(+1)
            event.accept()
            return
        if key == Qt.Key_Down:
            self.stop_direction.emit(-1)
            event.accept()
            return
        if key in {Qt.Key_Left, Qt.Key_Right, Qt.Key_Escape, Qt.Key_Q}:
            event.accept()
            return
        event.accept()

    def closeEvent(self, event):
        self.exit_requested.emit()
        event.accept()


class CLIJogDialog(QDialog):
    stop_requested = Signal()
    close_gui_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("CLI Jog Mode")
        self.setModal(True)
        self.setWindowModality(Qt.ApplicationModal)
        self.setMinimumWidth(560)

        layout = QVBoxLayout(self)
        message = QLabel(
            "CLI-style jog is active.\n\n"
            "Use the same keyboard controls as the working terminal jog:\n"
            "UP = forward\n"
            "DOWN = reverse\n"
            "RIGHT = faster by 1 ms/step\n"
            "LEFT = slower by 1 ms/step\n"
            "Q = quit jog mode\n\n"
            "Live jog status will continue printing in the terminal."
        )
        message.setWordWrap(True)
        layout.addWidget(message)

        button_row = QHBoxLayout()
        self.stop_button = QPushButton("Stop Jog")
        self.stop_button.clicked.connect(self.stop_requested.emit)
        button_row.addWidget(self.stop_button)

        self.close_gui_button = QPushButton("Close GUI")
        self.close_gui_button.clicked.connect(self.close_gui_requested.emit)
        button_row.addWidget(self.close_gui_button)
        layout.addLayout(button_row)

    def closeEvent(self, event):
        self.stop_requested.emit()
        event.accept()


class ZWOFramePreview(QWidget):
    roi_selected = Signal(int, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 280)
        self._placeholder_text = "No frame captured."
        self._pixmap = None
        self._frame_width = 0
        self._frame_height = 0
        self._analysis_roi = None
        self._dragging = False
        self._drag_start = None
        self._drag_end = None

    def set_placeholder_text(self, text):
        self._placeholder_text = text or ""
        self._pixmap = None
        self._frame_width = 0
        self._frame_height = 0
        self._analysis_roi = None
        self._dragging = False
        self._drag_start = None
        self._drag_end = None
        self.update()

    def set_frame_array(self, frame_array):
        if frame_array is None:
            self.set_placeholder_text("No frame captured.")
            return
        try:
            height, width = frame_array.shape[:2]
            image = QImage(frame_array.data, width, height, width, QImage.Format_Grayscale8).copy()
            self._pixmap = QPixmap.fromImage(image)
            self._frame_width = int(width)
            self._frame_height = int(height)
            self._placeholder_text = ""
            self._analysis_roi = self._clamp_roi(self._analysis_roi)
        except BaseException:
            self.set_placeholder_text("Frame captured, but preview rendering failed.")
            return
        self.update()

    def set_analysis_roi(self, roi):
        self._analysis_roi = self._clamp_roi(roi)
        self.update()

    def _clamp_roi(self, roi):
        if roi is None or self._frame_width <= 0 or self._frame_height <= 0:
            return None
        x, y, width, height = [int(v) for v in roi]
        x = max(0, min(x, self._frame_width - 1))
        y = max(0, min(y, self._frame_height - 1))
        width = max(1, min(width, self._frame_width - x))
        height = max(1, min(height, self._frame_height - y))
        return (x, y, width, height)

    def _frame_rect(self):
        if self._pixmap is None or self._frame_width <= 0 or self._frame_height <= 0:
            return QRect()
        widget_w = max(1, self.width() - 8)
        widget_h = max(1, self.height() - 8)
        scale = min(widget_w / self._frame_width, widget_h / self._frame_height)
        draw_w = max(1, int(round(self._frame_width * scale)))
        draw_h = max(1, int(round(self._frame_height * scale)))
        x = (self.width() - draw_w) // 2
        y = (self.height() - draw_h) // 2
        return QRect(x, y, draw_w, draw_h)

    def _map_widget_point_to_image(self, point):
        frame_rect = self._frame_rect()
        if (
            self._pixmap is None
            or frame_rect.isEmpty()
            or not frame_rect.contains(point)
            or self._frame_width <= 0
            or self._frame_height <= 0
        ):
            return None
        rel_x = (point.x() - frame_rect.x()) / max(1, frame_rect.width())
        rel_y = (point.y() - frame_rect.y()) / max(1, frame_rect.height())
        image_x = max(0, min(self._frame_width - 1, int(rel_x * self._frame_width)))
        image_y = max(0, min(self._frame_height - 1, int(rel_y * self._frame_height)))
        return (image_x, image_y)

    def _selection_from_points(self, start_point, end_point):
        if start_point is None or end_point is None:
            return None
        x1 = min(start_point[0], end_point[0])
        x2 = max(start_point[0], end_point[0])
        y1 = min(start_point[1], end_point[1])
        y2 = max(start_point[1], end_point[1])
        return self._clamp_roi((x1, y1, (x2 - x1) + 1, (y2 - y1) + 1))

    def _draw_roi(self, painter, roi, color, width=2, style=Qt.SolidLine):
        frame_rect = self._frame_rect()
        if roi is None or frame_rect.isEmpty() or self._frame_width <= 0 or self._frame_height <= 0:
            return
        scale_x = frame_rect.width() / float(self._frame_width)
        scale_y = frame_rect.height() / float(self._frame_height)
        x, y, roi_w, roi_h = roi
        draw_rect = QRectF(
            frame_rect.x() + (x * scale_x),
            frame_rect.y() + (y * scale_y),
            max(scale_x, roi_w * scale_x),
            max(scale_y, roi_h * scale_y),
        )
        pen = QPen(color, width, style)
        painter.setPen(pen)
        painter.drawRect(draw_rect)

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111111"))
        painter.setPen(QColor("#444444"))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        frame_rect = self._frame_rect()
        if self._pixmap is None or frame_rect.isEmpty():
            painter.setPen(QColor("#dddddd"))
            painter.drawText(self.rect().adjusted(12, 12, -12, -12), Qt.AlignCenter | Qt.TextWordWrap, self._placeholder_text)
            return

        painter.drawPixmap(frame_rect, self._pixmap)
        if self._analysis_roi is not None:
            self._draw_roi(painter, self._analysis_roi, QColor("#34c759"), width=2)
        if self._dragging:
            drag_roi = self._selection_from_points(self._drag_start, self._drag_end)
            if drag_roi is not None:
                self._draw_roi(painter, drag_roi, QColor("#ffd60a"), width=2, style=Qt.DashLine)

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return super().mousePressEvent(event)
        point = event.position().toPoint() if hasattr(event, "position") else event.pos()
        image_point = self._map_widget_point_to_image(point)
        if image_point is None:
            event.ignore()
            return
        self._dragging = True
        self._drag_start = image_point
        self._drag_end = image_point
        self.update()
        event.accept()

    def mouseMoveEvent(self, event):
        if not self._dragging:
            return super().mouseMoveEvent(event)
        point = event.position().toPoint() if hasattr(event, "position") else event.pos()
        image_point = self._map_widget_point_to_image(point)
        if image_point is not None:
            self._drag_end = image_point
            self.update()
        event.accept()

    def mouseReleaseEvent(self, event):
        if not self._dragging or event.button() != Qt.LeftButton:
            return super().mouseReleaseEvent(event)
        point = event.position().toPoint() if hasattr(event, "position") else event.pos()
        image_point = self._map_widget_point_to_image(point)
        if image_point is not None:
            self._drag_end = image_point
        self._dragging = False
        roi = self._selection_from_points(self._drag_start, self._drag_end)
        self._drag_start = None
        self._drag_end = None
        if roi is not None:
            self._analysis_roi = roi
            self.roi_selected.emit(*roi)
        self.update()
        event.accept()


class ZWOLineProfileWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(180)
        self._profile = None
        self._peak_index = None
        self._centroid_index = None
        self._axis_label = "Horizontal"

    def set_profile(self, profile, *, peak_index=None, centroid_index=None, axis_label="Horizontal"):
        self._profile = profile
        self._peak_index = peak_index
        self._centroid_index = centroid_index
        self._axis_label = axis_label
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#101419"))
        painter.setPen(QColor("#3a3f45"))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        inner = self.rect().adjusted(12, 16, -12, -20)
        if self._profile is None or len(self._profile) == 0:
            painter.setPen(QColor("#d0d4d8"))
            painter.drawText(inner, Qt.AlignCenter | Qt.TextWordWrap, "No profile yet.\nCapture or stream a frame to plot the slit profile.")
            return

        values = self._profile
        count = len(values)
        if count <= 0:
            painter.setPen(QColor("#d0d4d8"))
            painter.drawText(inner, Qt.AlignCenter, "No profile yet.")
            return

        max_value = max(float(v) for v in values)
        if max_value <= 0.0:
            max_value = 1.0

        polygon = QPolygonF()
        if count == 1:
            x = float(inner.left())
            y = float(inner.bottom())
            polygon.append(QPointF(x, y))
            polygon.append(QPointF(x, y))
        else:
            for idx, raw_value in enumerate(values):
                x = inner.left() + (idx / float(count - 1)) * inner.width()
                y = inner.bottom() - ((float(raw_value) / max_value) * inner.height())
                polygon.append(QPointF(x, y))

        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QColor("#59b9ff"), 2))
        painter.drawPolyline(polygon)

        def draw_marker(index_value, color, label):
            if index_value is None or count <= 0:
                return
            if count == 1:
                x = inner.left()
            else:
                x = inner.left() + (float(index_value) / float(max(1, count - 1))) * inner.width()
            painter.setPen(QPen(color, 1.5, Qt.DashLine))
            painter.drawLine(QPointF(x, inner.top()), QPointF(x, inner.bottom()))
            painter.setPen(color)
            painter.drawText(QRectF(x + 4, inner.top() + 2, 120, 18), label)

        draw_marker(self._peak_index, QColor("#ffd60a"), "Peak")
        draw_marker(self._centroid_index, QColor("#34c759"), "Centroid")

        painter.setPen(QColor("#d0d4d8"))
        painter.drawText(QRectF(inner.left(), 2, inner.width(), 14), Qt.AlignLeft | Qt.AlignVCenter, f"{self._axis_label} profile")


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
        self._mouse_jog_active = False
        self._mouse_jog_press_direction = 0
        self._mouse_jog_is_holding = False
        self._zwo_live_thread = None
        self._zwo_live_error_seen = False
        self._zwo_live_stop_requested = False
        self._zwo_live_last_frame_ts = None
        self._zwo_live_actual_fps = None
        self._zwo_live_info_last_update_ts = None
        self._zwo_resume_live_after_center = False
        self._zwo_resume_live_after_roi_change = False
        self._zwo_pending_camera_settings = None
        self._zwo_last_frame_array = None
        self._zwo_last_camera_status = {}
        self._zwo_analysis_local_roi = None
        self._arrow_jog_dialog = None
        self._arrow_jog_dialog_closing = False
        self._cli_jog_dialog = None
        self._cli_jog_dialog_closing = False

        self.setWindowTitle("Monochromator Control")
        self.resize(1200, 860)

        self._build_ui()
        if self.centralWidget() is not None:
            self.centralWidget().setFocusPolicy(Qt.StrongFocus)

        self._mouse_jog_hold_timer = QTimer(self)
        self._mouse_jog_hold_timer.setSingleShot(True)
        self._mouse_jog_hold_timer.timeout.connect(self._begin_mouse_hold_jog)

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
        self.workspace_splitter.setHandleWidth(10)
        self.workspace_splitter.setStyleSheet(
            "QSplitter::handle { background: #2c2f33; } "
            "QSplitter::handle:hover { background: #4c5258; }"
        )
        self.workspace_splitter.addWidget(self.workspace_tabs)
        self.workspace_splitter.addWidget(self._build_log_group())
        self.workspace_splitter.setStretchFactor(0, 4)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes([700, 180])

        self.camera_pane_scroll = self._wrap_workspace_page(self._build_zwo_camera_tab())
        self.camera_pane_scroll.setMinimumWidth(520)

        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.setHandleWidth(12)
        self.main_splitter.setStyleSheet(
            "QSplitter::handle { background: #2c2f33; } "
            "QSplitter::handle:hover { background: #4c5258; }"
        )
        self.main_splitter.addWidget(self.workspace_splitter)
        self.main_splitter.addWidget(self.camera_pane_scroll)
        self.main_splitter.setStretchFactor(0, 35)
        self.main_splitter.setStretchFactor(1, 65)
        self.main_splitter.setSizes([420, 780])
        root.addWidget(self.main_splitter, 1)

        self._configure_splitter_handles()

        self.setCentralWidget(central)

    def _wrap_workspace_page(self, widget):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        return scroll

    def _configure_splitter_handles(self):
        for index in range(1, self.main_splitter.count()):
            self.main_splitter.handle(index).setCursor(Qt.SplitHCursor)
        for index in range(1, self.workspace_splitter.count()):
            self.workspace_splitter.handle(index).setCursor(Qt.SplitVCursor)

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

        self.camera_pane_toggle_button = QPushButton("Hide Camera Pane")
        self.camera_pane_toggle_button.setCheckable(True)
        self.camera_pane_toggle_button.setChecked(True)
        self.camera_pane_toggle_button.clicked.connect(self._toggle_camera_pane)
        layout.addWidget(self.camera_pane_toggle_button)

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
        self.jog_steps_spin.setRange(1, 1)
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

        self.arrow_jog_checkbox = QCheckBox("CLI jog active")
        self.arrow_jog_checkbox.setChecked(False)
        self.arrow_jog_checkbox.hide()

        cli_row = QHBoxLayout()
        self.cli_jog_button = QPushButton("Start CLI Jog")
        self.cli_jog_button.clicked.connect(self._start_cli_jog)
        cli_row.addWidget(self.cli_jog_button)

        self.stop_cli_jog_button = QPushButton("Stop CLI Jog")
        self.stop_cli_jog_button.clicked.connect(self._stop_cli_jog)
        cli_row.addWidget(self.stop_cli_jog_button)
        layout.addLayout(cli_row)
        self.cli_jog_button.hide()
        self.stop_cli_jog_button.hide()

        jog_row = QHBoxLayout()
        self.ccw_button = QPushButton("CCW Step / Hold")
        self.ccw_button.pressed.connect(lambda: self._on_jog_button_pressed(-1))
        self.ccw_button.released.connect(lambda: self._on_jog_button_released(-1))
        jog_row.addWidget(self.ccw_button)

        self.cw_button = QPushButton("CW Step / Hold")
        self.cw_button.pressed.connect(lambda: self._on_jog_button_pressed(+1))
        self.cw_button.released.connect(lambda: self._on_jog_button_released(+1))
        jog_row.addWidget(self.cw_button)

        self.stop_jog_button = QPushButton("Stop")
        self.stop_jog_button.clicked.connect(self._stop_mouse_hold_jog)
        jog_row.addWidget(self.stop_jog_button)
        layout.addLayout(jog_row)

        self.jog_hint_label = QLabel(
            "Click a jog button once to move by the selected step count. Hold a jog button for continuous motion; "
            "release it to stop. Use the Jog speed field to control hold speed."
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

    def _build_zwo_camera_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        connection_box = QGroupBox("Camera Connection")
        connection_layout = QGridLayout(connection_box)

        self.zwo_sdk_path_edit = QLineEdit()
        self.zwo_sdk_path_edit.setPlaceholderText("ASICamera2 SDK path (optional; otherwise use $ZWO_ASI_LIB)")
        self.zwo_sdk_path_edit.setText(backend_module._find_zwo_sdk_path(compatible_only=True) or "")
        connection_layout.addWidget(QLabel("SDK path"), 0, 0)
        connection_layout.addWidget(self.zwo_sdk_path_edit, 0, 1, 1, 4)

        self.zwo_refresh_cameras_button = QPushButton("Refresh Cameras")
        self.zwo_refresh_cameras_button.clicked.connect(self._refresh_zwo_cameras)
        connection_layout.addWidget(self.zwo_refresh_cameras_button, 1, 0)

        self.zwo_camera_combo = QComboBox()
        self.zwo_camera_combo.setMinimumContentsLength(30)
        self.zwo_camera_combo.addItem("No cameras loaded", None)
        connection_layout.addWidget(QLabel("Camera"), 1, 1)
        connection_layout.addWidget(self.zwo_camera_combo, 1, 2, 1, 3)

        self.zwo_connect_button = QPushButton("Connect Camera")
        self.zwo_connect_button.clicked.connect(self._connect_zwo_camera)
        connection_layout.addWidget(self.zwo_connect_button, 2, 0)

        self.zwo_disconnect_button = QPushButton("Disconnect Camera")
        self.zwo_disconnect_button.clicked.connect(self._disconnect_zwo_camera)
        connection_layout.addWidget(self.zwo_disconnect_button, 2, 1)

        self.zwo_camera_connection_label = QLabel("No ZWO camera connected.")
        self.zwo_camera_connection_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        connection_layout.addWidget(self.zwo_camera_connection_label, 2, 2, 1, 3)

        layout.addWidget(connection_box)

        capture_box = QGroupBox("Acquisition")
        capture_layout = QGridLayout(capture_box)

        self.zwo_camera_exposure_spin = QDoubleSpinBox()
        self.zwo_camera_exposure_spin.setRange(0.01, 60000.0)
        self.zwo_camera_exposure_spin.setDecimals(3)
        self.zwo_camera_exposure_spin.setValue(50.0)
        self.zwo_camera_exposure_spin.setKeyboardTracking(False)
        self.zwo_camera_exposure_spin.editingFinished.connect(self._apply_zwo_camera_settings_from_editor)
        capture_layout.addWidget(QLabel("Exposure (ms)"), 0, 0)
        capture_layout.addWidget(self.zwo_camera_exposure_spin, 0, 1)

        self.zwo_camera_gain_spin = QSpinBox()
        self.zwo_camera_gain_spin.setRange(0, 1000)
        self.zwo_camera_gain_spin.setValue(100)
        self.zwo_camera_gain_spin.setKeyboardTracking(False)
        self.zwo_camera_gain_spin.editingFinished.connect(self._apply_zwo_camera_settings_from_editor)
        capture_layout.addWidget(QLabel("Gain"), 0, 2)
        capture_layout.addWidget(self.zwo_camera_gain_spin, 0, 3)

        self.zwo_apply_settings_button = QPushButton("Apply Settings")
        self.zwo_apply_settings_button.clicked.connect(self._apply_zwo_camera_settings)
        capture_layout.addWidget(self.zwo_apply_settings_button, 0, 4)

        self.zwo_capture_frame_button = QPushButton("Capture Single Frame")
        self.zwo_capture_frame_button.clicked.connect(self._capture_zwo_single_frame)
        capture_layout.addWidget(self.zwo_capture_frame_button, 1, 0)

        self.zwo_live_fps_spin = QDoubleSpinBox()
        self.zwo_live_fps_spin.setRange(0.5, 120.0)
        self.zwo_live_fps_spin.setDecimals(1)
        self.zwo_live_fps_spin.setSingleStep(1.0)
        self.zwo_live_fps_spin.setValue(30.0)
        capture_layout.addWidget(QLabel("Target FPS"), 1, 1)
        capture_layout.addWidget(self.zwo_live_fps_spin, 1, 2)

        self.zwo_start_live_button = QPushButton("Start Live")
        self.zwo_start_live_button.clicked.connect(self._start_zwo_live_preview)
        capture_layout.addWidget(self.zwo_start_live_button, 1, 3)

        self.zwo_stop_live_button = QPushButton("Stop Live")
        self.zwo_stop_live_button.clicked.connect(self._stop_zwo_live_preview)
        capture_layout.addWidget(self.zwo_stop_live_button, 1, 4)

        self.zwo_camera_status_summary = QLabel("No camera status available.")
        self.zwo_camera_status_summary.setWordWrap(True)
        capture_layout.addWidget(self.zwo_camera_status_summary, 2, 0, 1, 5)

        layout.addWidget(capture_box)

        roi_box = QGroupBox("ROI")
        roi_layout = QGridLayout(roi_box)

        self.zwo_roi_x_spin = QSpinBox()
        self.zwo_roi_x_spin.setRange(0, 100000)
        roi_layout.addWidget(QLabel("X"), 0, 0)
        roi_layout.addWidget(self.zwo_roi_x_spin, 0, 1)

        self.zwo_roi_y_spin = QSpinBox()
        self.zwo_roi_y_spin.setRange(0, 100000)
        roi_layout.addWidget(QLabel("Y"), 0, 2)
        roi_layout.addWidget(self.zwo_roi_y_spin, 0, 3)

        self.zwo_roi_w_spin = QSpinBox()
        self.zwo_roi_w_spin.setRange(8, 100000)
        self.zwo_roi_w_spin.setSingleStep(8)
        roi_layout.addWidget(QLabel("Width"), 1, 0)
        roi_layout.addWidget(self.zwo_roi_w_spin, 1, 1)

        self.zwo_roi_h_spin = QSpinBox()
        self.zwo_roi_h_spin.setRange(2, 100000)
        self.zwo_roi_h_spin.setSingleStep(2)
        roi_layout.addWidget(QLabel("Height"), 1, 2)
        roi_layout.addWidget(self.zwo_roi_h_spin, 1, 3)

        self.zwo_apply_roi_button = QPushButton("Apply ROI")
        self.zwo_apply_roi_button.clicked.connect(self._apply_zwo_camera_roi)
        roi_layout.addWidget(self.zwo_apply_roi_button, 0, 4)

        self.zwo_reset_roi_button = QPushButton("Full Frame")
        self.zwo_reset_roi_button.clicked.connect(self._reset_zwo_camera_roi)
        roi_layout.addWidget(self.zwo_reset_roi_button, 1, 4)

        roi_hint = QLabel("Width must be a multiple of 8. Height must be a multiple of 2. Smaller ROI can support higher live FPS.")
        roi_hint.setWordWrap(True)
        roi_layout.addWidget(roi_hint, 2, 0, 1, 5)

        layout.addWidget(roi_box)

        preview_box = QGroupBox("Live Preview / Analysis")
        preview_box_layout = QVBoxLayout(preview_box)
        preview_layout = QHBoxLayout()

        self.zwo_preview_label = ZWOFramePreview()
        self.zwo_preview_label.roi_selected.connect(self._on_zwo_preview_roi_selected)
        preview_layout.addWidget(self.zwo_preview_label, 2)

        side_panel = QWidget()
        side_layout = QVBoxLayout(side_panel)
        side_layout.setContentsMargins(0, 0, 0, 0)

        zwo_jog_box = QGroupBox("Jog While Viewing")
        zwo_jog_layout = QVBoxLayout(zwo_jog_box)

        zwo_speed_row = QHBoxLayout()
        zwo_speed_row.addWidget(QLabel("Jog speed (ms)"))
        self.zwo_jog_speed_spin = QSpinBox()
        self.zwo_jog_speed_spin.setRange(1, 1000)
        self.zwo_jog_speed_spin.setValue(1)
        zwo_speed_row.addWidget(self.zwo_jog_speed_spin)
        self.zwo_apply_jog_speed_button = QPushButton("Apply")
        self.zwo_apply_jog_speed_button.clicked.connect(self._apply_zwo_jog_speed)
        zwo_speed_row.addWidget(self.zwo_apply_jog_speed_button)
        zwo_jog_layout.addLayout(zwo_speed_row)

        zwo_jog_row = QHBoxLayout()
        self.zwo_ccw_button = QPushButton("CCW Step / Hold")
        self.zwo_ccw_button.pressed.connect(lambda: self._on_jog_button_pressed(-1))
        self.zwo_ccw_button.released.connect(lambda: self._on_jog_button_released(-1))
        zwo_jog_row.addWidget(self.zwo_ccw_button)

        self.zwo_cw_button = QPushButton("CW Step / Hold")
        self.zwo_cw_button.pressed.connect(lambda: self._on_jog_button_pressed(+1))
        self.zwo_cw_button.released.connect(lambda: self._on_jog_button_released(+1))
        zwo_jog_row.addWidget(self.zwo_cw_button)
        zwo_jog_layout.addLayout(zwo_jog_row)

        self.zwo_stop_jog_button = QPushButton("Stop")
        self.zwo_stop_jog_button.clicked.connect(self._stop_mouse_hold_jog)
        zwo_jog_layout.addWidget(self.zwo_stop_jog_button)

        zwo_jog_hint = QLabel(
            "Click once for one step. Hold for continuous jog. Release or press Stop to halt motion."
        )
        zwo_jog_hint.setWordWrap(True)
        zwo_jog_layout.addWidget(zwo_jog_hint)

        side_layout.addWidget(zwo_jog_box)

        self.zwo_camera_info_output = QPlainTextEdit()
        self.zwo_camera_info_output.setReadOnly(True)
        self.zwo_camera_info_output.setFocusPolicy(Qt.NoFocus)
        self.zwo_camera_info_output.setPlainText(
            "Phase 2 camera controls.\n"
            "- Refresh and select a ZWO camera\n"
            "- Connect / disconnect\n"
            "- Set exposure and gain\n"
            "- Capture a single still frame\n"
            "- Start / stop live preview"
        )
        side_layout.addWidget(self.zwo_camera_info_output, 1)

        preview_layout.addWidget(side_panel, 1)
        preview_box_layout.addLayout(preview_layout)

        profile_box = QGroupBox("Line / Slit Profile")
        profile_layout = QVBoxLayout(profile_box)

        profile_controls = QHBoxLayout()
        profile_controls.addWidget(QLabel("Profile axis"))
        self.zwo_profile_axis_combo = QComboBox()
        self.zwo_profile_axis_combo.addItem("Horizontal (sum rows)", "x")
        self.zwo_profile_axis_combo.addItem("Vertical (sum columns)", "y")
        self.zwo_profile_axis_combo.currentIndexChanged.connect(lambda _index: self._refresh_zwo_analysis_view())
        profile_controls.addWidget(self.zwo_profile_axis_combo)
        profile_controls.addStretch(1)
        profile_layout.addLayout(profile_controls)

        center_controls = QGridLayout()
        center_controls.addWidget(QLabel("Tolerance (px)"), 0, 0)
        self.zwo_center_tolerance_spin = QDoubleSpinBox()
        self.zwo_center_tolerance_spin.setRange(0.1, 1000.0)
        self.zwo_center_tolerance_spin.setDecimals(2)
        self.zwo_center_tolerance_spin.setSingleStep(0.25)
        self.zwo_center_tolerance_spin.setValue(2.0)
        center_controls.addWidget(self.zwo_center_tolerance_spin, 0, 1)

        center_controls.addWidget(QLabel("Max steps"), 0, 2)
        self.zwo_center_max_steps_spin = QSpinBox()
        self.zwo_center_max_steps_spin.setRange(1, 10000)
        self.zwo_center_max_steps_spin.setValue(40)
        center_controls.addWidget(self.zwo_center_max_steps_spin, 0, 3)

        center_controls.addWidget(QLabel("Settle (ms)"), 0, 4)
        self.zwo_center_settle_ms_spin = QDoubleSpinBox()
        self.zwo_center_settle_ms_spin.setRange(0.0, 5000.0)
        self.zwo_center_settle_ms_spin.setDecimals(1)
        self.zwo_center_settle_ms_spin.setSingleStep(5.0)
        self.zwo_center_settle_ms_spin.setValue(25.0)
        center_controls.addWidget(self.zwo_center_settle_ms_spin, 0, 5)
        profile_layout.addLayout(center_controls)

        assist_row = QHBoxLayout()
        self.zwo_center_centroid_button = QPushButton("Move To Centroid")
        self.zwo_center_centroid_button.clicked.connect(lambda: self._start_zwo_center_feature("centroid"))
        assist_row.addWidget(self.zwo_center_centroid_button)

        self.zwo_center_peak_button = QPushButton("Move To Peak")
        self.zwo_center_peak_button.clicked.connect(lambda: self._start_zwo_center_feature("peak"))
        assist_row.addWidget(self.zwo_center_peak_button)
        assist_row.addStretch(1)
        profile_layout.addLayout(assist_row)

        profile_hint = QLabel(
            "Drag directly on the preview to choose an analysis ROI. The drag box updates the live profile "
            "immediately and also fills the hardware ROI fields. Use the button below to make that selection "
            "the actual camera ROI."
        )
        profile_hint.setWordWrap(True)
        profile_layout.addWidget(profile_hint)

        roi_action_row = QHBoxLayout()
        self.zwo_apply_selected_roi_button = QPushButton("Use Selection As Camera ROI")
        self.zwo_apply_selected_roi_button.setToolTip(
            "Take the current drag-selection on the preview and apply it as the actual camera ROI."
        )
        self.zwo_apply_selected_roi_button.clicked.connect(self._apply_selected_preview_roi_to_camera)
        roi_action_row.addWidget(self.zwo_apply_selected_roi_button)

        self.zwo_clear_selected_roi_button = QPushButton("Clear Selection")
        self.zwo_clear_selected_roi_button.clicked.connect(self._clear_selected_preview_roi)
        roi_action_row.addWidget(self.zwo_clear_selected_roi_button)
        roi_action_row.addStretch(1)
        profile_layout.addLayout(roi_action_row)

        self.zwo_profile_plot = ZWOLineProfileWidget()
        profile_layout.addWidget(self.zwo_profile_plot)

        profile_stats = QGridLayout()
        self.zwo_profile_roi_label = QLabel("Analysis ROI: full visible frame")
        profile_stats.addWidget(self.zwo_profile_roi_label, 0, 0, 1, 2)

        self.zwo_profile_peak_label = QLabel("Peak: --")
        profile_stats.addWidget(self.zwo_profile_peak_label, 1, 0)
        self.zwo_profile_centroid_label = QLabel("Centroid: --")
        profile_stats.addWidget(self.zwo_profile_centroid_label, 1, 1)

        self.zwo_profile_peak_value_label = QLabel("Peak value: --")
        profile_stats.addWidget(self.zwo_profile_peak_value_label, 2, 0)
        self.zwo_profile_integrated_label = QLabel("Integrated intensity: --")
        profile_stats.addWidget(self.zwo_profile_integrated_label, 2, 1)

        self.zwo_profile_mean_label = QLabel("Mean DN: --")
        profile_stats.addWidget(self.zwo_profile_mean_label, 3, 0)
        self.zwo_profile_axis_label = QLabel("Axis: Horizontal")
        profile_stats.addWidget(self.zwo_profile_axis_label, 3, 1)
        profile_layout.addLayout(profile_stats)

        preview_box_layout.addWidget(profile_box)

        layout.addWidget(preview_box)
        return tab

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
        if not ready:
            self._stop_zwo_live_preview(wait=True)
            self._hide_cli_jog_dialog()
            self._set_cli_jog_active(False)
            self._clear_mouse_jog_state()
            self._zwo_last_frame_array = None
            self._zwo_last_camera_status = {}
            self._zwo_analysis_local_roi = None
        self._refresh_control_states()
        if not ready and hasattr(self, "zwo_camera_connection_label"):
            self._apply_zwo_status_to_view({"connected": False, "sdk_path": self._current_zwo_sdk_path()})
            self._set_zwo_preview_text("No frame captured.")

    def _set_busy(self, busy, message=None):
        self._busy = bool(busy)
        self._refresh_control_states()
        if message:
            self.connection_label.setText(message)

    def _log(self, message):
        self.log_output.appendPlainText(message)

    def _arrow_jog_status_text(self):
        return "Mouse jog active. Hold CW/CCW to move continuously; release to stop."

    def _task_is_active(self):
        return bool(self._task_thread is not None and self._task_thread.isRunning())

    def _is_jog_active(self):
        return bool(self.arrow_jog_checkbox.isChecked() or self._mouse_jog_active)

    def _clear_mouse_jog_state(self):
        if hasattr(self, "_mouse_jog_hold_timer") and self._mouse_jog_hold_timer is not None:
            self._mouse_jog_hold_timer.stop()
        self._mouse_jog_active = False
        self._mouse_jog_press_direction = 0
        self._mouse_jog_is_holding = False

    def _set_cli_jog_active(self, active):
        self.arrow_jog_checkbox.blockSignals(True)
        self.arrow_jog_checkbox.setChecked(bool(active))
        self.arrow_jog_checkbox.blockSignals(False)

    def _show_cli_jog_dialog(self):
        if self._cli_jog_dialog is None:
            dialog = CLIJogDialog(self)
            dialog.stop_requested.connect(self._stop_cli_jog)
            dialog.close_gui_requested.connect(self.close)
            dialog.finished.connect(self._on_cli_jog_dialog_finished)
            self._cli_jog_dialog = dialog
        self._cli_jog_dialog_closing = False
        self._cli_jog_dialog.show()
        self._cli_jog_dialog.raise_()
        self._cli_jog_dialog.activateWindow()

    def _hide_cli_jog_dialog(self):
        dialog = self._cli_jog_dialog
        if dialog is None:
            return
        self._cli_jog_dialog_closing = True
        try:
            dialog.blockSignals(True)
            dialog.hide()
            dialog.close()
        finally:
            dialog.blockSignals(False)
        self._cli_jog_dialog = None

    def _on_cli_jog_dialog_finished(self, _result):
        dialog = self.sender()
        if dialog is not None:
            dialog.deleteLater()
        if self._cli_jog_dialog is dialog:
            self._cli_jog_dialog = None
        if self._cli_jog_dialog_closing:
            self._cli_jog_dialog_closing = False
            return
        if self.arrow_jog_checkbox.isChecked():
            self._stop_cli_jog()

    def _on_jog_button_pressed(self, direction):
        if self.backend is None or self._closing:
            return
        if self._busy or self._task_is_active() or self._mouse_jog_press_direction != 0:
            return
        self._mouse_jog_press_direction = 1 if int(direction) > 0 else -1
        self._mouse_jog_is_holding = False
        self._mouse_jog_hold_timer.start(180)

    def _on_jog_button_released(self, direction):
        normalized_direction = 1 if int(direction) > 0 else -1
        if self._mouse_jog_press_direction != normalized_direction and not (
            self._mouse_jog_is_holding and self._jog_hold_direction == normalized_direction
        ):
            return

        self._mouse_jog_hold_timer.stop()
        if self._mouse_jog_is_holding:
            self._stop_mouse_hold_jog()
            return

        self._mouse_jog_press_direction = 0
        if self.backend is None or self._closing or self._busy or self._task_is_active():
            return
        label = "Jog CW" if normalized_direction > 0 else "Jog CCW"
        self._run_task(label, self.backend.jog_step, normalized_direction, 1, task_kind="jog_click", log_message=False)

    def _begin_mouse_hold_jog(self):
        if self.backend is None or self._closing or self._mouse_jog_press_direction == 0:
            self._clear_mouse_jog_state()
            self._refresh_control_states()
            return
        self._mouse_jog_is_holding = True
        self._mouse_jog_active = True
        self._jog_hold_direction = self._mouse_jog_press_direction
        self.connection_label.setText(self._arrow_jog_status_text())
        self._refresh_control_states()
        self._run_next_mouse_hold_step()

    def _run_next_mouse_hold_step(self):
        if (
            self.backend is None
            or self._closing
            or not self._mouse_jog_active
            or self._jog_hold_direction == 0
            or self._busy
            or self._task_is_active()
        ):
            return
        label = "Hold Jog CW" if self._jog_hold_direction > 0 else "Hold Jog CCW"
        self._run_task(label, self.backend.jog_step, self._jog_hold_direction, 1, task_kind="hold_jog", log_message=False)

    def _stop_mouse_hold_jog(self):
        if not self._mouse_jog_active and self._mouse_jog_press_direction == 0 and not self._mouse_jog_hold_timer.isActive():
            return
        self._clear_mouse_jog_state()
        self._jog_hold_direction = 0
        self._restore_post_jog_ui()

    def _show_arrow_jog_dialog(self):
        if self._arrow_jog_dialog is None:
            dialog = ArrowJogDialog(self)
            dialog.start_direction.connect(self._start_continuous_jog)
            dialog.stop_direction.connect(self._stop_continuous_jog)
            dialog.speed_delta.connect(self._adjust_jog_speed)
            dialog.exit_requested.connect(lambda: self._exit_arrow_jog_mode(cancel_active=True))
            dialog.close_gui_requested.connect(self.close)
            dialog.finished.connect(self._on_arrow_jog_dialog_finished)
            self._arrow_jog_dialog = dialog
        self._arrow_jog_dialog_closing = False
        self._arrow_jog_dialog.show()
        self._arrow_jog_dialog.raise_()
        self._arrow_jog_dialog.activateWindow()
        self._arrow_jog_dialog.setFocus(Qt.OtherFocusReason)

    def _hide_arrow_jog_dialog(self):
        dialog = self._arrow_jog_dialog
        if dialog is None:
            return
        self._arrow_jog_dialog_closing = True
        try:
            dialog.blockSignals(True)
            dialog.hide()
            dialog.close()
        finally:
            dialog.blockSignals(False)
        self._arrow_jog_dialog = None

    def _on_arrow_jog_dialog_finished(self, _result):
        dialog = self.sender()
        if dialog is not None:
            dialog.deleteLater()
        if self._arrow_jog_dialog is dialog:
            self._arrow_jog_dialog = None
        if self._arrow_jog_dialog_closing:
            self._arrow_jog_dialog_closing = False
            return
        if self.arrow_jog_checkbox.isChecked():
            self._exit_arrow_jog_mode(cancel_active=True)

    def _restore_post_jog_ui(self):
        self._refresh_control_states()
        try:
            self.arrow_jog_checkbox.clearFocus()
        except Exception:
            pass
        if self._closing:
            return
        focus_target = getattr(self, "workspace_tabs", None) or self.centralWidget()
        try:
            focus_target.setFocus(Qt.OtherFocusReason)
        except Exception:
            pass
        try:
            self.activateWindow()
        except Exception:
            pass

    def _exit_arrow_jog_mode(self, *, cancel_active=True):
        if self.arrow_jog_checkbox.isChecked():
            self.arrow_jog_checkbox.blockSignals(True)
            self.arrow_jog_checkbox.setChecked(False)
            self.arrow_jog_checkbox.blockSignals(False)

        self._hide_arrow_jog_dialog()
        self._jog_hold_direction = 0
        self._pending_jog_speed_ms = None

        if cancel_active and self.backend is not None and self._active_task_kind in {"continuous_jog", "continuous_jog_speed"}:
            try:
                self.backend.cancel()
            except Exception:
                pass

        if self.backend is not None and not self._busy and not self._task_is_active():
            ready_port = "unknown port"
            try:
                ready_port = self.backend.status().get("port") or "unknown port"
            except BaseException:
                pass
            self.connection_label.setText(f"Controller ready on {ready_port}.")
        elif self.backend is not None:
            self.connection_label.setText("Stopping arrow jog...")
        else:
            self.connection_label.setText("Backend not initialized.")

        self._log("[GUI] Arrow jog disarmed")
        self._restore_post_jog_ui()

    def _toggle_camera_pane(self, checked):
        visible = bool(checked)
        self.camera_pane_scroll.setVisible(visible)
        self.camera_pane_toggle_button.setText("Hide Camera Pane" if visible else "Show Camera Pane")
        if visible:
            self.main_splitter.setSizes([420, 780])
        else:
            self.main_splitter.setSizes([1, 0])

    def _refresh_control_states(self):
        ready = self.backend is not None
        live_active = self._zwo_live_thread is not None
        task_active = self._task_is_active()
        armed = bool(ready and self.arrow_jog_checkbox.isChecked() and not self._closing and not self._shutdown_complete)
        idle = bool(ready and not self._busy and not task_active and not self._closing and not armed)
        connect_idle = bool(not self._busy and not task_active and not self._closing and not armed and not live_active)
        hold_interaction = bool(self._mouse_jog_press_direction != 0 or self._mouse_jog_hold_timer.isActive() or self._mouse_jog_active)
        jog_button_enabled = bool(ready and not self._closing and (idle or hold_interaction))

        self.init_button.setEnabled(connect_idle)
        self.port_combo.setEnabled(connect_idle)
        self.port_refresh_button.setEnabled(connect_idle)
        self.refresh_button.setEnabled(bool(ready and not self._busy and not self._closing and not armed))
        self.cancel_button.setEnabled(bool(ready and not self._closing and not armed))
        self.close_button.setEnabled(bool(not self._shutdown_complete and not self._closing))
        self.camera_pane_toggle_button.setEnabled(bool(not self._closing))
        self.arrow_jog_checkbox.setEnabled(False)
        self.cli_jog_button.setEnabled(False)
        self.stop_cli_jog_button.setEnabled(False)
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
            self.zwo_jog_speed_spin,
            self.zwo_apply_jog_speed_button,
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
            widget.setEnabled(bool(idle))

        for widget in [
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
            self.zwo_sdk_path_edit,
            self.zwo_refresh_cameras_button,
            self.zwo_camera_combo,
            self.zwo_connect_button,
            self.zwo_disconnect_button,
            self.zwo_live_fps_spin,
            self.zwo_roi_x_spin,
            self.zwo_roi_y_spin,
            self.zwo_roi_w_spin,
            self.zwo_roi_h_spin,
            self.zwo_capture_frame_button,
            self.zwo_apply_roi_button,
            self.zwo_reset_roi_button,
        ]:
            widget.setEnabled(bool(idle and not live_active))

        zwo_jog_speed_enabled = bool(ready and not self._closing and not self._busy and not task_active and not self._mouse_jog_active)
        zwo_camera_connected = bool(self._zwo_last_camera_status.get("connected"))
        zwo_has_selected_roi = bool(self._zwo_last_frame_array is not None and self._zwo_analysis_local_roi is not None)
        zwo_settings_enabled = bool(
            ready
            and zwo_camera_connected
            and not self._busy
            and not task_active
            and not self._closing
        )
        zwo_center_enabled = bool(
            ready
            and zwo_camera_connected
            and self._zwo_last_frame_array is not None
            and not self._busy
            and not task_active
            and not self._closing
        )
        self.zwo_jog_speed_spin.setEnabled(zwo_jog_speed_enabled)
        self.zwo_apply_jog_speed_button.setEnabled(zwo_jog_speed_enabled)
        self.ccw_button.setEnabled(jog_button_enabled)
        self.cw_button.setEnabled(jog_button_enabled)
        self.stop_jog_button.setEnabled(bool(ready and not self._closing and self._mouse_jog_active))
        self.zwo_ccw_button.setEnabled(jog_button_enabled)
        self.zwo_cw_button.setEnabled(jog_button_enabled)
        self.zwo_stop_jog_button.setEnabled(bool(ready and not self._closing and self._mouse_jog_active))
        self.zwo_camera_exposure_spin.setEnabled(zwo_settings_enabled)
        self.zwo_camera_gain_spin.setEnabled(zwo_settings_enabled)
        self.zwo_apply_settings_button.setEnabled(zwo_settings_enabled)
        self.zwo_center_tolerance_spin.setEnabled(zwo_center_enabled)
        self.zwo_center_max_steps_spin.setEnabled(zwo_center_enabled)
        self.zwo_center_settle_ms_spin.setEnabled(zwo_center_enabled)
        self.zwo_center_centroid_button.setEnabled(zwo_center_enabled)
        self.zwo_center_peak_button.setEnabled(zwo_center_enabled)
        self.zwo_apply_selected_roi_button.setEnabled(
            bool(
                ready
                and zwo_camera_connected
                and zwo_has_selected_roi
                and not self._busy
                and not task_active
                and not self._closing
            )
        )
        self.zwo_clear_selected_roi_button.setEnabled(
            bool(self._zwo_last_frame_array is not None and not self._busy and not self._closing)
        )

        self.zwo_start_live_button.setEnabled(bool(idle and not live_active))
        self.zwo_stop_live_button.setEnabled(bool(live_active and not self._closing))

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

    def _format_zwo_camera_status(self, status):
        if not isinstance(status, dict):
            return "No camera status available."
        if not status.get("connected"):
            sdk_path = status.get("sdk_path") or "$ZWO_ASI_LIB"
            return f"Disconnected. SDK path: {sdk_path}"

        width = status.get("frame_width")
        height = status.get("frame_height")
        size_text = f"{width}x{height}" if width and height else "--"
        binning = status.get("binning")
        roi_x = status.get("roi_start_x")
        roi_y = status.get("roi_start_y")
        roi_text = f"ROI ({roi_x if roi_x is not None else '--'},{roi_y if roi_y is not None else '--'}) {size_text}"
        max_width = status.get("max_width")
        max_height = status.get("max_height")
        sensor_text = f"{max_width}x{max_height}" if max_width and max_height else "--"
        return (
            f"Connected to {status.get('camera_name', 'camera')} "
            f"(index {status.get('camera_index', '--')}) | "
            f"{roi_text} / Sensor {sensor_text} | "
            f"Binning {binning if binning is not None else '--'} | "
            f"Exposure {status.get('exposure_ms', '--')} ms | "
            f"Gain {status.get('gain', '--')}"
        )

    def _set_zwo_preview_text(self, text):
        self._zwo_last_frame_array = None
        self._zwo_analysis_local_roi = None
        self.zwo_preview_label.set_placeholder_text(text)
        self.zwo_profile_plot.set_profile(None)
        self.zwo_profile_roi_label.setText("Analysis ROI: full visible frame")
        self.zwo_profile_peak_label.setText("Peak: --")
        self.zwo_profile_centroid_label.setText("Centroid: --")
        self.zwo_profile_peak_value_label.setText("Peak value: --")
        self.zwo_profile_integrated_label.setText("Integrated intensity: --")
        self.zwo_profile_mean_label.setText("Mean DN: --")
        self.zwo_profile_axis_label.setText("Axis: Horizontal")

    def _set_zwo_preview_from_array(self, frame_array):
        if frame_array is None:
            self._set_zwo_preview_text("No frame captured.")
            return
        try:
            self._zwo_last_frame_array = frame_array
            self.zwo_preview_label.set_frame_array(frame_array)
            self._refresh_zwo_analysis_view()
        except BaseException:
            self._set_zwo_preview_text("Frame captured, but preview rendering failed.")

    def _update_zwo_camera_combo(self, cameras):
        current_index = self.zwo_camera_combo.currentData()
        self.zwo_camera_combo.blockSignals(True)
        self.zwo_camera_combo.clear()
        if not cameras:
            self.zwo_camera_combo.addItem("No cameras detected", None)
        else:
            selected_combo_index = 0
            for combo_index, camera in enumerate(cameras):
                self.zwo_camera_combo.addItem(camera.get("label", f"Camera {combo_index}"), camera.get("index"))
                if current_index is not None and camera.get("index") == current_index:
                    selected_combo_index = combo_index
            self.zwo_camera_combo.setCurrentIndex(selected_combo_index)
        self.zwo_camera_combo.blockSignals(False)

    def _apply_zwo_status_to_view(self, status):
        if not isinstance(status, dict):
            return
        self._zwo_last_camera_status = dict(status)
        self.zwo_camera_connection_label.setText(
            f"Connected: {status.get('camera_name')}" if status.get("connected") else "No ZWO camera connected."
        )
        self.zwo_camera_status_summary.setText(self._format_zwo_camera_status(status))
        if (
            status.get("exposure_ms") is not None
            and self._zwo_pending_camera_settings is None
            and not self._widget_or_child_has_focus(self.zwo_camera_exposure_spin)
        ):
            self.zwo_camera_exposure_spin.setValue(float(status["exposure_ms"]))
        if (
            status.get("gain") is not None
            and self._zwo_pending_camera_settings is None
            and not self._widget_or_child_has_focus(self.zwo_camera_gain_spin)
        ):
            self.zwo_camera_gain_spin.setValue(int(status["gain"]))
        max_width = status.get("max_width")
        max_height = status.get("max_height")
        if max_width is not None:
            self.zwo_roi_x_spin.setMaximum(int(max_width))
            self.zwo_roi_w_spin.setMaximum(int(max_width))
        if max_height is not None:
            self.zwo_roi_y_spin.setMaximum(int(max_height))
            self.zwo_roi_h_spin.setMaximum(int(max_height))
        if status.get("roi_start_x") is not None and not self._widget_or_child_has_focus(self.zwo_roi_x_spin):
            self.zwo_roi_x_spin.setValue(int(status["roi_start_x"]))
        if status.get("roi_start_y") is not None and not self._widget_or_child_has_focus(self.zwo_roi_y_spin):
            self.zwo_roi_y_spin.setValue(int(status["roi_start_y"]))
        if status.get("frame_width") is not None and not self._widget_or_child_has_focus(self.zwo_roi_w_spin):
            self.zwo_roi_w_spin.setValue(int(status["frame_width"]))
        if status.get("frame_height") is not None and not self._widget_or_child_has_focus(self.zwo_roi_h_spin):
            self.zwo_roi_h_spin.setValue(int(status["frame_height"]))

    def _current_zwo_profile_axis(self):
        axis = self.zwo_profile_axis_combo.currentData()
        return axis if axis in {"x", "y"} else "x"

    def _zwo_frame_origin(self):
        status = self._zwo_last_camera_status if isinstance(self._zwo_last_camera_status, dict) else {}
        return (
            int(status.get("roi_start_x") or 0),
            int(status.get("roi_start_y") or 0),
        )

    def _normalize_zwo_analysis_local_roi(self, frame_array, roi):
        if frame_array is None:
            return None
        frame_h, frame_w = frame_array.shape[:2]
        if frame_w <= 0 or frame_h <= 0:
            return None
        if roi is None:
            return None
        x, y, width, height = [int(v) for v in roi]
        x = max(0, min(x, frame_w - 1))
        y = max(0, min(y, frame_h - 1))
        width = max(1, min(width, frame_w - x))
        height = max(1, min(height, frame_h - y))
        if x == 0 and y == 0 and width == frame_w and height == frame_h:
            return None
        return (x, y, width, height)

    def _zwo_local_to_sensor_roi(self, local_roi):
        if local_roi is None:
            return None
        origin_x, origin_y = self._zwo_frame_origin()
        x, y, width, height = [int(v) for v in local_roi]
        return (origin_x + x, origin_y + y, width, height)

    def _align_sensor_roi_for_hardware(self, sensor_roi):
        if sensor_roi is None:
            return None
        x, y, width, height = [int(v) for v in sensor_roi]
        status = self._zwo_last_camera_status if isinstance(self._zwo_last_camera_status, dict) else {}
        max_width = int(status.get("max_width") or max(x + width, 8))
        max_height = int(status.get("max_height") or max(y + height, 2))

        width = max(8, ((width + 7) // 8) * 8)
        height = max(2, ((height + 1) // 2) * 2)

        width = min(width, max_width)
        height = min(height, max_height)
        if width % 8 != 0:
            width = max(8, width - (width % 8))
        if height % 2 != 0:
            height = max(2, height - (height % 2))

        x = max(0, min(x, max_width - 1))
        y = max(0, min(y, max_height - 1))
        if x + width > max_width:
            x = max(0, max_width - width)
        if y + height > max_height:
            y = max(0, max_height - height)
        return (x, y, width, height)

    def _populate_zwo_roi_inputs(self, sensor_roi):
        if sensor_roi is None:
            return
        x, y, width, height = [int(v) for v in sensor_roi]
        self.zwo_roi_x_spin.setValue(x)
        self.zwo_roi_y_spin.setValue(y)
        self.zwo_roi_w_spin.setValue(width)
        self.zwo_roi_h_spin.setValue(height)

    def _on_zwo_preview_roi_selected(self, x, y, width, height):
        local_roi = self._normalize_zwo_analysis_local_roi(self._zwo_last_frame_array, (x, y, width, height))
        self._zwo_analysis_local_roi = local_roi
        sensor_roi = self._align_sensor_roi_for_hardware(self._zwo_local_to_sensor_roi(local_roi))
        if sensor_roi is not None:
            self._populate_zwo_roi_inputs(sensor_roi)
        self._refresh_zwo_analysis_view()
        self._refresh_control_states()
        if sensor_roi is not None:
            sx, sy, sw, sh = sensor_roi
            self.zwo_camera_info_output.setPlainText(
                "Preview ROI selected.\n"
                f"Analysis ROI (visible frame): ({x}, {y}) {width} x {height}\n"
                f"Hardware ROI fields updated to: ({sx}, {sy}) {sw} x {sh}\n"
                "Click Apply ROI to crop the camera stream to that region."
            )

    def _refresh_zwo_analysis_view(self):
        if self._zwo_last_frame_array is None:
            self.zwo_profile_plot.set_profile(None)
            self.zwo_profile_roi_label.setText("Analysis ROI: full visible frame")
            self.zwo_profile_peak_label.setText("Peak: --")
            self.zwo_profile_centroid_label.setText("Centroid: --")
            self.zwo_profile_peak_value_label.setText("Peak value: --")
            self.zwo_profile_integrated_label.setText("Integrated intensity: --")
            self.zwo_profile_mean_label.setText("Mean DN: --")
            self.zwo_profile_axis_label.setText("Axis: Horizontal")
            self.zwo_preview_label.set_analysis_roi(None)
            return

        if np is None:
            self.zwo_profile_plot.set_profile(None)
            self.zwo_profile_roi_label.setText("Analysis ROI: NumPy not available")
            self.zwo_profile_peak_label.setText("Peak: --")
            self.zwo_profile_centroid_label.setText("Centroid: --")
            self.zwo_profile_peak_value_label.setText("Peak value: --")
            self.zwo_profile_integrated_label.setText("Integrated intensity: --")
            self.zwo_profile_mean_label.setText("Mean DN: --")
            self.zwo_profile_axis_label.setText("Axis: --")
            self.zwo_preview_label.set_analysis_roi(None)
            return

        frame_array = self._zwo_last_frame_array
        frame_h, frame_w = frame_array.shape[:2]
        local_roi = self._normalize_zwo_analysis_local_roi(frame_array, self._zwo_analysis_local_roi)
        self._zwo_analysis_local_roi = local_roi
        self.zwo_preview_label.set_analysis_roi(local_roi)

        if local_roi is None:
            x = 0
            y = 0
            width = frame_w
            height = frame_h
        else:
            x, y, width, height = local_roi

        region = frame_array[y : y + height, x : x + width]
        if region.size == 0:
            self.zwo_profile_plot.set_profile(None)
            return

        axis = self._current_zwo_profile_axis()
        axis_label = "Horizontal" if axis == "x" else "Vertical"
        region_f = region.astype(np.float64, copy=False)
        profile = region_f.sum(axis=0 if axis == "x" else 1)
        if profile.size == 0:
            self.zwo_profile_plot.set_profile(None)
            return

        peak_index = int(np.argmax(profile))
        peak_value = float(profile[peak_index])
        total_intensity = float(profile.sum())
        centroid_index = None
        if total_intensity > 0.0:
            coords = np.arange(profile.size, dtype=np.float64)
            centroid_index = float((coords * profile).sum() / total_intensity)

        sensor_roi = self._zwo_local_to_sensor_roi((x, y, width, height))
        sensor_x, sensor_y, _, _ = sensor_roi if sensor_roi is not None else (x, y, width, height)
        peak_sensor = (sensor_x + peak_index) if axis == "x" else (sensor_y + peak_index)
        centroid_sensor = None if centroid_index is None else ((sensor_x + centroid_index) if axis == "x" else (sensor_y + centroid_index))

        self.zwo_profile_plot.set_profile(
            profile.tolist(),
            peak_index=peak_index,
            centroid_index=centroid_index,
            axis_label=axis_label,
        )
        self.zwo_profile_roi_label.setText(
            f"Analysis ROI: sensor ({sensor_x}, {sensor_y}) {width} x {height}"
        )
        axis_name = "X" if axis == "x" else "Y"
        self.zwo_profile_peak_label.setText(
            f"Peak {axis_name}: local {peak_index} | sensor {peak_sensor}"
        )
        if centroid_sensor is None:
            self.zwo_profile_centroid_label.setText(f"Centroid {axis_name}: --")
        else:
            self.zwo_profile_centroid_label.setText(
                f"Centroid {axis_name}: local {centroid_index:.2f} | sensor {centroid_sensor:.2f}"
            )
        self.zwo_profile_peak_value_label.setText(f"Peak value: {peak_value:.1f}")
        self.zwo_profile_integrated_label.setText(f"Integrated intensity: {total_intensity:.1f}")
        self.zwo_profile_mean_label.setText(f"Mean DN: {float(region_f.mean()):.3f}")
        self.zwo_profile_axis_label.setText(f"Axis: {axis_label}")

    def _current_zwo_analysis_roi_for_backend(self):
        return self._normalize_zwo_analysis_local_roi(self._zwo_last_frame_array, self._zwo_analysis_local_roi)

    def _clear_selected_preview_roi(self):
        self._zwo_analysis_local_roi = None
        self._refresh_zwo_analysis_view()
        self._refresh_control_states()
        if self._zwo_last_frame_array is not None:
            self.zwo_camera_info_output.setPlainText(
                "Preview ROI selection cleared.\n"
                "Analysis is back to the full visible frame.\n"
                "Camera ROI is unchanged until you apply a new one."
            )

    def _apply_selected_preview_roi_to_camera(self):
        if self.backend is None or self._busy or self._closing:
            return
        local_roi = self._current_zwo_analysis_roi_for_backend()
        if local_roi is None:
            QMessageBox.warning(self, "ZWO ROI", "Drag a region on the preview first.")
            return
        sensor_roi = self._align_sensor_roi_for_hardware(self._zwo_local_to_sensor_roi(local_roi))
        if sensor_roi is None:
            QMessageBox.warning(self, "ZWO ROI", "Could not convert the selected preview ROI into a camera ROI.")
            return
        self._populate_zwo_roi_inputs(sensor_roi)
        self.zwo_camera_info_output.setPlainText(
            "Applying selected preview ROI to the camera.\n"
            "Live preview will stop, crop the camera stream, then resume."
        )
        self._apply_zwo_camera_roi(from_selected_preview=True)

    def _start_zwo_center_feature(self, feature):
        if self.backend is None or self._busy or self._closing:
            return
        if self._zwo_last_frame_array is None:
            QMessageBox.warning(self, "ZWO centering", "Capture or stream a frame before using centering assist.")
            return
        self._zwo_resume_live_after_center = bool(self._zwo_live_thread is not None)
        if self._zwo_resume_live_after_center and not self._ensure_zwo_live_preview_stopped(
            context="starting centering assist"
        ):
            self._zwo_resume_live_after_center = False
            return
        axis = self._current_zwo_profile_axis()
        label = "Center on centroid" if str(feature).lower() == "centroid" else "Center on peak"
        self.zwo_camera_info_output.setPlainText(f"{label} assist running...")
        self._run_task(
            label,
            self.backend.center_zwo_feature,
            feature,
            axis,
            self._current_zwo_analysis_roi_for_backend(),
            self.zwo_center_tolerance_spin.value(),
            self.zwo_center_max_steps_spin.value(),
            self.zwo_center_settle_ms_spin.value(),
            task_kind="zwo_center_feature",
        )

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

    def _current_zwo_sdk_path(self):
        text = self.zwo_sdk_path_edit.text().strip()
        return text or None

    def _refresh_zwo_camera_status_view(self, silent=False):
        if self.backend is None:
            self._apply_zwo_status_to_view({"connected": False, "sdk_path": self._current_zwo_sdk_path()})
            return
        try:
            status = self.backend.get_zwo_camera_status()
            self._apply_zwo_status_to_view(status)
        except BaseException:
            error_text = traceback.format_exc()
            if not silent:
                self._log("[GUI] ZWO camera status refresh failed")
                self._log(error_text)

    def _refresh_zwo_cameras(self):
        if self.backend is None or self._busy or self._closing:
            return
        self._run_task(
            "Refresh ZWO cameras",
            self.backend.list_zwo_cameras,
            self._current_zwo_sdk_path(),
            task_kind="zwo_camera_refresh",
        )

    def _connect_zwo_camera(self):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._ensure_zwo_live_preview_stopped(context="connecting the ZWO camera"):
            return
        camera_index = self.zwo_camera_combo.currentData()
        if camera_index is None:
            QMessageBox.warning(self, "No Camera Selected", "Refresh cameras first, then select a ZWO camera.")
            return
        self._run_task(
            "Connect ZWO camera",
            self.backend.connect_zwo_camera,
            int(camera_index),
            self.zwo_camera_exposure_spin.value(),
            self.zwo_camera_gain_spin.value(),
            self._current_zwo_sdk_path(),
            task_kind="zwo_camera_connect",
        )

    def _disconnect_zwo_camera(self):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._ensure_zwo_live_preview_stopped(context="disconnecting the ZWO camera"):
            return
        self._run_task(
            "Disconnect ZWO camera",
            self.backend.disconnect_zwo_camera,
            task_kind="zwo_camera_disconnect",
        )

    def _apply_zwo_camera_settings_from_editor(self):
        self._apply_zwo_camera_settings(auto_from_editor=True)

    def _apply_zwo_camera_settings(self, auto_from_editor=False):
        if self.backend is None or self._busy or self._closing:
            return
        if not self._zwo_last_camera_status.get("connected"):
            return
        exposure_value = float(self.zwo_camera_exposure_spin.value())
        gain_value = int(self.zwo_camera_gain_spin.value())
        current_exposure = self._zwo_last_camera_status.get("exposure_ms")
        current_gain = self._zwo_last_camera_status.get("gain")
        if (
            current_exposure is not None
            and current_gain is not None
            and abs(float(current_exposure) - exposure_value) < 1e-9
            and int(current_gain) == gain_value
        ):
            return
        self._zwo_pending_camera_settings = (exposure_value, gain_value)
        if auto_from_editor:
            self.zwo_camera_info_output.setPlainText(
                f"Applying camera settings...\nExposure: {exposure_value:.3f} ms\nGain: {gain_value}"
            )
        self._run_task(
            "Apply ZWO camera settings",
            self.backend.set_zwo_camera_settings,
            exposure_value,
            gain_value,
            task_kind="zwo_camera_settings",
        )

    def _capture_zwo_single_frame(self):
        if self.backend is None or self._busy or self._closing:
            return
        self.zwo_camera_info_output.setPlainText("Capturing single frame...")
        self._run_task(
            "Capture ZWO single frame",
            self.backend.capture_zwo_single_frame,
            task_kind="zwo_camera_capture",
        )

    def _validate_zwo_roi_inputs(self):
        width = int(self.zwo_roi_w_spin.value())
        height = int(self.zwo_roi_h_spin.value())
        if width % 8 != 0:
            QMessageBox.warning(self, "ROI", "ROI width must be a multiple of 8.")
            return None
        if height % 2 != 0:
            QMessageBox.warning(self, "ROI", "ROI height must be a multiple of 2.")
            return None
        return {
            "start_x": int(self.zwo_roi_x_spin.value()),
            "start_y": int(self.zwo_roi_y_spin.value()),
            "width": width,
            "height": height,
        }

    def _apply_zwo_camera_roi(self, from_selected_preview=False):
        if self.backend is None or self._busy or self._closing:
            return
        roi = self._validate_zwo_roi_inputs()
        if roi is None:
            return
        self._zwo_resume_live_after_roi_change = bool(self._zwo_live_thread is not None)
        if not self._ensure_zwo_live_preview_stopped(context="changing the ZWO ROI"):
            self._zwo_resume_live_after_roi_change = False
            return
        label = "Apply Selected ZWO ROI" if from_selected_preview else "Apply ZWO ROI"
        self._run_task(
            label,
            self.backend.set_zwo_camera_roi,
            roi["start_x"],
            roi["start_y"],
            roi["width"],
            roi["height"],
            task_kind="zwo_camera_roi",
        )

    def _reset_zwo_camera_roi(self):
        if self.backend is None or self._busy or self._closing:
            return
        self._zwo_analysis_local_roi = None
        self._refresh_zwo_analysis_view()
        self._zwo_resume_live_after_roi_change = bool(self._zwo_live_thread is not None)
        if not self._ensure_zwo_live_preview_stopped(context="resetting the ZWO ROI"):
            self._zwo_resume_live_after_roi_change = False
            return
        self._run_task(
            "Reset ZWO ROI",
            self.backend.reset_zwo_camera_roi,
            task_kind="zwo_camera_roi",
        )

    def _start_zwo_live_preview(self):
        if self.backend is None or self._busy or self._closing or self._zwo_live_thread is not None:
            return
        try:
            status = self.backend.get_zwo_camera_status()
        except BaseException:
            error_text = traceback.format_exc()
            self.zwo_camera_info_output.setPlainText(error_text)
            self._log("[GUI] Could not query ZWO camera status before live preview")
            self._log(error_text)
            QMessageBox.critical(self, "ZWO live preview", error_text)
            return

        if not status.get("connected"):
            QMessageBox.warning(self, "ZWO live preview", "Connect a ZWO camera before starting live preview.")
            return

        self._zwo_live_error_seen = False
        self._zwo_live_stop_requested = False
        self._zwo_live_last_frame_ts = None
        self._zwo_live_actual_fps = None
        self._zwo_live_info_last_update_ts = None
        self._zwo_live_thread = ZWOLivePreviewThread(self.backend, fps=self.zwo_live_fps_spin.value())
        self._zwo_live_thread.frame_ready.connect(self._on_zwo_live_frame_ready)
        self._zwo_live_thread.failed.connect(self._on_zwo_live_failed)
        self._zwo_live_thread.finished.connect(self._on_zwo_live_finished)
        self._refresh_control_states()
        self.zwo_camera_info_output.setPlainText(
            f"Starting live preview at {self.zwo_live_fps_spin.value():.1f} fps..."
        )
        self._log(f"[GUI] Starting ZWO live preview at {self.zwo_live_fps_spin.value():.1f} fps")
        self._zwo_live_thread.start()

    def _stop_zwo_live_preview(self, wait=False, timeout_ms=3000):
        thread = self._zwo_live_thread
        if thread is None:
            return True
        self._zwo_live_stop_requested = True
        thread.stop()
        if wait:
            thread.wait(int(timeout_ms))
            if not thread.isRunning() and self._zwo_live_thread is thread:
                self._zwo_live_thread = None
                self._refresh_control_states()
            return not thread.isRunning()
        return True

    def _ensure_zwo_live_preview_stopped(self, *, context, timeout_ms=5000):
        if self._zwo_live_thread is None:
            return True
        stopped = self._stop_zwo_live_preview(wait=True, timeout_ms=timeout_ms)
        if stopped:
            return True
        message = (
            f"Live preview did not stop cleanly before {context}.\n\n"
            "Click Stop Live, wait a moment, and try again."
        )
        self._log(f"[GUI] {context}: live preview thread did not stop within {timeout_ms} ms")
        self.zwo_camera_info_output.setPlainText(message)
        QMessageBox.warning(self, "ZWO live preview still running", message)
        return False

    def _on_zwo_live_frame_ready(self, payload):
        if not isinstance(payload, dict):
            return
        now = time.monotonic()
        if self._zwo_live_last_frame_ts is not None:
            dt = now - self._zwo_live_last_frame_ts
            if dt > 1e-6:
                instant_fps = 1.0 / dt
                if self._zwo_live_actual_fps is None:
                    self._zwo_live_actual_fps = instant_fps
                else:
                    self._zwo_live_actual_fps = 0.8 * self._zwo_live_actual_fps + 0.2 * instant_fps
        self._zwo_live_last_frame_ts = now
        self._apply_zwo_status_to_view(payload.get("status", {}))
        self._set_zwo_preview_from_array(payload.get("frame_array"))
        if self._zwo_live_info_last_update_ts is None or (now - self._zwo_live_info_last_update_ts) >= 0.20:
            self._zwo_live_info_last_update_ts = now
            actual_fps_text = "--" if self._zwo_live_actual_fps is None else f"{self._zwo_live_actual_fps:.1f}"
            self.zwo_camera_info_output.setPlainText(
                "Live preview running.\n"
                f"ROI: ({payload.get('status', {}).get('roi_start_x', '--')}, {payload.get('status', {}).get('roi_start_y', '--')}) "
                f"{payload.get('frame_width', '--')} x {payload.get('frame_height', '--')}\n"
                f"Actual FPS: {actual_fps_text}\n"
                f"Target FPS: {self.zwo_live_fps_spin.value():.1f}\n"
                "Drag on the image to update the analysis ROI and hardware ROI fields."
            )

    def _on_zwo_live_failed(self, error_text):
        self._zwo_live_error_seen = True
        self.zwo_camera_info_output.setPlainText(error_text)
        self._log("[GUI] ZWO live preview failed")
        self._log(error_text)
        if not self._closing:
            QMessageBox.critical(self, "ZWO live preview failed", error_text)
        self._stop_zwo_live_preview(wait=False)

    def _on_zwo_live_finished(self):
        thread = self.sender()
        if thread is not None:
            thread.deleteLater()
        self._zwo_live_thread = None
        self._refresh_control_states()
        if self._closing:
            return
        if self._zwo_live_error_seen:
            self._zwo_live_stop_requested = False
            self._zwo_live_error_seen = False
            self._zwo_live_last_frame_ts = None
            self._zwo_live_actual_fps = None
            return
        if self._zwo_live_stop_requested:
            self.zwo_camera_info_output.setPlainText("Live preview stopped.")
            self._log("[GUI] ZWO live preview stopped")
        self._zwo_live_stop_requested = False
        self._zwo_live_error_seen = False
        self._zwo_live_last_frame_ts = None
        self._zwo_live_actual_fps = None
        self._zwo_live_info_last_update_ts = None

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
            self._show_arrow_jog_dialog()
            self.connection_label.setText(self._arrow_jog_status_text())
            self._log(
                "[GUI] Arrow jog armed. Hold UP for forward, DOWN for reverse. "
                "Press Q to quit jog mode. RIGHT arrow speeds up by 1 ms/step. "
                "LEFT arrow slows down by 1 ms/step."
            )
            return

        self._exit_arrow_jog_mode(cancel_active=False)

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
            if self.backend is None:
                self.backend = backend_module.MonochromatorGUIBackend()
            snapshot = self.backend.initialize(selected_port)
            ready_port = snapshot.get("port") or init_target
            self._set_busy(False, f"Controller ready on {ready_port}.")
            self._set_backend_ready(True)
            self._update_status(snapshot)
            self._refresh_calibration_views(silent=True)
            self._refresh_zwo_camera_status_view(silent=True)
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
        ready_message = (
            "Controller ready."
            if task_kind == "cli_jog"
            else (self._arrow_jog_status_text() if self._is_jog_active() else "Controller ready.")
        )
        self._set_busy(False, "Closing controller..." if self._closing else ready_message)
        if isinstance(result, dict) and "step_count" in result:
            self._update_status(result)
        if task_kind == "cli_jog":
            self._set_cli_jog_active(False)
            self._hide_cli_jog_dialog()
            if self._closing:
                self._log("[GUI] CLI jog finished during shutdown")
                return
            self._log("[GUI] CLI jog exited")
            QTimer.singleShot(0, self._restore_post_jog_ui)
            return
        if task_kind == "hold_jog":
            if self._mouse_jog_active and self._jog_hold_direction != 0:
                QTimer.singleShot(0, self._run_next_mouse_hold_step)
            else:
                QTimer.singleShot(0, self._restore_post_jog_ui)
            return
        if task_kind in {"continuous_jog", "continuous_jog_speed"}:
            if self.arrow_jog_checkbox.isChecked() and (
                self._jog_hold_direction != 0 or self._pending_jog_speed_ms is not None
            ):
                QTimer.singleShot(0, self._continue_continuous_jog)
            else:
                QTimer.singleShot(0, self._restore_post_jog_ui)
            return
        if task_kind in {"s1_calibrate", "s2_calibrate", "wl_ref", "wl_fit", "wl_backup", "wl_clear", "zwo_calibration"}:
            self._refresh_calibration_views(silent=True)
        if task_kind == "zwo_calibration":
            self.zwo_info_output.setPlainText(self._format_zwo_result(result.get("_result")))
        elif task_kind == "zwo_camera_refresh":
            payload = result.get("_result", result)
            cameras = payload.get("cameras", [])
            self._update_zwo_camera_combo(cameras)
            self.zwo_camera_info_output.setPlainText(
                f"Detected {payload.get('camera_count', 0)} camera(s).\n"
                f"SDK path: {payload.get('sdk_path') or '$ZWO_ASI_LIB'}"
            )
            self._refresh_zwo_camera_status_view(silent=True)
        elif task_kind in {"zwo_camera_connect", "zwo_camera_disconnect", "zwo_camera_settings", "zwo_camera_roi"}:
            payload = result.get("_result", result)
            if task_kind == "zwo_camera_settings":
                self._zwo_pending_camera_settings = None
            self._apply_zwo_status_to_view(payload)
            if task_kind == "zwo_camera_disconnect":
                self._set_zwo_preview_text("No frame captured.")
                self.zwo_camera_info_output.setPlainText("ZWO camera disconnected.")
            elif task_kind == "zwo_camera_connect":
                self._zwo_analysis_local_roi = None
                self.zwo_camera_info_output.setPlainText("ZWO camera connected. Capture a single frame to verify the signal.")
            elif task_kind == "zwo_camera_roi":
                self._zwo_analysis_local_roi = None
                self._refresh_zwo_analysis_view()
                self.zwo_camera_info_output.setPlainText(
                    "ROI applied.\n"
                    f"ROI: ({payload.get('roi_start_x', '--')}, {payload.get('roi_start_y', '--')}) "
                    f"{payload.get('frame_width', '--')} x {payload.get('frame_height', '--')}\n"
                    "Smaller ROI can support higher live FPS."
                )
            else:
                self.zwo_camera_info_output.setPlainText("Camera settings applied.")
        elif task_kind == "zwo_camera_capture":
            payload = result.get("_result", result)
            self._apply_zwo_status_to_view(payload.get("status", {}))
            self._set_zwo_preview_from_array(payload.get("frame_array"))
            info_lines = ["Single-frame capture complete."]
            info_lines.append(f"Frame: {payload.get('frame_width', '--')} x {payload.get('frame_height', '--')}")
            info_lines.append(f"Min/Max: {payload.get('frame_min', '--')} / {payload.get('frame_max', '--')}")
            if payload.get("frame_mean") is not None:
                info_lines.append(f"Mean: {payload['frame_mean']:.3f}")
            self.zwo_camera_info_output.setPlainText("\n".join(info_lines))
        elif task_kind == "zwo_center_feature":
            payload = result.get("_result", result)
            self._apply_zwo_status_to_view(payload.get("status", {}))
            self._set_zwo_preview_from_array(payload.get("frame_array"))
            feature_name = str(payload.get("feature") or "feature").capitalize()
            axis_name = "X" if payload.get("axis") == "x" else "Y"
            analysis = payload.get("analysis") or {}
            info_lines = [f"{feature_name} centering assist complete."]
            info_lines.append(f"Axis: {axis_name}")
            info_lines.append(f"Converged: {'Yes' if payload.get('converged') else 'No'}")
            info_lines.append(f"Net steps moved: {payload.get('steps_moved_net', '--')}")
            info_lines.append(f"Iterations used: {payload.get('iterations_used', '--')}")
            if payload.get("probe_shift_per_cw_step") is not None:
                info_lines.append(f"Measured shift per CW step: {payload['probe_shift_per_cw_step']:.3f} px")
            if payload.get("initial_error_px") is not None and payload.get("final_error_px") is not None:
                info_lines.append(
                    f"Error: {payload['initial_error_px']:+.3f} px -> {payload['final_error_px']:+.3f} px "
                    f"(tol {payload.get('tolerance_px', '--')} px)"
                )
            if analysis.get("feature_sensor_position") is not None:
                info_lines.append(f"{feature_name} sensor {axis_name}: {analysis['feature_sensor_position']:.2f}")
            if analysis.get("integrated_intensity") is not None:
                info_lines.append(f"Integrated intensity: {analysis['integrated_intensity']:.1f}")
            self.zwo_camera_info_output.setPlainText("\n".join(info_lines))
        if task_kind == "jog_click":
            return
        if self._closing:
            self._log("[GUI] Operation finished during shutdown")
            return
        self._log("[GUI] Operation complete")

    def _task_failed(self, error_text):
        task_kind = self._active_task_kind
        fail_message = (
            "Operation failed."
            if task_kind == "cli_jog"
            else (self._arrow_jog_status_text() if self._is_jog_active() else "Operation failed.")
        )
        self._set_busy(False, "Closing controller..." if self._closing else fail_message)
        if task_kind == "cli_jog":
            self._set_cli_jog_active(False)
            self._hide_cli_jog_dialog()
            self._log("[GUI] CLI jog failed")
            self._log(error_text)
            if self._closing:
                return
            self.connection_label.setText("CLI jog failed. See log output.")
            QMessageBox.critical(self, "CLI jog failed", error_text)
            return
        if task_kind == "hold_jog":
            self._clear_mouse_jog_state()
            self._jog_hold_direction = 0
            if self._closing:
                return
            if "Operation cancelled" in error_text:
                self._log("[GUI] Hold jog cancelled")
                QTimer.singleShot(0, self._restore_post_jog_ui)
                return
            self._log("[GUI] Hold jog failed")
            self._log(error_text)
            self.connection_label.setText("Hold jog failed. See log output.")
            QTimer.singleShot(0, self._restore_post_jog_ui)
            QMessageBox.critical(self, "Hold jog failed", error_text)
            return
        if task_kind in {"continuous_jog", "continuous_jog_speed"}:
            self._jog_hold_direction = 0
            self._pending_jog_speed_ms = None
            if self._closing:
                return
            if "Operation cancelled" in error_text:
                self._log("[GUI] Jog cancelled")
                if not self.arrow_jog_checkbox.isChecked():
                    QTimer.singleShot(0, self._restore_post_jog_ui)
                return
            self._log("[GUI] Jog failed")
            self._log(error_text)
            if self.arrow_jog_checkbox.isChecked():
                self._exit_arrow_jog_mode(cancel_active=False)
            else:
                QTimer.singleShot(0, self._restore_post_jog_ui)
            self.connection_label.setText("Jog failed. See log output.")
            return
        if task_kind == "zwo_camera_settings":
            self._zwo_pending_camera_settings = None
        if task_kind and task_kind.startswith("zwo_camera"):
            self.zwo_camera_info_output.setPlainText(error_text)
        elif task_kind == "zwo_center_feature":
            self.zwo_camera_info_output.setPlainText(error_text)
        self._log("[GUI] Operation failed")
        self._log(error_text)
        if self._closing:
            return
        QMessageBox.critical(self, "Operation failed", error_text)

    def _task_finished(self):
        thread = self.sender()
        finished_task_kind = self._active_task_kind
        if thread is self._task_thread:
            self._task_thread = None
            self._active_task_kind = None
        if thread is not None:
            thread.deleteLater()
        self._refresh_control_states()
        if not self._closing and self.backend is not None and not self._is_jog_active():
            ready_port = "unknown port"
            try:
                ready_port = self.backend.status().get("port") or "unknown port"
            except BaseException:
                pass
            self.connection_label.setText(f"Controller ready on {ready_port}.")
        if (
            finished_task_kind == "zwo_center_feature"
            and self._zwo_resume_live_after_center
            and not self._closing
            and self.backend is not None
            and self._zwo_live_thread is None
        ):
            self._zwo_resume_live_after_center = False
            QTimer.singleShot(0, self._start_zwo_live_preview)
        elif finished_task_kind == "zwo_center_feature":
            self._zwo_resume_live_after_center = False
        if (
            finished_task_kind == "zwo_camera_roi"
            and self._zwo_resume_live_after_roi_change
            and not self._closing
            and self.backend is not None
            and self._zwo_live_thread is None
        ):
            self._zwo_resume_live_after_roi_change = False
            QTimer.singleShot(0, self._start_zwo_live_preview)
        elif finished_task_kind == "zwo_camera_roi":
            self._zwo_resume_live_after_roi_change = False
        if self._closing and not self._shutdown_complete and self._task_thread is None:
            self._finish_shutdown_after_task()

    def _refresh_status_manual(self):
        if self.backend is None or self._busy or self._task_is_active() or self._is_jog_active():
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
        if self.backend is not None and not self._busy and not self._task_is_active() and not self._is_jog_active():
            self._refresh_status_manual()

    def _start_cli_jog(self):
        if self.backend is None or self._busy or self._task_is_active() or self._closing or self.arrow_jog_checkbox.isChecked():
            return
        self._set_cli_jog_active(True)
        self._refresh_control_states()
        self._show_cli_jog_dialog()
        self.connection_label.setText(self._arrow_jog_status_text())
        self._log("[GUI] Starting CLI jog mode")
        self._run_task("CLI Jog Mode", self.backend.run_cli_jog_mode, task_kind="cli_jog", log_message=False)

    def _stop_cli_jog(self):
        if self.backend is None:
            self._hide_cli_jog_dialog()
            self._set_cli_jog_active(False)
            self._refresh_control_states()
            return
        if not self.arrow_jog_checkbox.isChecked():
            self._hide_cli_jog_dialog()
            self._refresh_control_states()
            return
        self.connection_label.setText("Stopping CLI jog...")
        self._log("[GUI] Stop requested for CLI jog")
        try:
            self.backend.stop_cli_jog_mode()
        except BaseException:
            error_text = traceback.format_exc()
            self._log("[GUI] Failed to request CLI jog stop")
            self._log(error_text)

    def _apply_jog_speed(self):
        new_ms = self.jog_speed_spin.value()
        if self._busy and self._active_task_kind not in {"continuous_jog", "continuous_jog_speed"}:
            return
        if self._jog_hold_direction != 0 or self._active_task_kind in {"continuous_jog", "continuous_jog_speed"}:
            self._pending_jog_speed_ms = new_ms
            return
        self._run_task("Set jog speed", self.backend.set_jog_speed_ms, new_ms)

    def _apply_zwo_jog_speed(self):
        if hasattr(self, "jog_speed_spin"):
            self.jog_speed_spin.setValue(self.zwo_jog_speed_spin.value())
        self._apply_jog_speed()

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
        if self._mouse_jog_active or self._mouse_jog_hold_timer.isActive() or self._mouse_jog_press_direction != 0:
            self._stop_mouse_hold_jog()
            if self._busy and self._active_task_kind == "hold_jog":
                self.backend.cancel()
            self._log("[GUI] Hold jog stop requested")
            return
        self._jog_hold_direction = 0
        if self._active_task_kind == "cli_jog" or self.arrow_jog_checkbox.isChecked():
            self.backend.stop_cli_jog_mode()
            self._log("[GUI] CLI jog stop requested")
            return
        self.backend.cancel()
        self._log("[GUI] Cancel requested")

    def _shutdown_backend(self):
        self.status_timer.stop()
        self._stop_zwo_live_preview(wait=True, timeout_ms=5000)
        if self.backend is not None:
            try:
                self.backend.stop_cli_jog_mode()
            except Exception:
                pass
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
        if (
            jog_speed_ms is not None
            and hasattr(self, "zwo_jog_speed_spin")
            and not self._widget_or_child_has_focus(self.zwo_jog_speed_spin)
        ):
            self.zwo_jog_speed_spin.setValue(int(jog_speed_ms))

    def closeEvent(self, event):
        if self._shutdown_complete:
            event.accept()
            super().closeEvent(event)
            return

        self._jog_hold_direction = 0
        self._pending_jog_speed_ms = None
        self._stop_mouse_hold_jog()
        self._stop_cli_jog()
        self._hide_arrow_jog_dialog()
        self._closing = True
        self._set_busy(self._busy, "Closing controller...")
        if not self._stop_zwo_live_preview(wait=True, timeout_ms=5000):
            self._closing = False
            self._set_busy(self._busy, "Live preview still stopping...")
            self._log("[GUI] Close delayed: ZWO live preview thread is still shutting down")
            event.ignore()
            return

        if self.backend is not None and self._task_thread is not None and self._task_thread.isRunning():
            self._log("[GUI] Close requested; cancelling active operation")
            if self._active_task_kind == "cli_jog":
                self.backend.stop_cli_jog_mode()
            else:
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
