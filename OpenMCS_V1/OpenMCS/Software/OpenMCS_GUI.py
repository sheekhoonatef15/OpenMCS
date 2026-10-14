"""OpenMCS production GUI.

Receives FPGA-accumulated MCS6 histogram frames from the Teensy USB bridge and
performs interactive signal/background analysis.
"""

from __future__ import annotations

import math
import json
from pathlib import Path
import os
import queue
import re
import csv
from collections import deque
import struct
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
from OpenMCS_protocol import (
    FrameParser, MCS_MAGIC, LIVE_BLOCK_MAGIC, MCS_TRAILER,
    MAX_BINS, MIN_DWELL_CYCLES, EXPECTED_CLOCK_HZ,
)
import serial
import serial.tools.list_ports

os.environ["PYQTGRAPH_QT_LIB"] = "PyQt6"
import pyqtgraph as pg
import pyqtgraph.exporters

from PyQt6.QtCore import QEvent, QSettings, QSignalBlocker, QThread, QTimer, Qt, QRect, QRectF, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QKeySequence, QPainter, QPen, QShortcut
from PyQt6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QColorDialog, QComboBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QDialog,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QMessageBox,
    QPushButton, QScrollArea, QSpinBox, QSplitter, QStatusBar, QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QVBoxLayout, QWidget,
)

APP_NAME = "OpenMCS"
APP_VERSION = "8.0.6-performance"
USB_BAUD = 115_200
# All released V3/V4 sketches in this project use the same line protocol.
# Match the stable family prefix so the maximum-accuracy V4 sketch connects.
TEENSY_ID = b"OPENMCS_TEENSY4_V"
STATUS_MAGIC = b"@@OPENMCS:"
DEFAULT_BINS = 10_000
MAX_MCS_DWELL_S = 0.001       # 1 ms/bin
MAX_MCS_PASS_SPAN_S = 20.0    # supports 16,384 bins at 1 ms/bin
BG = "#0b1016"
PANEL = "#121922"
PANEL_2 = "#17212c"
BORDER = "#293747"
TEXT = "#e7edf5"
MUTED = "#94a3b8"
CYAN = "#36c5f0"
GREEN = "#3ddc97"
AMBER = "#ffb454"
RED = "#ff6b6b"
LIGHT_BG = "#f4f7fb"


@dataclass(frozen=True)
class RegionMetrics:
    signal_start: int
    signal_end: int
    noise_start: int
    noise_end: int
    signal_bins: int
    noise_bins: int
    gross_signal: int
    noise_total: int
    noise_per_bin: float
    estimated_background: float
    net_signal: float
    uncertainty: float
    snr: float


def calculate_region_metrics(counts, signal_start, signal_end, noise_start, noise_end):
    """Background-scaled signal statistics for inclusive bin regions."""
    if counts.size == 0:
        raise ValueError("Histogram is empty")
    last = counts.size - 1
    signal_start = max(0, min(int(signal_start), last))
    signal_end = max(signal_start, min(int(signal_end), last))
    noise_start = max(0, min(int(noise_start), last))
    noise_end = max(noise_start, min(int(noise_end), last))
    signal_bins = signal_end - signal_start + 1
    noise_bins = noise_end - noise_start + 1
    gross = int(counts[signal_start : signal_end + 1].sum(dtype=np.uint64))
    noise_total = int(counts[noise_start : noise_end + 1].sum(dtype=np.uint64))
    noise_per_bin = noise_total / noise_bins
    scale = signal_bins / noise_bins
    background = scale * noise_total
    net = gross - background
    uncertainty = math.sqrt(max(0.0, gross + scale * scale * noise_total))
    snr = net / uncertainty if uncertainty else 0.0
    return RegionMetrics(
        signal_start, signal_end, noise_start, noise_end, signal_bins, noise_bins,
        gross, noise_total, noise_per_bin, background, net, uncertainty, snr,
    )


def linear_fit_active_rate(samples, pass_active_s):
    """Fit cumulative counts C(P)=m*P+b and return m/active-pass time."""
    if len(samples) < 2 or pass_active_s <= 0.0 or not math.isfinite(pass_active_s):
        return None
    passes = np.asarray([sample[0] for sample in samples], dtype=np.float64)
    totals = np.asarray([sample[1] for sample in samples], dtype=np.float64)
    passes -= passes.mean()
    totals -= totals.mean()
    denominator = float(np.dot(passes, passes))
    if denominator <= 0.0 or not math.isfinite(denominator):
        return None
    counts_per_pass = float(np.dot(passes, totals) / denominator)
    if counts_per_pass < 0.0 or not math.isfinite(counts_per_pass):
        return None
    return counts_per_pass / pass_active_s


def histogram_step_xy(x_values, counts):
    """Return staircase coordinates when x_values are LEFT EDGES of MCS bins.

    OpenMCS bin i spans [x[i], x[i] + dx].  For example, with a
    100 ns dwell: bin 0 = 0..100 ns, bin 1 = 100..200 ns, etc.
    """
    x = np.asarray(x_values, dtype=np.float64)
    y = np.asarray(counts)
    n = min(x.size, y.size)
    if n == 0:
        return np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64)
    x = x[:n]
    y = y[:n]

    if n == 1:
        # Safe fallback. Normal MCS displays contain multiple bins.
        dx = 1.0
    else:
        dx = float(x[-1] - x[-2])
        if not math.isfinite(dx) or dx <= 0.0:
            dx = 1.0

    # x[i] is the LEFT edge, not the center.  Append the right edge of
    # the final bin, then construct the explicit histogram staircase.
    edges = np.empty(n + 1, dtype=np.float64)
    edges[:-1] = x
    edges[-1] = x[-1] + dx
    sx = np.repeat(edges, 2)[1:-1]
    sy = np.repeat(y.astype(np.float64, copy=False), 2)
    return sx, sy




def available_ports():
    """Return serial ports with likely Teensy devices first."""
    ports = list(serial.tools.list_ports.comports())

    def is_teensy(port):
        text = " ".join(
            str(value or "")
            for value in (port.description, port.manufacturer, port.product, port.hwid)
        ).lower()
        return port.vid == 0x16C0 or "teensy" in text or "pjrc" in text

    ports.sort(key=lambda port: (not is_teensy(port), port.device))
    return ports


class SerialWorker(QThread):
    frame_received = pyqtSignal(object, object)
    connection_changed = pyqtSignal(bool, str)
    message = pyqtSignal(str, str)

    def __init__(self, port_name):
        super().__init__()
        self.port_name = port_name
        self._stop_event = threading.Event()
        self._commands = queue.Queue()
        self._status_buffer = bytearray()

    def _parse_instrument_status(self, data):
        """Display tagged Teensy messages without confusing them with MCS6 data."""
        self._status_buffer.extend(data)
        while True:
            marker = self._status_buffer.find(STATUS_MAGIC)
            if marker < 0:
                preserve = len(STATUS_MAGIC) - 1
                if len(self._status_buffer) > preserve:
                    del self._status_buffer[:-preserve]
                return
            newline = self._status_buffer.find(b"\n", marker)
            if newline < 0:
                if marker:
                    del self._status_buffer[:marker]
                return
            record = bytes(self._status_buffer[marker + len(STATUS_MAGIC):newline])
            del self._status_buffer[:newline + 1]
            try:
                level_bytes, message_bytes = record.split(b":", 1)
                level = level_bytes.decode("ascii", errors="replace").lower()
                message = message_bytes.decode("utf-8", errors="replace").strip()
            except ValueError:
                continue
            if level == "identity":
                continue
            if level not in ("info", "success", "warning", "error"):
                level = "info"
            if message:
                self.message.emit(message, level)

    @staticmethod
    def _verify_teensy(serial_port):
        """Reject a COM port unless the current OpenMCS Teensy sketch answers."""
        serial_port.reset_input_buffer()
        serial_port.write(b"i\n")
        serial_port.flush()
        reply = bytearray()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            waiting = serial_port.in_waiting
            chunk = serial_port.read(waiting if waiting else 1)
            if chunk:
                reply.extend(chunk)
                if TEENSY_ID in reply:
                    return bytes(reply)
                if len(reply) > 8192:
                    del reply[:-4096]
        raise RuntimeError(
            "The selected COM port did not identify as the OpenMCS Teensy. "
            "Select the Teensy COM port and upload the matching OpenMCS_Teensy4_0.ino sketch."
        )

    def send_command(self, command):
        self._commands.put(command.strip() + "\n")

    def stop(self):
        self._stop_event.set()

    def run(self):
        serial_port = None
        parser = FrameParser()
        try:
            serial_port = serial.Serial(
                self.port_name, USB_BAUD, timeout=0.05, write_timeout=1.0
            )
            startup_reply = self._verify_teensy(serial_port)
            self._parse_instrument_status(startup_reply)
            self.connection_changed.emit(True, self.port_name)
            while not self._stop_event.is_set():
                while True:
                    try:
                        command = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    serial_port.write(command.encode("ascii"))
                    serial_port.flush()
                waiting = serial_port.in_waiting
                data = serial_port.read(waiting if waiting else 1)
                if data:
                    self._parse_instrument_status(data)
                    for counts, metadata in parser.feed(data):
                        metadata["discarded_bytes"] = parser.discarded_bytes
                        metadata["invalid_frames"] = parser.invalid_frames
                        self.frame_received.emit(counts, metadata)
        except (serial.SerialException, OSError, RuntimeError) as error:
            self.message.emit(f"Serial connection error: {error}", "error")
        finally:
            if serial_port is not None:
                try:
                    serial_port.close()
                except serial.SerialException:
                    pass
            self.connection_changed.emit(False, self.port_name)


class MetricValue(QFrame):
    def __init__(self, title, value="—"):
        super().__init__()
        self.setObjectName("MetricValue")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(1)
        caption = QLabel(title)
        caption.setObjectName("MetricCaption")
        self.value = QLabel(value)
        self.value.setObjectName("MetricNumber")
        self.value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(caption)
        layout.addWidget(self.value)

    def set_value(self, value):
        self.value.setText(value)


class SelectionViewBox(pg.ViewBox):
    """ViewBox that reserves Ctrl + left-drag for histogram selection."""

    selectionDragged = pyqtSignal(float, float, bool)

    def mouseDragEvent(self, event, axis=None):
        ctrl_left_drag = (
            event.button() == Qt.MouseButton.LeftButton and
            bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier)
        )
        if not ctrl_left_drag:
            super().mouseDragEvent(event, axis=axis)
            return
        event.accept()
        start = self.mapSceneToView(event.buttonDownScenePos()).x()
        current = self.mapSceneToView(event.scenePos()).x()
        self.selectionDragged.emit(
            float(min(start, current)), float(max(start, current)),
            bool(event.isFinish()),
        )


class SelectAllLineEdit(QLineEdit):
    """Select all text whenever the field is clicked."""
    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        QTimer.singleShot(0, self.selectAll)


class InsetResizeHandle(QWidget):
    """Bottom-right handle that resizes only its FloatingOverview parent."""

    def __init__(self, inset):
        super().__init__(inset)
        self.inset = inset
        self._press_global = None
        self._start_size = None
        self._background = QColor("#526579")
        self._border = QColor("#94a3b8")
        self.setFixedSize(24, 24)
        self.setCursor(Qt.CursorShape.SizeFDiagCursor)
        self.setToolTip("Drag to resize overview only")
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)

    def setThemeColors(self, background, border):
        self._background = QColor(background)
        self._border = QColor(border)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), self._background)
        painter.setPen(QPen(self._border, 2.0))
        # Three diagonal grip marks make the resize target visually obvious.
        w = self.width() - 5
        h = self.height() - 5
        for offset in (0, 6, 12):
            painter.drawLine(w - offset, h, w, h - offset)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._press_global = event.globalPosition().toPoint()
            self._start_size = self.inset.size()
            self.grabMouse()
            event.accept()
            return
        event.ignore()

    def mouseMoveEvent(self, event):
        if self._press_global is None or self._start_size is None:
            event.ignore()
            return
        delta = event.globalPosition().toPoint() - self._press_global
        new_w = max(self.inset.minimumWidth(), self._start_size.width() + delta.x())
        new_h = max(self.inset.minimumHeight(), self._start_size.height() + delta.y())
        # Critical: resize ONLY the floating inset. No splitter, parent, or main plot changes.
        self.inset.resize(new_w, new_h)
        self.inset.raise_()
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._press_global is not None:
            self._press_global = None
            self._start_size = None
            self.releaseMouse()
            event.accept()
            return
        event.ignore()


class FloatingOverview(QFrame):
    """Layout-independent overview overlay. Drag title; resize with bottom-right grip."""
    navigateRequested = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("FloatingOverview")
        self.setMinimumSize(180, 120)
        self.resize(320, 190)
        self._drag_origin = None
        self._frame_origin = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(0)

        self.title = QLabel("Spectrum overview — drag to move")
        self.title.setFixedHeight(24)
        self.title.setCursor(Qt.CursorShape.SizeAllCursor)
        self.title.installEventFilter(self)
        layout.addWidget(self.title)

        self.plot = pg.PlotWidget()
        self.plot.setMenuEnabled(False)
        self.plot.hideAxis("left")
        self.plot.hideAxis("bottom")
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.scene().sigMouseClicked.connect(self._clicked)
        layout.addWidget(self.plot, 1)

        # Custom handle: QSizeGrip would target a top-level/subwindow and can
        # resize the OpenMCS window. This handle explicitly resizes only self.
        self.grip = InsetResizeHandle(self)
        self.grip.setThemeColors("#526579", "#cbd5e1")
        self.grip.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.grip.move(self.width() - self.grip.width() - 2,
                       self.height() - self.grip.height() - 2)
        self.grip.raise_()

    def eventFilter(self, obj, event):
        if obj is self.title:
            if event.type() == QEvent.Type.MouseButtonPress and event.button() == Qt.MouseButton.LeftButton:
                self._drag_origin = event.globalPosition().toPoint()
                self._frame_origin = self.pos()
                return True
            if event.type() == QEvent.Type.MouseMove and self._drag_origin is not None:
                delta = event.globalPosition().toPoint() - self._drag_origin
                self.move(self._frame_origin + delta)
                self.raise_()
                return True
            if event.type() == QEvent.Type.MouseButtonRelease:
                self._drag_origin = None
                self._frame_origin = None
                return True
        return super().eventFilter(obj, event)

    def _clicked(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            point = self.plot.getViewBox().mapSceneToView(event.scenePos())
            self.navigateRequested.emit(float(point.x()))

    def addItem(self, item):
        self.plot.addItem(item)

    def setRange(self, *args, **kwargs):
        self.plot.setRange(*args, **kwargs)

    def setThemeColors(self, background, border):
        self.setStyleSheet(
            f"QFrame#FloatingOverview {{ background:{background}; border:2px solid {border}; }}"
            f"QLabel {{ background:{background}; padding-left:6px; font-weight:600; }}"
        )
        self.plot.setBackground(background)
        # Deliberately contrast the custom resize handle with the panel.
        grip_bg = "#526579" if QColor(background).lightness() < 128 else "#cbd5e1"
        grip_border = "#f8fafc" if QColor(background).lightness() < 128 else "#334155"
        self.grip.setThemeColors(grip_bg, grip_border)

class OpenMCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.settings = QSettings("OpenMCS", "OpenMCS")
        self.worker = None
        self._live_preview_metadata = None
        self._live_preview_warning = False
        self.live_preview_timer = QTimer(self)
        self.live_preview_timer.setSingleShot(True)
        self.live_preview_timer.setInterval(33)
        self.live_preview_timer.timeout.connect(self._render_live_preview)
        self.latest_counts = None
        self.latest_metadata = None
        self.mcs_latest_counts = None
        self.mcs_latest_metadata = None
        self.mcs_accumulation_frames = 0
        self.counter_latest_counts = None
        self.counter_latest_metadata = None
        self.pass_number = 0
        self.accumulation_frames = 0
        self.accumulation_signature = None
        self.rejected_frames = 0
        self.frame_timestamps = deque(maxlen=512)
        self.frame_rate_ema = 0.0
        self._updating_regions = False
        self._auto_connect_attempted = False
        self.instrument_mode = "mcs"
        self.trace_color = CYAN
        self.counter_signal_color = CYAN
        self.counter_trigger_color = AMBER
        self.x_axis_mode = "time"
        self.counter_times = []
        self.counter_cps = []
        self.counter_trigger_cps = []
        self.counter_last_count = 0
        self.counter_last_trigger_count = 0
        self.counter_last_pass = 0
        self.counter_segment_count = 0
        self.counter_segment_signal_sum = 0
        self.counter_segment_trigger_sum = 0
        self.counter_segment_elapsed = 0.0
        self.counter_started = None
        self.counter_baseline_pending = True
        self.counter_segments_per_point = 1
        self.counter_accept_after = 0.0
        self.mode_accept_after = 0.0
        self.acquisition_running = False
        self.mcs_display_frozen = False
        self.mcs_view_initialized = False
        # User-requested live auto-range. Off by default so acquisition does not move the view.
        self.mcs_manual_autorange = False
        self.mcs_clear_pending = False
        self.configured_signature = None
        self.highlighted_bins = None
        # Frozen comparison/reference histogram. This is GUI-only and survives
        # new acquisitions until explicitly replaced or removed.
        self.reference_counts = None
        self.reference_metadata = None
        self.reference_curve = None
        self.reference_axis_mode = None
        self.acquisition_started_at = None
        self.acquisition_stopped_at = None
        self.acquisition_baseline_passes = 0
        self.acquisition_target_passes = None
        # MCS live-rate history. Rates are calculated from successive cumulative
        # FPGA frames, never from GUI paint/update frequency.
        self.mcs_rate_times = []  # retain full CPS history for troubleshooting
        self.mcs_count_rates = []
        self.mcs_pass_rates = []
        self.mcs_rate_last_time = None
        self.mcs_rate_last_total = None
        self.mcs_rate_last_passes = None
        self.mcs_rate_started = None
        self.mcs_rate_fit_samples = deque(maxlen=8)
        self.mcs_rate_fit_signature = None
        # First live MCS frame after every Start is baseline-only for CPS.
        self.mcs_rate_baseline_pending = True
        self.eta_pass_samples = deque()
        self.eta_window_seconds = 10.0
        self.eta_min_span_seconds = 1.0
        self.eta_last_rate = None
        self.eta_interval_values = []
        self.eta_interval_rates = []
        self.eta_display_eta = None
        self.eta_display_rate = None
        self.eta_last_display_update = time.monotonic()
        self.eta_display_period_s = 5.0
        self._counter_start_pending = False
        # Independent plot view ranges are preserved when switching modes.
        self._mcs_saved_view = None
        self._counter_saved_view = None
        # User-measured acquisition clock. The FPGA protocol continues to
        # identify the nominal 100 MHz design; this value corrects all physical
        # time and rate calculations in both MCS and Counter displays.
        self.clock_frequency_hz = self.settings.value(
            "calibration/clock_hz", float(EXPECTED_CLOCK_HZ), float
        )
        self._layout_editor = None
        self._layout_selected_widget = None
        self._layout_selected_style = ""
        self._layout_edit_active = False

        self.setWindowTitle(f"{APP_NAME} {APP_VERSION} — Multichannel Scaler")
        self.resize(1280, 780)
        self.setMinimumSize(980, 640)
        self._build_ui()
        self._connect_signals()
        self._install_shortcuts()
        self._restore_settings()
        self._set_connected(False)
        self.refresh_ports()
        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self.refresh_ports)
        self.port_timer.start(2000)
        self.acquisition_timer = QTimer(self)
        self.acquisition_timer.timeout.connect(self._update_acquisition_timing)
        self.acquisition_timer.start(250)
        QTimer.singleShot(350, self._try_initial_auto_connect)

    def _build_ui(self):
        scroll_area = QScrollArea()
        scroll_area.setObjectName("MainScrollArea")
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        root = QWidget()
        # Preserve a useful plot height on smaller displays. The main window
        # becomes vertically scrollable instead of compressing labels, toolbars,
        # and plots until text disappears.
        root.setMinimumSize(900, 820)
        scroll_area.setWidget(root)
        self.main_scroll_area = scroll_area
        self.setCentralWidget(scroll_area)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(14, 12, 14, 10)
        outer.setSpacing(10)

        header = QHBoxLayout()
        self.header_layout = header
        title_box = QVBoxLayout()
        self.app_title_label = QLabel("OpenMCS")
        self.app_title_label.setObjectName("CompactTitle" if os.environ.get("OPENQMA_EMBEDDED") else "AppTitle")
        self.app_subtitle_label = QLabel(
            "Triggered multichannel scaler · 100 MHz FPGA accumulation · "
            "GUI v6.7"
        )
        self.app_subtitle_label.setObjectName("AppSubtitle")
        self.app_subtitle_label.setWordWrap(True)
        self.app_subtitle_label.setMinimumWidth(0)
        if os.environ.get("OPENQMA_EMBEDDED"):
            self.app_subtitle_label.setVisible(False)
        title_box.addWidget(self.app_title_label)
        title_box.addWidget(self.app_subtitle_label)
        header.addLayout(title_box)
        header.addStretch(1)

        # Always-visible mode selector. Switching instruments should not
        # require opening Setup or finding a combo box among acquisition fields.
        mode_switch = QFrame()
        mode_switch.setObjectName("ModeSwitch")
        mode_layout = QHBoxLayout(mode_switch)
        mode_layout.setContentsMargins(4, 4, 4, 4)
        mode_layout.setSpacing(3)
        mode_label = QLabel("MODE")
        mode_label.setObjectName("ModeLabel")
        self.mcs_mode_button = QPushButton("MCS")
        self.counter_mode_button = QPushButton("COUNTER")
        for button in (self.mcs_mode_button, self.counter_mode_button):
            button.setObjectName("ModeButton")
            button.setCheckable(True)
            button.setMinimumWidth(86)
        self.mcs_mode_button.setToolTip("Triggered multichannel scaler (Ctrl+1)")
        self.counter_mode_button.setToolTip("Continuous CPS alignment counter (Ctrl+2)")
        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.setExclusive(True)
        self.mode_button_group.addButton(self.mcs_mode_button)
        self.mode_button_group.addButton(self.counter_mode_button)
        self.mcs_mode_button.setChecked(True)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mcs_mode_button)
        mode_layout.addWidget(self.counter_mode_button)
        header.addWidget(mode_switch)

        self.setup_button = QPushButton("Setup")
        self.setup_button.setToolTip("Show acquisition setup (Ctrl+, or F2)")
        self.shortcuts_button = QPushButton("Shortcuts")
        self.shortcuts_button.setCheckable(True)
        self.shortcuts_button.setToolTip("Show/hide keyboard shortcuts")
        self.theme_combo = QComboBox()
        self.theme_combo.addItem("Dark", "dark")
        self.theme_combo.addItem("Light", "light")
        self.theme_combo.setToolTip("Interface color theme")
        self.help_button = QPushButton("Help")
        self.help_button.setToolTip("Open help and keyboard shortcuts (F1)")
        self.layout_button = QPushButton("Edit layout")
        self.layout_button.setToolTip("Show, hide, and resize the workspace panels")
        self.header_actions = QWidget()
        self.header_actions_layout = QHBoxLayout(self.header_actions)
        self.header_actions_layout.setContentsMargins(0, 0, 0, 0)
        self.header_actions_layout.setSpacing(8)
        self.header_actions_layout.addWidget(self.setup_button)
        self.header_actions_layout.addWidget(self.layout_button)
        self.header_actions_layout.addWidget(self.shortcuts_button)
        self.header_actions_layout.addWidget(self.theme_combo)
        self.header_actions_layout.addWidget(self.help_button)
        header.addWidget(self.header_actions)
        self.connection_badge = QLabel("●  DISCONNECTED")
        self.connection_badge.setObjectName("ConnectionBadge")
        header.addWidget(self.connection_badge)
        outer.addLayout(header)

        # The right Status/ROI pane spans the full working height.  Everything
        # that supports acquisition lives in a compact left stack so the plot
        # receives essentially all remaining space.
        self.workspace_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.setHandleWidth(3)
        outer.addWidget(self.workspace_splitter, 1)

        left_workspace = QWidget()
        left_workspace_layout = QVBoxLayout(left_workspace)
        left_workspace_layout.setContentsMargins(0, 0, 0, 0)
        left_workspace_layout.setSpacing(8)
        self.workspace_splitter.addWidget(left_workspace)

        connection = QFrame()
        self.connection_bar = connection
        connection.setObjectName("ControlBar")
        connection_layout = QHBoxLayout(connection)
        self.connection_layout = connection_layout
        connection_layout.setContentsMargins(10, 5, 10, 5)
        self.instrument_label = QLabel("Instrument")
        connection_layout.addWidget(self.instrument_label)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(220)
        self.refresh_button = QPushButton("Refresh")
        self.connect_button = QPushButton("Connect")
        self.connect_button.setObjectName("PrimaryButton")
        self.auto_connect_check = QCheckBox("Auto-connect")
        self.auto_connect_check.setChecked(True)
        connection_layout.addWidget(self.port_combo, 1)
        connection_layout.addWidget(self.refresh_button)
        connection_layout.addWidget(self.auto_connect_check)
        connection_layout.addWidget(self.connect_button)
        left_workspace_layout.addWidget(connection)

        # Persistent experiment save target. This removes repeated Save dialogs
        # during acquisition: choose a folder and run name once, then use Save Data.
        save_bar = QFrame()
        self.save_bar = save_bar
        save_bar.setObjectName("ControlBar")
        save_layout = QHBoxLayout(save_bar)
        self.save_layout = save_layout
        save_layout.setContentsMargins(10, 5, 10, 5)
        save_layout.setSpacing(6)

        self.save_to_label = QLabel("Save to")
        save_layout.addWidget(self.save_to_label)
        self.save_directory_edit = QLineEdit()
        self.save_directory_edit.setPlaceholderText("Experiment folder")
        self.save_directory_edit.setToolTip("Folder used by Save Data")
        self.save_directory_edit.setMinimumWidth(220)

        self.browse_save_directory_button = QPushButton("Browse…")
        self.browse_save_directory_button.setToolTip("Choose experiment folder")

        save_layout.addWidget(self.save_directory_edit, 2)
        save_layout.addWidget(self.browse_save_directory_button)

        self.run_name_label = QLabel("Run name")
        save_layout.addWidget(self.run_name_label)
        self.run_name_edit = SelectAllLineEdit()
        self.run_name_edit.setPlaceholderText("sample_001")
        self.run_name_edit.setToolTip(
            "Base filename. Save Data always creates CSV and optionally creates PNG. "
            "The entered name is preserved; an existing name receives a numbered suffix."
        )
        self.run_name_edit.setMinimumWidth(150)
        save_layout.addWidget(self.run_name_edit, 1)

        self.data_settings_button = QPushButton("Data Settings…")
        self.data_settings_button.setToolTip(
            "Choose saved columns, column names/order, and visible acquisition-status fields"
        )
        save_layout.addWidget(self.data_settings_button)

        self.save_image_check = QCheckBox("Save image")
        self.save_image_check.setChecked(True)
        self.save_image_check.setToolTip("Save a PNG together with the CSV")
        save_layout.addWidget(self.save_image_check)
        self.save_data_button = QPushButton("Save Data")
        self.save_data_button.setObjectName("PrimaryButton")
        self.save_data_button.setToolTip(
            "Save CSV and, when selected, a spectrum image using the folder and run name"
        )
        save_layout.addWidget(self.save_data_button)
        left_workspace_layout.addWidget(save_bar)

        # Acquisition setup lives in the right-side column.  A vertical layout
        # prevents labels/buttons from colliding when the window is resized.
        acquisition = QGroupBox("Acquisition controls")
        self.acquisition_group = acquisition
        setup = QVBoxLayout(acquisition)
        setup.setContentsMargins(10, 8, 10, 8)
        setup.setSpacing(7)

        self.bins_spin = QSpinBox()
        self.bins_spin.setRange(1, MAX_BINS)
        self.bins_spin.setGroupSeparatorShown(True)
        self.bins_spin.setValue(DEFAULT_BINS)

        self.dwell_spin = QDoubleSpinBox()
        self.dwell_spin.setRange(0.001, 10_000_000.0)
        self.dwell_spin.setDecimals(3)
        self.dwell_spin.setValue(100.0)
        self.dwell_unit = QComboBox()
        for label, multiplier in (("ns", 1e-9), ("µs", 1e-6), ("ms", 1e-3), ("s", 1.0)):
            self.dwell_unit.addItem(label, multiplier)

        self.trigger_spin = QSpinBox()
        self.trigger_spin.setRange(1, 1_000_000)
        self.trigger_spin.setGroupSeparatorShown(True)
        self.trigger_spin.setSuffix(" Hz")
        self.trigger_spin.setValue(100)
        self.trigger_edge_combo = QComboBox()
        self.trigger_edge_combo.addItem("Rising edge", "r")
        self.trigger_edge_combo.addItem("Falling edge", "f")
        self.trigger_edge_combo.setToolTip(
            "Select which H2 edge the FPGA uses as MCS time zero. "
            "This does not change the Teensy H2/H4 waveforms."
        )

        # Normal MCS deterministic max-pass control. The FPGA enforces this in
        # the acquisition clock domain; physical trigger/event sources continue.
        self.max_passes_label = QLabel("Max passes")
        self.max_passes_spin = QSpinBox()
        self.max_passes_spin.setRange(1, 1_000_000_000)
        self.max_passes_spin.setGroupSeparatorShown(True)
        self.max_passes_spin.setValue(5_000)
        self.max_passes_spin.setToolTip(
            "Maximum passes accepted by the FPGA after each Start"
        )
        self.unlimited_passes_check = QCheckBox("Continuous mode (no pass limit)")
        self.unlimited_passes_check.setChecked(False)
        self.unlimited_passes_check.setToolTip(
            "Disable the max-pass limit. Teensy command m 0 selects continuous acquisition."
        )

        self.test_passes_label = QLabel("Passes")
        self.test_passes_spin = QSpinBox()
        self.test_passes_spin.setRange(1, 1_000_000_000)
        self.test_passes_spin.setGroupSeparatorShown(True)
        self.test_passes_spin.setValue(2_000)
        self.test_passes_spin.setToolTip(
            "FPGA passes accepted per Start; the self-test waveform continues afterward"
        )

        self.self_test_check = QCheckBox("Test mode / Self test")
        self.self_test_check.setToolTip(
            "Continuously generate the Teensy hardware self-test signals. This state persists across MCS/Counter mode changes."
        )
        self.counter_self_test_check = QCheckBox("Test mode / Self test")
        self.counter_self_test_check.setToolTip(
            "Same persistent Teensy self-test switch as MCS. Turning either switch on/off updates the other."
        )
        self.internal_signal_check = QCheckBox("Internal signal")
        self.internal_signal_check.setToolTip(
            "Continuously generate the signal on Teensy pin 9"
        )

        self.counter_dwell_label = QLabel("Counter dwell")
        self.counter_dwell_spin = QDoubleSpinBox()
        self.counter_dwell_spin.setRange(0.001, 50.0)
        self.counter_dwell_spin.setDecimals(3)
        self.counter_dwell_spin.setSingleStep(0.1)
        self.counter_dwell_spin.setSuffix(" s")
        self.counter_dwell_spin.setValue(0.100)
        self.counter_dwell_spin.setToolTip(
            "Displayed Counter dwell, up to 50 seconds. Long dwells are "
            "automatically combined from safe 32-bit FPGA dwell segments."
        )
        self.counter_hint = QLabel(
            "Both inputs are counted simultaneously during the same dwell:\n"
            "Signal input = FPGA H4  |  Trigger input = FPGA H2"
        )
        self.counter_hint.setObjectName("HintLabel")
        self.counter_hint.setWordWrap(True)

        self.quantized_label = QLabel()
        self.quantized_label.setObjectName("HintLabel")
        self.quantized_label.setWordWrap(True)

        self.configure_button = QPushButton("Apply configuration")
        self.configure_button.setObjectName("PrimaryButton")
        self.run_button = QPushButton("Run internal")
        self.run_button.setObjectName("RunButton")
        self.external_button = QPushButton("Run external")
        self.stop_button = QPushButton("Stop")
        self.single_button = QPushButton("Single trigger")
        self.reset_button = QPushButton("Clear accumulation")

        self.bins_label = QLabel("Bins")
        self.dwell_label = QLabel("Dwell")
        self.trigger_label = QLabel("Trigger")
        self.mode_description = QLabel("Triggered MCS")
        self.mode_description.setObjectName("HintLabel")
        self.mode_description.setWordWrap(True)

        setup_form = QFormLayout()
        setup_form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        setup_form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        setup_form.setHorizontalSpacing(10)
        setup_form.setVerticalSpacing(6)
        setup_form.addRow(self.bins_label, self.bins_spin)

        dwell_row = QWidget()
        dwell_box = QHBoxLayout(dwell_row)
        dwell_box.setContentsMargins(0, 0, 0, 0)
        dwell_box.setSpacing(5)
        dwell_box.addWidget(self.dwell_spin, 1)
        dwell_box.addWidget(self.dwell_unit)
        setup_form.addRow(self.dwell_label, dwell_row)
        setup_form.addRow(self.trigger_label, self.trigger_spin)
        setup_form.addRow("Trigger edge", self.trigger_edge_combo)
        setup_form.addRow(self.max_passes_label, self.max_passes_spin)
        setup_form.addRow("", self.unlimited_passes_check)
        setup_form.addRow(self.test_passes_label, self.test_passes_spin)
        setup_form.addRow(self.counter_dwell_label, self.counter_dwell_spin)
        setup.addLayout(setup_form)

        setup.addWidget(self.internal_signal_check)
        setup.addWidget(self.self_test_check)
        setup.addWidget(self.counter_self_test_check)
        setup.addWidget(self.counter_hint)
        setup.addWidget(self.quantized_label)
        setup.addWidget(self.mode_description)
        setup.addWidget(self.configure_button)

        self.calibration_group = QGroupBox("Clock-frequency calibration")
        cal = QGridLayout(self.calibration_group)
        cal_help = QLabel(
            "Enter the measured FPGA acquisition-clock frequency. This value "
            "is applied to MCS time bins and both Counter CPS channels."
        )
        cal_help.setWordWrap(True)
        self.clock_frequency_spin = QDoubleSpinBox()
        self.clock_frequency_spin.setRange(1.0, 1000.0)
        self.clock_frequency_spin.setDecimals(6)
        self.clock_frequency_spin.setSuffix(" MHz")
        self.clock_frequency_spin.setValue(self.clock_frequency_hz / 1e6)
        self.clock_frequency_apply_button = QPushButton("Apply clock frequency")
        self.clock_frequency_reset_button = QPushButton("Reset to 100 MHz")
        cal.addWidget(cal_help, 0, 0, 1, 2)
        cal.addWidget(QLabel("Measured clock"), 1, 0)
        cal.addWidget(self.clock_frequency_spin, 1, 1)
        cal.addWidget(self.clock_frequency_apply_button, 2, 0)
        cal.addWidget(self.clock_frequency_reset_button, 2, 1)
        setup.addWidget(self.calibration_group)

        run_row = QGridLayout()
        run_row.setHorizontalSpacing(6)
        run_row.setVerticalSpacing(6)
        run_row.addWidget(self.run_button, 0, 0)
        run_row.addWidget(self.external_button, 0, 1)
        run_row.addWidget(self.single_button, 1, 0)
        run_row.addWidget(self.stop_button, 1, 1)
        run_row.addWidget(self.reset_button, 2, 0, 1, 2)
        setup.addLayout(run_row)
        acquisition.setVisible(False)

        plot_frame = QFrame()
        self.plot_frame = plot_frame
        plot_frame.setObjectName("PlotPanel")
        plot_layout = QVBoxLayout(plot_frame)
        plot_layout.setContentsMargins(8, 8, 8, 8)
        title_toolbar = QHBoxLayout()
        plot_toolbar = QHBoxLayout()
        self.title_toolbar = title_toolbar
        self.plot_toolbar = plot_toolbar
        self.quick_run_button = QPushButton("▶ Start")
        self.quick_run_button.setObjectName("RunButton")
        self.quick_run_button.setToolTip("Start acquisition (F5)")
        self.quick_stop_button = QPushButton("■ Stop")
        self.quick_stop_button.setToolTip("Stop acquisition (Shift+F5 or Esc)")
        self.quick_clear_button = QPushButton("Clear")
        self.quick_clear_button.setToolTip("Clear FPGA accumulation (Ctrl+L)")
        self.plot_title = QLabel("Histogram — waiting for data")
        self.plot_title.setObjectName("SectionTitle")
        self.auto_y_check = QPushButton("Fit Y")
        self.auto_y_check.setCheckable(True)
        self.auto_y_check.setChecked(False)
        self.auto_y_check.setToolTip("Toggle live Y fitting. When enabled, Y expands with accumulating data; click again to switch it off.")
        self.log_y_check = QCheckBox("Log Y")
        self.plot_rate_check = QCheckBox("Plot count rate")
        self.plot_rate_check.setChecked(True)
        self.clear_rate_button = QPushButton("Clear CPS")
        self.clear_rate_button.setToolTip("Clear only MCS CPS history; preserve histogram")
        self.counter_signal_check = QCheckBox("H4 Signal (L)")
        self.counter_signal_check.setChecked(True)
        self.counter_signal_check.setToolTip("Show/hide Signal CPS on the left Y axis")
        self.counter_signal_color_button = QPushButton("●")
        self.counter_signal_color_button.setFixedWidth(30)
        self.counter_signal_color_button.setToolTip("Choose H4 Signal trace color")
        self.counter_trigger_check = QCheckBox("H2 Trigger (R)")
        self.counter_trigger_check.setChecked(True)
        self.counter_trigger_check.setToolTip("Show/hide Trigger CPS on the right Y axis")
        self.counter_trigger_color_button = QPushButton("●")
        self.counter_trigger_color_button.setFixedWidth(30)
        self.counter_trigger_color_button.setToolTip("Choose H2 Trigger trace color")
        self.counter_signal_check.setVisible(False)
        self.counter_signal_color_button.setVisible(False)
        self.counter_trigger_check.setVisible(False)
        self.counter_trigger_color_button.setVisible(False)
        self.plot_rate_check.setToolTip(
            "Show/hide detector CPS derived from FPGA histogram increments and active acquisition time"
        )
        self.roi_enable_check = QPushButton("ROI: OFF")
        self.roi_enable_check.setObjectName("ROIButton")
        self.roi_enable_check.setCheckable(True)
        self.roi_enable_check.setChecked(False)
        self.roi_enable_check.setToolTip(
            "Show signal/noise regions and live ROI calculations"
        )
        self.reference_button = QPushButton("Freeze reference")
        self.reference_button.setToolTip(
            "Freeze the currently displayed MCS histogram as a transparent comparison overlay"
        )
        self.clear_reference_button = QPushButton("Remove reference")
        self.clear_reference_button.setToolTip(
            "Remove the frozen comparison histogram"
        )
        self.clear_reference_button.setEnabled(False)

        self.difference_button = QPushButton("Difference: OFF")
        self.difference_button.setObjectName("ROIButton")
        self.difference_button.setCheckable(True)
        self.difference_button.setChecked(False)
        self.difference_button.setToolTip(
            "Show the background-subtracted signal ROI beside the raw MCS"
        )
        self.save_csv_button = QPushButton("Export CSV")
        self.save_plot_button = QPushButton("Export image")
        self.trace_color_button = QPushButton("Trace color")
        self.trace_color_button.setToolTip("Choose the MCS/counter trace color")
        title_toolbar.addWidget(self.plot_title, 1)
        title_toolbar.addWidget(self.roi_enable_check)
        title_toolbar.addWidget(self.difference_button)
        title_toolbar.addWidget(self.reference_button)
        title_toolbar.addWidget(self.clear_reference_button)
        plot_toolbar.addWidget(self.quick_run_button)
        plot_toolbar.addWidget(self.quick_stop_button)
        plot_toolbar.addWidget(self.quick_clear_button)
        plot_toolbar.addStretch(1)
        plot_toolbar.addWidget(self.auto_y_check)
        plot_toolbar.addWidget(self.log_y_check)
        plot_toolbar.addWidget(self.counter_signal_check)
        plot_toolbar.addWidget(self.counter_signal_color_button)
        plot_toolbar.addWidget(self.counter_trigger_check)
        plot_toolbar.addWidget(self.counter_trigger_color_button)
        plot_toolbar.addWidget(self.plot_rate_check)
        plot_toolbar.addWidget(self.clear_rate_button)
        plot_toolbar.addWidget(self.trace_color_button)
        plot_toolbar.addWidget(self.save_csv_button)
        plot_toolbar.addWidget(self.save_plot_button)
        plot_layout.addLayout(title_toolbar)
        plot_layout.addLayout(plot_toolbar)
        self.selection_view_box = SelectionViewBox()
        self.plot = pg.PlotWidget(viewBox=self.selection_view_box)
        self.plot.setBackground(BG)
        self.plot.showGrid(x=True, y=True, alpha=0.18)
        self.plot.setLabel("bottom", "Time after trigger", units="s")
        self.plot.getAxis("bottom").enableAutoSIPrefix(True)
        self.plot.setLabel("left", "Counts")
        self.plot.setMenuEnabled(True)
        self.plot.enableAutoRange(x=False, y=False)
        self.plot.setClipToView(True)
        self.plot.setDownsampling(auto=False)
        # Histogram counts cannot be negative. Prevent manual pan/zoom or
        # keyboard zoom from moving the MCS Y axis below zero.
        self.plot.setLimits(yMin=0.0)
        self.curve = self.plot.plot(
            pen=pg.mkPen(self.trace_color, width=1.25), connect="finite"
        )
        # Fixed full-spectrum overview. Main-view zoom never changes this scale.
        self.overview = FloatingOverview(self)
        self.overview.raise_()
        self.overview.show()
        self.overview_curve = pg.PlotCurveItem(pen=pg.mkPen(MUTED, width=1.0), connect="finite")
        self.overview.addItem(self.overview_curve)
        self.overview_zoom = pg.LinearRegionItem(values=(0, 1), movable=False,
            brush=pg.mkBrush(54, 197, 240, 55), pen=pg.mkPen(CYAN, width=1.2))
        self.overview.addItem(self.overview_zoom)
        self.overview.navigateRequested.connect(self._navigate_from_overview)
        self.plot.getViewBox().sigRangeChanged.connect(self._update_overview_zoom)
        QTimer.singleShot(0, self._position_overview)
        self.counter_trigger_view = pg.ViewBox()
        self.plot.scene().addItem(self.counter_trigger_view)
        self.counter_trigger_axis = self.plot.getAxis("right")
        self.counter_trigger_axis.linkToView(self.counter_trigger_view)
        # Reserve enough room for the independent trigger scale.  Relying on
        # AxisItem's automatic width can collapse this axis to almost nothing
        # after it has previously been hidden in MCS mode.
        self.counter_trigger_axis.setWidth(92)
        self.counter_trigger_axis.setStyle(
            showValues=True, tickLength=-5, autoExpandTextSpace=False,
            tickTextWidth=58,
        )
        self.counter_trigger_axis.setPen(pg.mkPen(self.counter_trigger_color))
        self.counter_trigger_axis.setTextPen(pg.mkPen(self.counter_trigger_color))
        self.counter_trigger_view.setXLink(self.plot)
        self.counter_trigger_view.setLimits(yMin=0.0)
        self.counter_trigger_curve = pg.PlotCurveItem(
            pen=pg.mkPen(self.counter_trigger_color, width=1.25), connect="finite",
            name="Trigger (H2)",
        )
        self.counter_trigger_view.addItem(self.counter_trigger_curve)
        self.plot.showAxis("right", show=False)
        self.counter_trigger_axis.setLabel(
            "Trigger input (H2)", units="cps", color=self.counter_trigger_color
        )
        self.counter_trigger_view.setVisible(False)
        self.counter_trigger_curve.setVisible(False)
        self.plot.getViewBox().sigResized.connect(
            self._sync_counter_trigger_view
        )
        self.signal_region = pg.LinearRegionItem(
            values=(10, 30), brush=pg.mkBrush(61, 220, 151, 42),
            pen=pg.mkPen(GREEN, width=1.5), movable=True,
        )
        self.noise_region = pg.LinearRegionItem(
            values=(70, 90), brush=pg.mkBrush(255, 180, 84, 38),
            pen=pg.mkPen(AMBER, width=1.5), movable=True,
        )
        self.signal_region.setZValue(5)
        self.noise_region.setZValue(4)
        self.plot.addItem(self.signal_region)
        self.plot.addItem(self.noise_region)
        self.highlight_region = pg.LinearRegionItem(
            values=(0, 1),
            brush=pg.mkBrush(54, 197, 240, 48),
            pen=pg.mkPen(CYAN, width=1.8),
            movable=False,
        )
        self.highlight_region.setZValue(6)
        self.highlight_region.setVisible(False)
        self.plot.addItem(self.highlight_region)
        self.difference_plot = pg.PlotWidget()
        self.difference_plot.setBackground(BG)
        self.difference_plot.showGrid(x=True, y=True, alpha=0.18)
        self.difference_plot.setLabel("bottom", "Time after trigger", units="s")
        self.difference_plot.getAxis("bottom").enableAutoSIPrefix(True)
        self.difference_plot.setLabel("left", "Net count rate", units="cps")
        self.difference_plot.setTitle("Signal − average noise")
        self.difference_plot.setMenuEnabled(True)
        self.difference_curve = self.difference_plot.plot(
            pen=pg.mkPen(GREEN, width=1.5), connect="finite"
        )
        self.plot_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.plot_splitter.setChildrenCollapsible(True)
        self.plot_splitter.setHandleWidth(7)
        self.plot_splitter.addWidget(self.plot)
        self.plot_splitter.addWidget(self.difference_plot)
        self.difference_plot.setVisible(False)
        # Secondary analysis plots may follow their own data. The large MCS
        # histogram is deliberately handled separately and is fitted only once.
        self.difference_plot.enableAutoRange(x=True, y=True)
        self.rate_plot = pg.PlotWidget()
        self.rate_plot.setBackground(BG)
        self.rate_plot.showGrid(x=True, y=True, alpha=0.18)
        self.rate_plot.setLabel("bottom", "Elapsed acquisition time", units="s")
        self.rate_plot.setLabel("left", "Robust active-window rate", units="cps")
        self.rate_plot.setMenuEnabled(True)
        self.rate_curve = self.rate_plot.plot(
            pen=pg.mkPen(GREEN, width=1.5), connect="finite"
        )
        self.rate_plot.setMinimumHeight(80)
        self.rate_plot.setVisible(False)
        self.rate_plot.enableAutoRange(x=True, y=True)
        self.vertical_plot_splitter = QSplitter(Qt.Orientation.Vertical)
        self.vertical_plot_splitter.setChildrenCollapsible(True)
        self.vertical_plot_splitter.setHandleWidth(7)
        self.vertical_plot_splitter.addWidget(self.plot_splitter)
        self.vertical_plot_splitter.addWidget(self.rate_plot)
        self.vertical_plot_splitter.setStretchFactor(0, 4)
        self.vertical_plot_splitter.setStretchFactor(1, 1)
        self.vertical_plot_splitter.setSizes([620, 180])
        plot_layout.addWidget(self.vertical_plot_splitter, 1)
        left_workspace_layout.addWidget(plot_frame, 1)

        self.side_tabs = QTabWidget()
        analysis_tab = QWidget()
        side_layout = QVBoxLayout(analysis_tab)
        side_layout.setContentsMargins(4, 0, 4, 0)
        side_layout.setSpacing(10)
        self.roi_group = QGroupBox("Regions of interest")
        roi_layout = QGridLayout(self.roi_group)
        self.signal_start = self._region_spin(10)
        self.signal_end = self._region_spin(29)
        self.noise_start = self._region_spin(70)
        self.noise_end = self._region_spin(89)
        signal_swatch = QLabel("● Signal")
        signal_swatch.setStyleSheet(f"color:{GREEN}; font-weight:600")
        noise_swatch = QLabel("● Noise / background")
        noise_swatch.setStyleSheet(f"color:{AMBER}; font-weight:600")
        roi_layout.addWidget(signal_swatch, 0, 0, 1, 2)
        roi_layout.addWidget(QLabel("First bin"), 1, 0)
        roi_layout.addWidget(QLabel("Last bin"), 1, 1)
        roi_layout.addWidget(self.signal_start, 2, 0)
        roi_layout.addWidget(self.signal_end, 2, 1)
        roi_layout.addWidget(noise_swatch, 3, 0, 1, 2)
        roi_layout.addWidget(QLabel("First bin"), 4, 0)
        roi_layout.addWidget(QLabel("Last bin"), 4, 1)
        roi_layout.addWidget(self.noise_start, 5, 0)
        roi_layout.addWidget(self.noise_end, 5, 1)
        self.reset_regions_button = QPushButton("Reset regions")
        roi_layout.addWidget(self.reset_regions_button, 6, 0, 1, 2)
        help_label = QLabel(
            "Drag either shaded region on the plot, or enter inclusive bin limits. "
            "Noise is scaled by region width before background subtraction."
        )
        help_label.setObjectName("HintLabel")
        help_label.setWordWrap(True)
        roi_layout.addWidget(help_label, 7, 0, 1, 2)
        side_layout.addWidget(self.roi_group)

        self.analysis_group = QGroupBox("Live ROI analysis")
        analysis_grid = QGridLayout(self.analysis_group)
        self.metric_signal = MetricValue("Gross signal")
        self.metric_background = MetricValue("Estimated background")
        self.metric_net = MetricValue("Net signal rate")
        self.metric_snr = MetricValue("SNR")
        self.metric_noise = MetricValue("Noise / bin")
        self.metric_rate = MetricValue("Net integrated counts")
        analysis_grid.addWidget(self.metric_signal, 0, 0)
        analysis_grid.addWidget(self.metric_background, 0, 1)
        analysis_grid.addWidget(self.metric_net, 1, 0)
        analysis_grid.addWidget(self.metric_snr, 1, 1)
        analysis_grid.addWidget(self.metric_noise, 2, 0)
        analysis_grid.addWidget(self.metric_rate, 2, 1)
        self.roi_detail = QLabel("No histogram available")
        self.roi_detail.setObjectName("HintLabel")
        self.roi_detail.setWordWrap(True)
        analysis_grid.addWidget(self.roi_detail, 3, 0, 1, 2)
        side_layout.addWidget(self.analysis_group)

        frame_group = QGroupBox("Acquisition status")
        self.frame_group = frame_group
        frame_form = QFormLayout(frame_group)
        frame_form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        frame_form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        frame_form.setHorizontalSpacing(12)
        frame_form.setVerticalSpacing(5)
        self.frame_passes = QLabel("—")
        self.frame_bins = QLabel("—")
        self.frame_dwell = QLabel("—")
        self.frame_span = QLabel("—")
        self.frame_total = QLabel("—")
        self.frame_rate = QLabel("—")
        self.frame_wire_limit = QLabel("—")
        self.frame_integrity = QLabel("—")
        self.frame_integrity.setWordWrap(True)
        frame_form.addRow("FPGA passes", self.frame_passes)
        frame_form.addRow("Bins", self.frame_bins)
        frame_form.addRow("Dwell", self.frame_dwell)
        frame_form.addRow("Acquisition span", self.frame_span)
        frame_form.addRow("Displayed counts", self.frame_total)
        frame_form.addRow("Frame rate", self.frame_rate)
        frame_form.addRow("5 Mbaud limit", self.frame_wire_limit)
        frame_form.addRow("Last frame", self.frame_integrity)
        self.frame_bytes = QLabel("—")
        frame_form.addRow("Frame size", self.frame_bytes)
        self.acquisition_elapsed = QLabel("—")
        self.acquisition_pass_rate = QLabel("—")
        self.acquisition_count_rate = QLabel("—")
        self.acquisition_time_left = QLabel("—")
        self.acquisition_finish_time = QLabel("—")
        frame_form.addRow("Acquisition time", self.acquisition_elapsed)
        frame_form.addRow("Pass rate", self.acquisition_pass_rate)
        frame_form.addRow("Active-window count rate", self.acquisition_count_rate)
        frame_form.addRow("Time remaining", self.acquisition_time_left)
        frame_form.addRow("Estimated completion", self.acquisition_finish_time)

        # Stable keys used by Data Settings. The label text may be customized
        # without changing the underlying measurement source.
        self.status_fields = {
            "fpga_passes": ("FPGA passes", self.frame_passes),
            "bins": ("Bins", self.frame_bins),
            "dwell": ("Dwell", self.frame_dwell),
            "span": ("Acquisition span", self.frame_span),
            "displayed_counts": ("Displayed counts", self.frame_total),
            "frame_rate": ("Frame rate", self.frame_rate),
            "wire_limit": ("5 Mbaud limit", self.frame_wire_limit),
            "integrity": ("Last frame", self.frame_integrity),
            "frame_size": ("Frame size", self.frame_bytes),
            "acq_time": ("Acquisition time", self.acquisition_elapsed),
            "pass_rate": ("Pass rate", self.acquisition_pass_rate),
            "count_rate": ("Active-window count rate", self.acquisition_count_rate),
            "time_remaining": ("Time remaining", self.acquisition_time_left),
            "completion": ("Estimated completion", self.acquisition_finish_time),
        }
        for _label, value_widget in self.status_fields.values():
            value_widget.setWordWrap(True)

        self.counter_status_group = QGroupBox("Simple counter statistics")
        counter_status_form = QFormLayout(self.counter_status_group)
        self.counter_stat_current = QLabel("—")
        self.counter_stat_trigger_current = QLabel("—")
        self.counter_stat_mean = QLabel("—")
        self.counter_stat_std = QLabel("—")
        self.counter_stat_rsd = QLabel("—")
        self.counter_stat_min = QLabel("—")
        self.counter_stat_max = QLabel("—")
        self.counter_stat_elapsed = QLabel("—")
        self.counter_stat_samples = QLabel("0")
        counter_status_form.addRow("Signal CPS (H4)", self.counter_stat_current)
        counter_status_form.addRow("Trigger CPS (H2)", self.counter_stat_trigger_current)
        counter_status_form.addRow("Mean CPS", self.counter_stat_mean)
        counter_status_form.addRow("Std. deviation", self.counter_stat_std)
        counter_status_form.addRow("Relative std. dev.", self.counter_stat_rsd)
        counter_status_form.addRow("Minimum CPS", self.counter_stat_min)
        counter_status_form.addRow("Maximum CPS", self.counter_stat_max)
        counter_status_form.addRow("Elapsed time", self.counter_stat_elapsed)
        counter_status_form.addRow("Saved samples", self.counter_stat_samples)
        self.counter_status_group.setVisible(False)
        status_tab = QWidget()
        status_layout = QVBoxLayout(status_tab)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(10)
        status_layout.addWidget(frame_group)
        status_layout.addWidget(self.counter_status_group)
        status_layout.addStretch(1)
        side_layout.addStretch(1)
        self.side_tabs.addTab(status_tab, "Status")
        self.side_tabs.addTab(analysis_tab, "ROI analysis")
        # Wide enough that status labels never truncate; no fixed maximum so
        # the user can drag the splitter wider on large displays.
        self.side_tabs.setMinimumWidth(360)
        self.workspace_splitter.addWidget(self.side_tabs)
        self.workspace_splitter.setStretchFactor(0, 1)
        self.workspace_splitter.setStretchFactor(1, 0)
        self.workspace_splitter.setSizes([1120, 400])

        self.selection_panel = QFrame()
        self.selection_panel.setObjectName("SelectionPanel")
        selection_layout = QHBoxLayout(self.selection_panel)
        selection_layout.setContentsMargins(10, 5, 10, 5)
        selection_caption = QLabel("Highlighted section")
        selection_caption.setObjectName("SelectionCaption")
        self.selection_info = QLabel(
            "Hold Ctrl and left-click a bin, or Ctrl+drag across a range"
        )
        self.selection_info.setObjectName("SelectionInfo")
        self.selection_info.setWordWrap(True)
        self.selection_info.setMinimumHeight(42)
        self.selection_info.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        selection_layout.addWidget(selection_caption)
        selection_layout.addWidget(self.selection_info, 1)
        plot_index = left_workspace_layout.indexOf(plot_frame)
        left_workspace_layout.insertWidget(max(0, plot_index), self.selection_panel)

        status = QStatusBar()
        self.setStatusBar(status)
        self.status_message = QLabel("Ready — connect an instrument")
        self.x_axis_label = QLabel("X axis")
        self.x_axis_label.setObjectName("HintLabel")
        self.x_axis_combo = QComboBox()
        self.x_axis_combo.addItem("Time", "time")
        self.x_axis_combo.addItem("Bin", "bin")
        self.x_axis_combo.setToolTip("Display the MCS horizontal axis as physical time or bin number")
        self.status_clock = QLabel("100 MHz · 10.000 ns/clock")
        self.status_clock.setObjectName("HintLabel")
        self.status_timing = QLabel("Acquisition — · ETA —")
        self.status_timing.setObjectName("HintLabel")
        self.status_frame_kind = QLabel("NO DATA")
        self.status_frame_kind.setObjectName("HintLabel")
        status.addWidget(self.status_message, 1)
        status.addPermanentWidget(self.status_frame_kind)
        status.addPermanentWidget(self.status_timing)
        status.addPermanentWidget(self.x_axis_label)
        status.addPermanentWidget(self.x_axis_combo)
        status.addPermanentWidget(self.status_clock)
        # PlotWidget is a QGraphicsView and normally consumes wheel events for
        # zooming. In this scrollable interface, the wheel always belongs to
        # the page. Dragging and the explicit zoom shortcuts still operate on
        # the plots.
        self._page_scroll_wheel_targets = {
            self.plot.viewport(),
            self.difference_plot.viewport(),
            self.rate_plot.viewport(),
        }
        for target in self._page_scroll_wheel_targets:
            target.installEventFilter(self)
        self._initialize_layout_editor_model()
        self._update_quantization()

    def _region_spin(self, value):
        spin = QSpinBox()
        spin.setRange(0, MAX_BINS - 1)
        spin.setGroupSeparatorShown(True)
        spin.setValue(value)
        return spin

    def _connect_signals(self):
        self.setup_button.clicked.connect(self._show_acquisition_popup)
        self.layout_button.clicked.connect(self._show_layout_editor)
        self.shortcuts_button.toggled.connect(self._toggle_shortcuts_window)
        self.theme_combo.currentIndexChanged.connect(self._apply_theme)
        self.help_button.clicked.connect(self.show_help)
        self.quick_run_button.clicked.connect(self.start_run)
        self.quick_stop_button.clicked.connect(self.stop_run)
        self.quick_clear_button.clicked.connect(self.reset_histogram)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.configure_button.clicked.connect(self.configure_acquisition)
        self.trigger_edge_combo.currentIndexChanged.connect(self._apply_trigger_edge)
        self.run_button.clicked.connect(self.start_run)
        self.external_button.clicked.connect(self.start_external)
        self.stop_button.clicked.connect(self.stop_run)
        self.single_button.clicked.connect(self.single_trigger)
        self.reset_button.clicked.connect(self.reset_histogram)
        self.save_csv_button.clicked.connect(self.save_csv)
        self.save_plot_button.clicked.connect(self.save_plot)
        self.browse_save_directory_button.clicked.connect(self._browse_save_directory)
        self.data_settings_button.clicked.connect(self._show_data_settings)
        self.save_data_button.clicked.connect(self.save_data)
        self.trace_color_button.clicked.connect(self.choose_trace_color)
        self.clock_frequency_apply_button.clicked.connect(
            self._apply_clock_frequency
        )
        self.clock_frequency_reset_button.clicked.connect(
            self._reset_clock_frequency
        )
        self.counter_signal_color_button.clicked.connect(
            self.choose_counter_signal_color
        )
        self.counter_trigger_color_button.clicked.connect(
            self.choose_counter_trigger_color
        )
        self.mcs_mode_button.clicked.connect(lambda: self._set_mode("mcs"))
        self.counter_mode_button.clicked.connect(lambda: self._set_mode("counter"))
        self.self_test_check.toggled.connect(self._self_test_setting_changed)
        self.counter_self_test_check.toggled.connect(self._counter_self_test_setting_changed)
        self.unlimited_passes_check.toggled.connect(
            lambda enabled: self.max_passes_spin.setEnabled(not enabled)
        )
        self.internal_signal_check.toggled.connect(self._internal_signal_changed)
        self.x_axis_combo.currentIndexChanged.connect(self._x_axis_changed)
        self.selection_view_box.selectionDragged.connect(
            self._highlight_plot_section
        )
        self.plot.scene().sigMouseClicked.connect(self._plot_mouse_clicked)
        self.dwell_spin.valueChanged.connect(self._update_quantization)
        self.dwell_unit.currentIndexChanged.connect(self._mcs_limits_changed)
        self.bins_spin.valueChanged.connect(self._mcs_limits_changed)
        self.auto_y_check.toggled.connect(self._set_live_fit_y)
        self.log_y_check.toggled.connect(self._apply_display_options)
        self.plot_rate_check.toggled.connect(self._set_rate_plot_enabled)
        self.clear_rate_button.clicked.connect(self._clear_mcs_rate_history)
        self.counter_signal_check.toggled.connect(
            self._update_counter_trace_visibility
        )
        self.counter_trigger_check.toggled.connect(
            self._update_counter_trace_visibility
        )
        self.roi_enable_check.toggled.connect(self._set_roi_enabled)
        self.difference_button.toggled.connect(self._set_difference_enabled)
        self.reference_button.clicked.connect(self._freeze_reference_histogram)
        self.clear_reference_button.clicked.connect(self._clear_reference_histogram)
        self.signal_region.sigRegionChangeFinished.connect(lambda: self._region_dragged("signal"))
        self.noise_region.sigRegionChangeFinished.connect(lambda: self._region_dragged("noise"))
        for spin in (self.signal_start, self.signal_end, self.noise_start, self.noise_end):
            spin.valueChanged.connect(self._spins_changed)
        self.reset_regions_button.clicked.connect(self.reset_regions)

    def _restore_settings(self):
        self.auto_connect_check.setChecked(self.settings.value("connection/auto", True, type=bool))
        self._load_data_settings()
        default_save_dir = str(Path.home() / "Documents" / "OpenMCS Data")
        self.save_directory_edit.setText(
            self.settings.value("data/save_directory", default_save_dir, str)
        )
        self.run_name_edit.setText(
            self.settings.value("data/run_name", "sample_001", str)
        )
        self.save_image_check.setChecked(
            self.settings.value("data/save_image", True, type=bool)
        )
        self.bins_spin.setValue(
            self.settings.value("acquisition/bins", DEFAULT_BINS, int)
        )
        self.dwell_spin.setValue(self.settings.value("acquisition/dwell_value", 100.0, float))
        self.dwell_unit.setCurrentIndex(self.settings.value("acquisition/dwell_unit", 0, int))
        self.trigger_spin.setValue(self.settings.value("acquisition/trigger_hz", 100, int))
        saved_edge = self.settings.value("acquisition/trigger_edge", "r", str)
        edge_index = self.trigger_edge_combo.findData(saved_edge)
        self.trigger_edge_combo.setCurrentIndex(max(0, edge_index))
        self.max_passes_spin.setValue(
            self.settings.value("acquisition/max_passes", 5_000, int)
        )
        self.unlimited_passes_check.setChecked(
            self.settings.value("acquisition/unlimited_passes", False, type=bool)
        )
        self.max_passes_spin.setEnabled(not self.unlimited_passes_check.isChecked())
        self.test_passes_spin.setValue(
            self.settings.value("acquisition/self_test_passes", 5_000, int)
        )
        saved_mode = self.settings.value("acquisition/mode", "mcs", str)
        self.signal_start.setValue(self.settings.value("roi/signal_start", 10, int))
        self.signal_end.setValue(self.settings.value("roi/signal_end", 29, int))
        self.noise_start.setValue(self.settings.value("roi/noise_start", 70, int))
        self.noise_end.setValue(self.settings.value("roi/noise_end", 89, int))
        self.roi_enable_check.setChecked(
            self.settings.value("roi/visible_v2", False, type=bool)
        )
        self.counter_dwell_spin.setValue(
            self.settings.value("counter/dwell_seconds", 0.100, float)
        )
        theme = self.settings.value("appearance/theme", "dark", str)
        self.theme_combo.setCurrentIndex(max(0, self.theme_combo.findData(theme)))
        self._set_trace_color(
            self.settings.value("appearance/trace_color", CYAN, str)
        )
        self._set_counter_signal_color(
            self.settings.value("appearance/counter_signal_color", CYAN, str)
        )
        self._set_counter_trigger_color(
            self.settings.value("appearance/counter_trigger_color", AMBER, str)
        )
        self.counter_signal_check.setChecked(
            self.settings.value("appearance/show_counter_signal", True, type=bool)
        )
        self.counter_trigger_check.setChecked(
            self.settings.value("appearance/show_counter_trigger", True, type=bool)
        )
        self.plot_rate_check.setChecked(
            self.settings.value("appearance/plot_count_rate", True, type=bool)
        )
        self._set_roi_enabled(self.roi_enable_check.isChecked())
        self._spins_changed()
        legacy_self_test = saved_mode == "self_test"
        self._set_mode(saved_mode if saved_mode in ("mcs", "counter") else "mcs")
        self.self_test_check.setChecked(
            legacy_self_test or self.settings.value(
                "acquisition/self_test", False, type=bool
            )
        )
        with QSignalBlocker(self.counter_self_test_check):
            self.counter_self_test_check.setChecked(self.self_test_check.isChecked())
        self.internal_signal_check.setChecked(
            self.settings.value("acquisition/internal_signal", False, type=bool)
        )
        self.clock_frequency_hz = self.settings.value(
            "calibration/clock_hz", float(EXPECTED_CLOCK_HZ), float
        )
        self.clock_frequency_spin.setValue(self.clock_frequency_hz / 1e6)
        saved_x_axis = self.settings.value("appearance/x_axis", "time", str)
        x_axis_index = self.x_axis_combo.findData(saved_x_axis)
        self.x_axis_combo.setCurrentIndex(max(0, x_axis_index))
        self._mcs_limits_changed()
        self._update_clock_status()
        workspace_state = self.settings.value("layout/workspace_splitter")
        vertical_state = self.settings.value("layout/vertical_plot_splitter")
        plot_state = self.settings.value("layout/plot_splitter")
        if workspace_state is not None:
            self.workspace_splitter.restoreState(workspace_state)
        if vertical_state is not None:
            self.vertical_plot_splitter.restoreState(vertical_state)
        if plot_state is not None:
            self.plot_splitter.restoreState(plot_state)
        self._restore_layout_customization()

    def _save_settings(self):
        values = {
            "connection/auto": self.auto_connect_check.isChecked(),
            "connection/port": self.port_combo.currentData() or "",
            "data/save_directory": self.save_directory_edit.text().strip(),
            "data/run_name": self.run_name_edit.text().strip(),
            "data/save_image": self.save_image_check.isChecked(),
            "acquisition/bins": self.bins_spin.value(),
            "acquisition/dwell_value": self.dwell_spin.value(),
            "acquisition/dwell_unit": self.dwell_unit.currentIndex(),
            "acquisition/trigger_hz": self.trigger_spin.value(),
            "acquisition/trigger_edge": self.trigger_edge_combo.currentData() or "r",
            "acquisition/max_passes": self.max_passes_spin.value(),
            "acquisition/unlimited_passes": self.unlimited_passes_check.isChecked(),
            "acquisition/self_test_passes": self.test_passes_spin.value(),
            "acquisition/mode": self.instrument_mode,
            "acquisition/self_test": self.self_test_check.isChecked(),
            "acquisition/internal_signal": self.internal_signal_check.isChecked(),
            "counter/dwell_seconds": self.counter_dwell_spin.value(),
            "appearance/theme": self.theme_combo.currentData(),
            "appearance/trace_color": self.trace_color,
            "appearance/counter_signal_color": self.counter_signal_color,
            "appearance/counter_trigger_color": self.counter_trigger_color,
            "appearance/show_counter_signal": self.counter_signal_check.isChecked(),
            "appearance/show_counter_trigger": self.counter_trigger_check.isChecked(),
            "appearance/plot_count_rate": self.plot_rate_check.isChecked(),
            "appearance/x_axis": self.x_axis_combo.currentData() or "time",
            "calibration/clock_hz": self.clock_frequency_hz,
            "roi/signal_start": self.signal_start.value(),
            "roi/signal_end": self.signal_end.value(),
            "roi/noise_start": self.noise_start.value(),
            "roi/noise_end": self.noise_end.value(),
            "roi/visible_v2": self.roi_enable_check.isChecked(),
        }
        for key, value in values.items():
            self.settings.setValue(key, value)
        self.settings.setValue(
            "layout/workspace_splitter", self.workspace_splitter.saveState()
        )
        self.settings.setValue(
            "layout/vertical_plot_splitter", self.vertical_plot_splitter.saveState()
        )
        self.settings.setValue("layout/plot_splitter", self.plot_splitter.saveState())
        self._save_layout_customization()

    def _install_shortcuts(self):
        self.shortcuts = []

        def bind(keys, callback):
            shortcut = QShortcut(QKeySequence(keys), self)
            shortcut.activated.connect(callback)
            self.shortcuts.append(shortcut)

        bind("F1", self.show_help)
        bind("F5", self.start_run)
        bind("Shift+F5", self.stop_run)
        bind("Escape", self.stop_run)
        bind("Ctrl+Return", self.configure_acquisition)
        bind("Ctrl+L", self.reset_histogram)
        bind("Ctrl+S", self.save_csv)
        bind("Ctrl+Shift+S", self.save_plot)
        bind("Ctrl+Shift+D", self.save_data)
        bind("Ctrl+R", self.roi_enable_check.toggle)
        bind("Ctrl+D", self.difference_button.toggle)
        bind("Ctrl+Y", self._fit_y_once)
        bind("Ctrl+A", self._auto_range_mcs_live)
        bind("Ctrl+G", self.log_y_check.toggle)
        bind("Ctrl+,", self._show_acquisition_popup)
        bind("F2", self._show_acquisition_popup)
        bind("Ctrl+1", lambda: self._set_mode("mcs"))
        bind("Ctrl+2", lambda: self._set_mode("counter"))
        bind("Ctrl+3", self._toggle_self_test)
        bind("Ctrl+F", self._freeze_reference_histogram)
        bind("Ctrl+Shift+F", self._clear_reference_histogram)

        # Keyboard histogram navigation. Plain arrows move the selected bin.
        # Ctrl+arrows zoom around that selected bin without changing selection.
        for key, step in (
            ("Left", -1), ("Right", 1),
            ("Up", 1), ("Down", -1),
        ):
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
            shortcut.activated.connect(
                lambda step=step: self._move_highlighted_bin(step)
            )
            self.shortcuts.append(shortcut)

        # Ctrl+horizontal arrows affect only the X axis.
        # Ctrl+vertical arrows affect only the Y axis.
        for key, axis, factor in (
            ("Ctrl+Right", "x", 0.5),   # X zoom in
            ("Ctrl+Left",  "x", 2.0),   # X zoom out
            ("Ctrl+Up",    "y", 0.5),   # Y zoom in
            ("Ctrl+Down",  "y", 2.0),   # Y zoom out
        ):
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
            shortcut.activated.connect(
                lambda axis=axis, factor=factor:
                    self._zoom_around_selected_bin(axis, factor)
            )
            self.shortcuts.append(shortcut)

    def _apply_theme(self, _index=None):
        theme = self.theme_combo.currentData() or "dark"
        app = QApplication.instance()
        if app is not None:
            app.setStyleSheet(DARK_STYLE_SHEET if theme == "dark" else LIGHT_STYLE_SHEET)
        plot_background = BG if theme == "dark" else LIGHT_BG
        axis_color = MUTED if theme == "dark" else "#526579"
        for plot in (self.plot, self.difference_plot, self.rate_plot):
            plot.setBackground(plot_background)
            for axis_name in ("left", "bottom"):
                axis = plot.getAxis(axis_name)
                axis.setTextPen(axis_color)
                axis.setPen(axis_color)

        # The overview is part of the same visual system as the main plot.
        # It is opaque in both themes and uses the theme's panel/border colors.
        if hasattr(self, "overview"):
            inset_bg = PANEL if theme == "dark" else "#ffffff"
            inset_border = BORDER if theme == "dark" else "#a8b6c5"
            self.overview.setThemeColors(inset_bg, inset_border)
            self.overview_curve.setPen(pg.mkPen(axis_color, width=1.0))
            highlight = QColor(CYAN)
            highlight.setAlpha(58 if theme == "dark" else 42)
            self.overview_zoom.setBrush(pg.mkBrush(highlight))
            # LinearRegionItem has no setPen() in several supported
            # pyqtgraph releases. Its two InfiniteLine boundaries own the pen.
            boundary_pen = pg.mkPen(CYAN, width=1.5)
            for boundary_line in self.overview_zoom.lines:
                boundary_line.setPen(boundary_pen)
        # Channel colors are explicit user choices and must not be derived
        # from either the selected GUI theme or the operating-system theme.
        if self.instrument_mode == "counter":
            self.plot.getAxis("left").setTextPen(self.counter_signal_color)
            self.plot.getAxis("left").setPen(self.counter_signal_color)
        self.counter_trigger_axis.setTextPen(self.counter_trigger_color)
        self.counter_trigger_axis.setPen(self.counter_trigger_color)

    def choose_trace_color(self):
        color = QColorDialog.getColor(
            QColor(self.trace_color), self, "Choose MCS trace color",
            QColorDialog.ColorDialogOption.DontUseNativeDialog,
        )
        if color.isValid():
            self._set_trace_color(color.name())

    def _set_trace_color(self, color_name):
        color = QColor(str(color_name))
        if not color.isValid():
            color = QColor(CYAN)
        self.trace_color = color.name()
        if self.instrument_mode == "mcs":
            self.curve.setPen(pg.mkPen(self.trace_color, width=1.25))
        text_color = "#081018" if color.lightness() > 150 else "#ffffff"
        self.trace_color_button.setText(f"Trace  {self.trace_color.upper()}")
        self.trace_color_button.setStyleSheet(
            f"background:{self.trace_color}; color:{text_color}; "
            f"border:1px solid {self.trace_color}; font-weight:650;"
        )

    def choose_counter_signal_color(self):
        color = QColorDialog.getColor(
            QColor(self.counter_signal_color), self, "Choose H4 Signal color",
            QColorDialog.ColorDialogOption.DontUseNativeDialog,
        )
        if color.isValid():
            self._set_counter_signal_color(color.name())

    def choose_counter_trigger_color(self):
        color = QColorDialog.getColor(
            QColor(self.counter_trigger_color), self, "Choose H2 Trigger color",
            QColorDialog.ColorDialogOption.DontUseNativeDialog,
        )
        if color.isValid():
            self._set_counter_trigger_color(color.name())

    @staticmethod
    def _color_button_style(color):
        text = "#081018" if QColor(color).lightness() > 150 else "#ffffff"
        return (
            f"background:{color}; color:{text}; border:1px solid {color}; "
            "font-weight:800; padding:0;"
        )

    def _set_counter_signal_color(self, color_name):
        color = QColor(str(color_name))
        if not color.isValid():
            color = QColor(CYAN)
        self.counter_signal_color = color.name()
        if self.instrument_mode == "counter":
            self.curve.setPen(pg.mkPen(self.counter_signal_color, width=1.25))
            self.plot.getAxis("left").setPen(self.counter_signal_color)
            self.plot.getAxis("left").setTextPen(self.counter_signal_color)
            self.plot.setLabel(
                "left", "Signal input (H4)", units="cps",
                color=self.counter_signal_color,
            )
        self.counter_signal_color_button.setStyleSheet(
            self._color_button_style(self.counter_signal_color)
        )

    def _set_counter_trigger_color(self, color_name):
        color = QColor(str(color_name))
        if not color.isValid():
            color = QColor(AMBER)
        self.counter_trigger_color = color.name()
        self.counter_trigger_curve.setPen(
            pg.mkPen(self.counter_trigger_color, width=1.25)
        )
        self.counter_trigger_axis.setPen(pg.mkPen(self.counter_trigger_color))
        self.counter_trigger_axis.setTextPen(pg.mkPen(self.counter_trigger_color))
        self.counter_trigger_axis.setLabel(
            "Trigger input (H2)", units="cps",
            color=self.counter_trigger_color,
        )
        self.counter_trigger_color_button.setStyleSheet(
            self._color_button_style(self.counter_trigger_color)
        )

    def show_help(self):
        QMessageBox.information(
            self,
            "OpenMCS help",
            "<h2>OpenMCS quick help</h2>"
            "<p><b>Mode:</b> use the always-visible MCS / COUNTER switch at the top. "
            "Changing mode safely stops a running acquisition. Ctrl+1 selects MCS; "
            "Ctrl+2 selects Counter; Ctrl+3 toggles the MCS Self test setting.</p>"
            "<p><b>Trace colors:</b> MCS uses its Trace button. In Counter mode, "
            "H4 Signal (L) and H2 Trigger (R) can each be shown or hidden, and "
            "the small colored button beside each channel changes its color.</p>"
            "<p><b>Setup</b> reveals bins, dwell, trigger, and counter settings. "
            "Keep it collapsed for a plot-focused acquisition view.</p>"
            "<p><b>Saving:</b> choose a Save to folder and Run name once, then use Save Data to write both CSV and PNG without another dialog. Existing names are protected with _002, _003, etc.</p>"
            "<p><b>MCS:</b> set Max passes (or Continuous / unlimited), apply the configuration, then use Run internal, "
            "Run external, or Stop. MCS dwell is limited to 1 ms/bin and each pass to 20 s total. The horizontal axis is physical time after trigger.</p>"
            "<p><b>Counter:</b> choose Simple counter, select the active dwell, "
            "then Start counter. No external trigger source is required.</p>"
            "<p><b>Self Test:</b> select MCS, enable Self test in Setup, click Prepare self-test, "
            "the dwell time and pass count, then click Start self-test. The stimulus "
            "continues independently while the FPGA accepts the selected passes. It generates one event/pass in bins "
            "4, 49, 499, and 3,999 for the selected dwell.</p>"
            "<p><b>ROI:</b> enable ROI, drag signal and noise regions, and optionally "
            "enable Difference for a side-by-side background-subtracted CPS plot.</p>"
            "<p><b>Interactive inspection:</b> Ctrl+left-click selects one bin; hold Ctrl and left-drag across "
            "the histogram. The highlighted interval and its bin/time range, "
            "sum, mean, peak, and centroid appear below the plot.</p>"
            "<h3>Keyboard shortcuts</h3>"
            "<pre>F5          Start acquisition\n"
            "Shift+F5    Stop acquisition\n"
            "Esc         Stop and disarm\n"
            "Ctrl+Enter  Apply/prepare configuration\n"
            "Ctrl+L      Clear accumulation\n"
            "Ctrl+S      Export CSV\n"
            "Ctrl+Shift+S Export plot image\n"
            "←/→ ↑/↓     Move selected bin by one\n"
            "Ctrl+→      Zoom X axis in around selected bin\n"
            "Ctrl+←      Zoom X axis out around selected bin\n"
            "Ctrl+↑      Zoom Y axis in around selected bin\n"
            "Ctrl+↓      Zoom Y axis out around selected bin\n"
            "Ctrl+R      Toggle ROI\n"
            "Ctrl+D      Toggle difference plot\n"
            "Ctrl+Y      Toggle Auto Y\n"
            "Ctrl+G      Toggle Log Y\n"
            "Ctrl+, / F2 Open setup\n"
            "Ctrl+1/2    MCS/counter mode\n"
            "Ctrl+3      Toggle MCS self-test setting\n"
            "F1          Help</pre>"
        )

    def refresh_ports(self):
        if self.worker is not None:
            return
        current = self.port_combo.currentData()
        preferred = current or self.settings.value("connection/port", "", str)
        ports = available_ports()
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(f"{port.device} — {port.description}", port.device)
        if preferred:
            index = self.port_combo.findData(preferred)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        self.connect_button.setEnabled(bool(ports))
        if not ports:
            self._show_status("No serial instruments detected", "warning")

    def _try_initial_auto_connect(self):
        if self._auto_connect_attempted or not self.auto_connect_check.isChecked():
            return
        self._auto_connect_attempted = True
        if self.port_combo.count() and self.worker is None:
            self.toggle_connection()

    def toggle_connection(self):
        if self.worker is not None:
            self.disconnect_serial()
            return
        port_name = self.port_combo.currentData()
        if not port_name:
            self._show_status("Select a COM port first", "warning")
            return
        self.connect_button.setEnabled(False)
        self._show_status(f"Connecting to {port_name}…")
        worker = SerialWorker(port_name)
        worker.frame_received.connect(self.handle_frame)
        worker.connection_changed.connect(self.handle_connection)
        worker.message.connect(self._show_status)
        self.worker = worker
        worker.start()

    def disconnect_serial(self):
        if self.worker is None:
            return
        self._show_status("Disconnecting…")
        self.worker.stop()
        if not self.worker.wait(1500):
            self._show_status("Serial connection is taking longer to close", "warning")

    def handle_connection(self, connected, port_name):
        if connected:
            self._set_connected(True)
            self.send(f"g {1 if self.internal_signal_check.isChecked() else 0}")
            self.send(f"h {1 if self.self_test_check.isChecked() else 0}")
            self._apply_trigger_edge()
            self._show_status(f"Connected to {port_name}", "success")
        else:
            if self.sender() is self.worker:
                self.worker = None
            self._set_connected(False)
            self.acquisition_running = False
            self._finish_acquisition_timing()
            self.configured_signature = None
            self._show_status("Disconnected", "warning")
            QTimer.singleShot(0, self.refresh_ports)

    def _set_connected(self, connected):
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connect_button.setEnabled(connected or self.port_combo.count() > 0)
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        for widget in (self.configure_button, self.run_button, self.external_button,
                       self.stop_button,
                       self.single_button, self.quick_run_button,
                       self.quick_stop_button, self.quick_clear_button):
            widget.setEnabled(connected)
        if self.instrument_mode == "counter" or self.self_test_check.isChecked():
            self.single_button.setEnabled(False)
        self.connection_badge.setText("●  CONNECTED" if connected else "●  DISCONNECTED")
        self.connection_badge.setProperty("connected", connected)
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)
        has_data = (
            bool(self.counter_times)
            if self.instrument_mode == "counter"
            else self.latest_counts is not None
        )
        self.save_csv_button.setEnabled(has_data)
        self.save_plot_button.setEnabled(has_data)
        self.reset_button.setEnabled(connected)

    def send(self, command):
        if self.worker is None:
            self._show_status("Connect to the Teensy first", "warning")
            return False
        self.worker.send_command(command)
        return True

    def _mcs_max_dwell_seconds(self):
        """Maximum dwell representable by the existing 32-bit FPGA protocol."""
        return 0xFFFFFFFF / self._clock_hz()

    def _mcs_limits_changed(self, _value=None):
        """Expose the full dwell range supported by the existing GUI/FPGA protocol."""
        multiplier = float(self.dwell_unit.currentData() or 1.0)
        max_seconds = self._mcs_max_dwell_seconds()
        max_display = max_seconds / multiplier
        self.dwell_spin.setMaximum(max_display)
        if self.dwell_spin.value() > max_display:
            self.dwell_spin.setValue(max_display)
        self.dwell_spin.setToolTip(
            f"Full 32-bit dwell range: up to {max_seconds:,.6f} s/bin at "
            f"{self._clock_hz()/1e6:,.6f} MHz. No GUI 1 ms/bin or 20 s/pass limit."
        )
        self._update_quantization()

    def _validate_mcs_timing(self):
        """GUI imposes no legacy dwell/pass-span limit; protocol range is enforced on conversion."""
        return True

    def _requested_dwell_cycles(self):
        seconds = self.dwell_spin.value() * self.dwell_unit.currentData()
        return max(MIN_DWELL_CYCLES, min(0xFFFFFFFF, round(seconds * self._clock_hz())))

    def _counter_timing(self):
        """Return segment cycles/count and actual displayed Counter dwell."""
        requested_s = float(self.counter_dwell_spin.value())
        total_cycles = max(MIN_DWELL_CYCLES, round(requested_s * self._clock_hz()))
        segments = max(1, math.ceil(total_cycles / 0xFFFFFFFF))
        segment_cycles = max(
            MIN_DWELL_CYCLES,
            min(0xFFFFFFFF, round(total_cycles / segments)),
        )
        actual_s = segment_cycles * segments / self._clock_hz()
        return segment_cycles, segments, actual_s

    def _update_quantization(self):
        clocks = self._requested_dwell_cycles()
        dwell_ns = clocks * 1e9 / self._clock_hz()
        span_s = clocks * self.bins_spin.value() / self._clock_hz()
        self.quantized_label.setText(
            f"Actual: {dwell_ns:,.3f} ns · {clocks:,} clocks · "
            f"pass span {self._format_time(span_s)}"
        )

    @staticmethod
    def _format_time(seconds):
        if seconds < 1e-6:
            return f"{seconds * 1e9:,.3f} ns"
        if seconds < 1e-3:
            return f"{seconds * 1e6:,.3f} µs"
        if seconds < 1:
            return f"{seconds * 1e3:,.3f} ms"
        return f"{seconds:,.3f} s"

    @staticmethod
    def _format_duration(seconds):
        if seconds is None or not math.isfinite(seconds) or seconds < 0:
            return "—"
        if seconds < 60:
            return f"{seconds:,.1f} s"
        total_seconds = int(round(seconds))
        hours, remainder = divmod(total_seconds, 3600)
        minutes, secs = divmod(remainder, 60)
        if hours:
            return f"{hours:d}:{minutes:02d}:{secs:02d}"
        return f"{minutes:d}:{secs:02d}"

    def _begin_acquisition_timing(self, target_passes=None, baseline_passes=None):
        self.acquisition_started_at = time.monotonic()
        self.eta_pass_samples.clear()
        self.eta_last_rate = None
        self.eta_interval_values.clear()
        self.eta_interval_rates.clear()
        self.eta_display_eta = None
        self.eta_display_rate = None
        self.eta_last_display_update = time.monotonic()
        # Start a new acquisition interval without clearing MCS CPS history.
        # Clear CPS is the only action that erases that history. Rebase the
        # cumulative FPGA counters so a restart cannot create a false CPS spike.
        self.mcs_rate_last_time = None
        self.mcs_rate_last_total = None
        self.mcs_rate_last_passes = None
        self.mcs_rate_fit_samples.clear()
        self.mcs_rate_fit_signature = None
        # Do not let Stop/Pause wall time or stale cumulative coordinates create
        # a false CPS point.  The first frame after Start only establishes a new baseline.
        self.mcs_rate_baseline_pending = True
        self.mcs_display_frozen = False
        self.mcs_clear_pending = False
        self.acquisition_stopped_at = None
        self.acquisition_baseline_passes = int(
            self.accumulation_frames if baseline_passes is None else baseline_passes
        )
        self.acquisition_target_passes = (
            None if target_passes is None else int(target_passes)
        )
        self._update_acquisition_timing()

    def _finish_acquisition_timing(self):
        if self.acquisition_started_at is not None and self.acquisition_stopped_at is None:
            self.acquisition_stopped_at = time.monotonic()
        self._update_acquisition_timing()

    def _current_measured_eta_rate(self):
        """Return the current rolling FPGA pass-rate estimate without changing display state."""
        if len(self.eta_pass_samples) < 3:
            return None
        ts = np.asarray([q[0] for q in self.eta_pass_samples], dtype=np.float64)
        ps = np.asarray([q[1] for q in self.eta_pass_samples], dtype=np.float64)
        span = float(ts[-1] - ts[0])
        if span < self.eta_min_span_seconds or np.ptp(ps) <= 0.0:
            return None
        tc = ts - ts.mean(); pc = ps - ps.mean()
        denom = float(np.dot(tc, tc))
        if denom <= 0.0 or not math.isfinite(denom):
            return None
        rate = float(np.dot(tc, pc) / denom)
        return rate if math.isfinite(rate) and rate > 0.0 else None

    def _update_acquisition_timing(self):
        self._apply_layout_custom_texts()
        if self.acquisition_started_at is None:
            self.acquisition_elapsed.setText("—")
            self.acquisition_pass_rate.setText("—")
            self.acquisition_count_rate.setText("—")
            self.acquisition_time_left.setText("—")
            self.acquisition_finish_time.setText("—")
            self.status_timing.setText("Acquisition — · ETA —")
            return

        end_time = self.acquisition_stopped_at or time.monotonic()
        elapsed = max(0.0, end_time - self.acquisition_started_at)
        if self.instrument_mode == "counter" and self.latest_metadata is not None:
            current_passes = int(self.latest_metadata.get("pass_count", 0))
        else:
            current_passes = int(self.accumulation_frames)
        completed = max(0, current_passes - self.acquisition_baseline_passes)

        # ETA display is a 5-second arithmetic mean of ETA estimates collected
        # from valid FPGA pass progression. Data acquisition itself is unaffected.
        now_display = time.monotonic()
        measured_rate = self._current_measured_eta_rate()
        if measured_rate is not None:
            self.eta_last_rate = measured_rate
            self.eta_interval_rates.append(measured_rate)
            if self.acquisition_target_passes is not None:
                rem = max(0, self.acquisition_target_passes - current_passes)
                self.eta_interval_values.append(rem / measured_rate)

        if now_display - self.eta_last_display_update >= self.eta_display_period_s:
            if self.eta_interval_rates:
                self.eta_display_rate = float(np.mean(self.eta_interval_rates))
            if self.eta_interval_values:
                self.eta_display_eta = float(np.mean(self.eta_interval_values))
            self.eta_interval_rates.clear()
            self.eta_interval_values.clear()
            self.eta_last_display_update = now_display

        rate = self.eta_display_rate or 0.0
        elapsed_text = self._format_duration(elapsed)
        self.acquisition_elapsed.setText(elapsed_text)
        self.acquisition_pass_rate.setText(
            self._format_rate(rate, "pass/s") if rate > 0 else "Waiting for data"
        )
        count_rate = self.mcs_count_rates[-1] if self.mcs_count_rates else 0.0
        self.acquisition_count_rate.setText(
            self._format_rate(count_rate, "cps") if count_rate >= 0 and self.mcs_count_rates
            else "Waiting for data"
        )

        if self.acquisition_target_passes is None:
            remaining_text = "Continuous"
            finish_text = "Continuous"
        else:
            remaining_passes = max(0, self.acquisition_target_passes - current_passes)
            if remaining_passes == 0:
                remaining_text = "Complete"
                finish_text = time.strftime("%H:%M:%S")
            elif rate > 0:
                remaining_seconds = (self.eta_display_eta if self.eta_display_eta is not None else remaining_passes / rate)
                remaining_text = self._format_duration(remaining_seconds)
                finish_text = time.strftime(
                    "%H:%M:%S", time.localtime(time.time() + remaining_seconds)
                )
            else:
                remaining_text = "Measuring…"
                finish_text = "Measuring…"
        self.acquisition_time_left.setText(remaining_text)
        self.acquisition_finish_time.setText(finish_text)
        self.status_timing.setText(
            f"Acquisition {elapsed_text} · ETA {remaining_text}"
        )

    @staticmethod
    def _format_rate(value, unit):
        value = float(value)
        magnitude = abs(value)
        if magnitude >= 1e9:
            return f"{value / 1e9:,.3f} G{unit}"
        if magnitude >= 1e6:
            return f"{value / 1e6:,.3f} M{unit}"
        if magnitude >= 1e3:
            return f"{value / 1e3:,.3f} k{unit}"
        return f"{value:,.3f} {unit}"

    def _clear_mcs_rate_history(self):
        """Clear only the MCS CPS history; histogram and acquisition are untouched."""
        self.mcs_rate_times.clear()
        self.mcs_count_rates.clear()
        self.mcs_pass_rates.clear()
        self.mcs_rate_last_time = None
        self.mcs_rate_last_total = None
        self.mcs_rate_last_passes = None
        self.mcs_rate_started = None
        self.mcs_rate_fit_samples.clear()
        self.mcs_rate_fit_signature = None
        self.rate_curve.setData([], [])
        self.acquisition_count_rate.setText("Waiting for data")
        self._show_status("MCS CPS history cleared; histogram preserved", "success")

    def _set_rate_plot_enabled(self, enabled):
        self.rate_plot.setVisible(bool(enabled) and self.instrument_mode == "mcs")
        if enabled and self.mcs_rate_times:
            self.rate_curve.setData(
                list(self.mcs_rate_times), list(self.mcs_count_rates)
            )
            self.rate_plot.getViewBox().autoRange(padding=0.04)

    def _sync_counter_trigger_view(self):
        """Keep the independent Trigger Y view aligned with the main X view."""
        self.counter_trigger_view.setGeometry(
            self.plot.getViewBox().sceneBoundingRect()
        )
        self.counter_trigger_view.linkedViewChanged(
            self.plot.getViewBox(), self.counter_trigger_view.XAxis
        )

    def _update_counter_trace_visibility(self, _checked=None):
        """Apply independent Counter-channel visibility and axis controls."""
        counter = self.instrument_mode == "counter"
        show_signal = counter and self.counter_signal_check.isChecked()
        show_trigger = counter and self.counter_trigger_check.isChecked()
        self.curve.setVisible(not counter or show_signal)
        self.counter_trigger_curve.setVisible(show_trigger)
        self.counter_trigger_view.setVisible(show_trigger)
        self.plot.showAxis("right", show=show_trigger)
        if show_trigger:
            # Reapply the width after showAxis(); some pyqtgraph versions
            # restore the last auto-computed (collapsed) width when unhidden.
            self.counter_trigger_axis.setWidth(92)
            self._sync_counter_trigger_view()
            # Counter 2 owns an independent scale and should track its data;
            # the MCS histogram view remains fully manual during acquisition.
            self.counter_trigger_view.enableAutoRange(axis="y", enable=True)

    def _update_mcs_live_rates(self, metadata, counts, now):
        """Estimate CPS with one continuous free-intercept linear fit.

        The cumulative histogram total is fitted against completed FPGA passes
        as C(P)=m*P+b.  The free intercept absorbs the unknown partial-read
        offset in non-atomic live MCS6 previews.  The slope gives counts per
        complete pass; no startup/steady-state estimator switch is used.
        """
        total = int(counts.sum(dtype=np.uint64))
        passes = int(metadata.get("pass_count", self.accumulation_frames))
        bins = int(metadata.get("bin_count", counts.size))
        dwell_cycles = int(metadata.get("dwell_cycles", 0))
        clock_hz = self._clock_hz()
        pass_active_s = (
            bins * dwell_cycles / clock_hz
            if bins > 0 and dwell_cycles > 0 and clock_hz > 0.0
            else 0.0
        )

        # Preserve the atomic final histogram, but do not mix it into the live
        # regression.  The displayed CPS therefore holds its last valid value
        # instead of acquiring an artificial endpoint.
        if not self.acquisition_running or metadata.get("atomic_final"):
            self.mcs_rate_last_time = now
            self.mcs_rate_last_total = total
            self.mcs_rate_last_passes = passes
            return

        # On every Start/restart, consume exactly one valid live frame as a
        # baseline and do not append a CPS sample. Existing CPS history is preserved.
        if self.mcs_rate_baseline_pending:
            self.mcs_rate_fit_samples.clear()
            self.mcs_rate_fit_signature = (bins, dwell_cycles)
            self.mcs_rate_last_time = now
            self.mcs_rate_last_total = total
            self.mcs_rate_last_passes = passes
            self.mcs_rate_baseline_pending = False
            return

        signature = (bins, dwell_cycles)
        if signature != self.mcs_rate_fit_signature:
            self.mcs_rate_fit_samples.clear()
            self.mcs_rate_fit_signature = signature

        # Repeated, reversed, or reset cumulative coordinates cannot belong to
        # the current fit. Rebase without erasing the existing CPS history.
        if self.mcs_rate_fit_samples:
            old_passes, old_total = self.mcs_rate_fit_samples[-1]
            if passes <= old_passes or total < old_total:
                self.mcs_rate_fit_samples.clear()
        self.mcs_rate_fit_samples.append((passes, total))

        cps = linear_fit_active_rate(self.mcs_rate_fit_samples, pass_active_s)
        pps = None
        if (self.mcs_rate_last_time is not None and
                self.mcs_rate_last_passes is not None):
            wall_dt = now - self.mcs_rate_last_time
            dp = passes - self.mcs_rate_last_passes
            if dp > 0:
                pps = dp / wall_dt if wall_dt > 0.02 else 0.0

        if cps is not None:
            previous_pass = (
                self.mcs_rate_last_passes
                if self.mcs_rate_last_passes is not None else passes
            )
            active_dt = max(0, passes - previous_pass) * pass_active_s
            elapsed = (self.mcs_rate_times[-1] if self.mcs_rate_times else 0.0) + active_dt
            self.mcs_rate_times.append(elapsed)
            self.mcs_count_rates.append(cps)
            if pps is not None and pps > 0.0:
                self.mcs_pass_rates.append(pps)

            if self.plot_rate_check.isChecked():
                self.rate_curve.setData(
                    list(self.mcs_rate_times),
                    list(self.mcs_count_rates)
                )
                self.rate_plot.getViewBox().autoRange(padding=0.04)

        self.mcs_rate_last_time = now
        self.mcs_rate_last_total = total
        self.mcs_rate_last_passes = passes

    def _toggle_shortcuts_window(self, visible):
        """Show/hide a persistent always-on-top keyboard shortcut reference."""
        dialog = getattr(self, "_shortcuts_dialog", None)

        if visible and dialog is None:
            dialog = QDialog(self)
            dialog.setWindowTitle("OpenMCS — Keyboard shortcuts")
            dialog.setModal(False)
            dialog.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, True)
            dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, False)
            dialog.setMinimumWidth(500)

            layout = QVBoxLayout(dialog)
            layout.setContentsMargins(14, 14, 14, 14)
            title = QLabel("Keyboard shortcuts")
            title.setObjectName("SectionTitle")
            layout.addWidget(title)

            rows = (
                ("F5", "Start acquisition"),
                ("Shift+F5 / Esc", "Stop acquisition"),
                ("Ctrl+Enter", "Apply / prepare configuration"),
                ("Ctrl+L", "Clear accumulation / session"),
                ("Ctrl+S", "Export CSV"),
                ("Ctrl+Shift+S", "Export plot image"),
                ("Ctrl+Shift+D", "Save Data"),
                ("Ctrl+, / F2", "Open Setup"),
                ("Ctrl+1", "MCS mode"),
                ("Ctrl+2", "Counter mode"),
                ("Ctrl+3", "Toggle MCS self-test"),
                ("Ctrl+F", "Freeze current histogram as reference"),
                ("Ctrl+Shift+F", "Remove reference histogram"),
                ("Ctrl+R", "Toggle ROI"),
                ("Ctrl+D", "Toggle Difference plot"),
                ("Ctrl+Y", "Toggle Auto Y"),
                ("Ctrl+G", "Toggle Log Y"),
                ("Ctrl + left-click", "Select MCS bin / nearest Counter point"),
                ("Ctrl + left-drag", "Highlight a histogram range"),
                ("Arrow keys", "Move selected bin / Counter point by one"),
                ("Ctrl+Right", "Zoom X axis in around selected bin"),
                ("Ctrl+Left", "Zoom X axis out around selected bin"),
                ("Ctrl+Up", "Zoom Y axis in around selected bin"),
                ("Ctrl+Down", "Zoom Y axis out around selected bin"),
            )

            grid = QGridLayout()
            grid.setHorizontalSpacing(22)
            grid.setVerticalSpacing(7)
            for row, (keys, action) in enumerate(rows):
                key_label = QLabel(keys)
                key_label.setStyleSheet(
                    "font-family:'Consolas','Cascadia Mono',monospace; font-weight:700;"
                )
                action_label = QLabel(action)
                grid.addWidget(key_label, row, 0)
                grid.addWidget(action_label, row, 1)
            layout.addLayout(grid)

            close_button = QPushButton("Close")
            close_button.clicked.connect(
                lambda: self.shortcuts_button.setChecked(False)
            )
            close_row = QHBoxLayout()
            close_row.addStretch(1)
            close_row.addWidget(close_button)
            layout.addLayout(close_row)

            def shortcuts_close_event(event):
                event.ignore()
                self.shortcuts_button.setChecked(False)

            dialog.closeEvent = shortcuts_close_event
            self._shortcuts_dialog = dialog

        if dialog is None:
            return

        if visible:
            dialog.show()
            dialog.raise_()
            dialog.activateWindow()
        else:
            dialog.hide()

    def _show_acquisition_popup(self):
        """Open the acquisition controls as a persistent always-on-top dialog."""
        if getattr(self, "_acquisition_dialog", None) is None:
            dialog = QDialog(self)
            dialog.setWindowTitle("OpenMCS — Acquisition setup")
            dialog.setModal(False)
            # Parent-owned tool window: stays above the OpenMCS main window,
            # but does not use the global always-on-top hint.
            dialog.setWindowFlag(Qt.WindowType.Tool, True)
            dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, False)
            dialog.setMinimumWidth(430)
            dialog.setSizeGripEnabled(True)

            layout = QVBoxLayout(dialog)
            layout.setContentsMargins(10, 10, 10, 10)

            # Controls are hidden at application startup and become visible only
            # when they are placed into the Setup dialog.
            layout.addWidget(self.acquisition_group)
            self.acquisition_group.setVisible(True)

            close_row = QHBoxLayout()
            close_row.addStretch(1)
            close_button = QPushButton("Close")
            close_button.clicked.connect(dialog.hide)
            close_row.addWidget(close_button)
            layout.addLayout(close_row)

            # Clicking the title-bar X hides rather than destroys the dialog.
            def setup_close_event(event):
                event.ignore()
                dialog.hide()

            dialog.closeEvent = setup_close_event
            self._acquisition_dialog = dialog

        self._acquisition_dialog.show()
        self._acquisition_dialog.raise_()
        self._acquisition_dialog.activateWindow()

    def _apply_trigger_edge(self, _index=None):
        """Configure only the FPGA H2 edge detector; Teensy stimulus is unchanged."""
        edge = self.trigger_edge_combo.currentData() or "r"
        if self.worker is not None:
            self.send(f"q {'d' if edge == 'f' else 'u'}")

    def configure_acquisition(self):
        self._apply_trigger_edge()
        if self.instrument_mode == "counter":
            clocks, segments, actual_dwell = self._counter_timing()
            self.counter_segments_per_point = segments
            self.counter_segment_count = 0
            self.counter_segment_signal_sum = 0
            self.counter_segment_trigger_sum = 0
            self.counter_segment_elapsed = 0.0
            if self.send("p"):
                self.send(f"c 1 {clocks}")
                self.configured_signature = (1, clocks)
                self._request_counter_baseline()
                self._show_status(
                    f"Counter prepared: {actual_dwell:,.6f} s displayed dwell"
                    f" ({segments} FPGA segment{'s' if segments != 1 else ''}) · history preserved",
                    "success",
                )
            return
        if self.self_test_check.isChecked():
            clocks = self._requested_dwell_cycles()
            requested_signature = (self.bins_spin.value(), clocks)
            if self.configured_signature == requested_signature:
                self._show_status(
                    "Self-test is already prepared with these settings; histogram preserved. Press Start.",
                    "success",
                )
            elif self.send("p"):
                self.send(f"c {self.bins_spin.value()} {clocks}")
                self.configured_signature = requested_signature
                self._show_status(
                    f"Self-test prepared: target {self.test_passes_spin.value():,} total passes · {self.bins_spin.value():,} bins × {clocks:,} clocks "
                    f"= {clocks * 1e9 / self._clock_hz():,.3f} ns/bin. "
                    "Geometry changed; existing FPGA histogram is preserved. Press Start.",
                    "success",
                )
            return
        if not self._validate_mcs_timing():
            return
        clocks = self._requested_dwell_cycles()
        requested_signature = (self.bins_spin.value(), clocks)
        max_passes = 0 if self.unlimited_passes_check.isChecked() else self.max_passes_spin.value()

        # CFG5 updates acquisition geometry without clearing histogram RAM or
        # the accumulated pass counter. Explicit Clear is the only operation
        # that erases the FPGA accumulation.
        if self.configured_signature == requested_signature:
            self.send(f"m {max_passes}")
            self._show_status(
                f"Configuration unchanged: {self.bins_spin.value():,} bins, "
                f"{clocks:,} clocks ({clocks * 1e9 / self._clock_hz():,.3f} ns). "
                "Existing FPGA histogram preserved.",
                "success",
            )
        elif self.send("p"):
            self.send(f"m {max_passes}")
            self.send(f"c {self.bins_spin.value()} {clocks}")
            self.configured_signature = requested_signature
            self._show_status(
                f"Configuration changed: {self.bins_spin.value():,} bins, {clocks:,} clocks "
                f"({clocks * 1e9 / self._clock_hz():,.3f} ns). "
                "Existing FPGA histogram preserved; new passes will accumulate on top.",
                "success",
            )

    def start_run(self):
        if self.instrument_mode == "mcs" and self.self_test_check.isChecked():
            clocks = self._requested_dwell_cycles()
            requested_signature = (self.bins_spin.value(), clocks)
            max_passes = self.test_passes_spin.value()
            # The Teensy owns the conditional configuration decision. If this
            # geometry is already active, "y" starts another physical test
            # burst without clearing the FPGA histogram. A different geometry
            # must be configured and therefore starts a new accumulation.
            preserves_histogram = True
            baseline_passes = self.accumulation_frames
            # Program the absolute accumulated-pass target. The Teensy
            # self-test remains continuous while the FPGA is armed/active.
            self.send(f"m {max_passes}")
            if self.send(f"y {clocks} {self.bins_spin.value()}"):
                self.configured_signature = requested_signature
                self.acquisition_running = True
                self._begin_acquisition_timing(
                    None if max_passes == 0 else max_passes,
                    baseline_passes=baseline_passes,
                )
                limit_text = (
                    "continuous" if max_passes == 0
                    else f"target {max_passes:,} total passes"
                )
                self._show_status(
                    f"Self-test running: {limit_text} · "
                    f"histogram preserved · "
                    f"{self.bins_spin.value():,} bins × {clocks:,} clocks "
                    f"({clocks * 1e9 / self._clock_hz():,.3f} ns/bin); "
                    "expected bins 4, 49, 499, 3,999…",
                )
            return
        if self.instrument_mode == "counter":
            clocks, segments, actual_dwell = self._counter_timing()
            requested_signature = (1, clocks)
            self.counter_segments_per_point = segments
            self.counter_segment_count = 0
            self.counter_segment_signal_sum = 0
            self.counter_segment_trigger_sum = 0
            self.counter_segment_elapsed = 0.0

            # START MUST ONLY START.  A CFG5 transaction clears/reconfigures the
            # FPGA and is intentionally kept out of the normal Start path.
            # Configure once when the requested dwell has actually changed.
            # Counter/MCS transitions are serialized deliberately.  CFG5 is a
            # stop/config transaction in the FPGA; issuing K immediately behind it
            # can race the stop/clear/config CDC and leave Counter producing zeros.
            # Always STOP -> CONFIGURE -> short settle -> K when entering Counter.
            if self._counter_start_pending:
                return
            self._counter_start_pending = True
            if not self.send("p"):
                self._counter_start_pending = False
                return
            self.send(f"c 1 {clocks}")
            self.configured_signature = requested_signature
            self.acquisition_running = False
            self._show_status("Preparing simple counter and synchronizing FPGA mode…")

            def finish_counter_start():
                self._counter_start_pending = False
                if self.instrument_mode != "counter" or self.worker is None:
                    return
                self.acquisition_running = True
                self._begin_acquisition_timing(baseline_passes=0)
                self._request_counter_baseline()
                if self.send("k"):
                    self._show_status(
                        f"Dual-input counter running · {actual_dwell:,.6f} s dwell"
                        f" · {segments} FPGA segment{'s' if segments != 1 else ''}/point",
                        "success",
                    )
                else:
                    self.acquisition_running = False
                    self._finish_acquisition_timing()
            QTimer.singleShot(100, finish_counter_start)
            return
        if not self._validate_mcs_timing():
            return
        frequency = self.trigger_spin.value()
        requested_signature = (
            self.bins_spin.value(), self._requested_dwell_cycles()
        )
        max_passes = 0 if self.unlimited_passes_check.isChecked() else self.max_passes_spin.value()
        baseline_passes = int(self.accumulation_frames)

        def finish_mcs_internal_start():
            if self.instrument_mode != "mcs" or self.worker is None:
                return
            self.send(f"f {frequency}")
            self.send(f"m {max_passes}")
            if self.send("r"):
                self.acquisition_running = True
                self._begin_acquisition_timing(
                    None if max_passes == 0 else max_passes,
                    baseline_passes=baseline_passes
                )
                limit_text = "continuous" if max_passes == 0 else f"target {max_passes:,} total passes"
                self._show_status(
                    f"Internal trigger running at {frequency:,} Hz · {limit_text} · "
                    f"{self.bins_spin.value():,} bins × {self._requested_dwell_cycles():,} clocks",
                    "success"
                )

        # ALWAYS program the geometry shown in Setup before every MCS Start.
        # Do not trust configured_signature here: it is GUI state and cannot prove
        # what geometry is currently active inside the FPGA after reconnects, mode
        # changes, resets, or power cycles.
        if not self.send("p"):
            return
        self.send(f"c {self.bins_spin.value()} {self._requested_dwell_cycles()}")
        self.configured_signature = requested_signature
        self.acquisition_running = False
        self._show_status(
            f"Configuring FPGA: {self.bins_spin.value():,} bins × "
            f"{self._requested_dwell_cycles():,} clocks before Start…"
        )
        QTimer.singleShot(300, finish_mcs_internal_start)

    def _internal_signal_changed(self, enabled):
        if self.worker is not None:
            self.send(f"g {1 if enabled else 0}")
        self._show_status(
            f"Internal pin-9 signal {'ON' if enabled else 'OFF'}",
            "success" if enabled else "info",
        )

    def stop_run(self):
        if self.send("p"):
            self.acquisition_running = False
            # Ignore trailing preview blocks, but accept the coherent full
            # frame published after the active pass and FIFO drain.
            if self.instrument_mode == "mcs":
                self.mcs_display_frozen = True
            self._finish_acquisition_timing()
            # A stopped counter is still a complete, exportable session.
            has_counter_data = bool(self.counter_times)
            if self.instrument_mode == "counter":
                self.save_csv_button.setEnabled(has_counter_data)
                self.save_plot_button.setEnabled(has_counter_data)
                self.status_frame_kind.setText("COUNTER STOPPED")
            else:
                self.status_frame_kind.setText("WAITING FOR COMPLETED FRAME")
            self._show_status(
                "Counter stopped" if self.instrument_mode == "counter" else
                "Stopped — waiting for completed final histogram"
            )

    def start_external(self):
        if not self._validate_mcs_timing():
            return
        requested_signature = (
            self.bins_spin.value(), self._requested_dwell_cycles()
        )
        max_passes = 0 if self.unlimited_passes_check.isChecked() else self.max_passes_spin.value()
        baseline_passes = int(self.accumulation_frames)

        def finish_mcs_external_start():
            if self.instrument_mode != "mcs" or self.worker is None:
                return
            self.send(f"m {max_passes}")
            if self.send("e"):
                self.acquisition_running = True
                self._begin_acquisition_timing(
                    None if max_passes == 0 else max_passes,
                    baseline_passes=baseline_passes
                )
                limit_text = "continuous" if max_passes == 0 else f"target {max_passes:,} total passes"
                self._show_status(
                    f"FPGA armed for external triggers · {limit_text} · "
                    f"{self.bins_spin.value():,} bins × {self._requested_dwell_cycles():,} clocks",
                    "success"
                )

        # ALWAYS program the geometry shown in Setup before every external Start.
        # configured_signature is only a GUI cache and is not authoritative FPGA state.
        if not self.send("p"):
            return
        self.send(f"c {self.bins_spin.value()} {self._requested_dwell_cycles()}")
        self.configured_signature = requested_signature
        self.acquisition_running = False
        self._show_status(
            f"Configuring FPGA: {self.bins_spin.value():,} bins × "
            f"{self._requested_dwell_cycles():,} clocks before external Start…"
        )
        QTimer.singleShot(300, finish_mcs_external_start)

    def single_trigger(self):
        if self.send("t"):
            self._show_status("Single test trigger requested")

    def _capture_mode_view(self, mode):
        """Save the visible X/Y ranges for a mode without changing its data."""
        try:
            main_range = [list(r) for r in self.plot.getViewBox().viewRange()]
            trigger_y = list(self.counter_trigger_view.viewRange()[1])
            state = (main_range, trigger_y)
            if mode == "counter":
                self._counter_saved_view = state
            else:
                self._mcs_saved_view = state
        except Exception:
            pass

    def _restore_mode_view(self, mode):
        """Restore the mode's previous plot view after its curves are repopulated."""
        state = self._counter_saved_view if mode == "counter" else self._mcs_saved_view
        if not state:
            return
        try:
            main_range, trigger_y = state
            self.plot.getViewBox().setRange(
                xRange=main_range[0], yRange=main_range[1], padding=0.0
            )
            if mode == "counter":
                self.counter_trigger_view.setYRange(trigger_y[0], trigger_y[1], padding=0.0)
        except Exception:
            pass

    def _set_mode(self, mode):
        mode = mode if mode in ("mcs", "counter") else "mcs"
        old_mode = self.instrument_mode
        if mode != old_mode:
            self._capture_mode_view(old_mode)
            if self.acquisition_running:
                self.stop_run()
            # Mode changes never clear either GUI data set and never change Test Mode.
        self.instrument_mode = mode
        self.mcs_mode_button.setChecked(mode == "mcs")
        self.counter_mode_button.setChecked(mode == "counter")
        self._mode_changed()
        if mode != old_mode:
            QTimer.singleShot(0, lambda mode=mode: self._restore_mode_view(mode))

    def _toggle_self_test(self):
        # Test mode is global, not an MCS-only mode.
        self.self_test_check.toggle()

    def _set_shared_self_test(self, enabled, source):
        other = self.counter_self_test_check if source is self.self_test_check else self.self_test_check
        with QSignalBlocker(other):
            other.setChecked(bool(enabled))
        if self.worker is not None:
            self.send(f"h {1 if enabled else 0}")
            self._show_status(
                f"Continuous self-test signals {'enabled' if enabled else 'disabled'}",
                "success" if enabled else "info",
            )
        self._mode_changed()

    def _self_test_setting_changed(self, enabled):
        self._set_shared_self_test(enabled, self.self_test_check)

    def _counter_self_test_setting_changed(self, enabled):
        self._set_shared_self_test(enabled, self.counter_self_test_check)

    def _mode_changed(self):
        self._cancel_live_preview()
        counter = self.instrument_mode == "counter"
        self_test = self.self_test_check.isChecked()
        self.x_axis_combo.setEnabled(not counter)
        self.x_axis_label.setEnabled(not counter)
        self.mode_description.setText(
            "Dual-input counter: H4 signal and H2 trigger are measured together" if counter else
            "Built-in 500.000 ns/bin hardware test" if self_test else
            "Triggered multichannel scaler"
        )
        if self_test:
            clocks = self._requested_dwell_cycles()
            dwell_ns = clocks * 1e9 / self._clock_hz()
            span_s = self.bins_spin.value() * clocks / self._clock_hz()
            self.quantized_label.setText(
                f"Self-test: {self.bins_spin.value():,} bins · target {self.test_passes_spin.value():,} total passes · {dwell_ns:,.3f} ns/bin · "
                f"{clocks:,} clocks · span {self._format_time(span_s)}"
            )
        elif not counter:
            self._update_quantization()
        self.bins_spin.setEnabled(not counter)
        for widget in (self.trigger_spin, self.single_button):
            widget.setEnabled(not counter and not self_test)
        # ROI is a display-only control and remains clickable regardless of
        # acquisition/self-test state while MCS is selected.
        self.roi_enable_check.setEnabled(not counter)
        self.difference_button.setEnabled(
            not counter and self.roi_enable_check.isChecked()
        )
        # Keep the Counter toolbar focused: its two channel/axis switches must
        # remain readable when the window is narrowed. These controls belong
        # only to the MCS histogram display.
        self.plot_rate_check.setVisible(not counter)
        self.clear_rate_button.setVisible(not counter)
        self.trace_color_button.setVisible(not counter)
        self.calibration_group.setVisible(True)
        self.overview.setVisible(not counter)
        self.roi_enable_check.setVisible(not counter)
        self.difference_button.setVisible(not counter)
        self.reference_button.setVisible(not counter)
        self.clear_reference_button.setVisible(not counter)
        for widget in (
            self.bins_label, self.bins_spin, self.dwell_label, self.dwell_spin,
            self.dwell_unit, self.trigger_label, self.trigger_spin,
            self.quantized_label,
        ):
            widget.setVisible(not counter)
        self.self_test_check.setVisible(not counter)
        self.counter_self_test_check.setVisible(counter)
        # The Teensy internal signal drives the H4 signal input and is useful
        # for testing in both MCS and Counter modes. Hide it only in the fixed
        # self-test, which owns its waveform configuration.
        self.internal_signal_check.setVisible(not self_test)
        self.internal_signal_check.setText(
            "Generate internal test signal on H4" if counter else
            "Internal signal"
        )
        self.internal_signal_check.setToolTip(
            "Generate the test pulse signal counted on FPGA H4"
            if counter else "Continuously generate the signal on Teensy pin 9"
        )
        self.trigger_label.setVisible(not counter and not self_test)
        self.trigger_spin.setVisible(not counter and not self_test)
        self.max_passes_label.setVisible(not counter and not self_test)
        self.max_passes_spin.setVisible(not counter and not self_test)
        self.unlimited_passes_check.setVisible(not counter and not self_test)
        self.max_passes_spin.setEnabled(
            not counter and not self_test and not self.unlimited_passes_check.isChecked()
        )
        self.test_passes_label.setVisible(self_test)
        self.test_passes_spin.setVisible(self_test)
        self.dwell_spin.setEnabled(not counter)
        self.dwell_unit.setEnabled(not counter)
        for widget in (self.counter_dwell_label, self.counter_dwell_spin,
                       self.counter_hint):
            widget.setVisible(counter)
        self.single_button.setVisible(not counter)
        self.run_button.setText("Start counter" if counter else "Start self-test" if self_test else "Run internal")
        self.external_button.setVisible(not counter and not self_test)
        self.quick_run_button.setText("▶ Counter" if counter else "▶ Self-test" if self_test else "▶ Start")
        self.configure_button.setText("Prepare counter" if counter else "Prepare self-test" if self_test else "Apply configuration")
        self.quick_clear_button.setText("Clear session" if counter else "Clear")
        self.reset_button.setText("Clear session" if counter else "Clear accumulation")
        self.quick_clear_button.setToolTip(
            "Delete the displayed counter session and clear the FPGA (Ctrl+L)"
            if counter else "Clear FPGA accumulation (Ctrl+L)"
        )
        if counter:
            # Counter history can become much longer than the visible window.
            # Keep every point renderable when the user pans back in time.
            self.plot.setClipToView(False)
            self.plot.setDownsampling(auto=False)
            # Counter has no ROI display, but preserve the MCS ROI/Difference
            # state exactly so returning to MCS restores it.
            self.signal_region.setVisible(False)
            self.noise_region.setVisible(False)
            self.difference_plot.setVisible(False)
            self.plot.setLabel("bottom", "Elapsed time", units="s")
            self.plot.getAxis("bottom").enableAutoSIPrefix(True)
            self.plot.setLimits(xMin=0.0, yMin=0.0)
            self.counter_trigger_view.setLimits(yMin=0.0)
            self.plot.getAxis("left").setWidth(92)
            self.counter_trigger_axis.setWidth(92)
            self.plot.getAxis("left").enableAutoSIPrefix(True)
            self.counter_trigger_axis.enableAutoSIPrefix(True)
            self.plot.setLabel(
                "left", "Signal input (H4)", units="cps",
                color=self.counter_signal_color,
            )
            self.plot.getAxis("left").setTextPen(self.counter_signal_color)
            self.plot.getAxis("left").setPen(self.counter_signal_color)
            self.rate_plot.setVisible(False)
            if self.reference_curve is not None:
                self.reference_curve.setVisible(False)
            self.latest_counts = self.counter_latest_counts
            self.latest_metadata = self.counter_latest_metadata
            self.curve.setData(self.counter_times, self.counter_cps)
            self.counter_trigger_curve.setData(
                self.counter_times, self.counter_trigger_cps
            )
            self.counter_signal_check.setVisible(True)
            self.counter_signal_color_button.setVisible(True)
            self.counter_trigger_check.setVisible(True)
            self.counter_trigger_color_button.setVisible(True)
            self.curve.setPen(pg.mkPen(self.counter_signal_color, width=1.25))
            self._update_counter_trace_visibility()
            self.frame_group.setVisible(False)
            self.counter_status_group.setVisible(True)
            self._update_counter_stats()
            self.plot_title.setText(
                (f"Simple counter — signal {self.counter_cps[-1]:,.3f} cps · "
                 f"trigger {self.counter_trigger_cps[-1]:,.3f} cps")
                if self.counter_cps else "Simple counter — waiting for data"
            )
            self._show_status(
                f"Dual-input counter selected · Signal H4 and Trigger H2 · {len(self.counter_times):,} saved point(s)"
            )
        else:
            self.counter_signal_check.setVisible(False)
            self.counter_signal_color_button.setVisible(False)
            self.counter_trigger_check.setVisible(False)
            self.counter_trigger_color_button.setVisible(False)
            self.counter_trigger_curve.setVisible(False)
            self.counter_trigger_view.setVisible(False)
            self.plot.showAxis("right", show=False)
            self.curve.setVisible(True)
            self.curve.setPen(pg.mkPen(self.trace_color, width=1.25))
            self.plot.setClipToView(True)
            self.plot.setDownsampling(auto=True, mode="peak")
            self.plot.setLimits(xMin=-0.8, yMin=0.0)
            self.plot.setLabel("left", "Counts")
            theme_axis_color = (
                "#526579" if self.theme_combo.currentData() == "light" else MUTED
            )
            self.plot.getAxis("left").setTextPen(theme_axis_color)
            self.plot.getAxis("left").setPen(theme_axis_color)
            self.rate_plot.setVisible(self.plot_rate_check.isChecked())
            self._update_reference_curve()
            roi_visible = self.roi_enable_check.isChecked()
            self.signal_region.setVisible(roi_visible)
            self.noise_region.setVisible(roi_visible)
            self.difference_plot.setVisible(
                roi_visible and self.difference_button.isChecked()
            )
            selected_counts = self.mcs_latest_counts
            selected_metadata = self.mcs_latest_metadata
            selected_passes = self.mcs_accumulation_frames
            self.latest_counts = selected_counts
            self.latest_metadata = selected_metadata
            self.accumulation_frames = selected_passes
            self.frame_group.setVisible(True)
            self.counter_status_group.setVisible(False)
            if selected_metadata is not None and selected_counts is not None:
                self._update_mcs_time_axis(selected_metadata)
                x_values = self._mcs_x_values(
                    np.arange(selected_counts.size), selected_metadata
                )
                step_x, step_y = histogram_step_xy(x_values, selected_counts)
                self.curve.setData(step_x, step_y)
                self.plot_title.setText("Self-test" if self_test else "Histogram")
                self._update_frame_stats()
            else:
                # Do not erase a previously acquired plot merely because the
                # display mode changed. Empty is valid only when MCS has no
                # stored data (for example after an explicit Clear).
                if self.mcs_latest_counts is None:
                    self.curve.setData([], [])
                    self.plot_title.setText("Histogram — waiting for data")
            self._show_status(
                "Self-test enabled · click Prepare self-test, then Start self-test · expected bins 4, 49, 499, 3,999"
                if self_test else "Triggered MCS selected"
            )
        has_mode_data = (
            bool(self.counter_times) if counter else
            self.mcs_latest_counts is not None
        )
        self.save_csv_button.setEnabled(has_mode_data)
        self.save_plot_button.setEnabled(has_mode_data)
        for widget in getattr(self, "_layout_named_widgets", {}).values():
            if bool(widget.property("layoutUserHidden")):
                widget.setVisible(False)

    def _position_overview(self):
        """Place the layout-independent overlay near the upper-right of OpenMCS."""
        if not hasattr(self, "overview"):
            return
        if self.overview.width() < 180 or self.overview.height() < 120:
            self.overview.resize(320, 190)
        # Only choose an initial position once; user movement is preserved.
        if self.overview.pos().x() == 0 and self.overview.pos().y() == 0:
            x = max(20, self.width() - self.overview.width() - 390)
            y = 150
            self.overview.move(x, y)
        self.overview.raise_()
        self.overview.show()

    def _navigate_from_overview(self, x):
        """Center the main MCS viewport where the overview was clicked."""
        if self.instrument_mode != "mcs":
            return
        view = self.plot.getViewBox()
        (xmin, xmax), _ = view.viewRange()
        half_width = max((xmax - xmin) * 0.5, 1e-15)
        view.setXRange(x - half_width, x + half_width, padding=0.0)

    def _initialize_layout_editor_model(self):
        """Register user-editable controls and their compact default layout."""
        self._layout_zones = {
            "Header buttons": (self.header_actions_layout, [
                self.setup_button, self.layout_button, self.shortcuts_button,
                self.theme_combo, self.help_button,
            ]),
            "Connection bar": (self.connection_layout, [
                self.instrument_label, self.port_combo, self.refresh_button,
                self.auto_connect_check, self.connect_button,
            ]),
            "Save bar": (self.save_layout, [
                self.save_to_label, self.save_directory_edit,
                self.browse_save_directory_button, self.run_name_label,
                self.run_name_edit, self.data_settings_button,
                self.save_image_check, self.save_data_button,
            ]),
            "Plot heading": (self.title_toolbar, [
                self.plot_title, self.roi_enable_check, self.difference_button,
                self.reference_button, self.clear_reference_button,
            ]),
            "Plot controls": (self.plot_toolbar, [
                self.quick_run_button, self.quick_stop_button,
                self.quick_clear_button, self.auto_y_check, self.log_y_check,
                self.counter_signal_check, self.counter_signal_color_button,
                self.counter_trigger_check, self.counter_trigger_color_button,
                self.plot_rate_check, self.clear_rate_button,
                self.trace_color_button, self.save_csv_button,
                self.save_plot_button,
            ]),
        }
        self._layout_default_zones = {
            name: list(data[1]) for name, data in self._layout_zones.items()
        }
        named = {
            "Application title": self.app_title_label,
            "Application subtitle": self.app_subtitle_label,
            "Mode selector": self.header_actions.parentWidget().findChild(QFrame, "ModeSwitch"),
            "Connection status": self.connection_badge,
            "Connection panel": self.connection_bar,
            "Save panel": self.save_bar,
            "Main plot panel": self.plot_frame,
            "Status and ROI panel": self.side_tabs,
            "Highlighted-section panel": self.selection_panel,
            "Histogram plot": self.plot,
            "Count-rate plot": self.rate_plot,
            "Spectrum overview": self.overview,
            "Acquisition status group": self.frame_group,
            "Counter status group": self.counter_status_group,
            "Bottom status bar": self.statusBar(),
        }
        for zone_name, (_layout, widgets) in self._layout_zones.items():
            for index, widget in enumerate(widgets):
                if widget not in named.values():
                    label = self._layout_widget_text(widget) or f"item {index + 1}"
                    named[f"{zone_name}: {label}"] = widget
        editable_types = (
            QLabel, QPushButton, QCheckBox, QComboBox, QLineEdit,
            QSpinBox, QDoubleSpinBox, QGroupBox, QTabWidget, QFrame,
        )
        for attribute, widget in sorted(vars(self).items()):
            if (isinstance(widget, editable_types)
                    and widget not in named.values()
                    and widget is not self.centralWidget()):
                named[attribute.replace("_", " ").title()] = widget
        self._layout_named_widgets = {
            name: widget for name, widget in named.items() if widget is not None
        }
        self._layout_default_properties = {}
        for name, widget in self._layout_named_widgets.items():
            self._layout_default_properties[name] = {
                "text": self._layout_widget_text(widget),
                "visible": widget.isVisible(),
                "minimum": (widget.minimumWidth(), widget.minimumHeight()),
                "maximum": (widget.maximumWidth(), widget.maximumHeight()),
            }

    @staticmethod
    def _layout_widget_text(widget):
        if isinstance(widget, QGroupBox):
            return widget.title()
        if hasattr(widget, "text") and callable(widget.text):
            return widget.text()
        return ""

    @staticmethod
    def _set_layout_widget_text(widget, text):
        if isinstance(widget, QGroupBox):
            widget.setTitle(text)
        elif hasattr(widget, "setText") and callable(widget.setText):
            widget.setText(text)

    def _zone_for_widget(self, widget):
        for name, (_layout, widgets) in self._layout_zones.items():
            if widget in widgets:
                return name
        return None

    def _rebuild_layout_zone(self, zone_name):
        layout, widgets = self._layout_zones[zone_name]
        while layout.count():
            layout.takeAt(0)
        for index, widget in enumerate(widgets):
            stretch = 0
            if zone_name == "Connection bar" and widget is self.port_combo:
                stretch = 1
            elif zone_name == "Save bar" and widget is self.save_directory_edit:
                stretch = 2
            elif zone_name == "Save bar" and widget is self.run_name_edit:
                stretch = 1
            elif zone_name == "Plot heading" and widget is self.plot_title:
                stretch = 1
            layout.addWidget(widget, stretch)
            if zone_name == "Plot controls" and widget is self.quick_clear_button:
                layout.addStretch(1)

    def _save_layout_customization(self):
        reverse = {widget: name for name, widget in self._layout_named_widgets.items()}
        payload = {
            "zones": {
                zone: [reverse[widget] for widget in widgets if widget in reverse]
                for zone, (_layout, widgets) in self._layout_zones.items()
            },
            "properties": {
                name: {
                    "text": (widget.property("layoutCustomText")
                             if bool(widget.property("layoutTextEdited")) else None),
                    "hidden": bool(widget.property("layoutUserHidden")),
                    "width": 0 if widget.maximumWidth() >= 16777215 else widget.maximumWidth(),
                    "height": 0 if widget.maximumHeight() >= 16777215 else widget.maximumHeight(),
                }
                for name, widget in self._layout_named_widgets.items()
            },
        }
        self.settings.setValue("layout/customization_v1", json.dumps(payload))

    def _restore_layout_customization(self):
        raw = self.settings.value("layout/customization_v1", "", str)
        if not raw:
            return
        try:
            payload = json.loads(raw)
            assigned = set()
            for zone, names in payload.get("zones", {}).items():
                if zone not in self._layout_zones:
                    continue
                widgets = [self._layout_named_widgets[name] for name in names
                           if name in self._layout_named_widgets]
                self._layout_zones[zone] = (self._layout_zones[zone][0], widgets)
                assigned.update(widgets)
            # Do not lose a control if an older/custom layout record omitted it.
            for zone, defaults in self._layout_default_zones.items():
                for widget in defaults:
                    if widget not in assigned and not any(
                            widget in data[1] for data in self._layout_zones.values()):
                        self._layout_zones[zone][1].append(widget)
            for zone in self._layout_zones:
                self._rebuild_layout_zone(zone)
            for name, values in payload.get("properties", {}).items():
                widget = self._layout_named_widgets.get(name)
                if widget is None:
                    continue
                saved_text = values.get("text")
                if saved_text is not None:
                    self._set_layout_widget_text(widget, saved_text)
                    widget.setProperty("layoutTextEdited", True)
                    widget.setProperty("layoutCustomText", saved_text)
                width = max(0, int(values.get("width", 0)))
                height = max(0, int(values.get("height", 0)))
                widget.setMinimumWidth(width)
                widget.setMaximumWidth(width if width else 16777215)
                widget.setMinimumHeight(height)
                widget.setMaximumHeight(height if height else 16777215)
                hidden = bool(values.get("hidden", False))
                widget.setProperty("layoutUserHidden", hidden)
                if hidden:
                    widget.setVisible(False)
        except (TypeError, ValueError, KeyError):
            self.settings.remove("layout/customization_v1")

    def _apply_layout_custom_texts(self):
        for widget in getattr(self, "_layout_named_widgets", {}).values():
            if bool(widget.property("layoutTextEdited")):
                custom = widget.property("layoutCustomText")
                if custom is not None:
                    self._set_layout_widget_text(widget, str(custom))

    def _select_layout_widget(self, widget):
        if widget is self._layout_selected_widget:
            return
        if isinstance(self._layout_selected_widget, QWidget):
            self._layout_selected_widget.setStyleSheet(self._layout_selected_style)
        self._layout_selected_widget = widget
        # Plot insets are QGraphicsWidget objects, not QWidget objects. They
        # support visibility and geometry, but not QWidget style sheets.
        self._layout_selected_style = (
            widget.styleSheet() if isinstance(widget, QWidget) else ""
        )
        if isinstance(widget, QWidget):
            widget.setStyleSheet(
                self._layout_selected_style + "; border: 2px dashed #ff2d78;"
            )
        if self._layout_editor is None:
            return
        name = next((key for key, value in self._layout_named_widgets.items()
                     if value is widget), widget.objectName() or type(widget).__name__)
        self._layout_editor_selected.setText(name)
        text_value = self._layout_widget_text(widget)
        self._layout_editor_text.setText(text_value)
        self._layout_editor_text.setEnabled(bool(text_value) or hasattr(widget, "setText"))
        self._layout_editor_visible.setChecked(widget.isVisible())
        self._layout_editor_width.setValue(
            0 if widget.minimumWidth() == 0 and widget.maximumWidth() >= 16777215
            else widget.width()
        )
        self._layout_editor_height.setValue(
            0 if widget.minimumHeight() == 0 and widget.maximumHeight() >= 16777215
            else widget.height()
        )
        zone = self._zone_for_widget(widget)
        self._layout_editor_zone.setCurrentText(zone or "Not in a movable bar")

    def eventFilter(self, watched, event):
        if (event.type() == QEvent.Type.Wheel and
                watched in getattr(self, "_page_scroll_wheel_targets", ())):
            scrollbar = self.main_scroll_area.verticalScrollBar()
            pixel_delta = event.pixelDelta().y()
            if pixel_delta:
                movement = pixel_delta
            else:
                steps = event.angleDelta().y() / 120.0
                movement = steps * max(20, scrollbar.singleStep() * 3)
            scrollbar.setValue(scrollbar.value() - int(round(movement)))
            event.accept()
            return True
        if (self._layout_edit_active and event.type() == QEvent.Type.MouseButtonPress
                and isinstance(watched, QWidget)):
            if self._layout_editor is not None and (
                    watched is self._layout_editor or self._layout_editor.isAncestorOf(watched)):
                return super().eventFilter(watched, event)
            candidate = watched
            registered = set(self._layout_named_widgets.values())
            while candidate is not None and candidate not in registered:
                candidate = candidate.parentWidget()
            if candidate in registered:
                self._select_layout_widget(candidate)
                return True
        return super().eventFilter(watched, event)

    def _show_layout_editor(self):
        if self._layout_editor is not None:
            self._layout_editor.show()
            self._layout_editor.raise_()
            self._layout_editor.activateWindow()
            return
        dialog = QDialog(self)
        self._layout_editor = dialog
        dialog.setModal(False)
        dialog.setWindowTitle("Edit layout")
        dialog.resize(500, 520)
        layout = QVBoxLayout(dialog)
        help_label = QLabel(
            "Click any highlighted-capable part of the main window, or select it "
            "below. Edit its text, visibility, size, order, or control-bar location. "
            "Buttons and fields in the five movable bars can be transferred between "
            "bars. Drag the plot separators and the spectrum overview directly."
        )
        help_label.setWordWrap(True)
        layout.addWidget(help_label)
        picker = QComboBox()
        picker.addItem("Select an element…", "")
        for name in self._layout_named_widgets:
            picker.addItem(name, name)
        layout.addWidget(picker)
        self._layout_editor_selected = QLabel("No element selected")
        self._layout_editor_selected.setObjectName("SectionTitle")
        layout.addWidget(self._layout_editor_selected)
        form = QFormLayout()
        self._layout_editor_text = QLineEdit()
        self._layout_editor_visible = QCheckBox("Visible")
        self._layout_editor_width = QSpinBox()
        self._layout_editor_width.setRange(0, 4000)
        self._layout_editor_width.setSpecialValueText("Automatic")
        self._layout_editor_height = QSpinBox()
        self._layout_editor_height.setRange(0, 2500)
        self._layout_editor_height.setSpecialValueText("Automatic")
        self._layout_editor_zone = QComboBox()
        self._layout_editor_zone.addItem("Not in a movable bar")
        self._layout_editor_zone.addItems(list(self._layout_zones))
        form.addRow("Text", self._layout_editor_text)
        form.addRow("Visibility", self._layout_editor_visible)
        form.addRow("Width", self._layout_editor_width)
        form.addRow("Height", self._layout_editor_height)
        form.addRow("Move to", self._layout_editor_zone)
        layout.addLayout(form)
        apply_button = QPushButton("Apply selected properties")
        move_row = QHBoxLayout()
        previous_button = QPushButton("Move left / up")
        next_button = QPushButton("Move right / down")
        move_area_button = QPushButton("Move to selected bar")
        move_row.addWidget(previous_button)
        move_row.addWidget(next_button)
        move_row.addWidget(move_area_button)
        layout.addWidget(apply_button)
        layout.addLayout(move_row)
        reset = QPushButton("Reset layout")
        close = QPushButton("Done")
        buttons = QHBoxLayout()
        buttons.addWidget(reset)
        buttons.addStretch(1)
        buttons.addWidget(close)
        layout.addStretch(1)
        layout.addLayout(buttons)

        def pick_element(_index):
            name = picker.currentData()
            if name:
                self._select_layout_widget(self._layout_named_widgets[name])

        def apply_properties():
            widget = self._layout_selected_widget
            if widget is None:
                return
            self._set_layout_widget_text(widget, self._layout_editor_text.text())
            widget.setProperty("layoutTextEdited", True)
            widget.setProperty("layoutCustomText", self._layout_editor_text.text())
            visible = self._layout_editor_visible.isChecked()
            widget.setProperty("layoutUserHidden", not visible)
            widget.setVisible(visible)
            width = self._layout_editor_width.value()
            height = self._layout_editor_height.value()
            widget.setMinimumWidth(width)
            widget.setMaximumWidth(width if width else 16777215)
            widget.setMinimumHeight(height)
            widget.setMaximumHeight(height if height else 16777215)

        def shift_selected(step):
            widget = self._layout_selected_widget
            zone = self._zone_for_widget(widget)
            if zone is None:
                return
            widgets = self._layout_zones[zone][1]
            index = widgets.index(widget)
            target = max(0, min(len(widgets) - 1, index + step))
            if target != index:
                widgets.insert(target, widgets.pop(index))
                self._rebuild_layout_zone(zone)

        def move_to_zone():
            widget = self._layout_selected_widget
            destination = self._layout_editor_zone.currentText()
            if widget is None or destination not in self._layout_zones:
                return
            source = self._zone_for_widget(widget)
            if source is None:
                self._show_status(
                    "That element can be resized or hidden, but only control-bar items can move between bars",
                    "warning",
                )
                return
            self._layout_zones[source][1].remove(widget)
            self._rebuild_layout_zone(source)
            self._layout_zones[destination][1].append(widget)
            self._rebuild_layout_zone(destination)
            self._layout_editor_zone.setCurrentText(destination)

        def reset_layout():
            # Reset is authoritative: remove persisted customization now so a
            # crash or forced close cannot resurrect the previous layout.
            self.settings.remove("layout/customization_v1")
            self.settings.remove("layout/workspace_splitter")
            self.settings.remove("layout/vertical_plot_splitter")
            self.settings.remove("layout/plot_splitter")
            for name, widgets in self._layout_default_zones.items():
                self._layout_zones[name] = (
                    self._layout_zones[name][0], list(widgets)
                )
                self._rebuild_layout_zone(name)
            for name, properties in self._layout_default_properties.items():
                widget = self._layout_named_widgets[name]
                self._set_layout_widget_text(widget, properties["text"])
                widget.setMinimumSize(*properties["minimum"])
                widget.setMaximumSize(*properties["maximum"])
                widget.setProperty("layoutUserHidden", False)
                widget.setProperty("layoutTextEdited", False)
                widget.setProperty("layoutCustomText", None)
                widget.setVisible(bool(properties["visible"]))
            for widget in (
                self.connection_bar, self.save_bar, self.plot_frame,
                self.side_tabs, self.selection_panel, self.statusBar(),
                self.overview,
            ):
                widget.setVisible(True)
            self.workspace_splitter.setSizes([1120, 400])
            self.vertical_plot_splitter.setSizes([620, 180])
            self.plot_splitter.setSizes([900, 300])
            self.overview.resize(320, 190)
            QTimer.singleShot(0, self._position_overview)
            self._mode_changed()

        def editor_finished(_result=0):
            if isinstance(self._layout_selected_widget, QWidget):
                self._layout_selected_widget.setStyleSheet(self._layout_selected_style)
            self._layout_selected_widget = None
            self._layout_edit_active = False
            QApplication.instance().removeEventFilter(self)
            self._save_layout_customization()
            self._layout_editor = None

        picker.currentIndexChanged.connect(pick_element)
        apply_button.clicked.connect(apply_properties)
        previous_button.clicked.connect(lambda: shift_selected(-1))
        next_button.clicked.connect(lambda: shift_selected(1))
        move_area_button.clicked.connect(move_to_zone)
        reset.clicked.connect(reset_layout)
        close.clicked.connect(dialog.accept)
        dialog.finished.connect(editor_finished)
        self._layout_edit_active = True
        QApplication.instance().installEventFilter(self)
        dialog.show()

    def _update_overview_data(self, x_values, counts):
        if self.instrument_mode != "mcs" or len(x_values) == 0:
            return
        step_x, step_y = histogram_step_xy(x_values, counts)
        self.overview_curve.setData(step_x, step_y)
        xmin, xmax = float(x_values[0]), float(x_values[-1])
        ymin, ymax = 0.0, max(1.0, float(np.max(counts)))
        self.overview.setRange(xRange=(xmin, xmax), yRange=(ymin, ymax), padding=0.02)
        self._update_overview_zoom()

    def _initialize_mcs_view_once(self, x_values, counts):
        """Fit the first histogram explicitly, then leave its viewport alone."""
        if self.instrument_mode == "mcs" and not self.mcs_view_initialized:
            x_array = np.asarray(x_values, dtype=np.float64)
            y_array = np.asarray(counts, dtype=np.float64)
            finite_x = x_array[np.isfinite(x_array)]
            finite_y = y_array[np.isfinite(y_array)]
            if finite_x.size:
                xmin = float(np.min(finite_x))
                xmax = float(np.max(finite_x))
                if xmax <= xmin:
                    half_width = max(0.5, abs(xmin) * 0.01)
                    xmin -= half_width
                    xmax += half_width
                self.plot.getViewBox().setXRange(xmin, xmax, padding=0.02)
            ymax = max(1.0, float(np.max(finite_y)) if finite_y.size else 1.0)
            self.plot.getViewBox().setYRange(0.0, ymax, padding=0.04)
            self.plot.enableAutoRange(x=False, y=False)
            self.mcs_view_initialized = True

    def _update_overview_zoom(self, *_args):
        if self.instrument_mode != "mcs":
            return
        (xmin, xmax), _ = self.plot.getViewBox().viewRange()
        self.overview_zoom.setRegion((xmin, xmax))

    def _clock_hz(self):
        return max(1.0, float(self.clock_frequency_hz))

    def _update_clock_status(self):
        clock_hz = self._clock_hz()
        self.status_clock.setText(
            f"{clock_hz / 1e6:,.6f} MHz · {1e9 / clock_hz:,.6f} ns/clock"
        )

    def _apply_clock_frequency(self):
        old_clock = self._clock_hz()
        old_mcs_range = self.plot.getViewBox().viewRange()[0]
        self.clock_frequency_hz = self.clock_frequency_spin.value() * 1e6
        new_clock = self._clock_hz()
        if self.counter_times and old_clock > 0:
            rate_scale = new_clock / old_clock
            time_scale = old_clock / new_clock
            self.counter_cps = [value * rate_scale for value in self.counter_cps]
            self.counter_trigger_cps = [
                value * rate_scale for value in self.counter_trigger_cps
            ]
            self.counter_times = [value * time_scale for value in self.counter_times]
            self.curve.setData(self.counter_times, self.counter_cps)
            self.counter_trigger_curve.setData(
                self.counter_times, self.counter_trigger_cps
            )
            self._update_counter_stats()
        self.settings.setValue("calibration/clock_hz", self.clock_frequency_hz)
        self._update_clock_status()
        self._update_quantization()
        if self.instrument_mode == "mcs" and self.latest_metadata is not None:
            self._x_axis_changed()
            if self.x_axis_mode == "time" and new_clock > 0:
                scale = old_clock / new_clock
                self.plot.getViewBox().setXRange(
                    old_mcs_range[0] * scale,
                    old_mcs_range[1] * scale,
                    padding=0.0,
                )
            self._update_frame_stats()
            self._update_roi_metrics()
        elif self.instrument_mode == "counter":
            self._show_status(
                "Clock frequency applied; new Counter samples use the calibrated CPS scale",
                "success",
            )
            return
        self._show_status(
            f"Clock frequency set to {self.clock_frequency_hz / 1e6:,.6f} MHz",
            "success",
        )

    def _reset_clock_frequency(self):
        self.clock_frequency_spin.setValue(EXPECTED_CLOCK_HZ / 1e6)
        self._apply_clock_frequency()

    def _clear_counter_history(self):
        self.counter_times = []
        self.counter_cps = []
        self.counter_trigger_cps = []
        self.counter_last_count = 0
        self.counter_last_trigger_count = 0
        self.counter_last_pass = 0
        # Explicit Clear is the ONLY operation that starts a new Counter
        # timeline. All subsequent samples remain referenced to this t=0.
        self.counter_started = time.monotonic()
        self.counter_baseline_pending = False
        self.counter_latest_counts = None
        self.counter_latest_metadata = None
        # Let any complete frame already queued before the clear command drain.
        self.counter_accept_after = 0.0
        if self.instrument_mode == "counter":
            self.curve.setData([], [])
            self.counter_trigger_curve.setData([], [])
            self.plot_title.setText("Simple counter — waiting for data")
            self._update_counter_stats()

    def _update_counter_stats(self):
        """Update the Counter-only status panel without modifying history."""
        samples = len(self.counter_cps)
        self.counter_stat_samples.setText(f"{samples:,}")
        elapsed = self.counter_times[-1] if self.counter_times else 0.0
        self.counter_stat_elapsed.setText(self._format_time(elapsed))
        if not samples:
            for label in (
                self.counter_stat_current, self.counter_stat_mean,
                self.counter_stat_trigger_current,
                self.counter_stat_std, self.counter_stat_rsd,
                self.counter_stat_min, self.counter_stat_max,
            ):
                label.setText("—")
            return
        values = np.asarray(self.counter_cps, dtype=np.float64)
        current = float(values[-1])
        mean = float(values.mean())
        std = float(values.std(ddof=1)) if samples > 1 else 0.0
        rsd = 100.0 * std / mean if mean else 0.0
        self.counter_stat_current.setText(f"{current:,.3f} cps")
        self.counter_stat_trigger_current.setText(
            f"{self.counter_trigger_cps[-1]:,.3f} cps"
        )
        self.counter_stat_mean.setText(f"{mean:,.3f} cps")
        self.counter_stat_std.setText(f"{std:,.3f} cps")
        self.counter_stat_rsd.setText(f"{rsd:,.3f} %")
        self.counter_stat_min.setText(f"{float(values.min()):,.3f} cps")
        self.counter_stat_max.setText(f"{float(values.max()):,.3f} cps")

    def _request_counter_baseline(self):
        """Compatibility hook for independent-dwell Counter frames.

        Counter frames now already contain one complete dwell, so Start/restart
        requires no cumulative baseline frame. Keep the session time/history and
        simply accept the next completed dwell.
        """
        self.counter_baseline_pending = False
        self.counter_accept_after = 0.0
        if self.counter_started is None:
            self.counter_started = time.monotonic()

    def _update_mcs_time_axis(self, metadata):
        del metadata  # axis scaling is handled directly by pyqtgraph
        if self.x_axis_mode == "bin":
            # Pyqtgraph retains the previous SI scale when auto-prefixing is
            # disabled.  Explicitly clear the seconds unit/prefix and restore
            # a unity scale so nanosecond mode cannot leave a 1e9 multiplier
            # on raw integer bin indices.
            for plot in (self.plot, self.difference_plot):
                axis = plot.getAxis("bottom")
                axis.enableAutoSIPrefix(False)
                axis.setScale(1.0)
                axis.setLabel(text="Bin", units="", unitPrefix="")
        else:
            # Supply seconds and let pyqtgraph select ns, us, ms or s from the
            # visible range. Never pre-scale the numerical X values.
            for plot in (self.plot, self.difference_plot):
                axis = plot.getAxis("bottom")
                axis.setScale(1.0)
                axis.setLabel(
                    text="Time after trigger", units="s", unitPrefix=""
                )
                axis.enableAutoSIPrefix(True)

    def _native_mcs_x_values(self, bins, metadata):
        values = np.asarray(bins, dtype=np.float64)
        if self.x_axis_mode == "bin":
            return values
        dwell_s = metadata["dwell_cycles"] / self._clock_hz()
        return values * dwell_s

    def _mcs_x_values(self, bins, metadata):
        return self._native_mcs_x_values(bins, metadata)

    def _bin_to_plot_x(self, bin_number):
        if self.latest_metadata is None:
            return float(bin_number)
        if self.x_axis_mode == "bin":
            native = float(bin_number)
        else:
            dwell_s = self.latest_metadata["dwell_cycles"] / self._clock_hz()
            native = float(bin_number) * dwell_s
        return native

    @staticmethod
    def _format_si_seconds(value):
        value = float(value)
        magnitude = abs(value)
        for scale, prefix in (
            (1.0, "s"), (1e-3, "ms"), (1e-6, "µs"),
            (1e-9, "ns"), (1e-12, "ps"),
        ):
            if magnitude >= scale or scale == 1e-12:
                return f"{value / scale:.6g} {prefix}"

    def _highlight_plot_section(self, x_start, x_end, finished=False):
        if (self.instrument_mode == "counter" or self.latest_counts is None or
                self.latest_metadata is None or self.latest_counts.size == 0):
            self.selection_info.setText(
                "A histogram must be displayed before a section can be highlighted"
            )
            return
        dwell_s = self.latest_metadata["dwell_cycles"] / self._clock_hz()
        if self.x_axis_mode == "bin":
            first_bin = math.floor(x_start)
            last_bin = math.floor(x_end)
        else:
            first_bin = math.floor(x_start / dwell_s)
            last_bin = math.floor(x_end / dwell_s)
        last_valid = self.latest_counts.size - 1
        first_bin = max(0, min(first_bin, last_valid))
        last_bin = max(0, min(last_bin, last_valid))
        if last_bin < first_bin:
            first_bin, last_bin = last_bin, first_bin
        self.highlighted_bins = (first_bin, last_bin)
        self._update_highlighted_section()
        if finished:
            self._show_status(
                f"Highlighted bins {first_bin:,}–{last_bin:,}", "success"
            )

    def _update_counter_highlight(self):
        """Show the currently selected Counter sample and its value."""
        if not self.counter_times or self.highlighted_bins is None:
            self.highlight_region.setVisible(False)
            return
        index = max(
            0, min(int(self.highlighted_bins[0]), len(self.counter_times) - 1)
        )
        self.highlighted_bins = (index, index)
        x = float(self.counter_times[index])
        cps = float(self.counter_cps[index])

        # A narrow vertical marker centered on the saved counter sample.
        if len(self.counter_times) > 1:
            if index == 0:
                width = abs(self.counter_times[1] - self.counter_times[0])
            else:
                width = abs(self.counter_times[index] - self.counter_times[index - 1])
            width = max(width * 0.20, 1e-9)
        else:
            width = 1e-6

        self.highlight_region.setRegion((x - width / 2, x + width / 2))
        self.highlight_region.setVisible(True)
        self.selection_info.setText(
            f"Counter sample {index + 1:,} of {len(self.counter_times):,}  |  "
            f"Time {self._format_si_seconds(x)}\n"
            f"Count rate {cps:,.3f} cps"
        )
        self._show_status(
            f"Selected counter sample {index + 1:,}: {cps:,.3f} cps", "success"
        )

    def _zoom_around_selected_bin(self, axis, factor):
        """Zoom one axis around the selected histogram bin."""
        if self.instrument_mode == "counter":
            if not self.counter_times or self.highlighted_bins is None:
                self._show_status(
                    "Select a counter point first with Ctrl+left-click, then use Ctrl+arrow to zoom",
                    "warning"
                )
                return
            selected_index = max(
                0, min(int(self.highlighted_bins[0]), len(self.counter_times) - 1)
            )
            center_x = float(self.counter_times[selected_index])
            center_y = float(self.counter_cps[selected_index])
            view_box = self.plot.getViewBox()
            (x_min, x_max), (_y_min, y_max) = view_box.viewRange()

            if axis == "x":
                x_half = max((x_max - x_min) * 0.5 * factor, 1e-12)
                view_box.setXRange(
                    center_x - x_half, center_x + x_half, padding=0.0
                )
                self._show_status(
                    "Counter X axis zoomed around selected point", "success"
                )
                return

            # CPS is nonnegative, so keep zero permanently visible just like MCS.
            current_top = max(float(y_max), 1e-12)
            new_top = max(current_top * factor, center_y * 1.05, 1.0)
            view_box.setYRange(0.0, new_top, padding=0.0)
            self._show_status(
                "Counter Y axis zoomed with zero baseline fixed", "success"
            )
            return

        if (self.latest_counts is None or
                self.latest_metadata is None or
                self.latest_counts.size == 0 or
                self.highlighted_bins is None):
            self._show_status(
                "Select a bin first with Ctrl+left-click, then use Ctrl+arrow to zoom",
                "warning"
            )
            return

        selected_bin = int(self.highlighted_bins[0])
        center_x = self._bin_to_plot_x(selected_bin + 0.5)
        center_y = float(self.latest_counts[selected_bin])

        view_box = self.plot.getViewBox()
        (x_min, x_max), (y_min, y_max) = view_box.viewRange()

        if axis == "x":
            # Horizontal shortcut changes ONLY horizontal magnification.
            x_half = max((x_max - x_min) * 0.5 * factor, 1e-30)
            view_box.setXRange(
                center_x - x_half, center_x + x_half,
                padding=0.0
            )
            self._show_status(
                "X axis zoomed around selected bin", "success"
            )
            return

        # Vertical zoom changes only Y. Raw MCS counts have a physical
        # zero baseline, so Y=0 always remains visible at the bottom.
        current_top = max(float(y_max), 1e-12)
        new_top = max(current_top * factor, center_y * 1.05, 1.0)
        view_box.setYRange(0.0, new_top, padding=0.0)
        self._show_status("Y axis zoomed with zero baseline fixed", "success")

    def _move_highlighted_bin(self, step):
        """Move the selected histogram bin by one bin with the arrow keys."""
        if self.instrument_mode == "counter":
            if not self.counter_times:
                return
            if self.highlighted_bins is None:
                selected_index = len(self.counter_times) - 1
            else:
                selected_index = int(self.highlighted_bins[0])
            selected_index = max(
                0, min(selected_index + int(step), len(self.counter_times) - 1)
            )
            self.highlighted_bins = (selected_index, selected_index)
            self._update_counter_highlight()
            return

        if (self.latest_counts is None or
                self.latest_metadata is None or
                self.latest_counts.size == 0):
            return

        # Arrow movement is specifically a single-bin inspection tool.
        # If a range is highlighted, start from its first bin.
        if self.highlighted_bins is None:
            selected_bin = 0
        else:
            selected_bin = int(self.highlighted_bins[0])

        selected_bin = max(
            0,
            min(selected_bin + int(step), self.latest_counts.size - 1)
        )

        self.highlighted_bins = (selected_bin, selected_bin)
        self._update_highlighted_section()
        self._show_status(f"Highlighted bin {selected_bin:,}", "success")

    def _plot_mouse_clicked(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return

        ctrl = bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier)

        if ctrl:
            # Ctrl + left-click selects a histogram bin or the nearest Counter
            # sample, depending on the active instrument mode.
            scene_pos = event.scenePos()
            if not self.plot.sceneBoundingRect().contains(scene_pos):
                return
            view_pos = self.plot.getViewBox().mapSceneToView(scene_pos)
            x = float(view_pos.x())

            if self.instrument_mode == "counter":
                if not self.counter_times:
                    return
                selected_index = int(
                    np.argmin(np.abs(np.asarray(self.counter_times) - x))
                )
                self.highlighted_bins = (selected_index, selected_index)
                self._update_counter_highlight()
                return

            if (self.latest_counts is None or
                    self.latest_metadata is None or self.latest_counts.size == 0):
                return

            if self.x_axis_mode == "bin":
                selected_bin = int(math.floor(x))
            else:
                dwell_s = self.latest_metadata["dwell_cycles"] / self._clock_hz()
                if dwell_s <= 0: return
                selected_bin = int(math.floor(x / dwell_s))

            selected_bin = max(
                0, min(selected_bin, self.latest_counts.size - 1)
            )
            self.highlighted_bins = (selected_bin, selected_bin)
            self._update_highlighted_section()
            self._show_status(f"Highlighted bin {selected_bin:,}", "success")
            return

        # Plain left-click clears only the highlighted inspection selection.
        self.highlighted_bins = None
        self.highlight_region.setVisible(False)
        self.selection_info.setText(
            "Hold Ctrl and left-click a bin, or Ctrl+drag across a range"
        )

    def _update_highlighted_section(self):
        if (self.highlighted_bins is None or self.latest_counts is None or
                self.latest_metadata is None or self.instrument_mode == "counter"):
            self.highlight_region.setVisible(False)
            return
        last_valid = self.latest_counts.size - 1
        first_bin = max(0, min(self.highlighted_bins[0], last_valid))
        last_bin = max(first_bin, min(self.highlighted_bins[1], last_valid))
        self.highlighted_bins = (first_bin, last_bin)
        self.highlight_region.setRegion((
            self._bin_to_plot_x(first_bin),
            self._bin_to_plot_x(last_bin + 1),
        ))
        self.highlight_region.setVisible(True)

        selected = self.latest_counts[first_bin:last_bin + 1]
        total = int(selected.sum(dtype=np.uint64))
        bin_count = int(selected.size)
        peak_offset = int(np.argmax(selected))
        peak_bin = first_bin + peak_offset
        peak_counts = int(selected[peak_offset])
        mean = total / bin_count
        dwell_s = self.latest_metadata["dwell_cycles"] / self._clock_hz()
        start_s = first_bin * dwell_s
        end_s = (last_bin + 1) * dwell_s
        duration_s = end_s - start_s
        if total:
            indices = np.arange(first_bin, last_bin + 1, dtype=np.float64)
            centroid_bin = float(np.dot(indices, selected.astype(np.float64)) / total)
            centroid = (f"{centroid_bin:,.3f} "
                        f"({self._format_si_seconds(centroid_bin * dwell_s)})")
        else:
            centroid = "—"
        reference_compatible = (
            self.reference_counts is not None and
            self.reference_metadata is not None and
            self.reference_counts.size == self.latest_counts.size and
            self.reference_metadata.get("dwell_cycles") == self.latest_metadata.get("dwell_cycles") and
            self.reference_metadata.get("clock_hz") == self.latest_metadata.get("clock_hz")
        )
        if first_bin == last_bin and reference_compatible:
            current_count = int(self.latest_counts[first_bin])
            reference_count = int(self.reference_counts[first_bin])
            current_passes = max(1, int(self.accumulation_frames))
            reference_passes = max(1, int(self.reference_metadata.get("pass_count", 1)))
            current_norm = current_count / current_passes
            reference_norm = reference_count / reference_passes
            delta_norm = current_norm - reference_norm
            if reference_norm > 0:
                ratio_text = f"{current_norm/reference_norm:,.5g}"
                percent_text = f"{100.0*delta_norm/reference_norm:+,.3f}%"
            else:
                ratio_text = "—"
                percent_text = "—"
            self.selection_info.setText(
                f"Bin {first_bin:,}  |  Time {self._format_si_seconds(start_s)}  |  "
                f"Current {current_count:,} ({current_norm:,.5g}/pass)  |  "
                f"Reference {reference_count:,} ({reference_norm:,.5g}/pass)\n"
                f"Δ {delta_norm:+,.5g}/pass  |  Ratio {ratio_text}  |  Change {percent_text}"
            )
        else:
            self.selection_info.setText(
                f"Bins {first_bin:,}–{last_bin:,} ({bin_count:,})  |  "
                f"Time {self._format_si_seconds(start_s)}–"
                f"{self._format_si_seconds(end_s)}  |  "
                f"Width {self._format_si_seconds(duration_s)}\n"
                f"Sum {total:,}  |  Mean {mean:,.3f}/bin  |  "
                f"Peak {peak_counts:,} at bin {peak_bin:,}  |  Centroid {centroid}"
            )

    def _x_axis_changed(self, _index=None):
        old_mode = self.x_axis_mode
        new_mode = self.x_axis_combo.currentData() or "time"
        old_range = self.plot.getViewBox().viewRange()[0]
        self.x_axis_mode = new_mode
        if self.instrument_mode == "counter":
            return
        metadata, counts = self.latest_metadata, self.latest_counts
        if metadata is None or counts is None:
            self._update_mcs_time_axis({})
            return
        self._update_mcs_time_axis(metadata)
        x_values = self._mcs_x_values(np.arange(counts.size), metadata)
        step_x, step_y = histogram_step_xy(x_values, counts)
        self.curve.setData(step_x, step_y)
        # Preserve the same physical/bin region, not the old raw numbers. A
        # seconds-scale range reused as bin numbers made the plot appear blank.
        if old_mode != new_mode:
            dwell_s = metadata["dwell_cycles"] / self._clock_hz()
            if dwell_s > 0:
                if old_mode == "time" and new_mode == "bin":
                    converted = (old_range[0] / dwell_s, old_range[1] / dwell_s)
                else:
                    converted = (old_range[0] * dwell_s, old_range[1] * dwell_s)
                self.plot.getViewBox().setXRange(*converted, padding=0.0)
        self._set_region_bounds(counts.size)
        self._update_roi_metrics()
        self._update_highlighted_section()
        self._update_reference_curve()
        self._update_overview_data(
            self._mcs_x_values(np.arange(counts.size), metadata), counts
        )

    def reset_histogram(self, _checked=False, silent=False):
        self._cancel_live_preview()
        if not silent:
            self.send("z")
        if self.instrument_mode == "mcs":
            # Ignore any pre-clear preview/final frame already in the serial
            # pipeline. The next explicit Start begins the new zeroed session.
            self.mcs_clear_pending = True
        self.frame_timestamps.clear()
        self.frame_rate_ema = 0.0
        if self.instrument_mode == "counter":
            self._clear_counter_history()
            self.latest_counts = None
            self.latest_metadata = None
            self._set_connected(self.worker is not None)
            if not silent:
                self._show_status(
                    "Counter session cleared — waiting for a new FPGA baseline",
                    "success",
                )
            return
        self.latest_counts = None
        self.latest_metadata = None
        self.mcs_latest_counts = None
        self.mcs_latest_metadata = None
        self.mcs_accumulation_frames = 0
        self.accumulation_frames = 0
        self.accumulation_signature = None
        self.mcs_rate_times.clear()
        self.mcs_count_rates.clear()
        self.mcs_pass_rates.clear()
        self.mcs_rate_last_time = None
        self.mcs_rate_last_total = None
        self.mcs_rate_last_passes = None
        self.mcs_rate_started = None
        self.mcs_rate_fit_samples.clear()
        self.mcs_rate_fit_signature = None
        self.rate_curve.setData([], [])
        self.curve.setData([], [])
        self.mcs_view_initialized = False
        self.status_frame_kind.setText("NO DATA")
        self.highlight_region.setVisible(False)
        self.selection_info.setText(
            "Hold Ctrl and left-click a bin, or Ctrl+drag across a range"
        )
        self.plot_title.setText("Histogram — waiting for data")
        self._clear_metrics()
        self._set_connected(self.worker is not None)
        if not silent:
            self._show_status("FPGA accumulation clear requested", "success")

    def _cancel_live_preview(self):
        self.live_preview_timer.stop()
        self._live_preview_metadata = None
        self._live_preview_warning = False

    def _render_live_preview(self):
        metadata = self._live_preview_metadata
        warning = self._live_preview_warning
        self._live_preview_metadata = None
        self._live_preview_warning = False
        if (metadata is None or self.instrument_mode != "mcs" or
                self.mcs_clear_pending or self.mcs_display_frozen or
                self.latest_counts is None):
            return
        self._update_mcs_time_axis(metadata)
        self._set_region_bounds(int(metadata["bin_count"]))
        x_values = self._mcs_x_values(np.arange(int(metadata["bin_count"])), metadata)
        step_x, step_y = histogram_step_xy(x_values, self.latest_counts)
        self.curve.setData(step_x, step_y)
        self._update_live_manual_autorange()
        self._initialize_mcs_view_once(x_values, self.latest_counts)
        self._update_overview_data(x_values, self.latest_counts)
        self.plot_title.setText(
            "Histogram"
        )
        self.status_frame_kind.setText(
            f"LIVE · pass {int(metadata['pass_count']):,} · bins "
            f"{int(metadata['block_start']):,}–"
            f"{int(metadata['block_start']) + int(metadata['block_count']) - 1:,}"
        )
        self._update_roi_metrics()
        self._update_highlighted_section()
        self.save_csv_button.setEnabled(True)
        self.save_plot_button.setEnabled(True)
        self.reset_button.setEnabled(True)
        self._show_status(
            "Live display fell behind; acquisition is intact and the completed final frame will resynchronize it"
            if warning else "Live preview updated",
            "warning" if warning else "success",
        )

    def handle_frame(self, counts, metadata):
        self.pass_number += 1
        if not metadata["total_ok"]:
            self.rejected_frames += 1
            self._show_status("Rejected frame: FPGA total does not match received bins", "error")
            return

        # Route by the FPGA's frame-type flag, not by whichever screen happens
        # to be selected when a queued USB frame reaches Qt. This prevents a
        # late Counter frame from replacing an MCS histogram (and vice versa)
        # during mode changes. MCB1 preview blocks are always MCS data.
        frame_is_counter = bool(metadata.get("counter_frame", False))
        if frame_is_counter != (self.instrument_mode == "counter"):
            return

        if self.instrument_mode == "mcs" and self.mcs_clear_pending:
            return

        # After Stop, discard any preview still in flight. Only the explicitly
        # flagged quiescent final frame is allowed to replace the display.
        if self.instrument_mode == "mcs" and self.mcs_display_frozen:
            if metadata.get("live_block") or not metadata.get("atomic_final"):
                return

        if not metadata.get("live_block"):
            # Full frames, including the authoritative final frame, paint now.
            self._cancel_live_preview()

        if metadata.get("live_block"):
            # MCB1 contains absolute FPGA BRAM values for one committed range.
            # Merge those values directly; never add them, because the BRAM is
            # already cumulative across passes. The completed final MCS6 frame
            # remains authoritative and replaces this preview after stopping.
            if self.instrument_mode == "counter":
                return
            total_bins = int(metadata["bin_count"])
            start = int(metadata["block_start"])
            length = int(metadata["block_count"])
            signature = (total_bins, metadata["dwell_cycles"], metadata["clock_hz"])
            if (self.mcs_latest_counts is None or
                    self.mcs_latest_counts.size != total_bins or
                    self.accumulation_signature != signature):
                self.mcs_latest_counts = np.zeros(total_bins, dtype=np.uint64)
                self.latest_counts = np.zeros(total_bins, dtype=np.uint64)
            self.mcs_latest_counts[start:start + length] = counts
            self.latest_counts[start:start + length] = counts
            self.accumulation_signature = signature
            self.configured_signature = (total_bins, metadata["dwell_cycles"])
            display_metadata = metadata.copy()
            display_metadata["pass_count"] = self.mcs_accumulation_frames
            self.latest_metadata = display_metadata
            self.mcs_latest_metadata = display_metadata.copy()

            self._live_preview_metadata = metadata.copy()
            self._live_preview_warning |= bool(metadata.get("live_stream_overflow"))
            if not self.live_preview_timer.isActive():
                self.live_preview_timer.start()
            return

        if self.instrument_mode == "counter":
            if metadata["bin_count"] != 2:
                return

            # New Simple Counter protocol: every MCS6 Counter frame is ONE
            # completed, independent dwell. counts[0]/counts[1] are not
            # cumulative and must never be differenced against the prior frame.
            segment_signal_count = int(counts[0])
            segment_trigger_count = int(counts[1])
            sample_number = int(metadata["pass_count"])
            segment_dwell_s = metadata["dwell_cycles"] / self._clock_hz()
            if segment_dwell_s <= 0.0:
                return

            # pass_count is now the monotonically increasing Counter sample ID.
            # Preserve the physical time axis and make a missed FIFO/UART sample
            # visible as a time gap instead of silently compressing the trace.
            if self.counter_last_pass > 0:
                sample_step = sample_number - self.counter_last_pass
                if sample_step <= 0:
                    # Clear/reconfigure restarted the sample numbering.
                    sample_step = 1
                elif sample_step > 1:
                    self._show_status(
                        f"Counter sample gap: expected {self.counter_last_pass + 1:,}, "
                        f"received {sample_number:,}", "warning"
                    )
            else:
                sample_step = 1

            # A requested Counter dwell can exceed the FPGA's 32-bit cycle
            # field. Combine consecutive complete FPGA segments into one GUI
            # sample; counts are summed before division by total active time.
            if sample_step > 1 and self.counter_segment_count:
                self.counter_segment_count = 0
                self.counter_segment_signal_sum = 0
                self.counter_segment_trigger_sum = 0
                self.counter_segment_elapsed = 0.0
            self.counter_last_pass = sample_number
            self.counter_segment_elapsed += sample_step * segment_dwell_s
            self.counter_segment_signal_sum += segment_signal_count
            self.counter_segment_trigger_sum += segment_trigger_count
            self.counter_segment_count += 1
            if self.counter_segment_count < self.counter_segments_per_point:
                self.status_frame_kind.setText(
                    f"COUNTER LIVE · segment {self.counter_segment_count}/"
                    f"{self.counter_segments_per_point}"
                )
                return

            signal_count = self.counter_segment_signal_sum
            trigger_count = self.counter_segment_trigger_sum
            dwell_s = segment_dwell_s * self.counter_segments_per_point
            elapsed_step = self.counter_segment_elapsed
            self.counter_segment_count = 0
            self.counter_segment_signal_sum = 0
            self.counter_segment_trigger_sum = 0
            self.counter_segment_elapsed = 0.0

            cps = signal_count / dwell_s
            trigger_cps = trigger_count / dwell_s
            elapsed = (self.counter_times[-1] if self.counter_times else 0.0) + elapsed_step
            self.counter_times.append(elapsed)
            self.counter_cps.append(cps)
            self.counter_trigger_cps.append(trigger_cps)
            self.counter_last_count = signal_count
            self.counter_last_trigger_count = trigger_count

            self.latest_metadata = metadata.copy()
            self.latest_counts = counts.astype(np.uint64)
            self.counter_latest_metadata = self.latest_metadata.copy()
            self.counter_latest_counts = self.latest_counts.copy()
            self.curve.setData(self.counter_times, self.counter_cps)
            self.counter_trigger_curve.setData(self.counter_times, self.counter_trigger_cps)
            self._update_counter_trace_visibility()
            # Bound horizontal navigation to the time interval that actually exists.
            # A tiny non-zero range is used before the second point arrives.
            if self.counter_times:
                counter_xmax = max(float(self.counter_times[-1]), 1e-9)
                self.plot.setLimits(xMin=0.0, xMax=counter_xmax, yMin=0.0)
                self.counter_trigger_view.setLimits(xMin=0.0, xMax=counter_xmax, yMin=0.0)
            # Counter is a live time-series: keep all axes fitted while it grows.
            self.plot.getViewBox().autoRange(padding=0.05)
            if self.counter_trigger_check.isChecked():
                self.counter_trigger_view.autoRange(padding=0.05)
            self.plot_title.setText(
                f"Simple counter — signal {cps:,.3f} cps · trigger {trigger_cps:,.3f} cps"
            )
            self.status_frame_kind.setText(
                f"COUNTER LIVE · sample {sample_number:,}"
            )
            self.save_csv_button.setEnabled(True)
            self.save_plot_button.setEnabled(True)
            self._update_counter_stats()
            return
        previous_signature = self.accumulation_signature
        self.latest_counts = counts.astype(np.uint64)
        self.accumulation_frames = metadata["pass_count"]
        self.accumulation_signature = (
            metadata["bin_count"], metadata["dwell_cycles"], metadata["clock_hz"]
        )
        self.configured_signature = (
            metadata["bin_count"], metadata["dwell_cycles"]
        )
        self.latest_metadata = metadata.copy()
        self.mcs_latest_counts = self.latest_counts.copy()
        self.mcs_latest_metadata = self.latest_metadata.copy()
        self.mcs_accumulation_frames = self.accumulation_frames
        now = time.monotonic()
        if self.acquisition_running and not metadata.get("atomic_final"):
            fpga_passes = int(self.accumulation_frames)
            if not self.eta_pass_samples:
                self.eta_pass_samples.append((now, fpga_passes))
            else:
                previous_passes = int(self.eta_pass_samples[-1][1])
                if fpga_passes < previous_passes:
                    # FPGA pass counter reset/rebased: start a fresh estimator.
                    self.eta_pass_samples.clear()
                    self.eta_last_rate = None
                    self.eta_pass_samples.append((now, fpga_passes))
                elif fpga_passes > previous_passes:
                    # Duplicate pass counts carry no new rate information.
                    self.eta_pass_samples.append((now, fpga_passes))

            # Keep a TIME window, not a frame-count window.
            cutoff = now - self.eta_window_seconds
            while len(self.eta_pass_samples) > 3 and self.eta_pass_samples[0][0] < cutoff:
                self.eta_pass_samples.popleft()
        if (self.acquisition_target_passes is not None and
                self.accumulation_frames >= self.acquisition_target_passes):
            self.acquisition_running = False
            self._finish_acquisition_timing()
        self._update_acquisition_timing()
        self.frame_timestamps.append(now)
        while (len(self.frame_timestamps) > 2 and
               now - self.frame_timestamps[0] > 2.0):
            self.frame_timestamps.popleft()
        if len(self.frame_timestamps) >= 2:
            elapsed = self.frame_timestamps[-1] - self.frame_timestamps[0]
            if elapsed >= 0.10:
                self.frame_rate_ema = (len(self.frame_timestamps) - 1) / elapsed
        bins = metadata["bin_count"]
        self._update_mcs_time_axis(metadata)
        self._set_region_bounds(bins)
        dwell_s = metadata["dwell_cycles"] / self._clock_hz()
        x_values = self._mcs_x_values(np.arange(bins), metadata)
        step_x, step_y = histogram_step_xy(x_values, self.latest_counts)
        self.curve.setData(step_x, step_y)
        # Hard X navigation bounds are the actual histogram bin edges. This prevents
        # panning/zooming into empty coordinates outside the acquired spectrum.
        if step_x.size >= 2:
            data_xmin = float(step_x[0])
            data_xmax = float(step_x[-1])
            if data_xmax > data_xmin:
                self.plot.setLimits(xMin=data_xmin, xMax=data_xmax, yMin=0.0)
        self._update_live_manual_autorange()
        self._initialize_mcs_view_once(x_values, self.latest_counts)
        self._update_overview_data(x_values, self.latest_counts)
        # Draw the scientific data first. Auxiliary CPS processing must never
        # be able to prevent the histogram from appearing.
        self._update_mcs_live_rates(metadata, self.latest_counts, now)
        # Frame arrival never owns the viewport. X/Y ranges are controlled only
        # by the user (mouse/keyboard) or an explicit display command.
        self.plot_title.setText(
            "Self-test" if self.self_test_check.isChecked() else "Histogram"
        )
        self.status_frame_kind.setText(
            f"{'COMPLETED' if metadata.get('atomic_final') else 'LIVE'} · "
            f"{self.accumulation_frames:,} passes"
        )
        self._update_frame_stats()
        self._update_roi_metrics()
        self._update_highlighted_section()
        self.save_csv_button.setEnabled(True)
        self.save_plot_button.setEnabled(True)
        self.reset_button.setEnabled(True)
        if metadata.get("trigger_queue_overflow"):
            self._show_status(
                "Critical: FPGA trigger queue overflowed; acquisition data is incomplete",
                "error",
            )
        elif metadata.get("fifo_overflow"):
            self._show_status(
                "Critical: FPGA completed-bin FIFO overflowed; acquisition data is incomplete",
                "error",
            )
        elif metadata.get("accumulator_overflow"):
            self._show_status(
                "Warning: at least one FPGA histogram bin saturated at 32 bits",
                "error",
            )
        else:
            self._show_status(
                "Completed frame verified" if metadata.get("atomic_final")
                else "Live preview verified",
                "success",
            )

    def _update_frame_stats(self):
        metadata, counts = self.latest_metadata, self.latest_counts
        if metadata is None or counts is None:
            return
        dwell_s = metadata["dwell_cycles"] / self._clock_hz()
        if metadata.get("atomic_final"):
            self.status_frame_kind.setText(
                f"COMPLETED · {self.accumulation_frames:,} passes"
            )
        elif not metadata.get("live_block"):
            self.status_frame_kind.setText(
                f"LIVE · {self.accumulation_frames:,} passes"
            )
        self.frame_passes.setText(f"{self.accumulation_frames:,}")
        self.frame_bins.setText(f"{metadata['bin_count']:,}")
        self.frame_dwell.setText(f"{dwell_s * 1e9:,.3f} ns ({metadata['dwell_cycles']:,} clocks)")
        self.frame_span.setText(self._format_time(dwell_s * metadata["bin_count"]))
        self.frame_total.setText(f"{int(counts.sum(dtype=np.uint64)):,}")
        frame_bytes = 19 + metadata["bin_count"] * 4 + 8 + 4
        wire_seconds = frame_bytes * 10 / 5_000_000
        maximum_rate = 1.0 / wire_seconds
        # Qt can deliver queued frames in a burst after a UI stall. Never show
        # that delivery burst as a physically impossible UART frame rate.
        displayed_rate = min(self.frame_rate_ema, maximum_rate)
        self.frame_rate.setText(
            f"{displayed_rate:,.2f} frames/s" if displayed_rate else "—"
        )
        self.frame_wire_limit.setText(f"{maximum_rate:,.2f} frames/s")
        self.frame_bytes.setText(f"{frame_bytes:,} bytes")
        invalid = metadata.get("invalid_frames", 0)
        if metadata.get("trigger_queue_overflow"):
            self.frame_integrity.setText("TRIGGER QUEUE OVERFLOW · data incomplete")
        elif metadata.get("fifo_overflow"):
            self.frame_integrity.setText("FIFO OVERFLOW · data incomplete")
        elif metadata.get("accumulator_overflow"):
            self.frame_integrity.setText("SATURATED · clear accumulation")
        else:
            self.frame_integrity.setText(
                "OK" if invalid == 0 and self.rejected_frames == 0
                else f"OK · parser resyncs {invalid:,} · rejected {self.rejected_frames:,}"
            )

    def _set_region_bounds(self, bins):
        maximum = max(0, bins - 1)
        for spin in (self.signal_start, self.signal_end, self.noise_start, self.noise_end):
            spin.setMaximum(maximum)
        self.signal_region.setBounds((0, self._bin_to_plot_x(bins)))
        self.noise_region.setBounds((0, self._bin_to_plot_x(bins)))
        self._spins_changed()

    def _region_dragged(self, which):
        if self._updating_regions:
            return
        region = self.signal_region if which == "signal" else self.noise_region
        start_spin, end_spin = (
            (self.signal_start, self.signal_end) if which == "signal"
            else (self.noise_start, self.noise_end)
        )
        low, high = sorted(region.getRegion())
        maximum = start_spin.maximum()
        bin_width = max(self._bin_to_plot_x(1), 1e-30)
        start = max(0, min(maximum, int(math.floor(low / bin_width))))
        end = max(start, min(maximum, int(math.ceil(high / bin_width) - 1)))
        self._updating_regions = True
        with QSignalBlocker(start_spin), QSignalBlocker(end_spin):
            start_spin.setValue(start)
            end_spin.setValue(end)
        region.setRegion((self._bin_to_plot_x(start), self._bin_to_plot_x(end + 1)))
        self._updating_regions = False
        self._update_roi_metrics()

    def _spins_changed(self):
        if self._updating_regions:
            return
        self._updating_regions = True
        for start_spin, end_spin, region in (
            (self.signal_start, self.signal_end, self.signal_region),
            (self.noise_start, self.noise_end, self.noise_region),
        ):
            if start_spin.value() > end_spin.value():
                with QSignalBlocker(end_spin):
                    end_spin.setValue(start_spin.value())
            region.setRegion((self._bin_to_plot_x(start_spin.value()),
                              self._bin_to_plot_x(end_spin.value() + 1)))
        self._updating_regions = False
        self._update_roi_metrics()

    def reset_regions(self):
        bins = self.latest_counts.size if self.latest_counts is not None else self.bins_spin.value()
        values = (
            (self.signal_start, max(0, bins // 10)),
            (self.signal_end, max(0, bins * 3 // 10 - 1)),
            (self.noise_start, max(0, bins * 7 // 10)),
            (self.noise_end, max(0, bins * 9 // 10 - 1)),
        )
        self._updating_regions = True
        for spin, value in values:
            with QSignalBlocker(spin):
                spin.setValue(value)
        self._updating_regions = False
        self._spins_changed()

    def _update_roi_metrics(self):
        if (
            not self.roi_enable_check.isChecked()
            or self.latest_counts is None
            or self.latest_metadata is None
        ):
            return
        metrics = calculate_region_metrics(
            self.latest_counts, self.signal_start.value(), self.signal_end.value(),
            self.noise_start.value(), self.noise_end.value(),
        )
        self.metric_signal.set_value(f"{metrics.gross_signal:,}")
        self.metric_background.set_value(f"{metrics.estimated_background:,.1f}")
        self.metric_snr.set_value(f"{metrics.snr:,.3f}")
        self.metric_noise.set_value(f"{metrics.noise_per_bin:,.3f}")
        dwell_s = self.latest_metadata["dwell_cycles"] / self._clock_hz()
        exposure = self.accumulation_frames * metrics.signal_bins * dwell_s
        net_rate = metrics.net_signal / exposure if exposure > 0 else 0.0
        self.metric_net.set_value(f"{net_rate:,.4g} cps")
        self.metric_rate.set_value(f"{metrics.net_signal:,.1f}")
        self._update_difference_plot(metrics, dwell_s)
        overlap = not (metrics.signal_end < metrics.noise_start or metrics.noise_end < metrics.signal_start)
        detail = (
            f"Signal bins {metrics.signal_start:,}–{metrics.signal_end:,} ({metrics.signal_bins:,}) · "
            f"Noise bins {metrics.noise_start:,}–{metrics.noise_end:,} ({metrics.noise_bins:,}) · "
            f"Net uncertainty ±{metrics.uncertainty:,.2f}"
        )
        if overlap:
            detail += " · Warning: regions overlap"
            self.roi_detail.setStyleSheet(f"color:{AMBER}")
        else:
            self.roi_detail.setStyleSheet("")
        self.roi_detail.setText(detail)

    def _update_difference_plot(self, metrics, dwell_s):
        if not self.difference_button.isChecked() or self.accumulation_frames <= 0:
            return
        first, last = metrics.signal_start, metrics.signal_end
        signal_counts = self.latest_counts[first:last + 1].astype(np.float64)
        exposure_per_bin = self.accumulation_frames * dwell_s
        difference_cps = (signal_counts - metrics.noise_per_bin) / exposure_per_bin
        bins = np.arange(first, last + 1, dtype=np.float64)
        x_values = self._mcs_x_values(bins, self.latest_metadata)
        self.difference_curve.setData(x_values, difference_cps)
        if self.difference_plot.isVisible():
            self.difference_plot.getViewBox().autoRange(padding=0.04)

    def _set_difference_enabled(self, enabled):
        enabled = bool(enabled and self.roi_enable_check.isChecked() and
                       self.instrument_mode == "mcs")
        if not enabled and self.difference_button.isChecked():
            with QSignalBlocker(self.difference_button):
                self.difference_button.setChecked(False)
        self.difference_button.setText("Difference: ON" if enabled else "Difference: OFF")
        self.difference_plot.setVisible(enabled)
        if enabled:
            self.plot_splitter.setSizes([1, 1])
            self._update_roi_metrics()

    def _clear_metrics(self):
        for metric in (self.metric_signal, self.metric_background, self.metric_net, self.metric_snr, self.metric_noise, self.metric_rate):
            metric.set_value("—")
        self.roi_detail.setText("No histogram available")
        for label in (self.frame_passes, self.frame_bins, self.frame_dwell,
                      self.frame_span, self.frame_total, self.frame_rate,
                      self.frame_wire_limit, self.frame_bytes, self.frame_integrity):
            label.setText("—")

    def _set_roi_enabled(self, enabled):
        """Pure display toggle for ROI overlays; never modifies acquired data."""
        enabled = bool(enabled)

        # This function intentionally does NOT:
        # - send any command to Teensy/FPGA
        # - clear/reconfigure acquisition
        # - change histogram/rate history
        # - change ROI limits or calculated values
        # - clear the Difference curve
        # It only controls visibility.
        self.roi_enable_check.setText("ROI: ON" if enabled else "ROI: OFF")
        self.signal_region.setVisible(enabled)
        self.noise_region.setVisible(enabled)

        if enabled:
            self.side_tabs.setCurrentIndex(1)
            self.difference_button.setEnabled(self.instrument_mode == "mcs")
            self._update_roi_metrics()
            self._show_status("ROI display ON")
        else:
            # Hide the Difference view without changing its check state or data.
            # If ROI is turned back on, the previous Difference selection/data
            # remains available.
            self.difference_plot.setVisible(False)
            self.side_tabs.setCurrentIndex(0)
            self._show_status("ROI display OFF")

    def _freeze_reference_histogram(self):
        """Freeze the current MCS histogram as a translucent comparison trace."""
        if (self.instrument_mode != "mcs" or
                self.latest_counts is None or self.latest_metadata is None):
            self._show_status("Display an MCS histogram before freezing a reference", "warning")
            return

        self.reference_counts = self.latest_counts.copy()
        self.reference_metadata = self.latest_metadata.copy()
        self.reference_metadata["pass_count"] = int(self.accumulation_frames)

        if self.reference_curve is None:
            self.reference_curve = self.plot.plot(
                pen=pg.mkPen(QColor(255, 180, 84, 105), width=1.0),
                connect="finite", skipFiniteCheck=True
            )
            self.reference_curve.setZValue(2)
            self.reference_curve.setClipToView(True)
            self.reference_curve.setDownsampling(auto=True, method="peak")
            self.plot.removeItem(self.reference_curve)
            self.plot.addItem(self.reference_curve, ignoreBounds=True)

        self._update_reference_curve(force=True)
        self.clear_reference_button.setEnabled(True)
        self.reference_button.setText("Replace reference")
        self._show_status(
            f"Reference frozen at {self.accumulation_frames:,} pass(es) · "
            "new acquisition will plot over it",
            "success"
        )

    def _update_reference_curve(self, force=False):
        """Display frozen reference cheaply; rebuild only when necessary."""
        if (self.reference_curve is None or self.reference_counts is None or
                self.reference_metadata is None):
            return
        if force or self.reference_axis_mode != self.x_axis_mode:
            x_values = self._mcs_x_values(
                np.arange(self.reference_counts.size), self.reference_metadata
            )
            self.reference_curve.setData(
                x_values, self.reference_counts, skipFiniteCheck=True
            )
            self.reference_axis_mode = self.x_axis_mode
        self.reference_curve.setVisible(self.instrument_mode == "mcs")

    def _clear_reference_histogram(self):
        """Remove only the GUI reference overlay; never alter acquired data."""
        self.reference_counts = None
        self.reference_metadata = None
        self.reference_axis_mode = None
        if self.reference_curve is not None:
            self.reference_curve.setData([], [])
            self.reference_curve.setVisible(False)
        self.clear_reference_button.setEnabled(False)
        self.reference_button.setText("Freeze reference")
        self._show_status("Reference histogram removed")

    def _auto_range_mcs_live(self):
        """Ctrl+A: fit every visible axis to the data in the active mode."""
        if self.instrument_mode == "counter":
            if self.counter_times:
                self.plot.getViewBox().autoRange(padding=0.05)
                if self.counter_trigger_check.isChecked():
                    self.counter_trigger_view.enableAutoRange(axis="y", enable=True)
                    self.counter_trigger_view.autoRange(padding=0.05)
                # Return to manual after the one-shot fit; future data remain visible
                # through the explicit Fit Y toggle rather than moving unexpectedly.
                self.plot.enableAutoRange(x=False, y=False)
                self.counter_trigger_view.enableAutoRange(axis="y", enable=False)
                self._show_status("Counter X/Y axes fitted to all stored data", "success")
            return
        if self.latest_counts is None or self.latest_metadata is None:
            self.auto_y_check.setChecked(True)
            return
        self.mcs_manual_autorange = True
        self.auto_y_check.blockSignals(True)
        self.auto_y_check.setChecked(True)
        self.auto_y_check.blockSignals(False)
        x_values = self._mcs_x_values(np.arange(self.latest_counts.size), self.latest_metadata)
        if len(x_values):
            self.plot.getViewBox().setXRange(float(x_values[0]), float(x_values[-1]), padding=0.02)
        self._update_live_manual_autorange()
        self.plot.enableAutoRange(x=False, y=False)
        self.mcs_view_initialized = True
        self._show_status("MCS X/Y axes fitted; live Fit Y enabled", "success")

    def _update_live_manual_autorange(self):
        """Keep a user-requested MCS auto-range current as counts accumulate."""
        if (not self.mcs_manual_autorange or self.instrument_mode != "mcs" or
                self.latest_counts is None or self.latest_counts.size == 0):
            return
        data = np.asarray(self.latest_counts, dtype=float)
        data = data[np.isfinite(data)]
        if not data.size:
            return
        vb = self.plot.getViewBox()
        if self.log_y_check.isChecked():
            positive = data[data > 0.0]
            if not positive.size:
                return
            lo = float(np.log10(np.min(positive)))
            hi = float(np.log10(np.max(positive)))
            span = max(hi - lo, 0.20)
            vb.setYRange(lo - 0.05 * span, hi + 0.10 * span, padding=0.0)
        else:
            vb.setYRange(0.0, max(1.0, float(np.max(data)) * 1.10), padding=0.0)

    def _set_live_fit_y(self, enabled):
        """Turn live MCS Y fitting on or off without changing acquisition."""
        if self.instrument_mode != "mcs":
            if enabled:
                self._fit_y_once()
            self.auto_y_check.blockSignals(True)
            self.auto_y_check.setChecked(False)
            self.auto_y_check.blockSignals(False)
            return
        self.mcs_manual_autorange = bool(enabled)
        if enabled:
            self._fit_y_once()
            self._show_status("Live Fit Y enabled", "success")
        else:
            self.plot.enableAutoRange(axis="y", enable=False)
            self._show_status("Live Fit Y disabled; Y range is now fixed", "success")

    def _fit_y_once(self, checked=False):
        """Fit Y now; when live Fit Y is enabled, keep expanding as counts accumulate."""
        log_y = self.log_y_check.isChecked()

        if self.instrument_mode == "counter":
            primary = np.asarray(self.counter_cps, dtype=float)
            secondary = np.asarray(self.counter_trigger_cps, dtype=float)
        else:
            primary = (np.asarray(self.latest_counts, dtype=float)
                       if self.latest_counts is not None else np.asarray([], dtype=float))
            secondary = np.asarray([], dtype=float)

        primary = primary[np.isfinite(primary)]
        if primary.size:
            vb = self.plot.getViewBox()
            if log_y:
                positive = primary[primary > 0.0]
                if positive.size:
                    # In pyqtgraph log mode ViewBox coordinates are log10(data).
                    lo_log = float(np.log10(np.min(positive)))
                    hi_log = float(np.log10(np.max(positive)))
                    span = max(hi_log - lo_log, 0.20)
                    vb.setLimits(yMin=None, yMax=None)
                    vb.setYRange(lo_log - 0.05 * span,
                                 hi_log + 0.10 * span, padding=0.0)
            else:
                top = max(1.0, float(np.max(primary)) * 1.10)
                vb.setLimits(yMin=0.0, yMax=None)
                vb.setYRange(0.0, top, padding=0.0)

        secondary = secondary[np.isfinite(secondary)]
        if (secondary.size and self.instrument_mode == "counter" and
                self.counter_trigger_check.isChecked()):
            if log_y:
                positive = secondary[secondary > 0.0]
                if positive.size:
                    lo_log = float(np.log10(np.min(positive)))
                    hi_log = float(np.log10(np.max(positive)))
                    span = max(hi_log - lo_log, 0.20)
                    self.counter_trigger_view.setYRange(
                        lo_log - 0.05 * span, hi_log + 0.10 * span, padding=0.0)
            else:
                top = max(1.0, float(np.max(secondary)) * 1.10)
                self.counter_trigger_view.setYRange(0.0, top, padding=0.0)

        self.plot.enableAutoRange(axis="y", enable=False)
        self.counter_trigger_view.enableAutoRange(axis="y", enable=False)
        self._show_status("Y axis fitted" + ("; live Fit Y is on" if self.instrument_mode == "mcs" and self.mcs_manual_autorange else "") if self.instrument_mode == "mcs" else "Y axis fitted to current data", "success")

    def _apply_display_options(self):
        """Apply display mode without changing the user's current viewport."""
        log_y = self.log_y_check.isChecked()
        self.plot.setLogMode(x=False, y=log_y)
        self.plot.enableAutoRange(axis="y", enable=False)
        self.counter_trigger_view.enableAutoRange(axis="y", enable=False)

    def _default_data_columns(self):
        return [
            ("bin", "Bin", True),
            ("time_ns", "Time (ns)", True),
            ("count", "Count", True),
            ("reference_count", "Reference count", True),
            ("in_signal_region", "In signal ROI", True),
            ("in_noise_region", "In noise ROI", True),
        ]

    def _load_data_settings(self):
        defaults = self._default_data_columns()
        saved_order = self.settings.value(
            "data/column_order", [x[0] for x in defaults]
        )
        if isinstance(saved_order, str):
            saved_order = [saved_order]
        known = {key for key, _, _ in defaults}
        order = [key for key in saved_order if key in known]
        order += [key for key, _, _ in defaults if key not in order]

        default_names = {key: name for key, name, _ in defaults}
        self.data_column_order = order
        self.data_column_names = {
            key: self.settings.value(
                f"data/column_name/{key}", default_names[key], str
            )
            for key in known
        }
        self.data_column_enabled = {
            key: self.settings.value(f"data/column_enabled/{key}", True, type=bool)
            for key in known
        }

        self.status_visibility = {}
        for key, (_label, _widget) in self.status_fields.items():
            self.status_visibility[key] = self.settings.value(
                f"status/show/{key}", True, type=bool
            )
        self._apply_status_order()
        self._apply_status_visibility()

    def _apply_status_order(self, order=None):
        """Reorder Acquisition status rows without changing measurement data."""
        if order is None:
            order = self.settings.value("status/order", list(self.status_fields.keys()))
            if isinstance(order, str):
                order = [order]
        order = [k for k in order if k in self.status_fields]
        order += [k for k in self.status_fields if k not in order]

        form = self.frame_group.layout()
        if not isinstance(form, QFormLayout):
            return

        rows = {}
        for key, (label_text, field) in self.status_fields.items():
            rows[key] = (form.labelForField(field), field, label_text)

        for label, field, _text in rows.values():
            if label is not None:
                form.removeWidget(label)
            form.removeWidget(field)

        for key in order:
            label, field, label_text = rows[key]
            if label is None:
                label = QLabel(label_text)
            form.addRow(label, field)

    def _apply_status_visibility(self):
        for key, (_label, widget) in self.status_fields.items():
            visible = bool(self.status_visibility.get(key, True))
            widget.setVisible(visible)
            # QFormLayout labelForField lets us hide the matching caption too.
            parent_layout = widget.parentWidget().layout() if widget.parentWidget() else None
            if isinstance(parent_layout, QFormLayout):
                label = parent_layout.labelForField(widget)
                if label is not None:
                    label.setVisible(visible)

    def _show_data_settings(self):
        """Editor for export schema and acquisition-status visibility."""
        dialog = QDialog(self)
        dialog.setWindowTitle("OpenMCS — Data settings")
        dialog.setModal(True)
        dialog.resize(720, 620)
        root = QVBoxLayout(dialog)

        tabs = QTabWidget()
        root.addWidget(tabs, 1)

        # ---- Saved columns ----
        columns_page = QWidget()
        columns_layout = QVBoxLayout(columns_page)
        columns_layout.addWidget(QLabel(
            "Check fields to save. Drag any ☰ row up or down to change CSV column order. "
            "Double-click a row to rename its CSV column heading."
        ))

        self._data_column_list = QListWidget()
        self._data_column_list.setDragDropMode(
            QListWidget.DragDropMode.InternalMove
        )
        self._data_column_list.setDefaultDropAction(Qt.DropAction.MoveAction)
        self._data_column_list.setDragEnabled(True)
        self._data_column_list.setAcceptDrops(True)
        self._data_column_list.setDropIndicatorShown(True)
        self._data_column_list.setDragDropOverwriteMode(False)

        default_labels = {k: n for k, n, _ in self._default_data_columns()}
        for key in self.data_column_order:
            item = QListWidgetItem()
            item.setData(Qt.ItemDataRole.UserRole, key)
            item.setFlags(
                item.flags() |
                Qt.ItemFlag.ItemIsUserCheckable |
                Qt.ItemFlag.ItemIsDragEnabled
            )
            item.setCheckState(
                Qt.CheckState.Checked if self.data_column_enabled.get(key, True)
                else Qt.CheckState.Unchecked
            )
            # Use a native QListWidgetItem for drag/drop. Qt's internal-move
            # operation does not reliably preserve setItemWidget() children;
            # that was why rows appeared to disappear after dragging.
            heading = self.data_column_names.get(key, default_labels[key])
            item.setText(f"☰   {heading}")
            item.setToolTip(
                "Drag this row to reorder. Double-click to rename the CSV column."
            )
            item.setFlags(
                item.flags() |
                Qt.ItemFlag.ItemIsEditable |
                Qt.ItemFlag.ItemIsDragEnabled
            )
            self._data_column_list.addItem(item)

        columns_layout.addWidget(self._data_column_list, 1)
        tabs.addTab(columns_page, "Saved columns")

        # ---- Acquisition status ----
        status_page = QWidget()
        status_layout = QVBoxLayout(status_page)
        status_layout.addWidget(QLabel(
            "Drag the ☰ rows to choose Acquisition status display order. "
            "Check/uncheck fields to show or hide them. This changes only the display; "
            "it never changes acquisition or saved data."
        ))
        self._status_list = QListWidget()
        self._status_list.setDragDropMode(QListWidget.DragDropMode.InternalMove)
        self._status_list.setDefaultDropAction(Qt.DropAction.MoveAction)
        self._status_list.setDragEnabled(True)
        self._status_list.setAcceptDrops(True)
        self._status_list.setDropIndicatorShown(True)
        self._status_list.setDragDropOverwriteMode(False)

        saved_status_order = self.settings.value(
            "status/order", list(self.status_fields.keys())
        )
        if isinstance(saved_status_order, str):
            saved_status_order = [saved_status_order]
        status_order = [k for k in saved_status_order if k in self.status_fields]
        status_order += [k for k in self.status_fields if k not in status_order]

        for key in status_order:
            label, _widget = self.status_fields[key]
            item = QListWidgetItem(f"☰   {label}")
            item.setData(Qt.ItemDataRole.UserRole, key)
            item.setCheckState(Qt.CheckState.Checked if self.status_visibility.get(key, True)
                               else Qt.CheckState.Unchecked)
            item.setFlags((item.flags() | Qt.ItemFlag.ItemIsUserCheckable |
                           Qt.ItemFlag.ItemIsDragEnabled) &
                          ~Qt.ItemFlag.ItemIsDropEnabled)
            item.setToolTip("Drag to reorder; check/uncheck to show or hide.")
            self._status_list.addItem(item)

        status_layout.addWidget(self._status_list, 1)
        tabs.addTab(status_page, "Acquisition status")

        buttons = QHBoxLayout()
        reset_button = QPushButton("Restore defaults")
        cancel_button = QPushButton("Cancel")
        apply_button = QPushButton("Apply")
        apply_button.setObjectName("PrimaryButton")
        buttons.addWidget(reset_button)
        buttons.addStretch(1)
        buttons.addWidget(cancel_button)
        buttons.addWidget(apply_button)
        root.addLayout(buttons)

        def restore_defaults():
            defaults = self._default_data_columns()
            self._data_column_list.clear()
            for key, name, enabled in defaults:
                item = QListWidgetItem()
                item.setData(Qt.ItemDataRole.UserRole, key)
                item.setFlags(
                    item.flags() |
                    Qt.ItemFlag.ItemIsUserCheckable |
                    Qt.ItemFlag.ItemIsDragEnabled
                )
                item.setCheckState(
                    Qt.CheckState.Checked if enabled else Qt.CheckState.Unchecked
                )
                item.setText(f"☰   {name}")
                item.setToolTip(
                    "Drag this row to reorder. Double-click to rename the CSV column."
                )
                item.setFlags(
                    item.flags() |
                    Qt.ItemFlag.ItemIsEditable |
                    Qt.ItemFlag.ItemIsDragEnabled
                )
                self._data_column_list.addItem(item)
            self._status_list.clear()
            for key, (label, _widget) in self.status_fields.items():
                item = QListWidgetItem(f"☰   {label}")
                item.setData(Qt.ItemDataRole.UserRole, key)
                item.setCheckState(Qt.CheckState.Checked)
                item.setFlags((item.flags() | Qt.ItemFlag.ItemIsUserCheckable |
                               Qt.ItemFlag.ItemIsDragEnabled) &
                              ~Qt.ItemFlag.ItemIsDropEnabled)
                self._status_list.addItem(item)

        def apply_settings():
            order, names, enabled = [], {}, {}
            for row in range(self._data_column_list.count()):
                item = self._data_column_list.item(row)
                key = item.data(Qt.ItemDataRole.UserRole)
                order.append(key)
                heading = item.text().strip()
                if heading.startswith("☰"):
                    heading = heading[1:].strip()
                names[key] = heading or key
                enabled[key] = item.checkState() == Qt.CheckState.Checked

            if not any(enabled.values()):
                QMessageBox.warning(
                    dialog, "No saved columns",
                    "Enable at least one CSV column."
                )
                return

            self.data_column_order = order
            self.data_column_names = names
            self.data_column_enabled = enabled
            status_order = []
            self.status_visibility = {}
            for row in range(self._status_list.count()):
                item = self._status_list.item(row)
                key = item.data(Qt.ItemDataRole.UserRole)
                status_order.append(key)
                self.status_visibility[key] = (
                    item.checkState() == Qt.CheckState.Checked
                )

            self.settings.setValue("data/column_order", order)
            for key in order:
                self.settings.setValue(f"data/column_name/{key}", names[key])
                self.settings.setValue(f"data/column_enabled/{key}", enabled[key])
            for key, visible in self.status_visibility.items():
                self.settings.setValue(f"status/show/{key}", visible)
            self.settings.setValue("status/order", status_order)

            self._apply_status_order(status_order)
            self._apply_status_visibility()
            dialog.accept()

        reset_button.clicked.connect(restore_defaults)
        cancel_button.clicked.connect(dialog.reject)
        apply_button.clicked.connect(apply_settings)
        dialog.exec()

    def _browse_save_directory(self):
        current = self.save_directory_edit.text().strip()
        if not current:
            current = str(Path.home() / "Documents")
        folder = QFileDialog.getExistingDirectory(
            self, "Choose OpenMCS data folder", current
        )
        if folder:
            self.save_directory_edit.setText(folder)
            self.settings.setValue("data/save_directory", folder)

    @staticmethod
    def _safe_run_name(name):
        """Return a Windows-safe base filename without changing its meaning."""
        name = str(name).strip()
        for char in '<>:"/\\\\|?*':
            name = name.replace(char, "_")
        name = name.rstrip(" .")
        return name or "openmcs_run"

    def _workflow_paths(self):
        folder_text = self.save_directory_edit.text().strip()
        if not folder_text:
            folder_text = str(Path.home() / "Documents" / "OpenMCS Data")
            self.save_directory_edit.setText(folder_text)
        folder = Path(folder_text).expanduser()
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            QMessageBox.critical(self, "Save folder error", str(error))
            return None, None, None

        run_name = self._safe_run_name(self.run_name_edit.text())
        if self.run_name_edit.text() != run_name:
            self.run_name_edit.setText(run_name)
        return folder, folder / f"{run_name}.csv", folder / f"{run_name}.png"

    def _write_current_csv(self, path):
        """Write current mode data to an exact path; no dialog and no data mutation."""
        if self.instrument_mode == "counter":
            if not self.counter_times:
                raise ValueError("There is no Counter data to save.")
            with open(path, "w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow((
                    "elapsed_seconds", "signal_counts_per_second",
                    "trigger_counts_per_second",
                ))
                writer.writerows(zip(
                    self.counter_times, self.counter_cps,
                    self.counter_trigger_cps,
                ))
            return

        if self.latest_counts is None or self.latest_metadata is None:
            raise ValueError("There is no MCS histogram to save.")

        metadata = self.latest_metadata
        dwell_s = metadata["dwell_cycles"] / self._clock_hz()
        bins = np.arange(self.latest_counts.size, dtype=np.uint32)
        if self.roi_enable_check.isChecked():
            signal_mask = (
                (bins >= self.signal_start.value()) &
                (bins <= self.signal_end.value())
            ).astype(np.uint8)
            noise_mask = (
                (bins >= self.noise_start.value()) &
                (bins <= self.noise_end.value())
            ).astype(np.uint8)
        else:
            signal_mask = np.zeros(bins.size, dtype=np.uint8)
            noise_mask = np.zeros(bins.size, dtype=np.uint8)

        with open(path, "w", newline="", encoding="utf-8") as stream:
            stream.write(
                f"# OpenMCS {APP_VERSION}; clock_hz={self._clock_hz():.6f}; "
                f"dwell_cycles={metadata['dwell_cycles']}; "
                f"displayed_passes={self.accumulation_frames}; "
                f"roi_enabled={int(self.roi_enable_check.isChecked())}\\n"
            )
            reference_compatible = (
                self.reference_counts is not None and
                self.reference_metadata is not None and
                self.reference_counts.size == self.latest_counts.size and
                self.reference_metadata.get("dwell_cycles") == metadata.get("dwell_cycles") and
                self.reference_metadata.get("clock_hz") == metadata.get("clock_hz")
            )
            writer = csv.writer(stream)
            active_columns = [
                key for key in self.data_column_order
                if self.data_column_enabled.get(key, True)
            ]
            writer.writerow([
                self.data_column_names.get(key, key)
                for key in active_columns
            ])
            for index in range(self.latest_counts.size):
                values = {
                    "bin": index,
                    "time_ns": f"{index * dwell_s * 1e9:.6f}",
                    "count": int(self.latest_counts[index]),
                    "reference_count": (
                        int(self.reference_counts[index]) if reference_compatible else ""
                    ),
                    "in_signal_region": int(signal_mask[index]),
                    "in_noise_region": int(noise_mask[index]),
                }
                writer.writerow([values[key] for key in active_columns])

    def _visible_acquisition_status_lines(self):
        """Return the currently enabled Acquisition status rows for image export."""
        lines = []
        order = self.settings.value("status/order", list(self.status_fields.keys()))
        if isinstance(order, str):
            order = [order]
        order = [key for key in order if key in self.status_fields]
        order += [key for key in self.status_fields if key not in order]

        for key in order:
            if not self.status_visibility.get(key, True):
                continue
            label, widget = self.status_fields[key]
            lines.append((label, widget.text()))
        return lines

    def _write_plot_with_status(self, path):
        """Export the plot plus the visible Acquisition status information."""
        import tempfile

        with tempfile.TemporaryDirectory() as temp_dir:
            plot_path = Path(temp_dir) / "plot.png"
            exporter = pg.exporters.ImageExporter(self.plot.getPlotItem())
            exporter.parameters()["width"] = 1800
            exporter.export(str(plot_path))

            plot_image = QImage(str(plot_path))
            if plot_image.isNull():
                raise RuntimeError("Could not render plot image.")

            status_lines = self._visible_acquisition_status_lines()
            margin = 28
            line_height = 28
            title_height = 38
            status_height = (
                margin * 2 + title_height + max(1, len(status_lines)) * line_height
            )

            result = QImage(
                plot_image.width(),
                plot_image.height() + status_height,
                QImage.Format.Format_ARGB32
            )
            # Match the selected GUI theme.
            light = self.theme_combo.currentData() == "light"
            background = QColor("#f4f7fb" if light else "#0b1016")
            foreground = QColor("#182536" if light else "#e7edf5")
            muted = QColor("#526579" if light else "#94a3b8")
            result.fill(background)

            painter = QPainter(result)
            painter.drawImage(0, 0, plot_image)

            y0 = plot_image.height() + margin
            title_font = QFont("Segoe UI", 12)
            title_font.setBold(True)
            painter.setFont(title_font)
            painter.setPen(foreground)
            painter.drawText(
                QRect(margin, y0, result.width() - 2 * margin, title_height),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                "Acquisition status"
            )

            label_font = QFont("Segoe UI", 10)
            painter.setFont(label_font)
            y = y0 + title_height
            label_width = 330
            for label, value in status_lines:
                painter.setPen(muted)
                painter.drawText(
                    QRect(margin, y, label_width, line_height),
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                    label
                )
                painter.setPen(foreground)
                painter.drawText(
                    QRect(margin + label_width, y,
                          result.width() - 2 * margin - label_width, line_height),
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                    value
                )
                y += line_height

            painter.end()
            if not result.save(str(path), "PNG"):
                raise RuntimeError("Could not save composed plot/status image.")

    def _write_current_plot(self, path):
        if self.instrument_mode == "counter":
            if not self.counter_times:
                raise ValueError("There is no Counter plot to save.")
        elif self.latest_counts is None:
            raise ValueError("There is no MCS histogram to save.")

        self._write_plot_with_status(path)

    @staticmethod
    def _increment_run_name(name):
        name = OpenMCSWindow._safe_run_name(name)
        match = re.match(r"^(.*?)(\d+)$", name)
        if match:
            prefix, digits = match.groups()
            return f"{prefix}{int(digits)+1:0{len(digits)}d}"
        return f"{name}_001"

    def save_data(self):
        """Save current data without changing the user-entered base name."""
        folder, csv_path, png_path = self._workflow_paths()
        if folder is None:
            return
        save_image = self.save_image_check.isChecked()

        if csv_path.exists() or (save_image and png_path.exists()):
            stem = csv_path.stem
            index = 1
            while True:
                candidate_csv = folder / f"{stem}_{index:03d}.csv"
                candidate_png = folder / f"{stem}_{index:03d}.png"
                if not candidate_csv.exists() and (not save_image or not candidate_png.exists()):
                    csv_path, png_path = candidate_csv, candidate_png
                    break
                index += 1
        try:
            self._write_current_csv(csv_path)
            if save_image:
                self._write_current_plot(png_path)
        except Exception as error:
            QMessageBox.critical(self, "Save Data failed", str(error))
            return

        self.settings.setValue("data/save_directory", str(folder))
        self.settings.setValue("data/run_name", self.run_name_edit.text().strip())
        self.settings.setValue("data/save_image", save_image)
        saved = f"{csv_path.name} + {png_path.name}" if save_image else csv_path.name
        self._show_status(f"Saved {saved}", "success")

    def save_csv(self):
        if self.instrument_mode == "counter":
            if not self.counter_times:
                return
            default = f"openmcs_counter_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            path, _ = QFileDialog.getSaveFileName(
                self, "Export counter history", default, "CSV files (*.csv)"
            )
            if not path:
                return
            if not path.lower().endswith(".csv"):
                path += ".csv"
            try:
                with open(path, "w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow((
                        "elapsed_seconds", "signal_counts_per_second",
                        "trigger_counts_per_second",
                    ))
                    writer.writerows(zip(
                        self.counter_times, self.counter_cps,
                        self.counter_trigger_cps,
                    ))
            except OSError as error:
                QMessageBox.critical(self, "Export failed", str(error))
                return
            self._show_status(f"Counter history exported to {path}", "success")
            return
        if self.latest_counts is None or self.latest_metadata is None:
            return
        default = f"openmcs_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "Export histogram and ROI data", default, "CSV files (*.csv)")
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"
        metadata = self.latest_metadata
        dwell_s = metadata["dwell_cycles"] / self._clock_hz()
        bins = np.arange(self.latest_counts.size, dtype=np.uint32)
        if self.roi_enable_check.isChecked():
            signal_mask = ((bins >= self.signal_start.value()) & (bins <= self.signal_end.value())).astype(np.uint8)
            noise_mask = ((bins >= self.noise_start.value()) & (bins <= self.noise_end.value())).astype(np.uint8)
        else:
            signal_mask = np.zeros(bins.size, dtype=np.uint8)
            noise_mask = np.zeros(bins.size, dtype=np.uint8)
        try:
            # csv.writer preserves the full uint64 accumulated count. A mixed
            # NumPy matrix would coerce the count column to float and could
            # lose integer precision in very long acquisitions.
            with open(path, "w", newline="", encoding="utf-8") as stream:
                stream.write(
                    f"# OpenMCS {APP_VERSION}; clock_hz={metadata['clock_hz']}; "
                    f"dwell_cycles={metadata['dwell_cycles']}; "
                    f"displayed_passes={self.accumulation_frames}; "
                    f"roi_enabled={int(self.roi_enable_check.isChecked())}\n"
                )
                writer = csv.writer(stream)
                writer.writerow(("bin", "time_ns", "count", "in_signal_region", "in_noise_region"))
                for index in range(self.latest_counts.size):
                    writer.writerow((
                        index,
                        f"{index * dwell_s * 1e9:.6f}",
                        int(self.latest_counts[index]),
                        int(signal_mask[index]),
                        int(noise_mask[index]),
                    ))
        except OSError as error:
            QMessageBox.critical(self, "Export failed", str(error))
            return
        self._show_status(f"Histogram exported to {path}", "success")

    def save_plot(self):
        if self.instrument_mode == "counter":
            if not self.counter_times:
                return
        elif self.latest_counts is None:
            return
        default = f"openmcs_{time.strftime('%Y%m%d_%H%M%S')}.png"
        path, _ = QFileDialog.getSaveFileName(self, "Export plot image", default, "PNG image (*.png)")
        if not path:
            return
        if not path.lower().endswith(".png"):
            path += ".png"
        try:
            self._write_plot_with_status(path)
        except Exception as error:
            QMessageBox.critical(self, "Image export failed", str(error))
            return
        self._show_status(f"Plot image exported to {path}", "success")

    def _show_status(self, message, level="info"):
        info_color = "#526579" if getattr(self, "theme_combo", None) and \
            self.theme_combo.currentData() == "light" else MUTED
        colors = {"info": info_color, "success": GREEN, "warning": AMBER, "error": RED}
        self.status_message.setText(message)
        self.status_message.setStyleSheet(f"color:{colors.get(level, MUTED)}")

    def closeEvent(self, event):
        # Safety shutdown: request the existing hardware STOP before closing USB.
        # Give the worker thread a short interval to dequeue and transmit it.
        if self.worker is not None:
            try:
                self.worker.send_command("p")
                time.sleep(0.15)
            except Exception:
                pass
        self._save_settings()
        layout_editor = getattr(self, "_layout_editor", None)
        if layout_editor is not None:
            layout_editor.close()
        shortcuts_dialog = getattr(self, "_shortcuts_dialog", None)
        if shortcuts_dialog is not None:
            shortcuts_dialog.closeEvent = lambda close_event: close_event.accept()
            shortcuts_dialog.close()
        dialog = getattr(self, "_acquisition_dialog", None)
        if dialog is not None:
            # Restore normal close behavior so the application cannot be kept
            # alive by the persistent modeless Setup dialog.
            dialog.closeEvent = lambda close_event: close_event.accept()
            dialog.close()
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1500)
        event.accept()


DARK_STYLE_SHEET = f"""
QWidget {{ background:{BG}; color:{TEXT}; font-family:"Segoe UI","Inter",sans-serif; font-size:10pt; }}
QMainWindow {{ background:{BG}; }}
QLabel#AppTitle {{ font-size:23pt; font-weight:700; color:#f5f8fc; }}
QLabel#CompactTitle {{ font-size:11pt; font-weight:700; color:#f5f8fc; }}
QLabel#AppSubtitle {{ font-size:9.5pt; color:{MUTED}; }}
QLabel#SectionTitle {{ font-size:12pt; font-weight:650; }}
QLabel#HintLabel {{ color:{MUTED}; font-size:9pt; }}
QLabel#ConnectionBadge {{ color:{RED}; background:#25171d; border:1px solid #59303a; border-radius:13px; padding:5px 10px; font-size:8.5pt; font-weight:700; }}
QLabel#ConnectionBadge[connected="true"] {{ color:{GREEN}; background:#11241f; border-color:#285746; }}
QFrame#ControlBar, QFrame#PlotPanel, QFrame#SelectionPanel, QGroupBox {{ background:{PANEL}; border:1px solid {BORDER}; border-radius:7px; }}
QLabel#SelectionCaption {{ color:{CYAN}; font-weight:700; padding-right:8px; }}
QLabel#SelectionInfo {{ color:{TEXT}; font-family:"Consolas","Cascadia Mono",monospace; font-size:9pt; }}
QFrame#ModeSwitch {{ background:#0e151d; border:1px solid #34465a; border-radius:7px; }}
QLabel#ModeLabel {{ color:{MUTED}; font-size:8pt; font-weight:700; padding:0 6px; }}
QGroupBox {{ margin-top:11px; padding-top:11px; font-weight:600; }}
QGroupBox::title {{ subcontrol-origin:margin; left:10px; padding:0 5px; color:#cbd6e4; }}
QFrame#MetricValue {{ background:{PANEL_2}; border:1px solid {BORDER}; border-radius:5px; }}
QLabel#MetricCaption {{ color:{MUTED}; font-size:8.5pt; }}
QLabel#MetricNumber {{ font-size:13pt; font-weight:650; color:#f4f8fc; }}
QPushButton, QComboBox, QSpinBox, QDoubleSpinBox {{ background:{PANEL_2}; border:1px solid #34465a; border-radius:5px; min-height:28px; padding:1px 9px; }}
QPushButton:hover, QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover {{ border-color:#52708e; background:#1b2835; }}
QPushButton:pressed {{ background:#0f1821; }}
QPushButton:disabled {{ color:#566575; border-color:#25313e; }}
QPushButton#PrimaryButton {{ background:#176b87; border-color:#238db0; color:white; font-weight:600; }}
QPushButton#PrimaryButton:hover {{ background:#1d7d9d; }}
QPushButton#RunButton {{ background:#176445; border-color:#268b63; color:white; font-weight:600; }}
QPushButton#RunButton:hover {{ background:#1c7752; }}
QPushButton#ModeButton {{ min-height:27px; padding:1px 13px; border-color:transparent; background:transparent; color:{MUTED}; font-weight:700; }}
QPushButton#ModeButton:hover {{ background:#1b2835; color:{TEXT}; }}
QPushButton#ModeButton:checked {{ background:#176b87; border-color:#36c5f0; color:white; }}
QPushButton#ROIButton {{ min-width:72px; font-weight:650; }}
QPushButton#ROIButton:checked {{ background:#176445; border-color:#3ddc97; color:white; }}
QComboBox::drop-down {{ border:0; width:24px; }}
QCheckBox {{ spacing:7px; }}
QScrollArea {{ background:transparent; }}
QTabWidget::pane {{ border:1px solid {BORDER}; border-radius:6px; background:{PANEL}; top:-1px; }}
QTabBar::tab {{ background:{PANEL_2}; border:1px solid {BORDER}; padding:7px 13px; min-width:78px; }}
QTabBar::tab:first {{ border-top-left-radius:5px; }}
QTabBar::tab:last {{ border-top-right-radius:5px; }}
QTabBar::tab:selected {{ background:{PANEL}; color:{CYAN}; border-bottom-color:{PANEL}; font-weight:650; }}
QStatusBar {{ background:#0d141c; border-top:1px solid {BORDER}; }}
QStatusBar::item {{ border:none; }}
QSplitter::handle {{ background:{BORDER}; width:2px; }}
QToolTip {{ color:{TEXT}; background:#1c2733; border:1px solid #41556a; }}
"""

LIGHT_STYLE_SHEET = DARK_STYLE_SHEET
for dark_color, light_color in {
    BG: LIGHT_BG,
    PANEL: "#ffffff",
    PANEL_2: "#eef3f8",
    BORDER: "#c8d3df",
    TEXT: "#17212b",
    MUTED: "#526579",
    "#f5f8fc": "#101820",
    "#cbd6e4": "#33465a",
    "#f4f8fc": "#101820",
    "#34465a": "#aebdcb",
    "#52708e": "#7891aa",
    "#1b2835": "#e3ebf3",
    "#0f1821": "#dbe5ef",
    "#566575": "#91a0ae",
    "#25313e": "#d5dee7",
    "#0d141c": "#e9eff5",
    "#1c2733": "#ffffff",
    "#41556a": "#9eafbf",
}.items():
    LIGHT_STYLE_SHEET = LIGHT_STYLE_SHEET.replace(dark_color, light_color)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setOrganizationName("OpenMCS")
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLE_SHEET)
    pg.setConfigOptions(antialias=False, foreground=MUTED)
    import pyqtgraph.exporters  # noqa: F401
    window = OpenMCSWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
