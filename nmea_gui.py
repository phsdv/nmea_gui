#!/usr/bin/env python3
"""
nmea_gui.py

Real-time GUI for monitoring a GNSS receiver on a serial/COM port
(e.g. COM4 on Windows, /dev/ttyUSB0 on Linux/macOS).

Reads NMEA 0183 sentences (GGA, RMC, GSA, GSV, GLL, VTG) from the port in
a background thread and displays live-updating:
  - Position / fix quality / HDOP / satellite count / speed
  - A per-satellite signal-strength (SNR) table
  - A live track plot and altitude trace
  - A scrolling raw NMEA console

Requirements:
    pip install PyQt6 pyserial matplotlib --break-system-packages

Usage:
    python3 nmea_gui.py
    (then enter your COM port, e.g. COM4, and click Connect)

    python3 nmea_gui.py --port COM4 --baud 9600   (auto-connects on launch, 9600 is now the default)
"""

import sys
import csv
import argparse
from collections import defaultdict, deque
from datetime import datetime

from PyQt6 import QtCore, QtGui, QtWidgets

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from math import floor, ceil, asin, sin, cos, radians, sqrt

try:
    import serial
except ImportError:
    serial = None

try:
    from serial.tools import list_ports as _list_ports
except ImportError:
    _list_ports = None


def list_available_ports():
    """Scan for connected serial/COM ports and return descriptive info for each."""
    if _list_ports is None:
        return []
    ports = []
    for p in sorted(_list_ports.comports(), key=lambda x: x.device):
        ports.append(dict(
            device=p.device,
            description=(p.description or "").strip(),
            manufacturer=(getattr(p, "manufacturer", None) or "").strip(),
            hwid=p.hwid or "",
            vid=p.vid,
            pid=p.pid,
            serial_number=(getattr(p, "serial_number", None) or "").strip(),
        ))
    return ports


def format_port_label(p):
    """Short one-line label for the port dropdown, e.g. 'COM4 — u-blox GNSS receiver'."""
    label = p["device"]
    extras = []
    if p["description"] and p["description"].lower() not in ("n/a", p["device"].lower()):
        extras.append(p["description"])
    if p["manufacturer"]:
        extras.append(p["manufacturer"])
    if extras:
        label += " — " + ", ".join(extras)
    return label


def format_port_details(p):
    """Longer detail string shown under the dropdown for the selected port."""
    bits = []
    if p.get("manufacturer"):
        bits.append(f"Manufacturer: {p['manufacturer']}")
    if p.get("vid") is not None and p.get("pid") is not None:
        bits.append(f"VID:PID = {p['vid']:04X}:{p['pid']:04X}")
    if p.get("serial_number"):
        bits.append(f"Serial #: {p['serial_number']}")
    if p.get("hwid"):
        bits.append(f"HWID: {p['hwid']}")
    return "   |   ".join(bits) if bits else "No additional device details available"


# --------------------------------------------------------------------------
# NMEA parsing helpers (same logic as nmea_analyzer.py, kept self-contained
# here so this GUI can run standalone)
# --------------------------------------------------------------------------

TALKER_SYSTEMS = {
    "GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
    "BD": "BeiDou", "GB": "BeiDou", "GN": "Multi/GNSS", "QZ": "QZSS",
}

FIX_QUALITY = {
    0: "Invalid", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS",
    4: "RTK Fixed", 5: "RTK Float", 6: "Estimated",
}


def checksum_ok(line: str) -> bool:
    line = line.strip()
    if "*" not in line or not line.startswith("$"):
        return False
    body, _, chk = line[1:].partition("*")
    try:
        expected = int(chk[:2], 16)
    except ValueError:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return calc == expected


def dm_to_decimal(value, hemisphere):
    if not value:
        return None
    dot = value.find(".")
    deg_len = dot - 2
    degrees = float(value[:deg_len])
    minutes = float(value[deg_len:])
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


class LiveNmeaState:
    """Holds the latest known GNSS state, updated in place as sentences arrive."""

    def __init__(self):
        self.time = None
        self.date = None
        self.lat = None
        self.lon = None
        self.alt_msl = None
        self.geoid_sep = None
        self.quality = None
        self.num_sats = None
        self.hdop = None
        self.speed_kn = None
        self.course = None
        # prn -> dict(elev, az, snr, system)
        self.sats = {}

    def feed_line(self, line):
        """Parse one NMEA sentence and update state. Returns True if a
        position/fix field changed (worth refreshing the main readout)."""
        line = line.strip()
        if not line.startswith("$") or not checksum_ok(line):
            return False
        body = line[1:].split("*")[0]
        f = body.split(",")
        talker = f[0][:2]
        sentence = f[0][2:]

        if sentence == "GGA":
            return self._handle_gga(f)
        elif sentence == "RMC":
            return self._handle_rmc(f)
        elif sentence == "GSV":
            self._handle_gsv(f, talker)
            return True
        return False

    def _handle_gga(self, f):
        try:
            self.time = f[1] or self.time
            if f[2] and f[4]:
                self.lat = dm_to_decimal(f[2], f[3])
                self.lon = dm_to_decimal(f[4], f[5])
            self.quality = int(f[6]) if f[6] else 0
            self.num_sats = int(f[7]) if f[7] else self.num_sats
            self.hdop = float(f[8]) if f[8] else self.hdop
            self.alt_msl = float(f[9]) if f[9] else self.alt_msl
            self.geoid_sep = float(f[11]) if f[11] else self.geoid_sep
        except (ValueError, IndexError):
            return False
        return True

    def _handle_rmc(self, f):
        try:
            self.time = f[1] or self.time
            self.date = f[9] or self.date
            self.speed_kn = float(f[7]) if f[7] else 0.0
            self.course = float(f[8]) if f[8] else self.course
        except (ValueError, IndexError):
            return False
        return True

    def _handle_gsv(self, f, talker):
        system = TALKER_SYSTEMS.get(talker, talker)
        i = 4
        while i < len(f):
            chunk = f[i:i + 4]
            if len(chunk) < 4:
                break
            prn, elev, az, snr = chunk
            if prn:
                self.sats[prn] = dict(
                    elev=elev or None, az=az or None,
                    snr=int(snr) if snr else None, system=system,
                )
            i += 4


# --------------------------------------------------------------------------
# Background serial reader thread
# --------------------------------------------------------------------------

class SerialWorker(QtCore.QThread):
    raw_line = QtCore.pyqtSignal(str)
    state_changed = QtCore.pyqtSignal(object)     # LiveNmeaState snapshot
    status_changed = QtCore.pyqtSignal(str, str)  # (level, message) level: info/ok/error
    finished_reading = QtCore.pyqtSignal()

    def __init__(self, port, baud, save_path=None, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.save_path = save_path
        self._stop = False
        self.state = LiveNmeaState()

    def stop(self):
        self._stop = True

    def run(self):
        if serial is None:
            self.status_changed.emit("error", "pyserial is not installed. "
                                      "Run: pip install pyserial --break-system-packages")
            self.finished_reading.emit()
            return

        try:
            ser = serial.Serial(self.port, baudrate=self.baud, timeout=1)
        except Exception as e:
            self.status_changed.emit("error", f"Could not open {self.port}: {e}")
            self.finished_reading.emit()
            return

        self.status_changed.emit("ok", f"Connected to {self.port} @ {self.baud} baud")

        save_fh = None
        if self.save_path:
            try:
                save_fh = open(self.save_path, "a", errors="ignore")
            except Exception as e:
                self.status_changed.emit("error", f"Could not open save file: {e}")

        try:
            with ser:
                while not self._stop:
                    try:
                        raw = ser.readline()
                    except Exception as e:
                        self.status_changed.emit("error", f"Read error: {e}")
                        break
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue
                    self.raw_line.emit(line)
                    if save_fh:
                        save_fh.write(line + "\n")
                        save_fh.flush()
                    if self.state.feed_line(line):
                        self.state_changed.emit(self.state)
        finally:
            if save_fh:
                save_fh.close()
            self.status_changed.emit("info", "Disconnected")
            self.finished_reading.emit()


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------

MAX_TRACK_POINTS = 5_000
MAX_CONSOLE_LINES = 500

COMMON_BAUDS = ["4800", "9600", "19200", "38400", "57600", "115200"]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, default_port="COM4", default_baud="9600"):
        super().__init__()
        self.setWindowTitle("Live GNSS / NMEA Monitor")
        self.resize(1200, 800)

        self.worker = None
        self.track_lats = deque(maxlen=MAX_TRACK_POINTS)
        self.track_lons = deque(maxlen=MAX_TRACK_POINTS)
        self.alt_history = deque(maxlen=MAX_TRACK_POINTS)
        self.hdop_history = deque(maxlen=MAX_TRACK_POINTS)
        self.session_fixes = []  # for CSV export: (time, lat, lon, alt, hdop, sats, quality)
        self.line_count = 0

        self._build_ui(default_port, default_baud)

    # -- UI construction -----------------------------------------------

    def _build_ui(self, default_port, default_baud):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        root.addLayout(self._build_connection_bar(default_port, default_baud))

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([540, 760])
        root.addWidget(splitter, stretch=1)

        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(MAX_CONSOLE_LINES)
        self.console.setFont(QtGui.QFont("Consolas, Monaco, monospace", 9))
        self.console.setFixedHeight(140)
        console_box = QtWidgets.QGroupBox("Raw NMEA")
        console_layout = QtWidgets.QVBoxLayout(console_box)
        console_layout.addWidget(self.console)
        root.addWidget(console_box)

        self.status_bar = self.statusBar()
        self.status_label = QtWidgets.QLabel("Not connected")
        self.status_bar.addWidget(self.status_label)

    def _build_connection_bar(self, default_port, default_baud):
        outer = QtWidgets.QVBoxLayout()
        bar = QtWidgets.QHBoxLayout()

        bar.addWidget(QtWidgets.QLabel("Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(340)
        self.port_combo.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.port_combo.setEditText(default_port)
        self.port_combo.currentIndexChanged.connect(self._update_port_details)
        self.port_combo.editTextChanged.connect(self._update_port_details)
        bar.addWidget(self.port_combo, stretch=1)

        self.refresh_ports_btn = QtWidgets.QPushButton("Refresh ports")
        self.refresh_ports_btn.clicked.connect(lambda: self.refresh_ports(keep_selection=True))
        bar.addWidget(self.refresh_ports_btn)

        bar.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(COMMON_BAUDS)
        self.baud_combo.setCurrentText(default_baud)
        self.baud_combo.setEditable(True)
        bar.addWidget(self.baud_combo)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        bar.addWidget(self.connect_btn)

        outer.addLayout(bar)

        bar2 = QtWidgets.QHBoxLayout()
        self.port_details_label = QtWidgets.QLabel("Click \"Refresh ports\" to scan for connected devices.")
        self.port_details_label.setStyleSheet("color: gray;")
        self.port_details_label.setWordWrap(True)
        bar2.addWidget(self.port_details_label, stretch=1)
        outer.addLayout(bar2)

        bar3 = QtWidgets.QHBoxLayout()
        self.save_checkbox = QtWidgets.QCheckBox("Save raw log to file")
        bar3.addWidget(self.save_checkbox)
        self.save_path_btn = QtWidgets.QPushButton("Choose...")
        self.save_path_btn.clicked.connect(self.choose_save_path)
        bar3.addWidget(self.save_path_btn)
        self.save_path = None
        self.save_path_label = QtWidgets.QLabel("(no file selected)")
        self.save_path_label.setStyleSheet("color: gray;")
        bar3.addWidget(self.save_path_label)

        bar3.addStretch(1)

        export_btn = QtWidgets.QPushButton("Export session CSV...")
        export_btn.clicked.connect(self.export_csv)
        bar3.addWidget(export_btn)

        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_all)
        bar3.addWidget(clear_btn)

        outer.addLayout(bar3)

        # Populate the port list right away so the dropdown isn't empty on launch.
        QtCore.QTimer.singleShot(0, lambda: self.refresh_ports(keep_selection=False, preferred=default_port))

        return outer

    def refresh_ports(self, keep_selection=True, preferred=None):
        """Rescan the system for connected serial ports and repopulate the dropdown."""
        current = preferred if preferred is not None else self.port_combo.currentText().strip()

        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        if _list_ports is None:
            self.port_combo.addItem(current or "COM4")
            self.port_details_label.setText(
                "pyserial is not installed, so ports can't be auto-detected. "
                "Run: pip install pyserial --break-system-packages")
            self.port_details_label.setStyleSheet("color: darkorange;")
            self.port_combo.blockSignals(False)
            return

        ports = list_available_ports()
        self._ports_info = {p["device"]: p for p in ports}

        if not ports:
            self.port_combo.addItem(current or "")
            self.port_details_label.setText(
                "No serial ports detected. Plug in your GNSS receiver/USB adapter and click \"Refresh ports\".")
            self.port_details_label.setStyleSheet("color: gray;")
        else:
            select_index = 0
            for i, p in enumerate(ports):
                self.port_combo.addItem(format_port_label(p), p["device"])
                if p["device"] == current:
                    select_index = i
            if keep_selection or preferred:
                self.port_combo.setCurrentIndex(select_index)
            self.port_details_label.setStyleSheet("color: gray;")

        self.port_combo.blockSignals(False)
        self._update_port_details()

    def _update_port_details(self, *_):
        device = self._current_port_device()
        info = getattr(self, "_ports_info", {}).get(device)
        if info:
            self.port_details_label.setText(format_port_details(info))
        elif hasattr(self, "_ports_info") and self._ports_info:
            self.port_details_label.setText(
                "Custom/typed port name — not in the detected list above.")
        # else: leave whatever status message refresh_ports already set

    def _current_port_device(self):
        """Return the actual device string (e.g. 'COM4') for whatever is selected/typed."""
        idx = self.port_combo.currentIndex()
        data = self.port_combo.itemData(idx) if idx >= 0 else None
        if data:
            return data
        return self.port_combo.currentText().strip()

    def _build_left_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        fix_box = QtWidgets.QGroupBox("Position && Fix")
        form = QtWidgets.QFormLayout(fix_box)
        self.lbl_time = QtWidgets.QLabel("--")
        self.lbl_lat = QtWidgets.QLabel("--")
        self.lbl_lon = QtWidgets.QLabel("--")
        self.lbl_alt = QtWidgets.QLabel("--")
        self.lbl_ellip = QtWidgets.QLabel("--")
        self.lbl_quality = QtWidgets.QLabel("--")
        self.lbl_sats = QtWidgets.QLabel("--")
        self.lbl_hdop = QtWidgets.QLabel("--")
        self.lbl_speed = QtWidgets.QLabel("--")
        self.lbl_course = QtWidgets.QLabel("--")

        big_font = QtGui.QFont()
        big_font.setPointSize(11)
        big_font.setBold(True)
        for lbl in (self.lbl_lat, self.lbl_lon):
            lbl.setFont(big_font)

        form.addRow("UTC Time:", self.lbl_time)
        form.addRow("Latitude:", self.lbl_lat)
        form.addRow("Longitude:", self.lbl_lon)
        form.addRow("Altitude (MSL):", self.lbl_alt)
        form.addRow("Ellipsoidal height:", self.lbl_ellip)
        form.addRow("Fix quality:", self.lbl_quality)
        form.addRow("Satellites used:", self.lbl_sats)
        form.addRow("HDOP:", self.lbl_hdop)
        form.addRow("Speed:", self.lbl_speed)
        form.addRow("Course:", self.lbl_course)
        layout.addWidget(fix_box)

        sat_box = QtWidgets.QGroupBox("Satellites in view")
        sat_layout = QtWidgets.QVBoxLayout(sat_box)
        self.sat_table = QtWidgets.QTableWidget(0, 5)
        self.sat_table.setHorizontalHeaderLabels(["PRN", "System", "Elev", "Az", "SNR"])
        self.sat_table.horizontalHeader().setStretchLastSection(True)
        self.sat_table.verticalHeader().setVisible(False)
        self.sat_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        sat_layout.addWidget(self.sat_table)
        layout.addWidget(sat_box, stretch=1)

        return panel

    def _build_right_panel(self):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)

        self.figure = Figure(figsize=(6, 5))
        self.canvas = FigureCanvas(self.figure)
        self.ax_track = self.figure.add_subplot(211)
        self.ax_alt = self.figure.add_subplot(212)
        self.figure.tight_layout(pad=2.5)
        layout.addWidget(self.canvas)

        self._init_plots()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setInterval(1000)  # redraw at most once per second
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start()

        return panel

    def _init_plots(self):
        self.ax_track.set_title("Live track")
        self.ax_track.set_xlabel("Longitude")
        self.ax_track.set_ylabel("Latitude")
        self.ax_alt.set_title("Altitude (MSL, m)")
        self.ax_alt.set_xlabel("Fix #")

    # -- connection management -------------------------------------------

    def toggle_connection(self):
        if self.worker is not None:
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        port = self._current_port_device()
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            self.set_status("error", "Invalid baud rate")
            return
        if not port:
            self.set_status("error", "Select or enter a port name, e.g. COM4")
            return

        save_path = self.save_path if self.save_checkbox.isChecked() else None

        self.worker = SerialWorker(port, baud, save_path=save_path)
        self.worker.raw_line.connect(self.on_raw_line)
        self.worker.state_changed.connect(self.on_state_changed)
        self.worker.status_changed.connect(self.on_status_changed)
        self.worker.finished_reading.connect(self.on_worker_finished)
        self.worker.start()

        self.connect_btn.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.refresh_ports_btn.setEnabled(False)
        self.baud_combo.setEnabled(False)

    def disconnect_port(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2000)
        self.connect_btn.setText("Connect")
        self.port_combo.setEnabled(True)
        self.refresh_ports_btn.setEnabled(True)
        self.baud_combo.setEnabled(True)

    def on_worker_finished(self):
        self.worker = None
        self.connect_btn.setText("Connect")
        self.port_combo.setEnabled(True)
        self.refresh_ports_btn.setEnabled(True)
        self.baud_combo.setEnabled(True)

    def choose_save_path(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save raw NMEA log as...", "nmea_capture.nmea",
            "NMEA log (*.nmea *.txt);;All files (*)")
        if path:
            self.save_path = path
            self.save_path_label.setText(path)
            self.save_checkbox.setChecked(True)

    # -- live update slots ------------------------------------------------

    def on_status_changed(self, level, message):
        self.set_status(level, message)

    def set_status(self, level, message):
        colors = {"ok": "green", "error": "red", "info": "gray"}
        self.status_label.setStyleSheet(f"color: {colors.get(level, 'black')};")
        self.status_label.setText(message)

    def on_raw_line(self, line):
        self.line_count += 1
        self.console.appendPlainText(line)

    def on_state_changed(self, state: LiveNmeaState):
        self.lbl_time.setText(self._fmt_time(state.time, state.date))
        self.lbl_lat.setText(f"{state.lat:.6f}" if state.lat is not None else "--")
        self.lbl_lon.setText(f"{state.lon:.6f}" if state.lon is not None else "--")
        self.lbl_alt.setText(f"{state.alt_msl:.2f} m" if state.alt_msl is not None else "--")
        if state.alt_msl is not None and state.geoid_sep is not None:
            self.lbl_ellip.setText(f"{state.alt_msl + state.geoid_sep:.2f} m")
        self.lbl_quality.setText(FIX_QUALITY.get(state.quality, str(state.quality)))
        self.lbl_sats.setText(str(state.num_sats) if state.num_sats is not None else "--")
        self.lbl_hdop.setText(f"{state.hdop:.2f}" if state.hdop is not None else "--")
        if state.speed_kn is not None:
            self.lbl_speed.setText(f"{state.speed_kn:.2f} kn ({state.speed_kn * 1.852:.2f} km/h)")
        if state.course is not None:
            self.lbl_course.setText(f"{state.course:.1f}°")

        self._update_sat_table(state.sats)

        if state.lat is not None and state.lon is not None:
            self.track_lats.append(state.lat)
            self.track_lons.append(state.lon)
            # d = distance(state.lat, state.lon, lat_ref=60, long_ref=6)
            # print(d)          # show distance of current position versus a reference point. Needs to be added to the gui
            if state.alt_msl is not None:
                self.alt_history.append(state.alt_msl)
            if state.hdop is not None:
                self.hdop_history.append(state.hdop)
            self.session_fixes.append((
                state.time, state.lat, state.lon, state.alt_msl,
                state.hdop, state.num_sats, state.quality,
            ))

    def _update_sat_table(self, sats):
        rows = sorted(sats.items(), key=lambda kv: -(kv[1]["snr"] or -1))
        self.sat_table.setRowCount(len(rows))
        for r, (prn, info) in enumerate(rows):
            snr = info["snr"]
            vals = [prn, info["system"] or "--", info["elev"] or "--",
                    info["az"] or "--", str(snr) if snr is not None else "--"]
            for c, v in enumerate(vals):
                item = QtWidgets.QTableWidgetItem(str(v))
                if c == 4 and snr is not None:
                    # simple signal-quality color coding
                    if snr >= 40:
                        item.setBackground(QtGui.QColor(200, 255, 200))
                    elif snr >= 25:
                        item.setBackground(QtGui.QColor(255, 255, 200))
                    else:
                        item.setBackground(QtGui.QColor(255, 210, 210))
                self.sat_table.setItem(r, c, item)

    @staticmethod
    def _fmt_time(time_str, date_str):
        if not time_str:
            return "--"
        try:
            hh, mm, ss = time_str[0:2], time_str[2:4], time_str[4:6]
            t = f"{hh}:{mm}:{ss} UTC"
        except Exception:
            return time_str
        if date_str and len(date_str) == 6:
            dd, mon, yy = date_str[0:2], date_str[2:4], date_str[4:6]
            return f"20{yy}-{mon}-{dd} {t}"
        return t

    # -- plotting -----------------------------------------------------------

    def refresh_plots(self):
        LONG_ROUND = 1000
        LAT_ROUND = 10000

        def minmax(data, ROUND):
            x_min = (floor(min(data) * ROUND)) / ROUND
            x_max = (ceil(max(data) * ROUND)) / ROUND
            return x_min, x_max

        if not self.track_lats:
            return
        self.ax_track.clear()
        self.ax_track.plot(list(self.track_lons), list(self.track_lats), "b.-", markersize=3)
        self.ax_track.plot(self.track_lons[-1], self.track_lats[-1], "ro", markersize=6)

        self.ax_track.set_xlim(minmax(self.track_lons, LONG_ROUND))
        self.ax_track.set_ylim(minmax(self.track_lats, LAT_ROUND))

        self.ax_track.set_title("Live track")
        self.ax_track.set_xlabel("Longitude")
        self.ax_track.set_ylabel("Latitude")
        self.ax_track.ticklabel_format(useOffset=False, style="plain")

        self.ax_alt.clear()
        if self.alt_history:
            self.ax_alt.plot(list(self.alt_history), "orange")
        self.ax_alt.set_title("Altitude (MSL, m)")
        self.ax_alt.set_xlabel("Fix #")

        self.figure.tight_layout(pad=2.5)
        self.canvas.draw_idle()

    # -- export / reset -------------------------------------------------

    def export_csv(self):
        if not self.session_fixes:
            QtWidgets.QMessageBox.information(self, "Export CSV", "No fixes captured yet.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export session as CSV", "nmea_session.csv", "CSV files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["time_utc", "lat", "lon", "alt_msl_m", "hdop", "num_sats", "fix_quality"])
            writer.writerows(self.session_fixes)
        QtWidgets.QMessageBox.information(self, "Export CSV", f"Saved {len(self.session_fixes)} fixes to:\n{path}")

    def clear_all(self):
        self.track_lats.clear()
        self.track_lons.clear()
        self.alt_history.clear()
        self.hdop_history.clear()
        self.session_fixes.clear()
        self.console.clear()
        self.sat_table.setRowCount(0)
        self.refresh_plots()

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2000)
        event.accept()

def distance(lat, long, lat_ref, long_ref):
    """ calculates distance between 2 points"""
    # d = 2R × sin⁻¹(√[sin²((θ₂ - θ₁)/2) + cosθ₁ × cosθ₂ × sin²((φ₂ - φ₁)/2)])
    # θ₁, φ₁ – First point latitude and longitude coordinates;
    # θ₂, φ₂ – Second point latitude and longitude coordinates;
    R = 6_371_000   #  Earth's radius: 6371 km but in m. Note, depends where you are on earth.
    a = sin(  (radians(lat_ref) - radians(lat))/2.0  )**2
    b = cos(radians(lat)) * cos(radians(lat_ref)) * sin( (radians(long) - radians(long_ref))/2.0)**2
    d = 2 * R * asin( sqrt(a + b ) )
    return d

# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Real-time Qt GUI for NMEA GNSS data")
    parser.add_argument("--port", default="COM4", help="Default port shown in the UI (default: COM4)")
    parser.add_argument("--baud", default="9600", help="Default baud rate shown in the UI (default: 9600)")
    parser.add_argument("--autoconnect", action="store_true",
                         help="Automatically connect on launch using --port/--baud")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(default_port=args.port, default_baud=args.baud)
    win.show()

    if args.autoconnect:
        QtCore.QTimer.singleShot(200, win.connect_port)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
