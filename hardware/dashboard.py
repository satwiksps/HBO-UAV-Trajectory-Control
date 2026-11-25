"""Local controller and experiment dashboard. Use --sim for a labelled practice rig."""
import argparse
import json
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
import sys
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets as Q
import pyqtgraph as pg

from experiments import DEMO_START, MATCHED_PAIRS, ROOT, GuidedComparison, Runner, Session
from hbo import BOUNDS, unit
from make_plots import export, read_csv
from results import mean_field, trial_groups, valid_trials
from rig_link import (Calibration, DEFAULT_CAP, FAULTS, GAINS, HARD_CAP, LIFT_US, RIG_MAX_ANGLE, START_TRIM_US,
                      SerialRig, calibrate, capture_pose)
from rig_sim import SIM_CAL, SimRig

BG = "#f5f6f3"
CARD = "#ffffff"
EDGE = "#cbd3cc"
PLOT = "#ffffff"
INK = "#202d2b"
MID = "#52625d"
TEAL = "#126b54"
AMBER = "#a85914"
RED = "#a42329"
BLUE = "#285e9a"
CAL_FILE = ROOT / "calibration.json"

STYLE = f"""
QWidget {{ color:{INK}; background:{BG}; font-family:'Segoe UI'; font-size:13px; }}
QMainWindow {{ background:{BG}; }}
QFrame#card {{ background:{CARD}; border:1px solid {EDGE}; }}
QFrame#card QLabel {{ background:transparent; }}
QLabel#eyebrow {{ font-family:'Consolas'; font-size:11px; font-weight:700; color:{TEAL}; }}
QLabel#display {{ font-size:26px; font-weight:700; }}
QLabel#muted {{ color:{MID}; }}
QLabel#section {{ font-size:15px; font-weight:650; }}
QLabel#metric {{ font-size:23px; font-weight:650; }}
QLabel#label {{ font-size:12px; color:{MID}; }}
QPushButton {{ background:{CARD}; border:1px solid #a9b9c4;
               padding:5px 9px; font-weight:600; min-height:16px; }}
QPushButton:hover {{ background:#e6eef3; }}
QPushButton:disabled {{ background:#edf0f2; color:#61717b; border-color:#d4dde3; }}
QPushButton#primary {{ background:#086b65; color:white; border-color:#086b65; }}
QPushButton#primary:hover {{ background:#07564f; }}
QPushButton#primary:disabled {{ background:#e1eae9; color:#566c6a; border-color:#cbd8d7; }}
QPushButton#stop {{ background:#fff2f2; color:#8d1c21; border-color:#ad2a30; padding:6px 14px; }}
QPushButton#stop:hover {{ background:#ffdddd; }}
QTabWidget::pane {{ border:0; margin-top:5px; }}
QTabBar::tab {{ padding:7px 13px; margin-right:3px; color:{MID}; background:{BG};
               border-bottom:2px solid {EDGE}; }}
QTabBar::tab:selected {{ color:{TEAL}; border-color:{TEAL}; font-weight:650; }}
QTabBar::tab:hover {{ color:{INK}; }}
QDoubleSpinBox, QSpinBox, QComboBox {{ background:{CARD}; border:1px solid #acc0cc;
                                      padding:4px 6px; min-height:16px;
                                      selection-background-color:#bbdfdb; }}
QComboBox QAbstractItemView {{ background:{CARD}; color:{INK}; selection-background-color:#bbdfdb; }}
QTableWidget {{ background:{PLOT}; alternate-background-color:#edf3f6;
                border:1px solid {EDGE}; gridline-color:{EDGE}; }}
QHeaderView::section {{ background:#e5edf2; color:{MID}; border:0; padding:5px; }}
QTableWidget::item {{ padding-left:5px; }}
QScrollArea {{ border:0; }}
"""


def label(text, style="", wrap=False):
    widget = Q.QLabel(text)
    widget.setObjectName(style)
    widget.setWordWrap(wrap)
    return widget


def button(text, action, style=""):
    b = Q.QPushButton(text)
    b.setObjectName(style)
    b.clicked.connect(lambda _checked=False: action())
    return b


def card(title=None):
    widget = Q.QFrame()
    widget.setObjectName("card")
    box = Q.QVBoxLayout(widget)
    box.setContentsMargins(11, 8, 11, 8)
    box.setSpacing(5)
    if title:
        box.addWidget(label(title, "section"))
    return widget, box


def decimal(lo, hi, value, decimals=1, step=1, suffix=""):
    w = Q.QDoubleSpinBox()
    w.setDecimals(decimals)
    w.setRange(lo, hi)
    w.setSingleStep(step)
    w.setSuffix(suffix)
    w.setValue(value)
    w.setKeyboardTracking(False)
    return w


def integer(lo, hi, value, suffix=""):
    w = Q.QSpinBox()
    w.setRange(lo, hi)
    w.setValue(value)
    w.setSuffix(suffix)
    w.setKeyboardTracking(False)
    return w


class TracePlot(pg.PlotWidget):
    def sizeHint(self):
        return QtCore.QSize(490, 160)

    def minimumSizeHint(self):
        return QtCore.QSize(185, self.minimumHeight())


def plot(x, y, legend=False):
    w = TracePlot(background=PLOT)
    w.setSizePolicy(Q.QSizePolicy.Policy.Expanding, Q.QSizePolicy.Policy.Expanding)
    w.setMinimumHeight(144)
    p = w.getPlotItem()
    p.setMenuEnabled(False)
    p.hideButtons()
    p.showGrid(x=True, y=True, alpha=.25)
    p.setLabel("bottom", x, color=MID)
    p.setLabel("left", y, color=MID)
    for side in ("left", "bottom"):
        p.getAxis(side).setPen(EDGE)
        p.getAxis(side).setTextPen(MID)
    if legend:
        p.addLegend(offset=(10, 8), labelTextColor=MID, labelTextSize="9pt")
    return w


class Dashboard(Q.QMainWindow):
    def __init__(self, simulated=False, port=None, data_dir=None, speed=1.0, comparison=None):
        super().__init__()
        self.simulated = simulated
        self.speed = speed if simulated else 1.0
        self.port_requested = port
        self.data_dir = data_dir
        self.runner = None
        self.calibration = SIM_CAL if simulated else self.load_calibration()
        self.comparison = comparison
        if comparison and self.calibration:
            if comparison.trim >= self.calibration.cap:
                raise ValueError("The comparison trim must be below the current ESC command cap.")
            self.calibration = replace(self.calibration, hover=comparison.trim)
        self.poses = {}
        self.pending_pose = None
        self.confirmed_calibration = False
        self.raw = deque(maxlen=80)
        self.samples = deque(maxlen=1100)
        self.last_seq = None
        self.last_plot = 0.0
        self.last_trial_count = -1
        self.last_map = None
        self.sent_saved = False
        self.elapsed_reference = 0.0
        self.last_tick = 0.0
        self.controls_error = ""
        self.export_pool = ThreadPoolExecutor(max_workers=1)
        self.export_future = None
        self.export_path = None
        self.review_path = None
        self.review_key = None
        self.setWindowTitle(("SIMULATION · " if simulated else "") + "Propeller Arm")
        self.setMinimumSize(620, 425)
        screen = Q.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1280, screen.width()-28), min(790, screen.height()-55))
        self.setup()
        self.clock = QtCore.QTimer(self)
        self.clock.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.clock.timeout.connect(self.update_loop)
        self.clock.start(20)
        self.kill_key = QtGui.QShortcut(QtGui.QKeySequence("Space"), self)
        self.kill_key.activated.connect(self.stop)
        if simulated:
            self.connect_rig()
        self.refresh()

    def load_calibration(self):
        if not CAL_FILE.exists():
            return None
        try:
            saved = Calibration.load(CAL_FILE)
            limit = min(saved.upper, saved.operating_limit or saved.upper, RIG_MAX_ANGLE)
            return replace(saved, operating_limit=limit) if limit >= 8.5 else None
        except (ValueError, OSError, TypeError):
            return None

    def setup(self):
        body = Q.QWidget()
        self.setCentralWidget(body)
        layout = Q.QVBoxLayout(body)
        layout.setContentsMargins(17, 13, 17, 11)
        layout.setSpacing(6)
        head = Q.QHBoxLayout()
        title_block = Q.QVBoxLayout()
        title_block.setSpacing(0)
        title = label("Propeller arm", "display")
        title_block.addWidget(title)
        title_block.addWidget(label("HBO PID TUNING", "eyebrow"))
        head.addLayout(title_block)
        head.addStretch()
        self.source = label("SIMULATION" if self.simulated else "HARDWARE")
        color = AMBER if self.simulated else TEAL
        self.source.setStyleSheet(f"color:{color};background:{CARD};border:1px solid {EDGE};"
                                  "padding:5px 9px;font-weight:bold;")
        head.addWidget(self.source, 0, QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.stop_button = button("STOP   Space", self.stop, "stop")
        head.addWidget(self.stop_button)
        layout.addLayout(head)
        self.tabs = Q.QTabWidget()
        for page, name in ((self.live_page, "PID bench"), (self.tune_page, "Bayesian tuning"),
                           (self.results_page, "Results"), (self.cal_page, "Calibration"),
                           (self.diagnostic_page, "Sensors"), (self.data_page, "Data")):
            scroll = Q.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setWidget(page())
            self.tabs.addTab(scroll, name)
        layout.addWidget(self.tabs, 1)
        self.footer = label("Disconnected.", "muted", True)
        self.footer.setMinimumHeight(24)
        layout.addWidget(self.footer)
        self.reflow()

    def reflow(self):
        if not hasattr(self, "cal_columns"):
            return
        narrow = self.width() < 1100
        narrow_live = self.width() < 800
        layout_key = (narrow, narrow_live)
        if getattr(self, "_layout_narrow", None) == layout_key:
            return
        self._layout_narrow = layout_key
        direction = Q.QBoxLayout.Direction.TopToBottom if narrow else Q.QBoxLayout.Direction.LeftToRight
        for columns in (self.tune_columns, self.cal_columns):
            columns.setDirection(direction)
        if self.tune_columns.itemAt(0).layout() is not (self.tune_controls if narrow else self.tune_charts):
            self.tune_columns.removeItem(self.tune_controls)
            self.tune_columns.removeItem(self.tune_charts)
            tuning_order = ((self.tune_controls, self.tune_charts) if narrow else
                            (self.tune_charts, self.tune_controls))
            for panel in tuning_order:
                self.tune_columns.addLayout(panel, 3 if panel is self.tune_charts else 1)
        self.live_columns.setDirection(
            Q.QBoxLayout.Direction.TopToBottom if narrow_live else Q.QBoxLayout.Direction.LeftToRight)
        # Keep the controls ahead of the plots when stacked on narrow screens.
        if self.live_columns.itemAt(0).layout() is not (self.live_controls if narrow_live else self.live_charts):
            self.live_columns.removeItem(self.live_controls)
            self.live_columns.removeItem(self.live_charts)
            order = ((self.live_controls, self.live_charts) if narrow_live else
                     (self.live_charts, self.live_controls))
            for item in order:
                self.live_columns.addLayout(item, int(item is self.live_charts))
        for panel in self.side_panels:
            panel.setMaximumWidth(16777215 if narrow_live else 330)
        for idx, tile in enumerate(self.metric_cards):
            self.metric_grid.addWidget(tile, idx//2 if narrow_live else 0, idx%2 if narrow_live else idx)
        for idx, control in enumerate(self.sensor_test_buttons):
            self.sensor_test_grid.addWidget(control, idx//2 if narrow_live else 0, idx%2 if narrow_live else idx)
        for idx, control in enumerate(self.data_actions):
            self.data_action_grid.addWidget(control, idx if narrow_live else 0, 0 if narrow_live else idx)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.reflow()

    def live_page(self):
        w = Q.QWidget()
        root = Q.QVBoxLayout(w)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        self.metric_grid = Q.QGridLayout()
        self.metric_cards = []
        self.tiles = []
        for name in ("Arm angle", "Target", "Tracking error", "ESC pulse"):
            tile, box = card()
            box.setSpacing(2)
            box.addWidget(label(name, "label"))
            value = label("—", "metric")
            box.addWidget(value)
            self.tiles.append(value)
            self.metric_cards.append(tile)
        root.addLayout(self.metric_grid)
        main = Q.QHBoxLayout()
        left = Q.QVBoxLayout()
        self.live_columns, self.live_charts = main, left
        left.setSpacing(8)
        traces, chart = card("Closed-loop angle")
        self.angle_plot = plot("Recent time [s]", "Angle [°]", True)
        self.angle_plot.setMinimumHeight(155)
        self.angle_plot.setMouseEnabled(x=False, y=False)
        self.target_line = self.angle_plot.plot(pen=pg.mkPen(AMBER, width=1.4, style=QtCore.Qt.PenStyle.DashLine),
                                                name="Target")
        self.angle_line = self.angle_plot.plot(pen=pg.mkPen(TEAL, width=2), name="Measured")
        self.angle_band = pg.LinearRegionItem((-.99, .99), orientation="horizontal",
                                              movable=False, brush=pg.mkBrush(18, 107, 84, 20),
                                              pen=pg.mkPen(None))
        self.angle_band.setZValue(-10)
        self.angle_plot.addItem(self.angle_band, ignoreBounds=True)
        self.angle_band.hide()
        self.angle_plot.setXRange(-12, 0, padding=0)
        chart.addWidget(self.angle_plot, 1)
        left.addWidget(traces, 3)
        cmd, chart = card()
        self.output_selector = Q.QComboBox()
        self.output_selector.addItems(("ESC command", "P / I / D terms"))
        self.output_selector.currentIndexChanged.connect(self.show_terms)
        chart.addWidget(self.output_selector)
        self.output_stack = Q.QStackedWidget()
        self.pulse_plot = plot("Recent time [s]", "Pulse [µs]")
        self.pulse_plot.setMinimumHeight(110)
        self.pulse_plot.setMouseEnabled(x=False, y=False)
        self.pulse_plot.setXLink(self.angle_plot)
        self.pulse_line = self.pulse_plot.plot(pen=pg.mkPen(BLUE, width=1.8))
        self.cap_line = pg.InfiniteLine(angle=0, pos=DEFAULT_CAP, label="ESC protection limit",
                                       labelOpts={"color": RED, "position": .13},
                                       pen=pg.mkPen(RED, style=QtCore.Qt.PenStyle.DotLine))
        self.pulse_plot.addItem(self.cap_line)
        self.pulse_plot.setYRange(941, 1160, padding=0)
        self.output_stack.addWidget(self.pulse_plot)
        self.term_plot = plot("Recent time [s]", "Contribution [µs]", True)
        self.term_plot.setMinimumHeight(110)
        self.term_plot.setMouseEnabled(x=False, y=False)
        self.term_plot.setXLink(self.angle_plot)
        self.term_lines = [self.term_plot.plot(pen=pg.mkPen(color, width=1.5), name=name)
                           for name, color in (("P", TEAL), ("I", AMBER), ("D", BLUE))]
        self.output_stack.addWidget(self.term_plot)
        chart.addWidget(self.output_stack, 1)
        left.addWidget(cmd, 2)
        main.addLayout(left, 1)
        side = Q.QVBoxLayout()
        self.live_controls = side
        side.setSpacing(8)
        rig, controls = card("Rig connection")
        rig.setMinimumWidth(265)
        connection = Q.QHBoxLayout()
        self.ports = Q.QComboBox()
        connection.addWidget(self.ports, 1)
        self.connect_button = button("Connect", self.toggle_connect)
        connection.addWidget(self.connect_button)
        refresh = button("Rescan", self.fill_ports)
        refresh.setToolTip("Rescan serial ports")
        refresh.setVisible(not self.simulated)
        connection.addWidget(refresh)
        controls.addLayout(connection)
        self.fill_ports()
        self.rig_state = label("Disconnected", "label", True)
        controls.addWidget(self.rig_state)
        self.arm_button = button("Arm rig", self.arm, "primary")
        controls.addWidget(self.arm_button)
        self.arm_hint = label("", "label", True)
        controls.addWidget(self.arm_hint)
        side.addWidget(rig)
        pid, pane = card("PID settings")
        pid.setMinimumWidth(265)
        self.side_panels = (rig, pid)
        form = Q.QFormLayout()
        form.setHorizontalSpacing(11)
        form.setVerticalSpacing(5)
        self.target = decimal(-82, 81, DEMO_START, 1, 1, " °")
        form.addRow("Target angle", self.target)
        self.gain_inputs = []
        gain_rows = Q.QGridLayout()
        gain_rows.setContentsMargins(0, 0, 0, 0)
        gain_rows.setVerticalSpacing(2)
        for index, (name, v, high) in enumerate(zip(("Kp", "Ki", "Kd"), GAINS, (2, 1, 1))):
            entry = decimal(0, high, v, 4, .01)
            self.gain_inputs.append(entry)
            caption = label(name, "label")
            caption.setBuddy(entry)
            gain_rows.addWidget(caption, 0, index)
            gain_rows.addWidget(entry, 1, index)
        pane.addLayout(form)
        pane.addLayout(gain_rows)
        self.hold_button = button("Hold angle", lambda: self.act("start_hold", self.target.value(), self.gains()), "primary")
        self.retarget_button = button("Apply target", lambda: self.act("change_target", self.target.value()))
        self.retarget_button.setToolTip("Move the live setpoint at up to 2° per second without stopping the hold.")
        hold_controls = Q.QHBoxLayout()
        hold_controls.addWidget(self.hold_button)
        hold_controls.addWidget(self.retarget_button)
        pane.addLayout(hold_controls)
        presets = Q.QHBoxLayout()
        self.controller_presets = [button(name, lambda name=name: self.controller_preset(name))
                                   for name in ("P", "PD", "PID")]
        for widget in self.controller_presets:
            presets.addWidget(widget)
        pane.addLayout(presets)
        self.dip_button = button("Apply −4 µs dip", lambda: self.act("request_dip"))
        self.demo_button = button("Run tracking demo", lambda: self.act("start_demo", False), "primary")
        perturb = Q.QHBoxLayout()
        perturb.addWidget(self.dip_button)
        perturb.addWidget(self.demo_button)
        pane.addLayout(perturb)
        side.addWidget(pid)
        side.addStretch()
        main.addLayout(side)
        root.addLayout(main, 1)
        return w

    def show_terms(self, index):
        self.output_stack.setCurrentIndex(index)

    def controller_preset(self, name):
        kp, ki, kd = GAINS
        values = (kp, ki if name == "PID" else 0, kd if name != "P" else 0)
        for entry, value in zip(self.gain_inputs, values):
            entry.setValue(value)
        self.controls_error = ""
        if self.runner:
            self.runner.status = f"{name} preset loaded. Arm and choose Hold angle."
        self.refresh()

    def tune_page(self):
        w = Q.QWidget()
        root = Q.QHBoxLayout(w)
        self.tune_columns = root
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        left = Q.QVBoxLayout()
        self.tune_charts = left
        self.tuning_views = Q.QTabWidget()
        progress, view = card()
        self.tuning_plot = plot("Search trial", "Tracking score ↓", True)
        self.random_points = self.tuning_plot.plot(pen=None, symbol="o", symbolBrush=None,
                                                   symbolPen=pg.mkPen(MID, width=1.5), name="Random")
        self.guided_points = self.tuning_plot.plot(pen=None, symbol="o", symbolBrush=TEAL,
                                                   symbolPen=None, name="Guided")
        self.fault_points = self.tuning_plot.plot(pen=None, symbol="x", symbolPen=pg.mkPen(RED, width=1.5))
        self.best_line = self.tuning_plot.plot(pen=pg.mkPen(AMBER, width=1.6), name="Best completed")
        view.addWidget(self.tuning_plot, 1)
        self.tuning_views.addTab(progress, "Progress")
        self.map_views = []
        self.images = []
        self.map_points = []
        self.map_picks = []
        self.map_status = []
        self.color_bars = []
        for name, units in (("Score map", "Score"), ("Noise map", "σ in log score")):
            panel, charts = card()
            self.map_status.append(label("Available after 8 search trials.", "muted"))
            charts.addWidget(self.map_status[-1])
            v = plot("Kp [µs/°]", "Kd [µs·s/°]")
            p = v.getPlotItem()
            p.setRange(xRange=(-.045, 1.045), yRange=(-.045, 1.045), padding=0)
            p.setLimits(xMin=-.045, xMax=1.045, yMin=-.045, yMax=1.045)
            for index, side in zip((0, 2), ("bottom", "left")):
                p.getAxis(side).setTicks([[(u, f"{val:.2g}") for u, val in zip(
                    np.linspace(0, 1, 3), np.geomspace(*BOUNDS[index], 3))]])
            im = pg.ImageItem()
            p.addItem(im)
            points = pg.ScatterPlotItem(size=5, brush=pg.mkBrush(INK), pen=pg.mkPen(PLOT))
            p.addItem(points)
            pick = pg.ScatterPlotItem(size=13, symbol="star", brush=pg.mkBrush(AMBER), pen=pg.mkPen(PLOT))
            p.addItem(pick)
            cbo = pg.ColorBarItem(values=(0, 1), colorMap=pg.colormap.get("viridis"),
                                  label=units, interactive=False, width=13)
            cbo.setImageItem(im, insert_in=p)
            self.color_bars.append(cbo)
            self.map_views.append(v)
            self.images.append(im)
            self.map_points.append(points)
            self.map_picks.append(pick)
            charts.addWidget(v, 1)
            self.tuning_views.addTab(panel, name)
        left.addWidget(self.tuning_views, 4)
        root.addLayout(left, 3)
        right = Q.QVBoxLayout()
        self.tune_controls = right
        runs, box = card("Experiments")
        runs.setMinimumWidth(300)
        runs.setMaximumWidth(365)
        form = Q.QFormLayout()
        self.trial_targets = []
        for name, value in zip(("Start", "Step up", "Step down"), (0, 20, 10)):
            field = decimal(-82, 81, value, 1, 1, " °")
            self.trial_targets.append(field)
            form.addRow(name, field)
        box.addLayout(form)
        self.baseline_button = button("Step test · 5 baseline runs",
                                      lambda: self.act("start_baseline", self.gains(), self.profile()))
        self.hbo_button = button("Run HBO · 30 trials + retests",
                                 lambda: self.act("start_hbo", self.profile()), "primary")
        self.repeat_button = button("Repeat test · 5 per method",
                                    lambda: self.act("start_repeat", self.profile(), self.gains()))
        self.load_pick_button = button("Load recommended gains", self.load_pick)
        for b in (self.baseline_button, self.hbo_button, self.repeat_button, self.load_pick_button):
            box.addWidget(b)
            b.setVisible(self.comparison is None)
        self.compare_button = button(f"Run comparison · {MATCHED_PAIRS} pairs",
                                     lambda: self.act("start_matched_hbo", self.comparison), "primary")
        box.addWidget(self.compare_button)
        self.compare_button.setVisible(self.comparison is not None)
        comparison = self.comparison
        if comparison:
            for entry, value in zip(self.trial_targets, comparison.targets):
                entry.setValue(value)
            manual_gains = " / ".join(f"{v:.5g}" for v in comparison.baseline)
            candidate_gains = " / ".join(f"{v:.5g}" for v in comparison.candidate)
            explanation = (f"Shared trim {comparison.trim} µs\n"
                           f"Manual PID   {manual_gains}\n"
                           f"HBO candidate   {candidate_gains}\n"
                           "Order alternates across pairs.")
        else:
            explanation = "Establish the manual baseline before tuning."
        self.pick_label = label(explanation, "muted", True)
        box.addWidget(self.pick_label)
        right.addWidget(runs)
        right.addStretch()
        record, box = card("Completed trials")
        self.trial_table = Q.QTableWidget(0, 4)
        self.trial_table.setMinimumHeight(110)
        self.trial_table.setSizePolicy(Q.QSizePolicy.Policy.Expanding, Q.QSizePolicy.Policy.Ignored)
        self.trial_table.setHorizontalHeaderLabels(["#", "Method", "Score", "MAE °"])
        hdr = self.trial_table.horizontalHeader()
        hdr.setSectionResizeMode(1, Q.QHeaderView.ResizeMode.Stretch)
        for i in (0, 2, 3):
            hdr.setSectionResizeMode(i, Q.QHeaderView.ResizeMode.ResizeToContents)
        self.trial_table.setEditTriggers(Q.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.trial_table.setSelectionMode(Q.QAbstractItemView.SelectionMode.NoSelection)
        self.trial_table.verticalHeader().hide()
        self.trial_table.setAlternatingRowColors(True)
        box.addWidget(self.trial_table)
        left.addWidget(record, 1)
        root.addLayout(right, 1)
        return w

    def results_page(self):
        widget = Q.QWidget()
        layout = Q.QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        summary, content = card()
        content.addWidget(label("Controller performance", "display", True))
        self.review_source = label("", "eyebrow")
        self.review_source.setWordWrap(True)
        self.review_summary = label("Open a recorded session or complete a test to review measured results.", "muted", True)
        actions = Q.QHBoxLayout()
        self.review_button = button("Open run", self.choose_review)
        self.active_review_button = button("Current run", self.review_active)
        self.review_export_button = button("Export plots", self.export_review)
        for control in (self.review_button, self.active_review_button, self.review_export_button):
            actions.addWidget(control)
        actions.addStretch()
        content.addLayout(actions)
        content.addWidget(self.review_source)
        content.addWidget(self.review_summary)
        layout.addWidget(summary)
        plot_card, chart = card("Tracking score by controller")
        self.review_plot = plot("Completed runs within group", "Score ↓", True)
        self.review_plot.setMinimumHeight(155)
        chart.addWidget(self.review_plot)
        layout.addWidget(plot_card, 2)
        stats, body = card("Step-response metrics")
        labels = ("Test group", "Runs", "Score mean ± SD", "MAE °", "Final MAE °", "Overshoot °", "Ripple °", "Settled ↑ / ↓")
        self.review_table = Q.QTableWidget(0, len(labels))
        self.review_table.setHorizontalHeaderLabels(labels)
        self.review_table.setMinimumHeight(118)
        self.review_table.setSizePolicy(Q.QSizePolicy.Policy.Ignored, Q.QSizePolicy.Policy.Expanding)
        self.review_table.verticalHeader().setDefaultSectionSize(24)
        self.review_table.verticalHeader().hide()
        self.review_table.setAlternatingRowColors(True)
        self.review_table.setEditTriggers(Q.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.review_table.horizontalHeader().setSectionResizeMode(0, Q.QHeaderView.ResizeMode.Stretch)
        for column in range(1, len(labels)):
            self.review_table.horizontalHeader().setSectionResizeMode(column, Q.QHeaderView.ResizeMode.ResizeToContents)
        body.addWidget(self.review_table)
        body.addWidget(label("Settled means the response stayed within ±1° through the end of the step.", "muted", True))
        layout.addWidget(stats, 1)
        return widget

    def review_active(self):
        self.review_path = None
        self.review_key = None
        self.update_results()

    def choose_review(self):
        path = Q.QFileDialog.getExistingDirectory(self, "Open a recorded session", str(ROOT/"runs"))
        if path:
            self.open_review(Path(path))

    def export_review(self):
        if self.review_path:
            self.launch_export(self.review_path)
        else:
            self.save_plots()

    def open_review(self, path):
        try:
            metadata = json.loads((path/"session.json").read_text(encoding="utf-8"))
            rows = read_csv(path/"trials.csv")
            if metadata["source"] not in {"HARDWARE", "SIMULATION"} or any(
                    r["source"] != metadata["source"] for r in rows):
                raise ValueError("The recorded session has inconsistent data sources.")
            self.render_results(rows, metadata, path.name)
            self.review_path = path
            self.refresh()
        except (ValueError, OSError, KeyError) as exc:
            self.error(f"Could not open recorded results. {exc}")

    def update_results(self):
        if self.runner and self.review_path is None:
            session = self.runner.session
            key = (session.path, len(session.trials), self.runner.pick, "validation" in session.metadata,
                   bool(session.metadata.get("matched_hbo", {}).get("result")))
            if self.review_key != key:
                self.review_key = key
                self.render_results(session.trials, session.metadata, session.path.name)

    def render_results(self, rows, metadata, run_name):
        self.review_source.setText(f"{metadata['source']}   /   {run_name}")
        groups = trial_groups(rows, metadata)
        self.review_plot.clear()
        self.review_plot.getPlotItem().legend.clear()
        self.review_table.setRowCount(len(groups))
        colors = (BLUE, TEAL, AMBER, "#785490", "#467e95", "#a64252")
        for index, (name, trials) in enumerate(groups.items()):
            completed = valid_trials(trials)
            scores = [float(row["score"]) for row in completed]
            color = colors[index % len(colors)]
            if scores:
                self.review_plot.plot(np.arange(1, len(scores)+1), scores, pen=pg.mkPen(color, width=1),
                                      symbol="o", symbolSize=7, symbolBrush=color, symbolPen=None, name=name)
            summary = f"{np.mean(scores):.2f} ± {np.std(scores, ddof=1):.2f}" if len(scores) > 1 else (
                f"{scores[0]:.2f}" if scores else "—")
            def metric(field):
                value = mean_field(completed, field)
                return f"{value:.2f}" if value is not None else "—"
            settled = [sum(np.isfinite(float(r[f"settle_{direction}_s"])) for r in completed)
                       for direction in ("up", "down")]
            values = (name, f"{len(completed)}/{len(trials)}", summary, metric("mae"), metric("steady"),
                      metric("overshoot"), metric("ripple"),
                      f"{settled[0]}/{len(completed)} · {settled[1]}/{len(completed)}")
            for col, value in enumerate(values):
                self.review_table.setItem(index, col, Q.QTableWidgetItem(value))
        if "matched_hbo" in metadata:
            comparison = metadata["matched_hbo"]
            result = comparison.get("result")
            if result:
                verdict = ("Met the paired improvement criterion." if result["improved"] else
                           "Did not meet the paired improvement criterion.")
                message = (f"Manual {np.mean(result['baseline']):.2f} · HBO {np.mean(result['candidate']):.2f}  |  "
                           f"HBO won {result['candidate_wins']}/{comparison['planned_pairs']} pairs. {verdict}")
                if result["scored_faults"]:
                    message += f" Scored-stage faults: {result['scored_faults']}."
            else:
                message = "Paired comparison in progress or interrupted. Review the completed runs below."
        elif "validation" in metadata:
            message = ("HBO met the improvement rule in both comparisons." if metadata["validation"]["improved"] else
                       "Keep the manual PID gains. HBO did not meet the improvement rule in both comparisons.")
            validation = metadata["validation"]
            message += (f"  Manual {np.mean(validation['baseline']):.2f} · HBO {np.mean(validation['candidate']):.2f}"
                        " on independent runs.")
        elif "hbo_pick" in metadata:
            message = "The HBO candidate is recorded. Independent validation is pending."
            manual = valid_trials(groups.get("Confirmation · manual", groups.get("Manual baseline", [])))
            candidate = valid_trials(groups.get("Confirmation · HBO pick", []))
            if manual and candidate:
                message += f"  Manual {mean_field(manual, 'score'):.2f} · HBO {mean_field(candidate, 'score'):.2f} so far."
        else:
            message = "Completed step tests will appear here."
        self.review_summary.setText(message)

    def cal_page(self):
        w = Q.QWidget()
        root = Q.QHBoxLayout(w)
        self.cal_columns = root
        left = Q.QVBoxLayout()
        steps, box = card("Three-pose angle calibration")
        box.addWidget(label("Disconnect ESC power. Move the arm by hand, hold each pose still, and capture it. "
                            "Measure the stops with a level or phone angle tool.", "muted", True))
        self.pose_buttons = {}
        self.pose_labels = {}
        for name, description in (("Lower", "Motor end down at its physical stop"),
                                  ("Level", "Arm horizontal, used as 0°"),
                                  ("Upper", "Motor end up at its physical stop")):
            row = Q.QHBoxLayout()
            capture = button("Capture " + name.lower(), lambda n=name: self.capture(n))
            capture.setMinimumWidth(118)
            self.pose_buttons[name] = capture
            row.addWidget(capture)
            row.addWidget(label(description, "label", True), 1)
            self.pose_labels[name] = label("—", "label")
            row.addWidget(self.pose_labels[name])
            box.addLayout(row)
        form = Q.QFormLayout()
        c = self.calibration
        self.lower = decimal(-86, -6, c.lower if c else -33.6, 1, 1, " °")
        self.upper = decimal(7, 85, c.upper if c else 26.0, 1, 1, " °")
        self.lower.setSpecialValueText("Measure")
        self.upper.setSpecialValueText("Measure")
        form.addRow("Measured lower stop", self.lower)
        form.addRow("Measured upper stop", self.upper)
        self.operating_limit = decimal(8.5, 85 if self.simulated else RIG_MAX_ANGLE,
                                       SIM_CAL.upper if self.simulated else
                                       (c.operating_limit if c else RIG_MAX_ANGLE), 1, .5, " °")
        form.addRow("Operating ceiling", self.operating_limit)
        box.addLayout(form)
        box.addWidget(label("Capture upper at the physical stop. PID targets stay below the operating ceiling "
                            "to leave room for overshoot.", "muted", True))
        self.apply_cal = button("Apply angle calibration", self.apply_calibration, "primary")
        box.addWidget(self.apply_cal)
        self.verify_checkbox = Q.QCheckBox("I verified level and both stops")
        self.verify_checkbox.setChecked(self.simulated)
        self.verify_checkbox.toggled.connect(self.verify_calibration)
        box.addWidget(self.verify_checkbox)
        self.cal_note = label("The angle estimate will appear after calibration.", "muted", True)
        box.addWidget(self.cal_note)
        left.addWidget(steps)
        left.addStretch()
        root.addLayout(left, 3)
        right = Q.QVBoxLayout()
        setup, box = card("Actuator settings")
        form = Q.QFormLayout()
        trim = self.comparison.trim if self.comparison else (c.hover if c else START_TRIM_US)
        self.hover = integer(1120, HARD_CAP-1, trim, " µs")
        self.cap = integer(LIFT_US+1, HARD_CAP, c.cap if c else DEFAULT_CAP, " µs")
        form.addRow("Feedforward trim", self.hover)
        form.addRow("ESC command maximum", self.cap)
        box.addLayout(form)
        self.apply_trim = button("Apply pulse settings", self.apply_pulse_settings)
        box.addWidget(self.apply_trim)
        box.addWidget(label("Feedforward trim is the starting estimate of the pulse needed to hold level. PID adjusts it. "
                            "The angle guard cuts drive if the arm approaches its operating ceiling too fast.", "muted", True))
        right.addWidget(setup)
        sensor, box = card("MPU6050 readings")
        self.raw_acc = label("Acceleration    —", "label", True)
        self.raw_gyro = label("Gyroscope       —", "label", True)
        self.imu_pulse = label("Angle             —", "label", True)
        for l in (self.raw_acc, self.raw_gyro, self.imu_pulse):
            box.addWidget(l)
        right.addWidget(sensor)
        right.addStretch()
        root.addLayout(right, 2)
        return w

    def diagnostic_page(self):
        w = Q.QWidget()
        root = Q.QVBoxLayout(w)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)
        summary, content = card("Telemetry quality")
        self.sensor_test_grid = Q.QGridLayout()
        self.ramp_button = button("Open-loop ramp", lambda: self.act("start_ramp"))
        self.noise_button = button("Motor-off noise test", lambda: self.act("start_noise"))
        self.poke_button = button("Mark poke", lambda: self.disturbance("Poke"))
        self.load_button = button("Mark added load", lambda: self.disturbance("Added load"))
        self.sensor_test_buttons = (self.ramp_button, self.noise_button, self.poke_button, self.load_button)
        content.addLayout(self.sensor_test_grid)
        self.diagnostic_summary = label("Connect to inspect incoming IMU data.", "muted", True)
        content.addWidget(self.diagnostic_summary)
        root.addWidget(summary)
        tracks, content = card("Tracking error")
        self.error_plot = plot("Recent time [s]", "Target − angle [°]")
        self.error_plot.setXRange(-12, 0, padding=0)
        self.error_line = self.error_plot.plot(pen=pg.mkPen(BLUE, width=1.8))
        self.error_plot.addItem(pg.InfiniteLine(angle=0, pos=0, pen=pg.mkPen(MID, width=1)))
        content.addWidget(self.error_plot, 1)
        root.addWidget(tracks, 1)
        sensor_views = Q.QTabWidget()
        gravity, content = card()
        self.gravity_plot = plot("Recent time [s]", "Acceleration [g]")
        self.gravity_plot.setXLink(self.error_plot)
        self.gravity_line = self.gravity_plot.plot(pen=pg.mkPen(TEAL, width=1.8))
        for value in (0.72, 1.28):
            self.gravity_plot.addItem(pg.InfiniteLine(
                angle=0, pos=value, pen=pg.mkPen(AMBER, style=QtCore.Qt.PenStyle.DotLine)))
        content.addWidget(self.gravity_plot, 1)
        sensor_views.addTab(gravity, "Acceleration")
        rates, content = card()
        self.rate_plot = plot("Recent time [s]", "Rate magnitude [°/s]")
        self.rate_plot.setXLink(self.error_plot)
        self.rate_line = self.rate_plot.plot(pen=pg.mkPen(TEAL, width=1.8))
        content.addWidget(self.rate_plot, 1)
        sensor_views.addTab(rates, "Gyro")
        root.addWidget(sensor_views, 1)
        return w

    def data_page(self):
        w = Q.QWidget()
        root = Q.QVBoxLayout(w)
        controls, box = card("Session files")
        self.path_label = label("Connect to start a session.", "muted", True)
        self.path_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        box.addWidget(self.path_label)
        self.data_action_grid = Q.QGridLayout()
        self.save_button = button("Save plots", self.save_plots, "primary")
        self.open_button = button("Open session folder", self.open_folder)
        self.data_actions = (self.save_button, self.open_button, button("Export another run", self.choose_export))
        box.addLayout(self.data_action_grid)
        self.export_status = label("CSV logs are saved as you work. PNGs are generated from the saved data.", "muted", True)
        box.addWidget(self.export_status)
        root.addWidget(controls)
        activity, box = card("Session events")
        self.event_list = Q.QPlainTextEdit()
        self.event_list.setReadOnly(True)
        self.event_list.setStyleSheet(f"background:{PLOT};border:1px solid {EDGE};padding:8px;")
        box.addWidget(self.event_list)
        root.addWidget(activity, 1)
        return w

    def fill_ports(self):
        previous = self.ports.currentText() or self.port_requested
        names = ["Practice rig"] if self.simulated else SerialRig.ports()
        self.ports.clear()
        self.ports.addItems(names or ["No serial ports"])
        if previous in names:
            self.ports.setCurrentText(previous)

    def gains(self):
        return tuple(item.value() for item in self.gain_inputs)

    def profile(self):
        return tuple(item.value() for item in self.trial_targets)

    def toggle_connect(self):
        if self.runner:
            self.disconnect()
        else:
            self.connect_rig()

    def connect_rig(self):
        if self.runner:
            return
        try:
            rig = SimRig() if self.simulated else SerialRig(self.ports.currentText())
        except (ValueError, OSError) as exc:
            self.error(f"Cannot open the selected port. Close Motor Bench or Serial Monitor. {exc}")
            return
        try:
            session = Session(self.simulated, self.data_dir)
        except OSError as exc:
            rig.close()
            self.error(f"Cannot create the session logs. {exc}")
            return
        self.runner = Runner(rig, self.calibration, session)
        self.runner.mark("Session connected", self.ports.currentText())
        self.last_tick = time.monotonic()
        self.last_plot = 0
        self.last_trial_count = -1
        self.last_map = None
        self.last_seq = None
        self.sent_saved = False
        self.raw.clear()
        self.samples.clear()
        self.poses.clear()
        self.pending_pose = None
        self.controls_error = ""
        self.verify_checkbox.setChecked(self.simulated)
        self.confirmed_calibration = self.simulated
        self.path_label.setText(f"{session.source} · {session.path.name}")
        self.path_label.setToolTip(str(session.path))
        self.runner.status = ("Practice rig connected. Arm to start a simulated experiment."
                              if self.simulated else "Waiting for the arm firmware and the MPU6050.")
        self.refresh()

    def disconnect(self):
        if self.runner:
            old, self.runner = self.runner, None
            try:
                old.close()
            except OSError:
                pass
        self.controls_error = ""
        self.pending_pose = None
        self.raw.clear()
        self.samples.clear()
        self.tiles_update(None)
        for line in (self.angle_line, self.target_line, self.pulse_line,
                     self.error_line, self.gravity_line, self.rate_line):
            line.setData([], [])
        self.diagnostic_summary.setText("Connect to inspect incoming IMU data.")
        self.refresh()

    def error(self, text):
        self.controls_error = text
        self.refresh()

    def act(self, name, *args):
        if not self.runner:
            self.error("Connect to the Uno first.")
            return
        self.controls_error = ""
        try:
            getattr(self.runner, name)(*args)
        except (ValueError, OSError) as exc:
            self.error(str(exc))
        self.refresh()

    def stop(self):
        self.controls_error = ""
        if self.runner:
            try:
                self.runner.stop()
            except OSError:
                self.disconnect()
                self.error("Serial link closed. Use the motor power cutoff if the arm is still moving.")
        self.refresh()

    def arm(self):
        if self.runner and self.runner.latest and self.runner.latest.armed:
            self.stop()
            return
        if not self.confirmed_calibration:
            self.error("Verify the angle calibration in the Calibration tab before arming.")
            return
        if not self.simulated:
            answer = Q.QMessageBox.question(
                self, "Arm the rig",
                f"Confirm the motor is securely mounted, the stops work, and you can cut motor power.\n"
                f"The current working limit is {self.calibration.cap} µs.\n\n"
                "Arming alone leaves the ESC at idle. Keep the area clear when you start a test.",
                Q.QMessageBox.StandardButton.Yes | Q.QMessageBox.StandardButton.Cancel,
                Q.QMessageBox.StandardButton.Cancel,
            )
            if answer != Q.QMessageBox.StandardButton.Yes:
                return
        self.act("arm")

    def verify_calibration(self, checked):
        self.confirmed_calibration = bool(checked and self.calibration and self.runner)
        self.refresh()

    def load_pick(self):
        if self.runner and self.runner.recommended and not self.runner.active:
            for w, value in zip(self.gain_inputs, self.runner.recommended):
                w.setValue(value)
            self.runner.status = "Recommended gains loaded. Arm and choose Hold angle."
        self.refresh()

    def disturbance(self, name):
        if self.runner and self.runner.experiment == "Hold":
            if self.simulated:
                action = self.runner.rig.poke if name == "Poke" else self.runner.rig.weight
                action()
            self.runner.mark(name)
            self.runner.status = f"{name} marked in the saved telemetry."

    def capture(self, name):
        r = self.runner
        if not r or not r.latest or not r.latest.imu or r.latest.armed or r.active:
            self.error("Connect the IMU and disarm before capturing a pose.")
            return
        if self.simulated:
            r.rig.held_angle = {"Lower": self.lower.value(), "Level": 0, "Upper": self.upper.value()}[name]
            # Let the model hold the illustrative pose while the averaging window fills.
            self.pending_pose = (name, r.time + 0.65)
            self.raw.clear()
            return
        self.store_pose(name)

    def store_pose(self, name):
        try:
            latest = self.raw[-1].ms
            self.poses[name] = capture_pose([s for s in self.raw if s.ms >= latest-600])
        except (ValueError, IndexError) as exc:
            self.error(str(exc) or "Wait for live IMU values.")
            return
        self.pose_labels[name].setText("Captured")
        self.controls_error = ""
        self.refresh()

    def profile_limits(self, c):
        lo, hi = c.targets
        for entry in (*self.trial_targets, self.target):
            entry.setRange(lo, hi)
        # Seed a profile only before the first experiment.
        if self.runner and not self.runner.session.trials:
            start = max(lo, min(-9, hi-4))
            profile = self.comparison.targets if self.comparison else (start, start+4, start+2)
            for entry, value in zip(self.trial_targets, profile):
                entry.setValue(value)

    def apply_calibration(self):
        try:
            limit = self.operating_limit.value()
            if self.simulated and limit == self.upper.value():
                limit = None
            c = calibrate(self.poses, self.lower.value(), self.upper.value(), self.hover.value(),
                          self.cap.value(), operating_limit=limit)
            self.set_calibration(c, True)
        except (ValueError, OSError, KeyError) as exc:
            self.error(f"Calibration needs three still poses and measured stops. {exc}")

    def apply_pulse_settings(self):
        if not self.calibration:
            self.error("Apply the three-pose calibration first.")
            return
        hover, cap = self.hover.value(), self.cap.value()
        if not 1120 <= hover < cap <= HARD_CAP:
            self.error("Set feedforward trim below the working limit.")
            return
        if self.runner and self.runner.session.trials:
            self.error("Start a new session before changing pulse settings used by scored trials.")
            return
        try:
            self.set_calibration(replace(self.calibration, hover=hover, cap=cap), False)
        except (ValueError, OSError) as exc:
            self.error(str(exc))

    def set_calibration(self, c, new_poses):
        if not self.runner:
            raise ValueError("Connect first.")
        if self.runner.session.trials:
            raise ValueError("Start a new session before changing the calibration used by scored trials.")
        self.runner.update_calibration(c)
        self.calibration = c
        self.profile_limits(c)
        self.controls_error = ""
        if not self.simulated and self.comparison is None:
            c.save(CAL_FILE)
        if new_poses:
            self.verify_checkbox.setChecked(self.simulated)
            self.confirmed_calibration = self.simulated
        self.refresh()

    def update_loop(self):
        if self.export_future and self.export_future.done():
            future, self.export_future = self.export_future, None
            try:
                saved = future.result()
                self.export_status.setText(f"Saved {len(saved)} plots in the run’s plots folder.")
                self.export_status.setToolTip(str(self.export_path / "plots"))
            except Exception as exc:
                self.export_status.setText(f"Plot export failed. {exc}")
            self.refresh()
        r = self.runner
        if not r:
            return
        now = time.monotonic()
        dt = max(0.001, now-self.last_tick)
        self.last_tick = now
        if not self.simulated and dt > .70 and r.latest and r.latest.armed:
            self.stop()
        try:
            rows = r.tick(min(dt, .20)*self.speed)
            for row in rows:
                if self.last_seq is not None and row.seq < self.last_seq:
                    r.motor_idle("Uno restarted. Verify calibration again before arming.")
                    self.verify_checkbox.setChecked(False)
                    self.raw.clear()
                    self.samples.clear()
                    self.sent_saved = False
                self.last_seq = row.seq
                self.raw.append(row)
                self.samples.append((r.time, row))
            if r.latest and r.latest.imu and not self.sent_saved:
                if self.calibration is not None:
                    r.update_calibration(self.calibration)
                    self.profile_limits(self.calibration)
                self.sent_saved = True
            if self.pending_pose and r.time >= self.pending_pose[1]:
                name, self.pending_pose = self.pending_pose[0], None
                self.store_pose(name)
                r.rig.held_angle = None
            if r.latest is None and r.time > 5:
                r.status = "No arm telemetry. Upload arm_controller.ino and reconnect."
            elif r.latest and r.latest.imu and self.calibration is None and r.experiment == "Monitor":
                r.status = "IMU streaming. Keep ESC power off and capture the three poses in Calibration."
        except (OSError, ValueError, RuntimeError) as exc:
            self.disconnect()
            self.error(f"Link or experiment stopped. {exc}")
            return
        if now-self.last_plot >= .09:
            self.last_plot = now
            self.update_plots()
            self.refresh()

    def tiles_update(self, row):
        valid = bool(row and row.angle_valid)
        pid_active = valid and row.mode == 1
        values = (f"{row.angle:.1f}°" if valid else "—",
                  f"{row.target:.1f}°" if pid_active else "—",
                  f"{row.error:+.1f}°" if pid_active else "—",
                  f"{row.pulse} µs" if row else "—")
        for tile, text in zip(self.tiles, values):
            tile.setText(text)

    def update_plots(self):
        r = self.runner
        self.tiles_update(r.latest)
        if self.samples:
            own = [row for _, row in self.samples]
            timestamp = np.array([row.ms for row in own], dtype=float)/1000
            timestamp -= timestamp[-1]
            kept = timestamp >= -12
            angle = np.array([row.angle if row.angle_valid else np.nan for row in own])[kept]
            target = np.array([row.target if row.angle_valid and row.mode == 1 else np.nan for row in own])[kept]
            self.angle_line.setData(timestamp[kept], angle, connect="finite")
            self.target_line.setData(timestamp[kept], target, connect="finite")
            visible = np.r_[angle[np.isfinite(angle)], target[np.isfinite(target)]]
            self.angle_plot.setXRange(-12, 0, padding=0)
            if visible.size:
                lower, upper = float(np.min(visible)), float(np.max(visible))
                center = (lower+upper)/2
                span = max(4, (upper-lower)*0.58)
                self.angle_plot.setYRange(center-span, center+span, padding=0)
            pulse_values = np.array([row.pulse for row in own])[kept]
            self.pulse_line.setData(timestamp[kept], pulse_values)
            last = own[-1]
            self.angle_band.setVisible(last.angle_valid and last.mode == 1)
            self.angle_band.setRegion((last.target-1, last.target+1))
            self.cap_line.setValue(own[-1].cap)
            lower_band = 1100 if np.min(pulse_values) >= 1100 else 941
            upper_band = max(1160, int(np.max(pulse_values))+8)
            self.cap_line.setVisible(own[-1].cap <= upper_band)
            self.pulse_plot.setYRange(lower_band, upper_band, padding=0)
            recent = [row for row, include in zip(own, kept) if include]
            t = timestamp[kept]
            raw = np.array([row.raw for row in recent], dtype=float)
            valid = np.array([row.imu for row in recent])
            errors = [row.error if row.angle_valid and row.mode == 1 else np.nan for row in recent]
            terms = np.array([row.terms if row.terms and row.mode == 1 else (np.nan,)*3 for row in recent])
            for term, line in zip(terms.T, self.term_lines):
                line.setData(t, term, connect="finite")
            self.error_line.setData(t, errors, connect="finite")
            self.gravity_line.setData(t, np.where(valid, np.linalg.norm(raw[:, :3], axis=1)/16384, np.nan),
                                      connect="finite")
            self.rate_line.setData(t, np.where(valid, np.linalg.norm(raw[:, 3:], axis=1)/131, np.nan),
                                   connect="finite")
            hz = (len(t)-1)/max(.001, t[-1]-t[0]) if len(t)>1 else 0
            self.diagnostic_summary.setText(
                f"{hz:.1f} frames/s  ·  IMU valid {np.mean(valid):.0%}  ·  "
                f"ESC command maximum {own[-1].cap} µs  ·  Error is shown during PID control.")
        self.update_tuning()
        self.update_results()
        if r.latest:
            raw = r.latest.raw
            self.raw_acc.setText("Acceleration [g]    " + "  /  ".join(f"{x/16384:+.3f}" for x in raw[:3])
                                 if r.latest.imu else "Acceleration [g]    —")
            self.raw_gyro.setText("Gyroscope [°/s]    " + "  /  ".join(f"{x/131:+.2f}" for x in raw[3:])
                                  if r.latest.imu else "Gyroscope [°/s]    —")
            self.imu_pulse.setText(f"Fused tilt    {r.latest.angle:+.2f}°    |    Accel tilt    {r.latest.accel_angle:+.2f}°"
                                  if r.latest.angle_valid else "Fused tilt    —")
        if not self.event_list.hasFocus():
            report = "\n".join(f"{e['elapsed_s']:7.1f} s    {e['event']}   {e['detail']}"
                               for e in r.session.events[-35:][::-1])
            if self.event_list.toPlainText() != report:
                self.event_list.setPlainText(report)

    def update_tuning(self):
        r = self.runner
        search = r.search_trials()
        if self.last_trial_count != len(r.session.trials):
            self.last_trial_count = len(r.session.trials)
            x = np.arange(1, len(search)+1)
            y = np.array([v["score"] for v in search])
            for name, plotter in (("Random", self.random_points), ("Guided", self.guided_points),
                                  ("Fault", self.fault_points)):
                mask = np.array([bool(v["fault"]) if name == "Fault" else
                                 v["group"] == name and not v["fault"] for v in search], dtype=bool)
                plotter.setData(x[mask], y[mask])
            best = np.minimum.accumulate(np.where([v["fault"] == 0 for v in search], y, np.inf))
            self.best_line.setData(x, np.where(np.isfinite(best), best, np.nan), connect="finite")
            recent = r.session.trials[-28:][::-1]
            self.trial_table.setRowCount(len(recent))
            for i, item in enumerate(recent):
                for col, text in enumerate((str(item["trial"]), item["group"],
                                             f"{item['score']:.2f}", f"{item['mae']:.2f}" if not item["fault"] else "—")):
                    self.trial_table.setItem(i, col, Q.QTableWidgetItem(text))
        if r.maps is not None and r.maps is not self.last_map:
            self.last_map = r.maps
            pts = unit([row["gains"] for row in search])
            for image, bar, items, pick, note, values in zip(
                    self.images, self.color_bars, self.map_points, self.map_picks, self.map_status, r.maps):
                vmax = max(float(values.max()), float(values.min()) + 1e-6)
                image.setImage(values, autoLevels=False)
                image.setRect(QtCore.QRectF(0, 0, 1, 1))
                bar.setLevels((float(values.min()), vmax))
                items.setData(pts[:, 0], pts[:, 2])
                if r.pick:
                    position = unit(r.pick)
                    pick.setData([position[0]], [position[2]])
                else:
                    pick.setData([], [])
                note.setText(f"Ki = {r.last_map_ki:.4f} · " +
                             ("star is retested pick" if r.pick else "dots are tried gains"))
        if r.pick:
            candidate = " / ".join(f"{v:.4f}" for v in r.pick)
            confirmation = r.session.metadata.get("confirmation", {})
            if "validation" in r.session.metadata:
                verdict = "Validated HBO recommendation" if r.recommended == r.pick else "Manual PID retained"
            else:
                verdict = "Confirmation passed" if confirmation.get("improved") else "Manual PID leads confirmation"
            self.pick_label.setText(f"HBO candidate {candidate}\n{verdict}")

    def refresh(self):
        r = self.runner
        s = r.latest if r else None
        armed = bool(s and s.armed and not s.fault)
        active = bool(r and r.active)
        connected = bool(r)
        ready = bool(r and r.ready)
        idle = bool(r and s and not armed and not active)
        self.ports.setEnabled(not connected)
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.arm_button.setText("Disarm" if armed else "Arm rig")
        self.arm_button.setEnabled(armed or (ready and self.confirmed_calibration and not active))
        for widget in (self.hold_button, self.ramp_button, self.baseline_button, self.hbo_button):
            widget.setEnabled(armed and not active)
        self.hbo_button.setEnabled(bool(r and s and s.terms is not None and armed and not active and not r.pick
                                        and len(r.baseline_trials()) >= 5))
        if r:
            self.hbo_button.setText("Resume HBO" if r.search_trials() and not r.pick and not active else
                                   "Run HBO · 30 search trials")
        self.repeat_button.setEnabled(bool(armed and not active and r.pick))
        compared = bool(r and r.session.metadata.get("matched_hbo", {}).get("result"))
        self.compare_button.setEnabled(bool(self.comparison and armed and ready and not active and not compared))
        self.load_pick_button.setEnabled(bool(r and r.recommended and not active))
        manual = bool(r and r.demo_stage is None)
        self.retarget_button.setEnabled(bool(manual and r.can_retarget))
        self.dip_button.setEnabled(bool(manual and r.can_dip))
        self.dip_button.setToolTip("Software disturbance · 2 s" if s and s.dip is not None else
                                   "Update the Uno for pulse dips and PID plots.")
        self.demo_button.setEnabled(bool(armed and ready and not active and s.terms is not None))
        self.demo_button.setToolTip("Hold −9°, move to −6°, and disarm automatically.")
        for button_widget in self.controller_presets:
            button_widget.setEnabled(not active)
        self.noise_button.setEnabled(idle and ready)
        for widget in (self.poke_button, self.load_button):
            widget.setEnabled(bool(r and r.experiment == "Hold" and s and s.mode == 1 and not s.launching))
        for w in (*self.pose_buttons.values(), self.apply_cal, self.lower, self.upper, self.operating_limit):
            w.setEnabled(idle)
        self.apply_trim.setEnabled(idle and self.calibration is not None and self.comparison is None)
        self.hover.setEnabled(self.comparison is None and not active)
        self.cap.setEnabled(self.comparison is None and not active)
        self.verify_checkbox.setEnabled(idle and ready)
        self.target.setEnabled(not active or bool(manual and r.can_retarget))
        for w in self.trial_targets:
            w.setEnabled(self.comparison is None and not active)
        for w in self.gain_inputs:
            w.setEnabled(not active)
        if not s:
            bits = "Waiting for arm firmware" if connected else "Disconnected"
        else:
            map_mode = {0: "Idle", 1: "PID active", 2: "Open loop"}
            bits = ("Armed" if armed else "Disarmed") + " · " + ("Soft launch" if s.launching else map_mode[s.mode])
            bits += f" · IMU 0x{s.addr:02X}" if s.imu else " · IMU unavailable"
            if s.fault:
                bits += " · " + FAULTS[s.fault]
        self.rig_state.setText(bits)
        if not r:
            hint = "Connect to the Uno to start."
        elif active:
            hint = "Test in progress."
        elif armed:
            hint = "Choose a test."
        elif not ready:
            hint = "Apply angle calibration and wait for a valid IMU reading."
        elif not self.confirmed_calibration:
            hint = "Verify the poses in Calibration."
        else:
            hint = "Arm the rig to enable powered tests."
        self.arm_hint.setText(hint)
        if ready:
            self.cal_note.setText(f"Check level and both physical stops. "
                                  f"Maximum target {self.calibration.targets[1]:+.1f}°.")
        else:
            self.cal_note.setText("A calibrated angle will appear once the Uno accepts the three poses.")
        self.footer.setText(self.controls_error or (r.status if r else "Disconnected."))
        self.footer.setStyleSheet(f"color:{RED if self.controls_error or (s and s.fault) else MID};")
        self.save_button.setEnabled(bool(r and s and not active and not armed and not self.export_future))
        self.open_button.setEnabled(bool(r))
        self.active_review_button.setEnabled(bool(r))
        self.review_export_button.setEnabled(bool(not self.export_future and
                                                 (self.review_path or (r and not r.active and not armed))))

    def save_plots(self):
        if self.runner and not self.runner.active:
            self.runner.session.stream.flush()
            self.launch_export(self.runner.session.path)

    def launch_export(self, path):
        if self.export_future:
            return
        self.export_status.setText("Building PNGs from the saved CSVs…")
        self.export_path = path
        self.export_future = self.export_pool.submit(export, path)
        self.tabs.setCurrentIndex(self.tabs.count()-1)
        self.refresh()

    def choose_export(self):
        if self.runner and self.runner.active:
            self.error("Stop the experiment before exporting.")
            return
        path = Q.QFileDialog.getExistingDirectory(self, "Select a recorded session", str(ROOT / "runs"))
        if path:
            self.launch_export(Path(path))

    def open_folder(self):
        if self.runner:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(self.runner.session.path)))

    def closeEvent(self, event):
        self.clock.stop()
        self.disconnect()
        self.export_pool.shutdown(wait=False, cancel_futures=True)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="Practice mode. All output marked as simulated.")
    parser.add_argument("--port")
    parser.add_argument("--connect", action="store_true", help="Connect on startup. The rig remains disarmed.")
    parser.add_argument("--calibration", action="store_true", help="Open the Calibration tab.")
    parser.add_argument("--review", type=Path, help="Open a saved session on the Results tab.")
    parser.add_argument("--matched-hbo", type=Path, help="Compare a saved guided candidate with manual PID.")
    args = parser.parse_args()
    app = Q.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setFont(QtGui.QFont("Segoe UI", 10))
    app.setStyleSheet(STYLE)
    try:
        comparison = GuidedComparison.load(args.matched_hbo) if args.matched_hbo else None
        if comparison and args.sim:
            comparison = replace(comparison, trim=SIM_CAL.hover)
        window = Dashboard(args.sim, args.port, comparison=comparison)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        Q.QMessageBox.critical(None, "Could not open the comparison", str(exc))
        return 1
    if args.calibration:
        window.tabs.setCurrentIndex(3)
    if args.review:
        window.open_review(args.review)
        window.tabs.setCurrentIndex(2)
    if comparison and not args.calibration and not args.review:
        window.tabs.setCurrentIndex(1)
    window.show()
    if args.connect and not args.sim:
        QtCore.QTimer.singleShot(0, window.connect_rig)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
