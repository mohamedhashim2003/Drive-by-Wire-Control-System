import sys
import serial
import serial.tools.list_ports
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QFrame, QGridLayout, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QColor, QPalette, QPainter, QPen, QBrush

import pyqtgraph as pg

# ── Constants ──────────────────────────────────────────────────────
BAUD_RATE    = 115200
BUFFER_SIZE  = 200          
REFRESH_MS   = 50           

MODE_NAMES   = ["ECO", "NORMAL", "SPORT"]
MODE_COLORS  = ["#00C896", "#4A9EFF", "#FF4D6D"]
MODE_CMDS    = ["M0", "M1", "M2"]

MODE_DESC = {
    "ECO":    "Logarithmic curve — soft pedal, max efficiency",
    "NORMAL": "Linear 1:1 mapping — standard response",
    "SPORT":  "Square-root curve — aggressive front-loaded response",
}

# ── Dark theme ─────────────────────────────────────────────
BG_DEEP    = "#0D0F14"
BG_PANEL   = "#151820"
BG_CARD    = "#1C2030"
ACCENT     = "#4A9EFF"
TEXT_PRI   = "#E8EAF0"
TEXT_SEC   = "#6B7280"
BORDER     = "#252A3A"


# ══════════════════════════════════════════════════════════════════
# Serial Reader from arduino
# ══════════════════════════════════════════════════════════════════
class SerialReader(QThread):
    data_received = pyqtSignal(float, float, float, int, int)
    connection_lost = pyqtSignal()

    def __init__(self, port, baud):
        super().__init__()
        self.port   = port
        self.baud   = baud
        self._running = True
        self.ser    = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            
            self.ser.readline()
        except serial.SerialException:
            self.connection_lost.emit()
            return

        while self._running:
            try:
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                parts = raw.split(",")
                if len(parts) == 5:
                    pedal = float(parts[0])
                    tps   = float(parts[1])
                    error = float(parts[2])
                    pwm   = int(parts[3])
                    mode  = int(parts[4])
                    self.data_received.emit(pedal, tps, error, pwm, mode)
            except Exception:
                pass

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode())

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


# ══════════════════════════════════════════════════════════════════
# Graph of Angles
# ══════════════════════════════════════════════════════════════════
class GaugeWidget(QWidget):
    def __init__(self, label, color, parent=None):
        super().__init__(parent)
        self.label  = label
        self.color  = QColor(color)
        self.value  = 0.0          # 0–90 degrees
        self.setMinimumSize(160, 160)

    def setValue(self, v):
        self.value = max(0.0, min(90.0, v))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h   = self.width(), self.height()
        margin = 18
        rect   = painter.viewport()
        side   = min(w, h) - 2 * margin
        cx, cy = w // 2, h // 2

        from PyQt5.QtCore import QRectF
        arc_rect = QRectF(cx - side//2, cy - side//2, side, side)

        # Background arc
        bg_pen = QPen(QColor(BORDER), 10, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(bg_pen)
        painter.drawArc(arc_rect, 225 * 16, -270 * 16)

        # Value arc
        span = int(-270 * 16 * (self.value / 90.0))
        val_pen = QPen(self.color, 10, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(val_pen)
        painter.drawArc(arc_rect, 225 * 16, span)

        # Center text
        painter.setPen(QColor(TEXT_PRI))
        painter.setFont(QFont("Consolas", 18, QFont.Bold))
        painter.drawText(arc_rect, Qt.AlignCenter, f"{self.value:.1f}°")

        # Label below
        painter.setPen(QColor(TEXT_SEC))
        painter.setFont(QFont("Consolas", 9))
        label_rect = QRectF(0, cy + side//2 - 10, w, 24)
        painter.drawText(label_rect, Qt.AlignCenter, self.label)


# ══════════════════════════════════════════════════════════════════
# Main Window
# ══════════════════════════════════════════════════════════════════
class ThrottleGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_thread = None
        self.current_mode  = 1   # NORMAL default

        # Data buffers
        self.buf_pedal = deque([0.0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.buf_tps   = deque([0.0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.buf_error = deque([0.0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

        self._build_ui()
        self._apply_theme()

        # Graph refresh timer
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_graph)
        self.timer.start(REFRESH_MS)

    # ── UI Construction ────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("Throttle Control Interface")
        self.setMinimumSize(1000, 680)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)

        # ── Top bar ────────────────────────────────────────────────
        top = QHBoxLayout()

        title = QLabel("Electronic Throttle Control Interface")
        title.setFont(QFont("Consolas", 16, QFont.Bold))
        title.setStyleSheet(f"color: {ACCENT}; letter-spacing: 4px;")
        top.addWidget(title)

        top.addStretch()

        # Port selector
        self.port_combo = QComboBox()
        self.port_combo.setFixedWidth(140)
        self._populate_ports()
        top.addWidget(QLabel("Port:"))
        top.addWidget(self.port_combo)

        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setFixedWidth(110)
        self.btn_connect.clicked.connect(self._toggle_connection)
        top.addWidget(self.btn_connect)

        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color: #FF4D6D; font-size: 18px;")
        top.addWidget(self.status_dot)

        root.addLayout(top)
        root.addWidget(self._hline())

        # ── Middle row: gauges + mode selector ────────────────────
        mid = QHBoxLayout()
        mid.setSpacing(16)

        # Pedal gauge
        gauge_box = self._card()
        gauge_layout = QVBoxLayout(gauge_box)
        self.gauge_pedal = GaugeWidget("PEDAL INPUT", "#4A9EFF")
        self.gauge_tps   = GaugeWidget("VALVE ANGLE", "#00C896")
        g_row = QHBoxLayout()
        g_row.addWidget(self.gauge_pedal)
        g_row.addWidget(self.gauge_tps)
        gauge_layout.addLayout(g_row)
        mid.addWidget(gauge_box, stretch=2)

        # Right panel: mode + stats
        right_col = QVBoxLayout()
        right_col.setSpacing(12)

        # Drive mode card
        mode_card = self._card()
        mode_layout = QVBoxLayout(mode_card)
        mode_title = QLabel("DRIVE MODE")
        mode_title.setFont(QFont("Consolas", 10, QFont.Bold))
        mode_title.setStyleSheet(f"color: {TEXT_SEC}; letter-spacing: 2px;")
        mode_layout.addWidget(mode_title)

        self.mode_buttons = []
        for i, name in enumerate(MODE_NAMES):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.setFont(QFont("Consolas", 11, QFont.Bold))
            btn.setFixedHeight(44)
            btn.clicked.connect(lambda checked, idx=i: self._set_mode(idx))
            self.mode_buttons.append(btn)
            mode_layout.addWidget(btn)

        self.mode_buttons[1].setChecked(True)   

        self.mode_desc_label = QLabel(MODE_DESC["NORMAL"])
        self.mode_desc_label.setWordWrap(True)
        self.mode_desc_label.setStyleSheet(f"color: {TEXT_SEC}; font-size: 11px;")
        mode_layout.addWidget(self.mode_desc_label)
        right_col.addWidget(mode_card)

        # Stats card
        stats_card = self._card()
        stats_layout = QGridLayout(stats_card)
        stats_layout.setSpacing(8)

        self.lbl_error = self._stat_value("0.0°")
        self.lbl_pwm   = self._stat_value("0")

        stats_layout.addWidget(self._stat_label("ERROR"),  0, 0)
        stats_layout.addWidget(self.lbl_error,             0, 1)
        stats_layout.addWidget(self._stat_label("PWM OUT"),1, 0)
        stats_layout.addWidget(self.lbl_pwm,               1, 1)

        right_col.addWidget(stats_card)
        mid.addLayout(right_col, stretch=1)

        root.addLayout(mid)

        # ── Graph ──────────────────────────────────────────────────
        graph_card = self._card()
        graph_layout = QVBoxLayout(graph_card)

        graph_label = QLabel("ANGLE TRACE")
        graph_label.setFont(QFont("Consolas", 10, QFont.Bold))
        graph_label.setStyleSheet(f"color: {TEXT_SEC}; letter-spacing: 2px;")
        graph_layout.addWidget(graph_label)

        pg.setConfigOption("background", BG_CARD)
        pg.setConfigOption("foreground", TEXT_SEC)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setYRange(0, 90)
        self.plot_widget.showGrid(x=False, y=True, alpha=0.15)
        self.plot_widget.getAxis("left").setLabel("Degrees (°)")
        self.plot_widget.setFixedHeight(200)

        self.curve_pedal = self.plot_widget.plot(
            pen=pg.mkPen(color="#4A9EFF", width=2), name="Pedal"
        )
        self.curve_tps = self.plot_widget.plot(
            pen=pg.mkPen(color="#00C896", width=2), name="TPS"
        )

        legend = self.plot_widget.addLegend(offset=(10, 10))

        graph_layout.addWidget(self.plot_widget)
        root.addWidget(graph_card)

    # ── Theme ──────────────────────────────────────────────────────
    def _apply_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {BG_DEEP};
                color: {TEXT_PRI};
                font-family: Consolas, monospace;
            }}
            QComboBox, QComboBox QAbstractItemView {{
                background-color: {BG_CARD};
                color: {TEXT_PRI};
                border: 1px solid {BORDER};
                border-radius: 6px;
                padding: 4px 8px;
            }}
            QPushButton {{
                background-color: {BG_CARD};
                color: {TEXT_PRI};
                border: 1px solid {BORDER};
                border-radius: 6px;
                padding: 6px 14px;
            }}
            QPushButton:hover {{
                border-color: {ACCENT};
            }}
            QPushButton#connect_btn {{
                background-color: {ACCENT};
                color: #000;
                font-weight: bold;
            }}
        """)
        self._update_mode_buttons()

    def _update_mode_buttons(self):
        for i, btn in enumerate(self.mode_buttons):
            if btn.isChecked():
                btn.setStyleSheet(
                    f"background-color: {MODE_COLORS[i]}22;"
                    f"color: {MODE_COLORS[i]};"
                    f"border: 1px solid {MODE_COLORS[i]};"
                    f"border-radius: 6px; padding: 6px; font-weight: bold;"
                )
            else:
                btn.setStyleSheet(
                    f"background-color: {BG_CARD}; color: {TEXT_SEC};"
                    f"border: 1px solid {BORDER}; border-radius: 6px; padding: 6px;"
                )

    # ── Helper Functions ────────────────────────────────────────────────────
    def _card(self):
        f = QFrame()
        f.setStyleSheet(
            f"background-color: {BG_CARD}; border: 1px solid {BORDER};"
            f"border-radius: 10px;"
        )
        return f

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color: {BORDER};")
        return line

    def _stat_label(self, text):
        l = QLabel(text)
        l.setStyleSheet(f"color: {TEXT_SEC}; font-size: 11px; letter-spacing: 1px;")
        return l

    def _stat_value(self, text):
        l = QLabel(text)
        l.setFont(QFont("Consolas", 16, QFont.Bold))
        l.setStyleSheet(f"color: {TEXT_PRI};")
        return l

    def _populate_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("No ports found")

    # ── Connection ─────────────────────────────────────────────────
    def _toggle_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.serial_thread = None
            self.btn_connect.setText("CONNECT")
            self.status_dot.setStyleSheet("color: #FF4D6D; font-size: 18px;")
        else:
            port = self.port_combo.currentText()
            if not port or port == "No ports found":
                return
            self.serial_thread = SerialReader(port, BAUD_RATE)
            self.serial_thread.data_received.connect(self._on_data)
            self.serial_thread.connection_lost.connect(self._on_disconnect)
            self.serial_thread.start()
            self.btn_connect.setText("DISCONNECT")
            self.status_dot.setStyleSheet("color: #00C896; font-size: 18px;")
            # Send current mode immediately on connect
            self._send_mode()

    def _on_disconnect(self):
        self.btn_connect.setText("CONNECT")
        self.status_dot.setStyleSheet("color: #FF4D6D; font-size: 18px;")

    # ── Data handling ───────────────────────────────────────────────
    def _on_data(self, pedal, tps, error, pwm, mode):
        self.buf_pedal.append(pedal)
        self.buf_tps.append(tps)
        self.buf_error.append(error)

        self.gauge_pedal.setValue(pedal)
        self.gauge_tps.setValue(tps)

        error_color = "#FF4D6D" if abs(error) > 5 else TEXT_PRI
        self.lbl_error.setText(f"{error:+.1f}°")
        self.lbl_error.setStyleSheet(f"color: {error_color}; font-size: 16px; font-weight: bold;")
        self.lbl_pwm.setText(str(abs(pwm)))

    def _refresh_graph(self):
        self.curve_pedal.setData(list(self.buf_pedal))
        self.curve_tps.setData(list(self.buf_tps))

    # ── Mode switching ─────────────────────────────────────────────
    def _set_mode(self, idx):
        self.current_mode = idx
        for i, btn in enumerate(self.mode_buttons):
            btn.setChecked(i == idx)
        self._update_mode_buttons()
        self.mode_desc_label.setText(MODE_DESC[MODE_NAMES[idx]])
        self._send_mode()

    def _send_mode(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.send(MODE_CMDS[self.current_mode])

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ThrottleGUI()
    win.show()
    sys.exit(app.exec_())
