"""
Espectrometro V7 - ESP32 + OPT101 + Motor paso a paso
Adquisicion espectral en tiempo real con correccion por responsividad.

Mejoras sobre V6:
  - Selector de puerto serial en la UI (ya no hardcodeado)
  - Rango espectral ampliado: 350 - 750 nm  (antes 400-700)
  - Duracion de barrido ajustada: 10.65 s
  - "INICIAR BARRIDO" separado del control manual del motor:
      w → envía el comando + activa t=0 y recoleccion de datos
      SUBIR / BAJAR → mueven el motor SIN recolectar datos
  - Boton "DETENER MEDICION": para la recoleccion sin detener el motor
  - Tarjetas de lectura completas: λ | V_out | I_photo | R(λ) | I_corr | ADC

Mapeo tiempo-longitud de onda (sin encoder de posicion):
    lambda(t) = 350 + (t / 10.65) * 400   [nm]

Correccion por responsividad espectral del OPT101:
    I_corr(λ) = max(V_out - V_dark, 0) / R_I(λ)
donde R_I(λ) es la responsividad tipica [A/W] interpolada del datasheet.
"""
from __future__ import annotations

import csv
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QBrush, QColor, QLinearGradient
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

pg.setConfigOptions(antialias=True, background="#0a0c10", foreground="#c8d0e0")

# ── Calibracion ───────────────────────────────────────────────────────────────
BAUD_RATE       = 115200
WAVELENGTH_START = 350.0    # nm — inicio del espectro util
WAVELENGTH_END   = 750.0    # nm — fin del espectro util
SCAN_DURATION_S  = 10.65    # s  — tiempo del barrido antihorario ('w')
V_DARK_V         = 0.0075   # V  — pedestal oscuro del OPT101 (datasheet sec. 6.5)
R_FEEDBACK_OHM   = 1_000_000.0  # Ω — transimpedancia interna del OPT101

CSV_OUTPUT = Path("lectura_opt101_espectral_V7.csv")
PNG_OUTPUT = Path("lectura_opt101_espectral_V7.png")

# Responsividad espectral tipica del OPT101, puntos tomados del datasheet [nm, A/W].
# Para correccion relativa importa la forma de la curva, no el valor absoluto.
_RESP_PTS = np.array([
    [300,  0.02], [350,  0.10], [400,  0.19], [450,  0.27],
    [500,  0.33], [550,  0.38], [600,  0.43], [650,  0.46],
    [700,  0.48], [750,  0.50], [800,  0.52], [850,  0.54],
    [900,  0.56], [950,  0.55], [1000, 0.45], [1050, 0.25],
    [1100, 0.05],
], dtype=float)

ADC_RE = re.compile(
    r"raw:\s*(?P<raw>\d+)\s*\|\s*(?P<voltage>\d+(?:\.\d+)?)\s*V",
    re.IGNORECASE,
)


# ── Utilidades ────────────────────────────────────────────────────────────────

def responsivity_at(wl_nm: float) -> float:
    return float(np.interp(wl_nm, _RESP_PTS[:, 0], _RESP_PTS[:, 1]))


def wavelength_for_elapsed(t_s: float) -> float:
    frac = max(0.0, min(1.0, t_s / SCAN_DURATION_S))
    return WAVELENGTH_START + frac * (WAVELENGTH_END - WAVELENGTH_START)


def wavelength_to_rgb(wl: float) -> tuple[int, int, int]:
    """Aproximacion CIE de color visible para una longitud de onda."""
    if wl < 380:
        r, g, b = 0.0, 0.0, 0.0
    elif wl < 440:
        r, g, b = -(wl - 440) / 60, 0.0, 1.0
    elif wl < 490:
        r, g, b = 0.0, (wl - 440) / 50, 1.0
    elif wl < 510:
        r, g, b = 0.0, 1.0, -(wl - 510) / 20
    elif wl < 580:
        r, g, b = (wl - 510) / 70, 1.0, 0.0
    elif wl < 645:
        r, g, b = 1.0, -(wl - 645) / 65, 0.0
    elif wl <= 750:
        r, g, b = 1.0, 0.0, 0.0
    else:
        r, g, b = 0.0, 0.0, 0.0
    return (round(r * 255), round(g * 255), round(b * 255))


def parse_adc_line(line: str) -> tuple[int, float] | None:
    m = ADC_RE.search(line)
    if not m:
        return None
    return int(m.group("raw")), float(m.group("voltage"))


# ── Modelo de datos ───────────────────────────────────────────────────────────

@dataclass
class Sample:
    timestamp_s: float
    wavelength_nm: float
    raw: int
    voltage_v: float

    @property
    def signal_v(self) -> float:
        """V_out menos el pedestal oscuro; nunca negativo."""
        return max(self.voltage_v - V_DARK_V, 0.0)

    @property
    def photodiode_current_na(self) -> float:
        """Corriente estimada del fotodiodo: I = V_signal / R_F [nA]."""
        return self.signal_v / R_FEEDBACK_OHM * 1e9

    @property
    def responsivity_a_per_w(self) -> float:
        return responsivity_at(self.wavelength_nm)

    @property
    def corrected_intensity(self) -> float:
        """Intensidad corregida: elimina el sesgo de responsividad del sensor."""
        return self.signal_v / max(self.responsivity_a_per_w, 1e-9)


def save_csv(samples: list[Sample], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["# v_dark_v", "scan_duration_s", "lambda_start_nm", "lambda_end_nm"])
        w.writerow([V_DARK_V, SCAN_DURATION_S, WAVELENGTH_START, WAVELENGTH_END])
        w.writerow([
            "timestamp_s", "wavelength_nm", "raw_adc",
            "voltage_v", "signal_v", "photodiode_current_na",
            "responsivity_a_per_w", "corrected_intensity",
        ])
        for s in samples:
            w.writerow([
                f"{s.timestamp_s:.3f}", f"{s.wavelength_nm:.3f}", s.raw,
                f"{s.voltage_v:.6f}", f"{s.signal_v:.6f}",
                f"{s.photodiode_current_na:.6f}",
                f"{s.responsivity_a_per_w:.6f}",
                f"{s.corrected_intensity:.6f}",
            ])


# ── Hoja de estilos ───────────────────────────────────────────────────────────

APP_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0a0c10;
    color: #c8d0e0;
    font-family: 'Space Mono', 'Courier New', monospace;
    font-size: 11px;
}
QFrame#header { border-bottom: 1px solid #1b2130; background: transparent; }
QFrame#logoMark {
    border: 1px solid rgba(0, 229, 255, 0.55);
    border-radius: 8px;
    background: rgba(0, 229, 255, 0.10);
}
QLabel#logoDot  { color: #00e5ff; font-size: 18px; }
QLabel#titleLabel {
    color: #f0f4fb;
    font-family: 'Syne', 'Arial', sans-serif;
    font-size: 18px; font-weight: 800;
}
QLabel#subtitleLabel { color: #424a60; font-size: 9px; letter-spacing: 2px; }
QLabel#statusPill {
    border: 1px solid #1b2130; border-radius: 13px;
    background: #0b0e16; color: #5a6178;
    padding: 6px 12px; font-size: 9px; letter-spacing: 2px;
}
QLabel#sampleTotal        { color: #eef3fb; font-size: 24px; font-weight: 700; }
QLabel#sampleTotalCaption { color: #424a60; font-size: 8px;  letter-spacing: 2px; }
QFrame#toolbar { background: transparent; }
QFrame#plotCard, QFrame#readoutCard, QFrame#barCard {
    background: #0b0e16; border: 1px solid #1b2130; border-radius: 8px;
}
QLabel#plotTitle { color: #e6ecf6; font-size: 12px; font-weight: 700; }
QLabel#plotTag {
    border: 1px solid rgba(0, 229, 255, 0.55); border-radius: 4px;
    background: rgba(0, 229, 255, 0.10); color: #00e5ff;
    padding: 3px 8px; font-size: 8px; letter-spacing: 1px;
}
QLabel#readoutLabel, QLabel#barLabel { color: #424a60; font-size: 8px; letter-spacing: 2px; }
QLabel#readoutValue    { color: #eef3fb; font-size: 20px; font-weight: 700; }
QLabel#readoutValueBig { color: #00e5ff; font-size: 26px; font-weight: 700; }
QLabel#readoutUnit     { color: #424a60; font-size: 10px; }
QFrame#swatch {
    border: 1px solid rgba(255,255,255,0.25);
    border-radius: 3px; background: #1b2130;
}
QProgressBar#scanProgress {
    min-height: 9px; max-height: 9px;
    border: 1px solid #1b2130; border-radius: 4px;
    background: rgba(139, 92, 246, 0.14); text-align: center;
}
QProgressBar#scanProgress::chunk {
    border-radius: 4px;
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0.00 #8b5cf6,
        stop:0.15 #3b82f6,
        stop:0.30 #06b6d4,
        stop:0.48 #22c55e,
        stop:0.65 #eab308,
        stop:0.82 #f97316,
        stop:1.00 #ef4444);
}
QPushButton {
    padding: 8px 13px; border: 1px solid #1e2330; border-radius: 6px;
    background: #0d1119; color: #c8d0e0;
    font-size: 10px; letter-spacing: 1px;
}
QPushButton:hover    { border-color: #00e5ff; color: #00e5ff; background: rgba(0,229,255,0.05); }
QPushButton:pressed  { background: rgba(0,229,255,0.12); }
QPushButton:disabled { color: #2a3040; border-color: #151b26; }
QPushButton#btnScan {
    background: rgba(0,229,255,0.07); border-color: rgba(0,229,255,0.40);
    color: #00e5ff; font-weight: 700; letter-spacing: 2px;
}
QPushButton#btnScan:hover    { background: rgba(0,229,255,0.15); }
QPushButton#btnStopMotor     { border-color: #ff3b54; color: #ff3b54; }
QPushButton#btnStopMotor:hover { background: rgba(255,59,84,0.08); }
QPushButton#btnStopMeas      { border-color: #f97316; color: #f97316; }
QPushButton#btnStopMeas:hover { background: rgba(249,115,22,0.08); }
QComboBox {
    padding: 6px 10px; border: 1px solid #1e2330; border-radius: 6px;
    background: #0d1119; color: #c8d0e0; font-size: 10px;
}
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background: #0d1119; color: #c8d0e0;
    selection-background-color: rgba(0,229,255,0.12);
}
QStatusBar {
    background: #111318; color: #5a6070; font-size: 10px;
    border-top: 1px solid #1e2330; padding: 2px 10px;
}
QStatusBar::item { border: none; }
"""


# ── Widgets de visualizacion ──────────────────────────────────────────────────

class SpectrumPlot(pg.PlotWidget):
    """PlotWidget de PyQtGraph para una curva espectral con relleno de color."""

    _GRADIENT_WLS = (400, 440, 490, 510, 580, 645, 700)

    def __init__(self, y_label: str, gradient: bool = False, parent=None):
        super().__init__(parent=parent)
        self._setup_axes(y_label)
        pen   = pg.mkPen("#00e5ff", width=1.5) if gradient else pg.mkPen("#5a6070", width=1.5)
        brush = self._wavelength_brush() if gradient else QBrush(QColor(90, 96, 112, 38))
        self._curve = pg.PlotCurveItem(antialias=True)
        self._curve.setPen(pen)
        self._curve.setFillLevel(0)
        self._curve.setBrush(brush)
        self.addItem(self._curve)
        self._empty_lbl = pg.TextItem(
            "Esperando muestras...", color="#5a6070", anchor=(0.5, 0.5)
        )
        self.addItem(self._empty_lbl)
        self._empty_lbl.setPos((WAVELENGTH_START + WAVELENGTH_END) / 2, 0.5)

    def _setup_axes(self, y_label: str) -> None:
        self.setBackground("#0a0c10")
        pi = self.getPlotItem()
        pi.setLabel("bottom", "Longitud de onda (nm)", color="#5a6070", size="9pt")
        pi.setLabel("left", y_label, color="#5a6070", size="9pt")
        pi.showGrid(x=True, y=True, alpha=0.15)
        for name in ("bottom", "left", "top", "right"):
            ax = pi.getAxis(name)
            ax.setPen(pg.mkPen("#1e2330"))
            ax.setTextPen(pg.mkPen("#5a6070"))
        pi.setXRange(WAVELENGTH_START, WAVELENGTH_END, padding=0.02)
        pi.setYRange(0, 1, padding=0.05)

    def _wavelength_brush(self) -> QBrush:
        grad = QLinearGradient(WAVELENGTH_START, 0, WAVELENGTH_END, 0)
        span = WAVELENGTH_END - WAVELENGTH_START
        for wl in self._GRADIENT_WLS:
            stop = (wl - WAVELENGTH_START) / span
            r, g, b = wavelength_to_rgb(wl)
            grad.setColorAt(max(0.0, min(1.0, stop)), QColor(r, g, b, 160))
        return QBrush(grad)

    def update_data(self, x: list[float], y: list[float]) -> None:
        if len(x) < 2:
            self._curve.setData([], [])
            self._empty_lbl.setVisible(True)
            return
        self._empty_lbl.setVisible(False)
        y_max = max(y) if y else 1.0
        if y_max > 0:
            self.getPlotItem().setYRange(0, max(y_max * 1.1, 1e-6), padding=0)
        self._curve.setData(x=x, y=y)

    def clear_data(self) -> None:
        self._curve.setData([], [])
        self._empty_lbl.setVisible(True)
        self.getPlotItem().setYRange(0, 1, padding=0.05)


class PlotCard(QFrame):
    def __init__(self, title: str, tag: str, plot: SpectrumPlot):
        super().__init__()
        self.setObjectName("plotCard")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        hdr = QWidget()
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(14, 10, 14, 8)
        hl.setSpacing(10)
        t = QLabel(title)
        t.setObjectName("plotTitle")
        tg = QLabel(tag)
        tg.setObjectName("plotTag")
        hl.addWidget(t)
        hl.addStretch()
        hl.addWidget(tg)
        lay.addWidget(hdr)
        lay.addWidget(plot, stretch=1)


class ReadoutCard(QFrame):
    def __init__(self, label: str, unit: str, *, big: bool = False, swatch: bool = False):
        super().__init__()
        self.setObjectName("readoutCard")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(4)

        top_row = QHBoxLayout()
        top_row.setSpacing(6)
        if swatch:
            self._sw = QFrame()
            self._sw.setObjectName("swatch")
            self._sw.setFixedSize(10, 10)
            top_row.addWidget(self._sw)
        else:
            self._sw = None
        lbl = QLabel(label.upper())
        lbl.setObjectName("readoutLabel")
        top_row.addWidget(lbl)
        top_row.addStretch()
        lay.addLayout(top_row)

        val_row = QHBoxLayout()
        val_row.setSpacing(5)
        self._val = QLabel("--")
        self._val.setObjectName("readoutValueBig" if big else "readoutValue")
        self._val.setMinimumWidth(78)
        unit_lbl = QLabel(unit)
        unit_lbl.setObjectName("readoutUnit")
        val_row.addWidget(self._val)
        val_row.addWidget(unit_lbl)
        val_row.addStretch()
        lay.addLayout(val_row)

    def set(self, value: str, *, sw_color: str | None = None) -> None:
        self._val.setText(value)
        if self._sw is not None:
            self._sw.setStyleSheet(
                f"QFrame#swatch {{ background: {sw_color or '#1b2130'}; }}"
            )


class ScanProgressCard(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("barCard")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(6)

        lbl = QLabel("POSICION DE BARRIDO")
        lbl.setObjectName("barLabel")
        lay.addWidget(lbl)

        self._bar = QProgressBar()
        self._bar.setObjectName("scanProgress")
        self._bar.setRange(0, 1000)
        self._bar.setTextVisible(False)
        lay.addWidget(self._bar)

        scale = QHBoxLayout()
        for txt in (
            f"{WAVELENGTH_START:.0f}",
            f"{(WAVELENGTH_START + WAVELENGTH_END) / 2:.0f}",
            f"{WAVELENGTH_END:.0f} nm",
        ):
            l = QLabel(txt)
            l.setObjectName("readoutLabel")
            scale.addWidget(l)
        scale.setStretch(0, 1)
        scale.setStretch(1, 1)
        scale.setStretch(2, 1)
        lay.addLayout(scale)

    def set_wavelength(self, wl: float | None) -> None:
        if wl is None:
            self._bar.setValue(0)
            return
        span = WAVELENGTH_END - WAVELENGTH_START
        frac = (wl - WAVELENGTH_START) / span if span > 0 else 0.0
        self._bar.setValue(round(max(0.0, min(1.0, frac)) * 1000))


# ── Aplicacion principal ──────────────────────────────────────────────────────

class SpectrometerApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.samples: list[Sample] = []
        self._esp32 = None
        self._latest: Sample | None = None
        self._scan_t0: float | None = None
        self._collecting = False

        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._read_serial)

        self._build_ui()
        self._set_state("idle")
        self._refresh_ports()

    # ── Construccion de la UI ─────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setWindowTitle(f"Espectrometro OPT101 V7  ·  {WAVELENGTH_START:.0f}–{WAVELENGTH_END:.0f} nm  ·  ESP32")
        self.resize(1240, 820)
        self.setMinimumSize(980, 640)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 12)
        root.setSpacing(10)

        root.addWidget(self._build_header())
        root.addWidget(self._build_connection_bar())
        root.addWidget(self._build_toolbar())

        self.raw_plot  = SpectrumPlot("Voltaje bruto (V)",          gradient=False)
        self.corr_plot = SpectrumPlot("Intensidad corregida (u.a.)", gradient=True)

        plot_grid = QGridLayout()
        plot_grid.setContentsMargins(0, 0, 0, 0)
        plot_grid.setHorizontalSpacing(14)
        plot_grid.addWidget(
            PlotCard("OPT101 — Espectro Bruto", "SIN CORRECCIÓN", self.raw_plot), 0, 0
        )
        plot_grid.addWidget(
            PlotCard("OPT101 — Corregido por Responsividad", "I = V / R(λ)", self.corr_plot), 0, 1
        )
        plot_grid.setColumnStretch(0, 1)
        plot_grid.setColumnStretch(1, 1)
        root.addLayout(plot_grid, stretch=1)

        root.addWidget(self._build_readouts())

    def _build_header(self) -> QWidget:
        frm = QFrame()
        frm.setObjectName("header")
        h = QHBoxLayout(frm)
        h.setContentsMargins(0, 0, 0, 10)
        h.setSpacing(12)

        logo = QFrame()
        logo.setObjectName("logoMark")
        logo.setFixedSize(32, 32)
        ll = QVBoxLayout(logo)
        ll.setContentsMargins(0, 0, 0, 0)
        dot = QLabel("●")
        dot.setObjectName("logoDot")
        dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ll.addWidget(dot)

        blk = QWidget()
        bl = QVBoxLayout(blk)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(2)
        title = QLabel("Espectrometro — Lectura Espectral")
        title.setObjectName("titleLabel")
        sub = QLabel(
            f"OPT101 · {WAVELENGTH_START:.0f}–{WAVELENGTH_END:.0f} nm · "
            f"BARRIDO ANGULAR {SCAN_DURATION_S:.2f} s · OPTOELECTRÓNICA"
        )
        sub.setObjectName("subtitleLabel")
        bl.addWidget(title)
        bl.addWidget(sub)

        self._pill = QLabel()
        self._pill.setObjectName("statusPill")
        self._pill.setMinimumWidth(195)
        self._pill.setAlignment(Qt.AlignmentFlag.AlignCenter)

        cnt_blk = QWidget()
        cl = QVBoxLayout(cnt_blk)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        self._sample_cnt = QLabel("0")
        self._sample_cnt.setObjectName("sampleTotal")
        self._sample_cnt.setAlignment(Qt.AlignmentFlag.AlignRight)
        cap = QLabel("MUESTRAS")
        cap.setObjectName("sampleTotalCaption")
        cap.setAlignment(Qt.AlignmentFlag.AlignRight)
        cl.addWidget(self._sample_cnt)
        cl.addWidget(cap)

        h.addWidget(logo)
        h.addWidget(blk)
        h.addStretch()
        h.addWidget(self._pill)
        h.addWidget(cnt_blk)
        return frm

    def _build_connection_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("toolbar")
        h = QHBoxLayout(bar)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(8)

        h.addWidget(QLabel("PUERTO:"))

        self._port_cb = QComboBox()
        self._port_cb.setMinimumWidth(140)
        self._port_cb.setToolTip("Puerto serial del ESP32")
        h.addWidget(self._port_cb)

        btn_ref = QPushButton("⟳")
        btn_ref.setFixedWidth(34)
        btn_ref.setToolTip("Actualizar lista de puertos")
        btn_ref.clicked.connect(self._refresh_ports)
        h.addWidget(btn_ref)

        self._btn_connect = QPushButton("CONECTAR")
        self._btn_connect.clicked.connect(self._connect)
        h.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton("DESCONECTAR")
        self._btn_disconnect.setObjectName("btnStopMotor")
        self._btn_disconnect.clicked.connect(self._disconnect)
        self._btn_disconnect.setEnabled(False)
        h.addWidget(self._btn_disconnect)

        h.addStretch()
        return bar

    def _build_toolbar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("toolbar")
        h = QHBoxLayout(bar)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(8)

        # ── Seccion: Datos ───────────────────
        h.addWidget(self._sep_label("DATOS"))

        btn_reset = QPushButton("⟳ REINICIAR")
        btn_reset.setToolTip("Borra todas las muestras y reinicia el barrido")
        btn_reset.clicked.connect(self._reset)
        h.addWidget(btn_reset)

        btn_png = QPushButton("💾 GUARDAR PNG")
        btn_png.setToolTip("Exporta ambas graficas como PNG")
        btn_png.clicked.connect(self._save_png_dialog)
        h.addWidget(btn_png)

        btn_csv = QPushButton("📄 GUARDAR CSV")
        btn_csv.setToolTip("Exporta los datos de la medicion como CSV")
        btn_csv.clicked.connect(self._save_csv_dialog)
        h.addWidget(btn_csv)

        h.addWidget(self._vsep())

        # ── Seccion: Barrido ─────────────────
        h.addWidget(self._sep_label("BARRIDO"))

        btn_scan = QPushButton("▶ INICIAR BARRIDO")
        btn_scan.setObjectName("btnScan")
        btn_scan.setToolTip(
            f"Envia 'w' al ESP32 e inicia la recoleccion de datos\n"
            f"({WAVELENGTH_START:.0f}–{WAVELENGTH_END:.0f} nm en {SCAN_DURATION_S:.2f} s)"
        )
        btn_scan.clicked.connect(self._cmd_scan)
        h.addWidget(btn_scan)

        btn_stop_meas = QPushButton("⏹ DETENER MEDICIÓN")
        btn_stop_meas.setObjectName("btnStopMeas")
        btn_stop_meas.setToolTip("Detiene la recoleccion de datos sin parar el motor")
        btn_stop_meas.clicked.connect(self._cmd_stop_meas)
        h.addWidget(btn_stop_meas)

        h.addWidget(self._vsep())

        # ── Seccion: Motor ───────────────────
        h.addWidget(self._sep_label("MOTOR"))

        for label, cmd, tip, obj_name in [
            ("↑ SUBIR",         "s", "Motor horario continuo — regresa al inicio (sin recolectar datos)", None),
            ("↓ BAJAR",         "w", "Motor antihorario 11.55 s — sin recolectar datos", None),
            ("■ DETENER MOTOR", "x", "Detiene el motor y la recoleccion de datos", "btnStopMotor"),
        ]:
            btn = QPushButton(label)
            if obj_name:
                btn.setObjectName(obj_name)
            btn.setToolTip(tip)
            btn.clicked.connect(lambda _ch=False, c=cmd, lbl=label: self._cmd_motor(c, lbl))
            h.addWidget(btn)

        h.addStretch()
        return bar

    def _build_readouts(self) -> QWidget:
        wrapper = QWidget()
        lay = QGridLayout(wrapper)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(10)

        self._r_lambda = ReadoutCard("lambda", "nm",    big=True, swatch=True)
        self._r_vout   = ReadoutCard("V_out",  "V")
        self._r_vsig   = ReadoutCard("V_señal","V")
        self._r_iphoto = ReadoutCard("I_photo", "nA")
        self._r_resp   = ReadoutCard("R(λ)",   "A/W")
        self._r_icorr  = ReadoutCard("I_corr", "u.a.")
        self._r_adc    = ReadoutCard("ADC",    "/ 4095")
        self._r_bar    = ScanProgressCard()

        widgets = [
            self._r_lambda, self._r_vout, self._r_vsig, self._r_iphoto,
            self._r_resp,   self._r_icorr, self._r_adc, self._r_bar,
        ]
        for i, w in enumerate(widgets):
            lay.addWidget(w, 0, i)
        for i in range(len(widgets) - 1):
            lay.setColumnStretch(i, 10)
        lay.setColumnStretch(len(widgets) - 1, 22)

        self._update_readouts(None)
        return wrapper

    @staticmethod
    def _vsep() -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("QFrame { color: #1e2330; max-width: 1px; }")
        return sep

    @staticmethod
    def _sep_label(text: str) -> QLabel:
        lbl = QLabel(text + ":")
        lbl.setStyleSheet("color: #5a6070; font-size: 9px; letter-spacing: 2px;")
        return lbl

    # ── Serial ────────────────────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        try:
            import serial.tools.list_ports
            ports = [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            ports = []
        self._port_cb.clear()
        if ports:
            self._port_cb.addItems(ports)
        else:
            self._port_cb.addItem("(sin puertos)")

    def _connect(self) -> None:
        try:
            import serial as _serial
        except ImportError:
            QMessageBox.critical(
                self, "Error de dependencia",
                "pyserial no esta instalado.\n\nEjecuta:\n  pip install pyserial PyQt6 pyqtgraph numpy"
            )
            return

        port = self._port_cb.currentText()
        if not port or port.startswith("("):
            QMessageBox.warning(self, "Sin puerto", "Selecciona un puerto serial valido.")
            return

        try:
            self._esp32 = _serial.Serial(port, BAUD_RATE, timeout=0)
            QTimer.singleShot(2000, self._on_connected)
            self._timer.start()
            self._set_state("connecting")
            self._btn_connect.setEnabled(False)
            self._btn_disconnect.setEnabled(True)
            self.statusBar().showMessage(f"Conectando a {port}...")
        except Exception as exc:
            QMessageBox.critical(self, "Error de conexion", str(exc))
            self._set_state("error")

    def _on_connected(self) -> None:
        if self._esp32 is not None and self._esp32.is_open:
            self._esp32.reset_input_buffer()
            self._set_state("live")
            self.statusBar().showMessage(
                f"Conectado — {self._esp32.port} @ {BAUD_RATE} baud  |  "
                f"usa INICIAR BARRIDO para comenzar la medicion"
            )

    def _disconnect(self) -> None:
        self._timer.stop()
        self._collecting = False
        self._scan_t0 = None
        if self._esp32 is not None:
            try:
                self._esp32.close()
            except Exception:
                pass
            self._esp32 = None
        self._set_state("idle")
        self._btn_connect.setEnabled(True)
        self._btn_disconnect.setEnabled(False)
        self.statusBar().showMessage("Desconectado del ESP32")

    def _send(self, cmd: str) -> bool:
        if self._esp32 is None or not self._esp32.is_open:
            QMessageBox.warning(self, "Sin conexion", "Primero conecta el ESP32.")
            return False
        try:
            self._esp32.write(f"{cmd}\n".encode("utf-8"))
            return True
        except Exception as exc:
            self.statusBar().showMessage(f"Error al enviar '{cmd}': {exc}")
            return False

    def _read_serial(self) -> None:
        if self._esp32 is None:
            return
        for _ in range(50):
            try:
                raw = self._esp32.readline()
            except Exception:
                break
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                break

            parsed = parse_adc_line(line)
            if parsed is None:
                self.statusBar().showMessage(line[:120])
                continue

            adc_raw, voltage = parsed

            if not self._collecting or self._scan_t0 is None:
                # Actualiza el readout en vivo aunque no haya barrido activo
                self._latest = Sample(
                    timestamp_s=0.0,
                    wavelength_nm=WAVELENGTH_START,
                    raw=adc_raw,
                    voltage_v=voltage,
                )
                continue

            elapsed = time.time() - self._scan_t0
            if elapsed > SCAN_DURATION_S:
                self._collecting = False
                self._scan_t0 = None
                self._set_state("live")
                self.statusBar().showMessage(
                    f"Barrido completado ({len(self.samples)} muestras) — "
                    "el motor puede seguir funcionando"
                )
                continue

            wl = wavelength_for_elapsed(elapsed)
            s = Sample(timestamp_s=elapsed, wavelength_nm=wl, raw=adc_raw, voltage_v=voltage)
            self.samples.append(s)
            self._latest = s
            self.statusBar().showMessage(
                f"{wl:7.2f} nm | ADC={adc_raw:4d} | V={voltage:.4f} V | "
                f"Vsig={s.signal_v:.4f} V | I={s.photodiode_current_na:.3f} nA | "
                f"R={s.responsivity_a_per_w:.4f} A/W | I_corr={s.corrected_intensity:.4f}"
            )

        self._refresh_plots()

    # ── Comandos ──────────────────────────────────────────────────────────────

    def _cmd_scan(self) -> None:
        """Inicia el barrido espectral: resetea datos, envia 'w', activa t=0."""
        if not self._esp32:
            QMessageBox.warning(self, "Sin conexion", "Primero conecta el ESP32.")
            return
        self._reset(silent=True)
        if self._send("w"):
            self._scan_t0 = time.time()
            self._collecting = True
            self._set_state("scanning")
            self.statusBar().showMessage(
                f"Barrido iniciado — {SCAN_DURATION_S:.2f} s para cubrir "
                f"{WAVELENGTH_START:.0f}–{WAVELENGTH_END:.0f} nm"
            )

    def _cmd_stop_meas(self) -> None:
        """Detiene solo la recoleccion de datos; el motor sigue corriendo."""
        self._collecting = False
        self._scan_t0 = None
        self._set_state("live" if self._esp32 else "idle")
        self.statusBar().showMessage("Medicion detenida — motor continua (detenerlo con DETENER MOTOR)")

    def _cmd_motor(self, cmd: str, label: str) -> None:
        """Envia un comando de motor sin activar la recoleccion de datos."""
        if not self._send(cmd):
            return
        if cmd == "x":
            self._collecting = False
            self._scan_t0 = None
            self._set_state("live" if self._esp32 else "idle")
        self.statusBar().showMessage(f"Motor — {label.strip()} (cmd: {cmd!r})")

    # ── Actualizacion de graficas y readouts ──────────────────────────────────

    def _reset(self, silent: bool = False) -> None:
        self.samples.clear()
        self._latest = None
        self._scan_t0 = None
        self._collecting = False
        if self._esp32:
            try:
                self._esp32.reset_input_buffer()
            except Exception:
                pass
        self.raw_plot.clear_data()
        self.corr_plot.clear_data()
        self._sample_cnt.setText("0")
        self._update_readouts(None)
        if not silent:
            self.statusBar().showMessage("Datos reiniciados — listo para nuevo barrido")

    def _refresh_plots(self) -> None:
        if not self.samples:
            return
        wls  = [s.wavelength_nm for s in self.samples]
        vout = [s.voltage_v     for s in self.samples]
        corr = [s.corrected_intensity for s in self.samples]
        self.raw_plot.update_data(wls, vout)
        self.corr_plot.update_data(wls, corr)
        self._sample_cnt.setText(str(len(self.samples)))
        self._update_readouts(self._latest)

    def _update_readouts(self, s: Sample | None) -> None:
        if s is None:
            self._r_lambda.set("--", sw_color="#1b2130")
            for card in (self._r_vout, self._r_vsig, self._r_iphoto,
                         self._r_resp, self._r_icorr, self._r_adc):
                card.set("--")
            self._r_bar.set_wavelength(None)
            return
        r, g, b = wavelength_to_rgb(s.wavelength_nm)
        self._r_lambda.set(f"{s.wavelength_nm:.1f}", sw_color=f"rgb({r},{g},{b})")
        self._r_vout.set(f"{s.voltage_v:.4f}")
        self._r_vsig.set(f"{s.signal_v:.4f}")
        self._r_iphoto.set(f"{s.photodiode_current_na:.3f}")
        self._r_resp.set(f"{s.responsivity_a_per_w:.4f}")
        self._r_icorr.set(f"{s.corrected_intensity:.4f}")
        self._r_adc.set(str(s.raw))
        self._r_bar.set_wavelength(s.wavelength_nm)

    def _set_state(self, state: str) -> None:
        STATES = {
            "idle":      ("● ESPERANDO",       "#5a6178", "#1b2130",               "#0b0e16"),
            "connecting":("● CONECTANDO",      "#eab308", "rgba(234,179,8,0.45)",  "#12100a"),
            "live":      ("● EN VIVO",         "#00e5ff", "rgba(0,229,255,0.55)",  "#071016"),
            "scanning":  ("● BARRIDO ACTIVO",  "#22c55e", "rgba(34,197,94,0.55)",  "#071208"),
            "error":     ("● SIN PUERTO",      "#ff3b54", "rgba(255,59,84,0.55)",  "#16070a"),
        }
        txt, col, bord, bg = STATES.get(state, STATES["idle"])
        self._pill.setText(txt)
        self._pill.setStyleSheet(
            f"QLabel#statusPill {{"
            f"  border: 1px solid; border-radius: 13px; padding: 6px 12px;"
            f"  font-size: 9px; letter-spacing: 2px;"
            f"  color: {col}; border-color: {bord}; background: {bg};"
            f"}}"
        )

    # ── Guardado ──────────────────────────────────────────────────────────────

    def _save_csv_dialog(self) -> None:
        if not self.samples:
            QMessageBox.information(self, "Sin datos", "No hay muestras para guardar.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar CSV", str(CSV_OUTPUT), "CSV (*.csv);;Todos los archivos (*)"
        )
        if path:
            save_csv(self.samples, Path(path))
            self.statusBar().showMessage(f"CSV guardado: {path}")

    def _save_png_dialog(self) -> None:
        if not self.samples:
            QMessageBox.information(self, "Sin datos", "No hay muestras para guardar.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar graficas PNG", str(PNG_OUTPUT), "PNG (*.png);;Todos los archivos (*)"
        )
        if not path:
            return
        base = Path(path)
        try:
            from pyqtgraph.exporters import ImageExporter
            for plot, suffix in [
                (self.raw_plot,  "_sin_correccion"),
                (self.corr_plot, "_con_correccion"),
            ]:
                exp = ImageExporter(plot.getPlotItem())
                exp.parameters()["width"] = 1920
                out_path = base.with_name(f"{base.stem}{suffix}{base.suffix}")
                exp.export(str(out_path))
            self.statusBar().showMessage(
                f"PNG guardados: {base.stem}_sin_correccion y {base.stem}_con_correccion"
            )
        except Exception as exc:
            QMessageBox.critical(self, "Error al exportar PNG", str(exc))

    # ── Cierre ────────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        self._timer.stop()
        if self.samples:
            save_csv(self.samples, CSV_OUTPUT)
            self.statusBar().showMessage(f"Datos guardados automaticamente en {CSV_OUTPUT}")
        if self._esp32 is not None:
            try:
                self._esp32.close()
            except Exception:
                pass
        event.accept()


# ── Punto de entrada ──────────────────────────────────────────────────────────

def main() -> None:
    try:
        import serial  # noqa: F401
    except ImportError:
        print("ERROR: pyserial no instalado.")
        print("Ejecuta:  pip install pyserial PyQt6 pyqtgraph numpy")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    win = SpectrometerApp()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
