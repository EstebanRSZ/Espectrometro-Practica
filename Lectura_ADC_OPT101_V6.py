"""
Lectura y grafica del OPT101 conectado al ADC34 de un ESP32 WROOM-32U.

El firmware del ESP32 envia por Serial lineas como:
    OPT101 -> raw: 1234 | 0.994 V

Este script:
  - Lee el puerto serial COM7 a 115200 baudios.
  - Extrae raw ADC y voltaje.
  - Asigna cada muestra a una longitud de onda dentro de un barrido configurable.
  - Corrige la medicion por la respuesta espectral tipica del OPT101.
  - Muestra una interfaz PyQt6 + PyQtGraph con graficas separadas para la medicion
    OPT101 sin corregir y corregida.
  - Permite reiniciar la lectura, guardar las graficas como PNG y exportar CSV.
  - Guarda los datos en CSV al cerrar.

La cadena fisica del sensor es:
    Luz P_opt(lambda) [W]
      -> Fotodiodo: I_D = R_I(lambda) * P_opt          [A]
      -> Transimpedancia 1 MΩ: V_out = I_D * R_F + V_B [V]

La correccion aplicada recupera la potencia optica relativa:
    P_opt(lambda) = (V_out(lambda) - V_dark) / (R_I(lambda) * R_F)

Como R_F = 1 MΩ es constante, para comparacion espectral relativa basta:
    intensidad_corregida(lambda) = (V_out(lambda) - V_dark) / R_I(lambda)

donde:
  V_dark ~ 7.5 mV es el voltaje de pedestal en oscuridad (datasheet sec. 6.5).
  R_I(lambda) es la responsividad espectral tipica del fotodiodo [A/W].

Sin restar V_dark, el offset contamina la correccion, especialmente en UV/azul
donde la senal es pequeña y el offset es una fraccion significativa de V_out.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QBrush, QColor, QLinearGradient
from PyQt6.QtWidgets import (
    QApplication,
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

# -------------------- Configuracion --------------------
SERIAL_PORT = "COM7"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0

# Ajusta estos valores segun tu monocromador/red/motor/calibracion.
# Rango visible usado para comparar contra el espectro del flash de celular.
WAVELENGTH_START_NM = 400.0
WAVELENGTH_END_NM = 700.0
POINTS_PER_SCAN = 300

# Si quieres usar el ADC crudo como intensidad, cambia a "raw".
INTENSITY_MODE = "voltage"  # "voltage" o "raw"

CSV_OUTPUT = Path("lectura_opt101_spectral.csv")
PNG_OUTPUT = Path("lectura_opt101_spectral.png")

# Voltaje de pedestal en oscuridad del OPT101.
# Datasheet sec. 6.5: tipico 7.5 mV, maximo 10 mV.
# Para mayor precision, tapar el sensor completamente, leer V_out en reposo
# y asignar ese valor aqui antes de hacer un barrido.
V_DARK_V: float = 0.0075

# Resistencia de transimpedancia interna del OPT101.
# Con el circuito estandar: I_D = (V_out - V_dark) / R_F.
TRANSIMPEDANCE_OHM: float = 1_000_000.0

# Tabla tipica de responsividad espectral del OPT101/silicio, tomada de la
# curva del datasheet como puntos aproximados. Ajusta estos valores si haces
# una digitalizacion mas precisa de la grafica del datasheet.
# Unidades: A/W. Para correccion relativa importa principalmente la forma.
OPT101_RESPONSIVITY_A_PER_W = [
    (300.0, 0.02),
    (350.0, 0.10),
    (400.0, 0.19),
    (450.0, 0.27),
    (500.0, 0.33),
    (550.0, 0.38),
    (600.0, 0.43),
    (650.0, 0.46),
    (700.0, 0.48),
    (750.0, 0.50),
    (800.0, 0.52),
    (850.0, 0.54),
    (900.0, 0.56),
    (950.0, 0.55),
    (1000.0, 0.45),
    (1050.0, 0.25),
    (1100.0, 0.05),
]


ADC_LINE_RE = re.compile(
    r"raw:\s*(?P<raw>\d+)\s*\|\s*(?P<voltage>\d+(?:\.\d+)?)\s*V",
    re.IGNORECASE,
)


@dataclass
class Sample:
    timestamp_s: float
    wavelength_nm: float
    raw: int
    voltage_v: float

    @property
    def signal_v(self) -> float:
        """Voltaje de senal: V_out menos el pedestal oscuro. Nunca negativo."""
        return max(self.voltage_v - V_DARK_V, 0.0)

    @property
    def photodiode_current_a(self) -> float:
        """Corriente estimada del fotodiodo OPT101 a partir de la transimpedancia."""
        return self.signal_v / TRANSIMPEDANCE_OHM

    @property
    def photodiode_current_na(self) -> float:
        return self.photodiode_current_a * 1e9

    @property
    def intensity(self) -> float:
        if INTENSITY_MODE.lower() == "raw":
            return float(self.raw)
        return self.signal_v

    @property
    def responsivity_a_per_w(self) -> float:
        return interpolate(OPT101_RESPONSIVITY_A_PER_W, self.wavelength_nm)

    @property
    def corrected_intensity(self) -> float:
        # P_opt(lambda) ∝ (V_out - V_dark) / R_I(lambda)
        # Se usa signal_v (no intensity) para que la correccion fisica sea siempre
        # en dominio de voltaje, independientemente de INTENSITY_MODE.
        responsivity = max(self.responsivity_a_per_w, 1e-9)
        return self.signal_v / responsivity


def interpolate(points: list[tuple[float, float]], x_value: float) -> float:
    """Interpolacion lineal con saturacion en los extremos."""
    ordered = sorted(points)
    if x_value <= ordered[0][0]:
        return ordered[0][1]
    if x_value >= ordered[-1][0]:
        return ordered[-1][1]

    for (x0, y0), (x1, y1) in zip(ordered, ordered[1:]):
        if x0 <= x_value <= x1:
            fraction = (x_value - x0) / (x1 - x0)
            return y0 + fraction * (y1 - y0)

    return ordered[-1][1]


def wavelength_for_index(index: int) -> float:
    """Convierte el numero de muestra del barrido a longitud de onda."""
    if POINTS_PER_SCAN <= 1:
        return WAVELENGTH_START_NM

    scan_index = index % POINTS_PER_SCAN
    fraction = scan_index / (POINTS_PER_SCAN - 1)
    return WAVELENGTH_START_NM + fraction * (WAVELENGTH_END_NM - WAVELENGTH_START_NM)


def parse_adc_line(line: str) -> tuple[int, float] | None:
    """Extrae raw y voltaje de una linea enviada por el ESP32."""
    match = ADC_LINE_RE.search(line)
    if not match:
        return None

    raw = int(match.group("raw"))
    voltage = float(match.group("voltage"))
    return raw, voltage


def save_csv(samples: list[Sample], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        # Primera fila: parametros de calibracion usados
        writer.writerow([f"# v_dark_v={V_DARK_V:.6f}", f"intensity_mode={INTENSITY_MODE}"])
        writer.writerow(
            [
                "timestamp_s",
                "wavelength_nm",
                "raw_adc",
                "voltage_v",
                "signal_v",
                "photodiode_current_a",
                "photodiode_current_na",
                "intensity",
                "opt101_responsivity_a_per_w",
                "corrected_intensity",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.timestamp_s:.3f}",
                    f"{sample.wavelength_nm:.3f}",
                    sample.raw,
                    f"{sample.voltage_v:.6f}",
                    f"{sample.signal_v:.6f}",
                    f"{sample.photodiode_current_a:.12f}",
                    f"{sample.photodiode_current_na:.6f}",
                    f"{sample.intensity:.6f}",
                    f"{sample.responsivity_a_per_w:.6f}",
                    f"{sample.corrected_intensity:.6f}",
                ]
            )


# -------------------- Visual Utilities --------------------

def wavelength_to_rgb(wl: float) -> tuple[int, int, int]:
    """CIE approximation, ported from simulacion_difraccion.html."""
    if 380 <= wl < 440:
        r, g, b = -(wl - 440) / 60, 0.0, 1.0
    elif wl < 490:
        r, g, b = 0.0, (wl - 440) / 50, 1.0
    elif wl < 510:
        r, g, b = 0.0, 1.0, -(wl - 510) / 20
    elif wl < 580:
        r, g, b = (wl - 510) / 70, 1.0, 0.0
    elif wl < 645:
        r, g, b = 1.0, -(wl - 645) / 65, 0.0
    else:
        r, g, b = 1.0, 0.0, 0.0
    return (round(r * 255), round(g * 255), round(b * 255))


APP_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0a0c10;
    color: #c8d0e0;
    font-family: 'Space Mono', 'Courier New', monospace;
    font-size: 11px;
}
QFrame#header {
    border-bottom: 1px solid #1b2130;
    background: transparent;
}
QFrame#logoMark {
    border: 1px solid rgba(0, 229, 255, 0.55);
    border-radius: 8px;
    background: rgba(0, 229, 255, 0.10);
}
QLabel#logoDot {
    color: #00e5ff;
    font-size: 18px;
}
QLabel#titleLabel {
    color: #f0f4fb;
    font-family: 'Syne', 'Arial', sans-serif;
    font-size: 18px;
    font-weight: 800;
}
QLabel#subtitleLabel {
    color: #424a60;
    font-size: 9px;
    letter-spacing: 2px;
}
QLabel#statusPill {
    border: 1px solid #1b2130;
    border-radius: 13px;
    background: #0b0e16;
    color: #5a6178;
    padding: 6px 12px;
    font-size: 9px;
    letter-spacing: 2px;
}
QLabel#sampleTotal {
    color: #eef3fb;
    font-size: 25px;
    font-weight: 700;
}
QLabel#sampleTotalCaption {
    color: #424a60;
    font-size: 8px;
    letter-spacing: 2px;
}
QFrame#toolbar {
    background: transparent;
}
QFrame#plotCard, QFrame#readoutCard, QFrame#barCard {
    background: #0b0e16;
    border: 1px solid #1b2130;
    border-radius: 8px;
}
QLabel#plotTitle {
    color: #e6ecf6;
    font-size: 12px;
    font-weight: 700;
}
QLabel#plotTag {
    border: 1px solid rgba(0, 229, 255, 0.55);
    border-radius: 4px;
    background: rgba(0, 229, 255, 0.10);
    color: #00e5ff;
    padding: 3px 8px;
    font-size: 8px;
    letter-spacing: 1px;
}
QLabel#readoutLabel, QLabel#barLabel {
    color: #424a60;
    font-size: 8px;
    letter-spacing: 2px;
}
QLabel#readoutValue {
    color: #eef3fb;
    font-size: 24px;
    font-weight: 700;
}
QLabel#readoutValueBig {
    color: #00e5ff;
    font-size: 31px;
    font-weight: 700;
}
QLabel#readoutUnit {
    color: #424a60;
    font-size: 10px;
}
QFrame#swatch {
    border: 1px solid rgba(255, 255, 255, 0.25);
    border-radius: 3px;
    background: #1b2130;
}
QProgressBar#scanProgress {
    min-height: 9px;
    max-height: 9px;
    border: 1px solid #1b2130;
    border-radius: 4px;
    background: rgba(139, 92, 246, 0.14);
    text-align: center;
}
QProgressBar#scanProgress::chunk {
    border-radius: 4px;
    background: qlineargradient(
        x1: 0, y1: 0, x2: 1, y2: 0,
        stop: 0 #8b5cf6,
        stop: 0.18 #3b82f6,
        stop: 0.35 #06b6d4,
        stop: 0.52 #22c55e,
        stop: 0.70 #eab308,
        stop: 0.86 #f97316,
        stop: 1 #ef4444
    );
}
QPushButton {
    padding: 9px 14px;
    border: 1px solid #1e2330;
    border-radius: 6px;
    background: #0d1119;
    color: #c8d0e0;
    font-size: 10px;
    letter-spacing: 1px;
}
QPushButton:hover {
    border-color: #00e5ff;
    color: #00e5ff;
    background: rgba(0, 229, 255, 0.05);
}
QPushButton:pressed {
    background: rgba(0, 229, 255, 0.12);
}
QPushButton#motorW, QPushButton#motorS,
QPushButton#motorI, QPushButton#motorX {
    font-size: 9px;
    padding: 8px 10px;
    color: #5a6070;
}
QPushButton#motorW:hover, QPushButton#motorS:hover,
QPushButton#motorI:hover, QPushButton#motorX:hover {
    color: #00e5ff;
    border-color: #00e5ff;
}
QPushButton#motorX:hover {
    color: #ff3b54;
    border-color: #ff3b54;
}
QStatusBar {
    background: #111318;
    color: #5a6070;
    font-size: 10px;
    border-top: 1px solid #1e2330;
    padding: 2px 10px;
}
QStatusBar::item { border: none; }
"""

# -------------------- Plot Widget --------------------

class SpectrumPlot(pg.PlotWidget):
    """
    PyQtGraph PlotWidget for a single spectral channel.

    gradient=True  → corrected spectrum: cyan line + wavelength-colored fill
    gradient=False → raw spectrum: gray line + dim gray fill
    """

    _GRADIENT_WLS = (400, 450, 500, 550, 600, 650, 700)

    def __init__(
        self,
        title: str,
        y_label: str,
        gradient: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent=parent)
        self._gradient = gradient
        self._configure_appearance(title, y_label)

        pen = pg.mkPen("#00e5ff", width=1.5) if gradient else pg.mkPen("#5a6070", width=1.5)
        brush = self._make_gradient_brush() if gradient else QBrush(QColor(90, 96, 112, 38))

        self._curve = pg.PlotCurveItem(antialias=True)
        self._curve.setPen(pen)
        self._curve.setFillLevel(0)
        self._curve.setBrush(brush)
        self.addItem(self._curve)

        self._empty_label = pg.TextItem(
            text="Esperando muestras...",
            color="#5a6070",
            anchor=(0.5, 0.5),
        )
        self.addItem(self._empty_label)
        self._empty_label.setPos((WAVELENGTH_START_NM + WAVELENGTH_END_NM) / 2, 0.5)

    def _configure_appearance(self, title: str, y_label: str) -> None:
        self.setBackground("#0a0c10")
        pi = self.getPlotItem()
        if title:
            pi.setTitle(
                f'<span style="color:#5a6070; font-size:9pt;'
                f' font-family:Space Mono,monospace; letter-spacing:2px;">'
                f"{title.upper()}</span>"
            )
        pi.setLabel("bottom", "Longitud de onda (nm)", color="#5a6070", size="9pt")
        pi.setLabel("left", y_label, color="#5a6070", size="9pt")
        pi.showGrid(x=True, y=True, alpha=0.15)
        for ax_name in ("bottom", "left", "top", "right"):
            ax = pi.getAxis(ax_name)
            ax.setPen(pg.mkPen("#1e2330"))
            ax.setTextPen(pg.mkPen("#5a6070"))
        pi.setXRange(WAVELENGTH_START_NM, WAVELENGTH_END_NM, padding=0.02)
        pi.setYRange(0, 1, padding=0.05)

    def _make_gradient_brush(self) -> QBrush:
        """Wavelength-colored QLinearGradient in data coordinates (nm)."""
        grad = QLinearGradient(WAVELENGTH_START_NM, 0, WAVELENGTH_END_NM, 0)
        wl_range = WAVELENGTH_END_NM - WAVELENGTH_START_NM
        for wl in self._GRADIENT_WLS:
            stop = (wl - WAVELENGTH_START_NM) / wl_range
            r, g, b = wavelength_to_rgb(wl)
            grad.setColorAt(stop, QColor(r, g, b, 160))
        return QBrush(grad)

    def set_data(self, x_values: list[float], y_values: list[float]) -> None:
        if len(x_values) < 2:
            self._curve.setData([], [])
            self._empty_label.setVisible(True)
            return
        self._empty_label.setVisible(False)
        y_max = max(y_values) if y_values else 1.0
        if y_max > 0:
            self.getPlotItem().setYRange(0, y_max * 1.1, padding=0)
        self._curve.setData(x=x_values, y=y_values)

    def clear(self) -> None:
        self._curve.setData([], [])
        self._empty_label.setVisible(True)
        self.getPlotItem().setYRange(0, 1, padding=0.05)


class PlotCard(QFrame):
    """Contenedor visual para una grafica, equivalente a las tarjetas del HTML."""

    def __init__(self, title: str, tag: str, plot: SpectrumPlot) -> None:
        super().__init__()
        self.setObjectName("plotCard")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(16, 12, 16, 10)
        header_layout.setSpacing(10)

        title_label = QLabel(title)
        title_label.setObjectName("plotTitle")
        tag_label = QLabel(tag)
        tag_label.setObjectName("plotTag")

        header_layout.addWidget(title_label)
        header_layout.addStretch()
        header_layout.addWidget(tag_label)
        layout.addWidget(header)
        layout.addWidget(plot, stretch=1)


class ReadoutCard(QFrame):
    """Tarjeta compacta para las lecturas numericas del dashboard."""

    def __init__(self, label: str, unit: str, *, big: bool = False, swatch: bool = False) -> None:
        super().__init__()
        self.setObjectName("readoutCard")
        self._unit = unit
        self._has_swatch = swatch

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(5)

        label_row = QHBoxLayout()
        label_row.setSpacing(7)
        if swatch:
            self._swatch = QFrame()
            self._swatch.setObjectName("swatch")
            self._swatch.setFixedSize(11, 11)
            label_row.addWidget(self._swatch)
        else:
            self._swatch = None

        label_widget = QLabel(label.upper())
        label_widget.setObjectName("readoutLabel")
        label_row.addWidget(label_widget)
        label_row.addStretch()
        layout.addLayout(label_row)

        value_row = QHBoxLayout()
        value_row.setSpacing(6)
        self._value = QLabel("--")
        self._value.setObjectName("readoutValueBig" if big else "readoutValue")
        self._value.setMinimumWidth(90)
        self._unit_label = QLabel(unit)
        self._unit_label.setObjectName("readoutUnit")
        value_row.addWidget(self._value)
        value_row.addWidget(self._unit_label)
        value_row.addStretch()
        layout.addLayout(value_row)

    def set_value(self, value: str, *, swatch_color: str | None = None) -> None:
        self._value.setText(value)
        if self._swatch is not None:
            self._swatch.setStyleSheet(
                f"QFrame#swatch {{ background: {swatch_color or '#1b2130'}; }}"
            )


class ScanBarCard(QFrame):
    """Barra de posicion de barrido, portada del readout inferior del HTML."""

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("barCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(8)

        label = QLabel("POSICION DE BARRIDO")
        label.setObjectName("barLabel")
        layout.addWidget(label)

        self._progress = QProgressBar()
        self._progress.setObjectName("scanProgress")
        self._progress.setRange(0, 1000)
        self._progress.setTextVisible(False)
        layout.addWidget(self._progress)

        scale_row = QHBoxLayout()
        for text in (
            f"{WAVELENGTH_START_NM:.0f}",
            f"{(WAVELENGTH_START_NM + WAVELENGTH_END_NM) / 2:.0f}",
            f"{WAVELENGTH_END_NM:.0f} nm",
        ):
            lbl = QLabel(text)
            lbl.setObjectName("readoutLabel")
            scale_row.addWidget(lbl)
        scale_row.setStretch(0, 1)
        scale_row.setStretch(1, 1)
        scale_row.setStretch(2, 1)
        layout.addLayout(scale_row)

    def set_wavelength(self, wavelength_nm: float | None) -> None:
        if wavelength_nm is None:
            self._progress.setValue(0)
            return
        span = WAVELENGTH_END_NM - WAVELENGTH_START_NM
        fraction = 0.0 if span <= 0 else (wavelength_nm - WAVELENGTH_START_NM) / span
        self._progress.setValue(round(max(0.0, min(1.0, fraction)) * 1000))


# -------------------- Main Application --------------------

class Opt101App(QMainWindow):
    def __init__(self, serial_module, serial_port: str = SERIAL_PORT) -> None:
        super().__init__()
        self._serial = serial_module
        self._serial_port = serial_port
        self.samples: list[Sample] = []
        self.start_time = time.time()
        self._esp32 = None
        self._latest_sample: Sample | None = None

        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._read_serial)

        self._build_ui()
        self._set_connection_state("idle")

    def _build_ui(self) -> None:
        self.setWindowTitle("Espectrometro - Lectura espectral OPT101")
        self.resize(1180, 760)
        self.setMinimumSize(960, 640)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(22, 16, 22, 16)
        root_layout.setSpacing(14)

        root_layout.addWidget(self._build_header())
        root_layout.addWidget(self._build_toolbar())

        self.raw_plot = SpectrumPlot(
            title="",
            y_label="Intensidad Bruta (V)",
            gradient=False,
        )
        self.corrected_plot = SpectrumPlot(
            title="",
            y_label="Intensidad Corregida (u.a.)",
            gradient=True,
        )

        plot_grid = QGridLayout()
        plot_grid.setContentsMargins(0, 0, 0, 0)
        plot_grid.setHorizontalSpacing(16)
        plot_grid.setVerticalSpacing(16)
        plot_grid.addWidget(
            PlotCard("OPT101 - Espectro bruto", "SIN CORRECCION", self.raw_plot),
            0,
            0,
        )
        plot_grid.addWidget(
            PlotCard(
                "OPT101 - Corregido por responsividad",
                "I = V / R(lambda)",
                self.corrected_plot,
            ),
            0,
            1,
        )
        plot_grid.setColumnStretch(0, 1)
        plot_grid.setColumnStretch(1, 1)
        root_layout.addLayout(plot_grid, stretch=1)

        root_layout.addWidget(self._build_readouts())

    def _build_header(self) -> QWidget:
        header = QFrame()
        header.setObjectName("header")
        layout = QHBoxLayout(header)
        layout.setContentsMargins(0, 0, 0, 13)
        layout.setSpacing(14)

        logo = QFrame()
        logo.setObjectName("logoMark")
        logo.setFixedSize(34, 34)
        logo_layout = QVBoxLayout(logo)
        logo_layout.setContentsMargins(0, 0, 0, 0)
        logo_dot = QLabel("●")
        logo_dot.setObjectName("logoDot")
        logo_dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo_layout.addWidget(logo_dot)

        title_block = QWidget()
        title_layout = QVBoxLayout(title_block)
        title_layout.setContentsMargins(0, 0, 0, 0)
        title_layout.setSpacing(2)
        title = QLabel("Espectrometro - Lectura Espectral")
        title.setObjectName("titleLabel")
        subtitle = QLabel("OPT101 - BARRIDO ANGULAR - OPTOELECTRONICA")
        subtitle.setObjectName("subtitleLabel")
        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)

        self._status_pill = QLabel()
        self._status_pill.setObjectName("statusPill")
        self._status_pill.setMinimumWidth(178)
        self._status_pill.setAlignment(Qt.AlignmentFlag.AlignCenter)

        sample_block = QWidget()
        sample_layout = QVBoxLayout(sample_block)
        sample_layout.setContentsMargins(0, 0, 0, 0)
        sample_layout.setSpacing(0)
        self._sample_total = QLabel("0")
        self._sample_total.setObjectName("sampleTotal")
        self._sample_total.setAlignment(Qt.AlignmentFlag.AlignRight)
        sample_caption = QLabel("MUESTRAS")
        sample_caption.setObjectName("sampleTotalCaption")
        sample_caption.setAlignment(Qt.AlignmentFlag.AlignRight)
        sample_layout.addWidget(self._sample_total)
        sample_layout.addWidget(sample_caption)

        layout.addWidget(logo)
        layout.addWidget(title_block)
        layout.addStretch()
        layout.addWidget(self._status_pill)
        layout.addWidget(sample_block)
        return header

    def _build_toolbar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("toolbar")
        h = QHBoxLayout(bar)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(9)

        btn_reset = QPushButton("REINICIAR")
        btn_reset.clicked.connect(self._reset)
        btn_png = QPushButton("GUARDAR PNG")
        btn_png.clicked.connect(self._save_png_dialog)
        btn_csv = QPushButton("GUARDAR CSV")
        btn_csv.clicked.connect(self._save_csv_dialog)
        h.addWidget(btn_reset)
        h.addWidget(btn_png)
        h.addWidget(btn_csv)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("QFrame { color: #1e2330; max-width: 1px; }")
        h.addSpacing(8)
        h.addWidget(sep)
        h.addSpacing(8)

        motor_lbl = QLabel("MOTOR")
        motor_lbl.setStyleSheet(
            "color: #5a6070; font-size: 9px; letter-spacing: 2px;"
        )
        h.addWidget(motor_lbl)

        for label_text, cmd, obj_name in [
            ("W → SUBIR (ANTIHORARIO)", "w", "motorW"),
            ("S → BAJAR (HORARIO)", "s", "motorS"),
            ("I → AUTO", "i", "motorI"),
            ("X → DETENER", "x", "motorX"),
        ]:
            btn = QPushButton(label_text)
            btn.setObjectName(obj_name)
            btn.clicked.connect(lambda _checked, c=cmd: self._send_motor_command(c))
            h.addWidget(btn)

        h.addStretch()

        return bar

    def _build_readouts(self) -> QWidget:
        wrapper = QWidget()
        layout = QGridLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        self._lambda_readout = ReadoutCard("lambda detector", "nm", big=True, swatch=True)
        self._voltage_readout = ReadoutCard("voltaje opt101", "V")
        self._current_readout = ReadoutCard("corriente opt101", "nA")
        self._adc_readout = ReadoutCard("adc crudo", "/ 4095")
        self._scan_bar = ScanBarCard()

        layout.addWidget(self._lambda_readout, 0, 0)
        layout.addWidget(self._voltage_readout, 0, 1)
        layout.addWidget(self._current_readout, 0, 2)
        layout.addWidget(self._adc_readout, 0, 3)
        layout.addWidget(self._scan_bar, 0, 4)
        layout.setColumnStretch(0, 13)
        layout.setColumnStretch(1, 10)
        layout.setColumnStretch(2, 10)
        layout.setColumnStretch(3, 10)
        layout.setColumnStretch(4, 22)

        self._update_readouts(None)
        return wrapper

    def start(self) -> None:
        self._set_connection_state("connecting")
        try:
            self._esp32 = self._serial.Serial(self._serial_port, BAUD_RATE, timeout=0)
            QTimer.singleShot(2000, self._serial_ready)
            self._timer.start()
        except self._serial.SerialException as exc:
            self._set_connection_state("error")
            self.statusBar().showMessage(f"No se pudo abrir {self._serial_port}: {exc}")
            QMessageBox.critical(
                self,
                "Puerto serial no disponible",
                f"No se pudo abrir {self._serial_port}.\n\n"
                "Revisa que el ESP32 esté conectado y que "
                "PlatformIO/Monitor Serial esté cerrado.",
            )

    def _serial_ready(self) -> None:
        if self._esp32 is not None:
            self._esp32.reset_input_buffer()
            self._set_connection_state("live")
            self.statusBar().showMessage(
                f"Leyendo {self._serial_port} a {BAUD_RATE} baudios"
            )

    def _read_serial(self) -> None:
        if self._esp32 is None:
            return

        for _ in range(50):
            raw_line = self._esp32.readline().decode("utf-8", errors="ignore").strip()
            if not raw_line:
                break

            parsed = parse_adc_line(raw_line)
            if parsed is None:
                self.statusBar().showMessage(raw_line)
                continue

            raw, voltage = parsed
            wavelength = wavelength_for_index(len(self.samples))
            sample = Sample(
                timestamp_s=time.time() - self.start_time,
                wavelength_nm=wavelength,
                raw=raw,
                voltage_v=voltage,
            )
            self.samples.append(sample)
            self._latest_sample = sample
            self.statusBar().showMessage(
                f"{sample.wavelength_nm:7.2f} nm | raw={sample.raw:4d} | "
                f"V={sample.voltage_v:.4f}  sig={sample.signal_v:.4f} V | "
                f"I={sample.photodiode_current_na:.3f} nA | "
                f"R={sample.responsivity_a_per_w:.3f} A/W | "
                f"corr={sample.corrected_intensity:.4f}"
            )

        self._update_plots()

    def _update_plots(self) -> None:
        scan = self.samples[-POINTS_PER_SCAN:]
        wls = [s.wavelength_nm for s in scan]
        self.raw_plot.set_data(wls, [s.intensity for s in scan])
        self.corrected_plot.set_data(wls, [s.corrected_intensity for s in scan])
        self._sample_total.setText(f"{len(self.samples):,}".replace(",", "."))
        self._update_readouts(self._latest_sample)

    def _reset(self) -> None:
        self.samples.clear()
        self._latest_sample = None
        self.start_time = time.time()
        if self._esp32 is not None:
            self._esp32.reset_input_buffer()
        self.raw_plot.clear()
        self.corrected_plot.clear()
        self._sample_total.setText("0")
        self._update_readouts(None)
        self._set_connection_state("live" if self._esp32 is not None else "idle")
        self.statusBar().showMessage("Lectura reiniciada")

    def _send_motor_command(self, command: str) -> None:
        if self._esp32 is None:
            QMessageBox.warning(
                self,
                "Puerto serial no disponible",
                "No hay conexión activa con el ESP32.",
            )
            return
        try:
            self._esp32.write(f"{command}\n".encode("utf-8"))
            self.statusBar().showMessage(f"Comando enviado al ESP32: {command}")
        except self._serial.SerialException as exc:
            self.statusBar().showMessage(f"No se pudo enviar '{command}': {exc}")
            QMessageBox.critical(
                self,
                "Error serial",
                f"No se pudo enviar el comando '{command}'.\n\n{exc}",
            )

    def _save_csv_dialog(self) -> None:
        if not self.samples:
            QMessageBox.information(self, "Sin datos", "No hay muestras para guardar.")
            return
        selected, _ = QFileDialog.getSaveFileName(
            self,
            "Guardar CSV",
            str(CSV_OUTPUT),
            "CSV (*.csv);;Todos los archivos (*.*)",
        )
        if selected:
            save_csv(self.samples, Path(selected))
            self.statusBar().showMessage(f"CSV guardado en {selected}")

    def _save_png_dialog(self) -> None:
        if not self.samples:
            QMessageBox.information(self, "Sin datos", "No hay muestras para guardar.")
            return
        selected, _ = QFileDialog.getSaveFileName(
            self,
            "Guardar gráficas PNG",
            str(PNG_OUTPUT),
            "PNG (*.png);;Todos los archivos (*.*)",
        )
        if not selected:
            return
        base_path = Path(selected)
        try:
            from pyqtgraph.exporters import ImageExporter
            for plot, suffix in [
                (self.raw_plot, "_sin_correccion"),
                (self.corrected_plot, "_con_correccion"),
            ]:
                exp = ImageExporter(plot.getPlotItem())
                exp.parameters()["width"] = 1920
                out = base_path.with_name(
                    f"{base_path.stem}{suffix}{base_path.suffix}"
                )
                exp.export(str(out))
            self.statusBar().showMessage(
                f"PNG guardados: {base_path.stem}_sin_correccion "
                f"y {base_path.stem}_con_correccion"
            )
        except Exception as exc:
            QMessageBox.critical(
                self, "Error al exportar", f"No se pudo guardar PNG:\n{exc}"
            )

    def closeEvent(self, event) -> None:
        self._timer.stop()
        if self.samples:
            save_csv(self.samples, CSV_OUTPUT)
        if self._esp32 is not None:
            self._esp32.close()
        event.accept()

    def _set_connection_state(self, state: str) -> None:
        states = {
            "idle": ("● ESPERANDO", "#5a6178", "#1b2130", "#0b0e16"),
            "connecting": ("● CONECTANDO", "#eab308", "rgba(234, 179, 8, 0.45)", "#12100a"),
            "live": ("● LECTURA EN VIVO", "#00e5ff", "rgba(0, 229, 255, 0.55)", "#071016"),
            "error": ("● SIN PUERTO", "#ff3b54", "rgba(255, 59, 84, 0.55)", "#16070a"),
        }
        text, color, border, bg = states.get(state, states["idle"])
        self._status_pill.setText(text)
        self._status_pill.setStyleSheet(
            "QLabel#statusPill {"
            "border: 1px solid;"
            "border-radius: 13px;"
            "padding: 6px 12px;"
            "font-size: 9px;"
            f"color: {color}; border-color: {border}; background: {bg};"
            "}"
        )

    def _update_readouts(self, sample: Sample | None) -> None:
        if sample is None:
            self._lambda_readout.set_value("--", swatch_color="#1b2130")
            self._voltage_readout.set_value("0.000")
            self._current_readout.set_value("0.000")
            self._adc_readout.set_value("0")
            self._scan_bar.set_wavelength(None)
            return

        r, g, b = wavelength_to_rgb(sample.wavelength_nm)
        self._lambda_readout.set_value(
            f"{sample.wavelength_nm:.1f}",
            swatch_color=f"rgb({r}, {g}, {b})",
        )
        self._voltage_readout.set_value(f"{sample.voltage_v:.3f}")
        self._current_readout.set_value(f"{sample.photodiode_current_na:.3f}")
        self._adc_readout.set_value(str(sample.raw))
        self._scan_bar.set_wavelength(sample.wavelength_nm)


# -------------------- Entry Point --------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dashboard en vivo para el OPT101 conectado a un ESP32."
    )
    parser.add_argument(
        "port",
        nargs="?",
        default=SERIAL_PORT,
        help=f"Puerto serial del ESP32. Por defecto: {SERIAL_PORT}",
    )
    args = parser.parse_args()

    missing_dependencies: list[str] = []

    try:
        import serial
    except ModuleNotFoundError:
        missing_dependencies.append("pyserial")
        serial = None

    if missing_dependencies:
        print("Faltan dependencias para ejecutar este script:")
        for dependency in missing_dependencies:
            print(f"  - {dependency}")
        print("Ejecuta: pip install pyserial PyQt6 pyqtgraph numpy")
        return

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)

    window = Opt101App(serial, args.port)
    window.show()
    window.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
