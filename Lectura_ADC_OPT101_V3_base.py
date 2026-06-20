"""
Lectura y grafica del OPT101 conectado al ADC34 de un ESP32 WROOM-32U.

El firmware del ESP32 envia por Serial lineas como:
    OPT101 -> raw: 1234 | 0.994 V

Este script:
  - Lee el puerto serial COM7 a 115200 baudios.
  - Extrae raw ADC y voltaje.
  - Asigna cada muestra a una longitud de onda dentro de un barrido configurable.
  - Corrige la medicion por la respuesta espectral tipica del OPT101.
  - Muestra una interfaz Tkinter con graficas separadas para la medicion
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

import csv
import re
import time
import tkinter as tk
import zlib
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from pathlib import Path



# -------------------- Configuracion --------------------
SERIAL_PORT = "COM7"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0

# Calibracion temporal del barrido.
# El ESP32 no envia pasos ni posicion; solo emite una lectura del OPT101 cada
# ~300 ms. Por eso la longitud de onda se estima con el tiempo real del
# barrido medido experimentalmente.
WAVELENGTH_START_NM = 425.0
WAVELENGTH_END_NM = 725.0
SCAN_DURATION_S = 11.55
TOTAL_TRAVEL_MM = 80.0
MOTOR_REV_TRAVEL_MM = 47.0
MOTOR_REV_DURATION_S = 8.38
REVERSE_WAVELENGTH_AXIS = False
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



def wavelength_for_elapsed_time(elapsed_s: float) -> float:
    """Convierte tiempo transcurrido del barrido a longitud de onda."""
    if SCAN_DURATION_S <= 0:
        return WAVELENGTH_START_NM

    fraction = max(0.0, min(1.0, elapsed_s / SCAN_DURATION_S))
    if REVERSE_WAVELENGTH_AXIS:
        fraction = 1.0 - fraction
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
        writer.writerow(
            [
                f"# v_dark_v={V_DARK_V:.6f}",
                f"intensity_mode={INTENSITY_MODE}",
                f"wavelength_start_nm={WAVELENGTH_START_NM:.3f}",
                f"wavelength_end_nm={WAVELENGTH_END_NM:.3f}",
                f"scan_duration_s={SCAN_DURATION_S:.3f}",
                f"total_travel_mm={TOTAL_TRAVEL_MM:.3f}",
                f"motor_rev_travel_mm={MOTOR_REV_TRAVEL_MM:.3f}",
                f"motor_rev_duration_s={MOTOR_REV_DURATION_S:.3f}",
                f"reverse_wavelength_axis={REVERSE_WAVELENGTH_AXIS}",
            ]
        )
        writer.writerow(
            [
                "timestamp_s",
                "wavelength_nm",
                "raw_adc",
                "voltage_v",
                "signal_v",          # voltage_v - V_dark (senal optica real)
                "intensity",         # senal usada para grafica sin correccion
                "opt101_responsivity_a_per_w",
                "corrected_intensity",  # signal_v / responsivity
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
                    f"{sample.intensity:.6f}",
                    f"{sample.responsivity_a_per_w:.6f}",
                    f"{sample.corrected_intensity:.6f}",
                ]
            )


class PlotCanvas(ttk.Frame):
    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        line_color: str,
        y_label: str = "Intensidad",
    ) -> None:
        super().__init__(parent, padding=8)
        self.title = title
        self.line_color = line_color
        self.y_label = y_label
        self.x_values: list[float] = []
        self.y_values: list[float] = []

        title_label = ttk.Label(self, text=title, font=("Segoe UI", 11, "bold"))
        title_label.pack(anchor="w")

        self.canvas = tk.Canvas(self, height=280, background="#ffffff", highlightthickness=1, highlightbackground="#ced4da")
        self.canvas.pack(fill="both", expand=True, pady=(6, 0))
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_data(self, x_values: list[float], y_values: list[float]) -> None:
        self.x_values = x_values
        self.y_values = y_values
        self.redraw()

    def clear(self) -> None:
        self.set_data([], [])

    def redraw(self) -> None:
        width = max(self.canvas.winfo_width(), 300)
        height = max(self.canvas.winfo_height(), 220)
        self.canvas.delete("all")

        margin_left = 58
        margin_right = 18
        margin_top = 18
        margin_bottom = 44
        plot_left = margin_left
        plot_top = margin_top
        plot_right = width - margin_right
        plot_bottom = height - margin_bottom

        self.canvas.create_rectangle(plot_left, plot_top, plot_right, plot_bottom, outline="#adb5bd", fill="#ffffff")

        if len(self.x_values) >= 2:
            x_min = min(self.x_values)
            x_max = max(self.x_values)
            if x_min == x_max:
                x_min -= 1.0
                x_max += 1.0
        else:
            x_min = min(WAVELENGTH_START_NM, WAVELENGTH_END_NM)
            x_max = max(WAVELENGTH_START_NM, WAVELENGTH_END_NM)
        if self.y_values and max(self.y_values) > 0:
            y_data_max = max(self.y_values)
            y_min = -0.05 * y_data_max
            y_max = 1.10 * y_data_max
        else:
            y_min = 0.0
            y_max = 1.0

        for tick in range(6):
            fraction = tick / 5
            y = plot_bottom - fraction * (plot_bottom - plot_top)
            value = y_min + fraction * (y_max - y_min)
            self.canvas.create_line(plot_left, y, plot_right, y, fill="#e9ecef")
            self.canvas.create_text(plot_left - 8, y, text=f"{value:.4g}", anchor="e", fill="#495057", font=("Segoe UI", 8))

        for tick in range(7):
            fraction = tick / 6
            x = plot_left + fraction * (plot_right - plot_left)
            value = x_min + fraction * (x_max - x_min)
            self.canvas.create_line(x, plot_top, x, plot_bottom, fill="#f1f3f5")
            self.canvas.create_text(x, plot_bottom + 16, text=f"{value:.0f}", anchor="n", fill="#495057", font=("Segoe UI", 8))

        self.canvas.create_text(
            (plot_left + plot_right) / 2,
            height - 0.5,
            text="Longitud de onda (nm)",
            anchor="s",
            fill="#343a40",
            font=("Segoe UI", 9),
        )
        self.canvas.create_text(
            12,
            (plot_top + plot_bottom) / 2,
            text=self.y_label,
            anchor="center",
            angle=90,
            fill="#343a40",
            font=("Segoe UI", 9),
        )

        if len(self.x_values) < 2:
            self.canvas.create_text(
                (plot_left + plot_right) / 2,
                (plot_top + plot_bottom) / 2,
                text="Esperando muestras...",
                fill="#6c757d",
                font=("Segoe UI", 10),
            )
            return

        points: list[float] = []
        for x_value, y_value in zip(self.x_values, self.y_values):
            x_fraction = (x_value - x_min) / (x_max - x_min)
            y_fraction = (y_value - y_min) / (y_max - y_min)
            x = plot_left + x_fraction * (plot_right - plot_left)
            y = plot_bottom - y_fraction * (plot_bottom - plot_top)
            points.extend([x, y])

        self.canvas.create_line(*points, fill=self.line_color, width=2, smooth=True)


class PngPlotRenderer:
    FONT_5X7 = {
        " ": ["000", "000", "000", "000", "000", "000", "000"],
        ".": ["0", "0", "0", "0", "0", "0", "1"],
        ",": ["0", "0", "0", "0", "0", "1", "1"],
        "-": ["0000", "0000", "0000", "1111", "0000", "0000", "0000"],
        "_": ["0000", "0000", "0000", "0000", "0000", "0000", "1111"],
        "(": ["01", "10", "10", "10", "10", "10", "01"],
        ")": ["10", "01", "01", "01", "01", "01", "10"],
        "/": ["0001", "0001", "0010", "0010", "0100", "0100", "1000"],
        "%": ["10001", "10010", "00100", "01000", "10010", "00010", "00001"],
        "0": ["111", "101", "101", "101", "101", "101", "111"],
        "1": ["010", "110", "010", "010", "010", "010", "111"],
        "2": ["111", "001", "001", "111", "100", "100", "111"],
        "3": ["111", "001", "001", "111", "001", "001", "111"],
        "4": ["101", "101", "101", "111", "001", "001", "001"],
        "5": ["111", "100", "100", "111", "001", "001", "111"],
        "6": ["111", "100", "100", "111", "101", "101", "111"],
        "7": ["111", "001", "001", "010", "010", "100", "100"],
        "8": ["111", "101", "101", "111", "101", "101", "111"],
        "9": ["111", "101", "101", "111", "001", "001", "111"],
        "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
        "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
        "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
        "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
        "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
        "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
        "G": ["01111", "10000", "10000", "10111", "10001", "10001", "01111"],
        "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
        "I": ["111", "010", "010", "010", "010", "010", "111"],
        "J": ["00111", "00010", "00010", "00010", "10010", "10010", "01100"],
        "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
        "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
        "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
        "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
        "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
        "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
        "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
        "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
        "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
        "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
        "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
        "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
        "W": ["10001", "10001", "10001", "10101", "10101", "11011", "10001"],
        "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
        "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
        "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    }

    def __init__(self, width: int = 1100, height: int = 520) -> None:
        self.width = width
        self.height = height
        self.pixels = bytearray([255, 255, 255] * width * height)

    def draw_plot(
        self,
        x_values: list[float],
        y_values: list[float],
        line_color: tuple[int, int, int],
        title: str = "OPT101 - INTENSIDAD ESPECTRAL",
        x_label: str = "LONGITUD DE ONDA (NM)",
        y_label: str = "INTENSIDAD",
    ) -> bytes:
        self._fill((255, 255, 255))
        left, top, right, bottom = 118, 72, self.width - 34, self.height - 92
        self._rect(left, top, right, bottom, (173, 181, 189))

        if len(x_values) >= 2:
            x_min = min(x_values)
            x_max = max(x_values)
            if x_min == x_max:
                x_min -= 1.0
                x_max += 1.0
        else:
            x_min = min(WAVELENGTH_START_NM, WAVELENGTH_END_NM)
            x_max = max(WAVELENGTH_START_NM, WAVELENGTH_END_NM)
        if y_values and max(y_values) > 0:
            y_data_max = max(y_values)
            y_min = -0.05 * y_data_max
            y_max = 1.10 * y_data_max
        else:
            y_min = 0.0
            y_max = 1.0

        self._text((self.width - self._text_width(title, 3)) // 2, 22, title, (33, 37, 41), scale=3)

        for tick in range(6):
            fraction = tick / 5
            y = int(bottom - fraction * (bottom - top))
            value = y_min + fraction * (y_max - y_min)
            self._line(left, y, right, y, (233, 236, 239))
            tick_text = f"{value:.4g}"
            self._text(left - 16 - self._text_width(tick_text, 2), y - 7, tick_text, (73, 80, 87), scale=2)

        for tick in range(7):
            fraction = tick / 6
            x = int(left + fraction * (right - left))
            value = x_min + fraction * (x_max - x_min)
            self._line(x, top, x, bottom, (241, 243, 245))
            tick_text = f"{value:.0f}"
            self._text(x - self._text_width(tick_text, 2) // 2, bottom + 14, tick_text, (73, 80, 87), scale=2)

        self._rect(left, top, right, bottom, (173, 181, 189))
        self._text((left + right - self._text_width(x_label, 2)) // 2, self.height - 34, x_label, (52, 58, 64), scale=2)
        self._text(22, (top + bottom + self._text_width(y_label, 2)) // 2, y_label, (52, 58, 64), scale=2, angle=90)

        if len(x_values) >= 2:
            mapped = []
            for x_value, y_value in zip(x_values, y_values):
                x_fraction = (x_value - x_min) / (x_max - x_min)
                y_fraction = (y_value - y_min) / (y_max - y_min)
                x_fraction = min(max(x_fraction, 0.0), 1.0)
                y_fraction = min(max(y_fraction, 0.0), 1.0)
                x = int(left + x_fraction * (right - left))
                y = int(bottom - y_fraction * (bottom - top))
                mapped.append((x, y))

            for (x0, y0), (x1, y1) in zip(mapped, mapped[1:]):
                self._line(x0, y0, x1, y1, line_color, width=3)

        return self._to_png()

    def _fill(self, color: tuple[int, int, int]) -> None:
        self.pixels[:] = bytes(color) * (self.width * self.height)

    def _set_pixel(self, x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            index = (y * self.width + x) * 3
            self.pixels[index:index + 3] = bytes(color)

    def _line(self, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int], width: int = 1) -> None:
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        while True:
            radius = width // 2
            for yy in range(y0 - radius, y0 + radius + 1):
                for xx in range(x0 - radius, x0 + radius + 1):
                    self._set_pixel(xx, yy, color)
            if x0 == x1 and y0 == y1:
                break
            error2 = 2 * error
            if error2 >= dy:
                error += dy
                x0 += sx
            if error2 <= dx:
                error += dx
                y0 += sy

    def _rect(self, left: int, top: int, right: int, bottom: int, color: tuple[int, int, int]) -> None:
        self._line(left, top, right, top, color)
        self._line(right, top, right, bottom, color)
        self._line(right, bottom, left, bottom, color)
        self._line(left, bottom, left, top, color)

    def _text_width(self, text: str, scale: int = 1) -> int:
        width = 0
        for char in text.upper():
            glyph = self.FONT_5X7.get(char, self.FONT_5X7[" "])
            width += (len(glyph[0]) + 1) * scale
        return max(width - scale, 0)

    def _text(
        self,
        x: int,
        y: int,
        text: str,
        color: tuple[int, int, int],
        scale: int = 1,
        angle: int = 0,
    ) -> None:
        cursor = 0
        for char in text.upper():
            glyph = self.FONT_5X7.get(char, self.FONT_5X7[" "])
            self._glyph(x, y, glyph, color, scale, cursor, angle)
            cursor += (len(glyph[0]) + 1) * scale

    def _glyph(
        self,
        x: int,
        y: int,
        glyph: list[str],
        color: tuple[int, int, int],
        scale: int,
        offset: int,
        angle: int,
    ) -> None:
        for row_index, row in enumerate(glyph):
            for col_index, pixel in enumerate(row):
                if pixel != "1":
                    continue
                for dy in range(scale):
                    for dx in range(scale):
                        px = offset + col_index * scale + dx
                        py = row_index * scale + dy
                        if angle == 90:
                            self._set_pixel(x + py, y - px, color)
                        else:
                            self._set_pixel(x + px, y + py, color)

    def _to_png(self) -> bytes:
        def chunk(chunk_type: bytes, data: bytes) -> bytes:
            checksum = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
            return len(data).to_bytes(4, "big") + chunk_type + data + checksum.to_bytes(4, "big")

        raw_rows = bytearray()
        row_size = self.width * 3
        for y in range(self.height):
            raw_rows.append(0)
            start = y * row_size
            raw_rows.extend(self.pixels[start:start + row_size])

        header = b"\x89PNG\r\n\x1a\n"
        ihdr = self.width.to_bytes(4, "big") + self.height.to_bytes(4, "big") + bytes([8, 2, 0, 0, 0])
        return header + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(bytes(raw_rows), 9)) + chunk(b"IEND", b"")


class Opt101App:
    def __init__(self, root: tk.Tk, serial_module) -> None:
        self.root = root
        self.serial = serial_module
        self.samples: list[Sample] = []
        self.start_time = time.time()
        self.scan_start_time: float | None = None
        self.esp32 = None
        self.is_running = True

        root.title("OPT101 - Lectura espectral")
        root.geometry("1120x760")
        root.minsize(840, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value=f"Abriendo {SERIAL_PORT} a {BAUD_RATE} baudios...")
        self.count_var = tk.StringVar(value="0 muestras")

        toolbar = ttk.Frame(root, padding=(10, 10, 10, 6))
        toolbar.pack(fill="x")

        ttk.Button(toolbar, text="Reiniciar", command=self.reset).pack(side="left")
        ttk.Button(toolbar, text="Guardar PNG", command=self.save_png_dialog).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Guardar CSV", command=self.save_csv_dialog).pack(side="left", padx=(8, 0))

        motor_controls = ttk.Frame(toolbar)
        motor_controls.pack(side="left", padx=(18, 0))
        ttk.Button(motor_controls, text="W -> Subir (Antihorario)", command=lambda: self.send_motor_command("w")).pack(side="left")
        ttk.Button(motor_controls, text="S -> Bajar (Horario)", command=lambda: self.send_motor_command("s")).pack(side="left", padx=(6, 0))
        ttk.Button(motor_controls, text="I -> Auto", command=lambda: self.send_motor_command("i")).pack(side="left", padx=(6, 0))
        ttk.Button(motor_controls, text="X -> Detener", command=lambda: self.send_motor_command("x")).pack(side="left", padx=(6, 0))

        ttk.Label(toolbar, textvariable=self.count_var).pack(side="right")

        plot_area = ttk.Frame(root, padding=(10, 0, 10, 10))
        plot_area.pack(fill="both", expand=True)
        plot_area.columnconfigure(0, weight=1)
        plot_area.rowconfigure(0, weight=1)
        plot_area.rowconfigure(1, weight=1)

        self.raw_plot = PlotCanvas(plot_area, "OPT101 sin correccion", "#6c757d", y_label="Intensidad Bruta (V)")
        self.raw_plot.grid(row=0, column=0, sticky="nsew", pady=(0, 8))

        self.corrected_plot = PlotCanvas(plot_area, "OPT101 con correccion por responsividad", "#0072b2", y_label="Intensidad Corregida (u.a.)")
        self.corrected_plot.grid(row=1, column=0, sticky="nsew")

        status = ttk.Label(root, textvariable=self.status_var, anchor="w", padding=(10, 0, 10, 10))
        status.pack(fill="x")

    def start(self) -> None:
        try:
            self.esp32 = self.serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
            self.root.after(2000, self._serial_ready)
            self.root.after(20, self.read_serial)
        except self.serial.SerialException as exc:
            self.status_var.set(f"No se pudo abrir {SERIAL_PORT}: {exc}")
            messagebox.showerror(
                "Puerto serial no disponible",
                f"No se pudo abrir {SERIAL_PORT}.\n\nRevisa que el ESP32 este conectado y que PlatformIO/Monitor Serial este cerrado.",
            )

    def _serial_ready(self) -> None:
        if self.esp32 is not None:
            self.esp32.reset_input_buffer()
            self.status_var.set(f"Leyendo {SERIAL_PORT} a {BAUD_RATE} baudios")

    def read_serial(self) -> None:
        if not self.is_running:
            return

        if self.esp32 is not None:
            for _ in range(50):
                raw_line = self.esp32.readline().decode("utf-8", errors="ignore").strip()
                if not raw_line:
                    break

                parsed = parse_adc_line(raw_line)
                if parsed is None:
                    self.status_var.set(raw_line)
                    continue

                raw, voltage = parsed
                now = time.time()
                if self.scan_start_time is None:
                    self.scan_start_time = now
                wavelength = wavelength_for_elapsed_time(now - self.scan_start_time)
                sample = Sample(
                    timestamp_s=now - self.start_time,
                    wavelength_nm=wavelength,
                    raw=raw,
                    voltage_v=voltage,
                )
                self.samples.append(sample)
                self.status_var.set(
                    f"{sample.wavelength_nm:7.2f} nm | raw={sample.raw:4d} | "
                    f"V={sample.voltage_v:.4f} sig={sample.signal_v:.4f} V | "
                    f"R={sample.responsivity_a_per_w:.3f} A/W | "
                    f"corr={sample.corrected_intensity:.4f}"
                )

            self.update_plots()

        self.root.after(20, self.read_serial)

    def update_plots(self) -> None:
        scan_samples = self.samples[-POINTS_PER_SCAN:]
        wavelengths = [item.wavelength_nm for item in scan_samples]
        raw_intensities = [item.intensity for item in scan_samples]
        corrected_intensities = [item.corrected_intensity for item in scan_samples]

        self.raw_plot.set_data(wavelengths, raw_intensities)
        self.corrected_plot.set_data(wavelengths, corrected_intensities)
        self.count_var.set(f"{len(self.samples)} muestras")

    def reset(self) -> None:
        self.samples.clear()
        self.start_time = time.time()
        self.scan_start_time = None
        if self.esp32 is not None:
            self.esp32.reset_input_buffer()
        self.raw_plot.clear()
        self.corrected_plot.clear()
        self.count_var.set("0 muestras")
        self.status_var.set("Lectura reiniciada")

    def prepare_new_scan(self) -> None:
        self.samples.clear()
        self.start_time = time.time()
        self.scan_start_time = self.start_time
        if self.esp32 is not None:
            self.esp32.reset_input_buffer()
        self.raw_plot.clear()
        self.corrected_plot.clear()
        self.count_var.set("0 muestras")

    def send_motor_command(self, command: str) -> None:
        if self.esp32 is None:
            messagebox.showwarning("Puerto serial no disponible", "No hay conexion activa con el ESP32.")
            return

        try:
            if command.lower() in {"w", "i"}:
                self.prepare_new_scan()
            self.esp32.write(f"{command}\n".encode("utf-8"))
            self.status_var.set(f"Comando enviado al ESP32: {command}")
        except self.serial.SerialException as exc:
            self.status_var.set(f"No se pudo enviar '{command}': {exc}")
            messagebox.showerror("Error serial", f"No se pudo enviar el comando '{command}'.\n\n{exc}")

    def save_csv_dialog(self) -> None:
        if not self.samples:
            messagebox.showinfo("Sin datos", "No hay muestras para guardar.")
            return

        selected = filedialog.asksaveasfilename(
            title="Guardar CSV",
            defaultextension=".csv",
            initialfile=CSV_OUTPUT.name,
            filetypes=[("CSV", "*.csv"), ("Todos los archivos", "*.*")],
        )
        if selected:
            save_csv(self.samples, Path(selected))
            self.status_var.set(f"CSV guardado en {selected}")

    def save_png_dialog(self) -> None:
        if not self.samples:
            messagebox.showinfo("Sin datos", "No hay muestras para guardar.")
            return

        selected = filedialog.asksaveasfilename(
            title="Guardar graficas PNG",
            defaultextension=".png",
            initialfile=PNG_OUTPUT.name,
            filetypes=[("PNG", "*.png"), ("Todos los archivos", "*.*")],
        )
        if selected:
            base_path = Path(selected)
            scan_samples = self.samples[-POINTS_PER_SCAN:]
            wavelengths = [item.wavelength_nm for item in scan_samples]
            raw_intensities = [item.intensity for item in scan_samples]
            corrected_intensities = [item.corrected_intensity for item in scan_samples]

            raw_path = base_path.with_name(f"{base_path.stem}_sin_correccion{base_path.suffix}")
            corrected_path = base_path.with_name(f"{base_path.stem}_con_correccion{base_path.suffix}")
            renderer = PngPlotRenderer()
            raw_path.write_bytes(
                renderer.draw_plot(
                    wavelengths,
                    raw_intensities,
                    (108, 117, 125),
                    title="OPT101 SIN CORRECCION",
                    y_label="INTENSIDAD BRUTA (V)",
                )
            )
            corrected_path.write_bytes(
                renderer.draw_plot(
                    wavelengths,
                    corrected_intensities,
                    (0, 114, 178),
                    title="OPT101 CON CORRECCION POR RESPONSIVIDAD",
                    y_label="INTENSIDAD CORREGIDA (U.A.)",
                )
            )
            self.status_var.set(f"PNG guardados: {raw_path.name}, {corrected_path.name}")

    def close(self) -> None:
        self.is_running = False
        if self.samples:
            save_csv(self.samples, CSV_OUTPUT)
            self.status_var.set(f"Datos guardados en {CSV_OUTPUT.resolve()}")
        if self.esp32 is not None:
            self.esp32.close()
        self.root.destroy()


def main() -> None:
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
        print("Ejecuta: C:/Python313/python.exe -m pip install pyserial")
        return

    root = tk.Tk()
    app = Opt101App(root, serial)
    app.start()
    root.mainloop()


if __name__ == "__main__":
    main()
