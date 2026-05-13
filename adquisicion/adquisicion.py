"""
Script principal de adquisicion en tiempo real.

Flujo:
  1. Abre el puerto serial del ESP32 (115200 baud).
  2. Lee lineas; ignora las que empiezan con '#' salvo '#DARK:' y '#SWEEP_END'.
  3. Parsea pares "lambda_nm,V_raw" y los acumula.
  4. Al recibir '#SWEEP_END': resta dark, aplica correccion OPT101,
     grafica el espectro final y guarda un CSV con timestamp.

Uso:
    python adquisicion.py --puerto COM3          (Windows)
    python adquisicion.py --puerto /dev/ttyUSB0  (Linux/Mac)
    python adquisicion.py --puerto /dev/ttyUSB0 --demo
"""

import argparse
import sys
import os
import csv
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Agregar el directorio padre al path para importar correccion_espectral
sys.path.insert(0, str(Path(__file__).parent))
from correccion_espectral import corregir_espectro, s_norm

# ── Constantes ────────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
DIR_DATOS   = Path(__file__).parent.parent / 'datos'
DIR_DATOS.mkdir(exist_ok=True)


# ── Modo demo: genera datos sinteticos de un flash LED blanco ──────────────────

def _generar_datos_demo():
    """Simula un espectro de flash LED blanco con ruido."""
    rng = np.random.default_rng(42)
    lambdas = np.linspace(400, 700, 400)
    # Espectro LED blanco tipico: pico azul ~450 nm + fosforo verde-amarillo ~550 nm
    spd = (
        2.5 * np.exp(-0.5 * ((lambdas - 450) / 18) ** 2) +   # pico azul bomba
        1.0 * np.exp(-0.5 * ((lambdas - 550) / 60) ** 2) +   # fosforo
        0.4 * np.exp(-0.5 * ((lambdas - 610) / 40) ** 2)     # hombro rojo
    )
    # Aplicar respuesta OPT101 para simular lo que mediria el sensor
    v_dark  = 120
    v_raw   = (spd * s_norm(lambdas) * 2000).astype(int) + v_dark
    v_raw  += rng.integers(-15, 15, size=len(lambdas))
    return lambdas, v_raw, v_dark


class AdquisicionDemo:
    """Fuente de datos sintetica para pruebas sin hardware."""

    def __init__(self):
        self._lambdas, self._v_raw, self._v_dark = _generar_datos_demo()
        self._idx = 0
        self.v_dark = float(self._v_dark)

    def readline(self):
        if self._idx == 0:
            return b'#HOMING\n'
        if self._idx == 1:
            return f'#DARK:{int(self.v_dark)}\n'.encode()
        if self._idx == 2:
            return b'#SWEEP_START\n'
        i = self._idx - 3
        if i < len(self._lambdas):
            line = f'{self._lambdas[i]:.2f},{self._v_raw[i]}\n'
            self._idx += 1
            return line.encode()
        self._idx += 1
        return b'#SWEEP_END\n'

    def write(self, data):
        pass

    def close(self):
        pass

    def __iter__(self):
        while True:
            line = self.readline()
            self._idx += 1
            yield line.decode('ascii', errors='replace')
            if b'SWEEP_END' in line:
                break


# ── Clase principal de adquisicion ────────────────────────────────────────────

class Espectrometro:

    def __init__(self, puerto: str, demo: bool = False):
        self.puerto = puerto
        self.demo   = demo
        self.ser    = None

        self.lambdas:  list[float] = []
        self.v_raws:   list[int]   = []
        self.v_dark:   float       = 0.0
        self.en_sweep: bool        = False

        # Figura matplotlib para actualizacion en tiempo real
        plt.ion()
        self.fig, (self.ax_raw, self.ax_corr) = plt.subplots(
            2, 1, figsize=(10, 7), sharex=True
        )
        self.fig.suptitle('Espectrometro de Barrido Angular — Adquisicion en vivo')
        self._configurar_ejes()
        self.fig.tight_layout()
        self.fig.canvas.draw()

    def _configurar_ejes(self):
        for ax in (self.ax_raw, self.ax_corr):
            ax.set_xlabel('Longitud de onda (nm)')
            ax.set_xlim(380, 720)
            ax.grid(True, alpha=0.3)
            _pintar_visible(ax)
        self.ax_raw.set_ylabel('ADC raw (−dark)')
        self.ax_raw.set_title('Señal medida (sin correccion)')
        self.ax_corr.set_ylabel('Intensidad corregida (u.a.)')
        self.ax_corr.set_title('Espectro corregido I_real = I_medida / S_norm(λ)')

    def abrir_puerto(self):
        if self.demo:
            print('[DEMO] Usando datos sinteticos — no se requiere hardware.')
            self.ser = AdquisicionDemo()
            return
        try:
            import serial
            self.ser = serial.Serial(self.puerto, BAUD_RATE, timeout=2)
            print(f'Puerto {self.puerto} abierto a {BAUD_RATE} baud.')
        except ImportError:
            sys.exit('Error: instala pyserial →  pip install pyserial')
        except Exception as e:
            sys.exit(f'Error abriendo {self.puerto}: {e}')

    def _procesar_linea(self, linea: str) -> bool:
        """
        Procesa una linea del serial.
        Devuelve True cuando se recibe '#SWEEP_END'.
        """
        linea = linea.strip()
        if not linea:
            return False

        if linea.startswith('#'):
            print(f'  [estado] {linea}')
            if linea.startswith('#DARK:'):
                try:
                    self.v_dark = float(linea.split(':')[1])
                except ValueError:
                    pass
            elif linea == '#SWEEP_START':
                self.en_sweep = True
                self.lambdas.clear()
                self.v_raws.clear()
            elif linea == '#SWEEP_END':
                return True
            return False

        if not self.en_sweep:
            return False

        try:
            parts = linea.split(',')
            lam   = float(parts[0])
            v_raw = int(parts[1])
            self.lambdas.append(lam)
            self.v_raws.append(v_raw)

            # Actualizar grafica cada 20 puntos
            if len(self.lambdas) % 20 == 0:
                self._actualizar_grafica_live()
        except (ValueError, IndexError):
            pass  # linea malformada; ignorar

        return False

    def _actualizar_grafica_live(self):
        lam  = np.array(self.lambdas)
        vraw = np.array(self.v_raws, dtype=float)
        i_medida = np.maximum(vraw - self.v_dark, 0.0)
        i_corr   = corregir_espectro(lam, i_medida)

        self.ax_raw.cla()
        self.ax_corr.cla()
        self._configurar_ejes()

        self.ax_raw.plot(lam, i_medida, 'b-', linewidth=0.8)
        self.ax_corr.plot(lam, i_corr, 'r-', linewidth=0.8)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def adquirir(self):
        """Bucle principal de lectura serial."""
        self.abrir_puerto()
        print('Esperando datos del ESP32... (Ctrl+C para cancelar)\n')
        try:
            for linea in self.ser:
                if isinstance(linea, bytes):
                    linea = linea.decode('ascii', errors='replace')
                sweep_terminado = self._procesar_linea(linea)
                if sweep_terminado:
                    break
        except KeyboardInterrupt:
            print('\nAdquisicion interrumpida por el usuario.')
        finally:
            if self.ser:
                self.ser.close()

        if self.lambdas:
            self._finalizar()

    def _finalizar(self):
        lam      = np.array(self.lambdas)
        vraw     = np.array(self.v_raws, dtype=float)
        i_medida = np.maximum(vraw - self.v_dark, 0.0)
        i_corr   = corregir_espectro(lam, i_medida)

        # Grafica final con ambos espectros
        self.ax_raw.cla()
        self.ax_corr.cla()
        self._configurar_ejes()

        self.ax_raw.plot(lam, i_medida, 'b-', linewidth=1.2, label='Medido')
        self.ax_raw.legend()
        self.ax_corr.plot(lam, i_corr, 'r-', linewidth=1.2, label='Corregido')
        self.ax_corr.legend()

        self.fig.canvas.draw()
        plt.ioff()

        # Guardar CSV
        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = DIR_DATOS / f'medicion_{ts}.csv'
        _guardar_csv(csv_path, lam, vraw, i_medida, i_corr, self.v_dark)
        print(f'\nDatos guardados en: {csv_path}')
        print(f'  {len(lam)} puntos  |  dark = {self.v_dark:.0f}')

        # Mostrar estadisticas basicas
        if len(i_corr) > 0:
            i_peak = lam[np.argmax(i_corr)]
            print(f'  Pico espectral corregido: {i_peak:.1f} nm')

        plt.show()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pintar_visible(ax):
    """Colorea las bandas del espectro visible como fondo."""
    colores = [
        (380, 450, '#8B00FF'),
        (450, 495, '#0000FF'),
        (495, 570, '#00CC00'),
        (570, 590, '#FFFF00'),
        (590, 620, '#FF8000'),
        (620, 700, '#FF0000'),
    ]
    for l0, l1, c in colores:
        ax.axvspan(l0, l1, alpha=0.07, color=c)


def _guardar_csv(ruta, lam, v_raw, i_medida, i_corr, v_dark):
    with open(ruta, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['# v_dark', v_dark])
        w.writerow(['lambda_nm', 'v_raw', 'i_medida', 'i_corregida'])
        for row in zip(lam, v_raw.astype(int), i_medida, i_corr):
            w.writerow([f'{row[0]:.3f}', int(row[1]),
                        f'{row[2]:.2f}', f'{row[3]:.4f}'])


# ── Punto de entrada ──────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(
        description='Adquisicion de espectro — Espectrometro de Barrido Angular'
    )
    p.add_argument('--puerto', default='COM3',
                   help='Puerto serial del ESP32 (ej. COM3 o /dev/ttyUSB0)')
    p.add_argument('--demo', action='store_true',
                   help='Modo demo con datos sinteticos (sin hardware)')
    return p.parse_args()


if __name__ == '__main__':
    args = _parse_args()
    esp = Espectrometro(puerto=args.puerto, demo=args.demo)
    esp.adquirir()
