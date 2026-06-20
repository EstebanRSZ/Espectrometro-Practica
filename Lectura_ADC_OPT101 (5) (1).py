"""
Espectrómetro OPT101 - Adquisición, Corrección Espectral y Control de Motor.

Modificaciones:
  - Panel de control para el motor (Subir, Bajar, Parar) enviando comandos UART.
  - Monitor en tiempo real de Voltaje/Corriente (Calculada con Rf=1MΩ).
  - Ajuste dinámico del Umbral de Disparo (Trigger) para lidiar con luz ambiente.
"""

import argparse
import csv
from datetime import datetime
import math
import pathlib
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# ==================== Configuración Global ====================
SERIAL_PORT = "COM7"
BAUD_RATE = 115200
CSV_OUTPUT = "espectro_opt101.csv"

# Parámetros físicos del barrido
TIEMPO_BARRIDO_TOTAL_S = 10.6   
WAVELENGTH_START_NM = 400.0    
WAVELENGTH_END_NM = 700.0
RANGO_NM = WAVELENGTH_END_NM - WAVELENGTH_START_NM
FACTOR_NM_POR_SEGUNDO = RANGO_NM / TIEMPO_BARRIDO_TOTAL_S  

V_DARK_V = 0.008  # Ruido base interno del integrado en oscuridad absoluta

# Tabla de Responsividad Espectral del OPT101 (Datasheet)
RESPONSIVITY_DATA = [
    (300, 0.05), (350, 0.10), (400, 0.19), (450, 0.25),
    (500, 0.32), (550, 0.37), (600, 0.42), (650, 0.46),
    (700, 0.50), (750, 0.54), (800, 0.57), (850, 0.60),
    (900, 0.60), (950, 0.53), (1000, 0.42), (1050, 0.26), (1100, 0.12)
]

BG = "#121212"
FG = "#E0E0E0"
ACCENT_RAW = "#FF5722"  
ACCENT_CORR = "#00E5FF" 


# ==================== Funciones Matemáticas ====================

def interpolate_responsivity(wavelength: float) -> float:
    if wavelength <= RESPONSIVITY_DATA[0][0]:
        return RESPONSIVITY_DATA[0][1]
    if wavelength >= RESPONSIVITY_DATA[-1][0]:
        return RESPONSIVITY_DATA[-1][1]
    
    for i in range(len(RESPONSIVITY_DATA) - 1):
        w0, r0 = RESPONSIVITY_DATA[i]
        w1, r1 = RESPONSIVITY_DATA[i+1]
        if w0 <= wavelength <= w1:
            return r0 + (r1 - r0) * (wavelength - w0) / (w1 - w0)
    return 0.45


class Sample:
    def __init__(self, raw: int, voltage_v: float, timestamp_s: float, wavelength_nm: float = 0.0):
        self.raw = raw
        self.voltage_v = voltage_v
        self.timestamp_s = timestamp_s  
        self.wavelength_nm = wavelength_nm

    @property
    def current_ua(self) -> float:
        """Calcula la fotocorriente real en microamperios (I = V / Rf), Rf = 1 MΩ."""
        return (self.voltage_v / 1.0) # V / 1 MOhm = uA

    @property
    def signal_v(self) -> float:
        val = self.voltage_v - V_DARK_V
        return max(val, 0.0)

    @property
    def corrected_intensity(self) -> float:
        """Aplica la teoría de corrección mitigando el ruido base del extremo azul."""
        # Si el voltaje útil es extremadamente bajo (ruido de cuantización),
        # lo forzamos a cero para evitar que la división por R_I lo infle falsamente.
        if self.signal_v < 0.015:  # Umbral de confianza de 15 mV
            return 0.0
            
        r_i = interpolate_responsivity(self.wavelength_nm)
        if r_i <= 0:
            return 0.0
        return self.signal_v / r_i


# ==================== Almacenamiento ====================


def save_csv(samples: list[Sample], filename: str) -> None:
    try:
        path = pathlib.Path(filename)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_S", "Wavelength_nm", "Raw_ADC", "Voltage_V", "Corrected_Intensity"])
            for s in samples:
                if WAVELENGTH_START_NM <= s.wavelength_nm <= WAVELENGTH_END_NM:
                    writer.writerow([
                        f"{s.timestamp_s:.3f}", f"{s.wavelength_nm:.1f}",
                        s.raw, f"{s.voltage_v:.4f}", f"{s.corrected_intensity:.4f}"
                    ])
    except Exception as e:
        print(f"Error al escribir CSV: {e}")


# ==================== Interfaz Gráfica ====================

class Opt101App:
    def __init__(self, root: tk.Tk, serial_module, port: str):
        self._root = root
        self._serial_mod = serial_module
        self._port = port
        self._esp32 = None
        
        self.samples: list[Sample] = []
        self.trigger_activated = False
        self.t_inicio_espectro = 0.0
        self.start_app_time = time.time()
        
        # Umbral por defecto inicial (Modificable desde la UI)
        self.umbral_trigger_v = 0.050 
        
        self._root.title("Espectrómetro OPT101 - Control y Adquisición")
        self._root.geometry("1150x750")
        self._root.configure(bg=BG)
        
        self._setup_styles()
        self._create_widgets()
        self._init_plots()
        self._connect_serial()
        
        self._poll_id = self._root.after(10, self._poll_serial)

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("default")
        style.configure(".", background=BG, foreground=FG)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=FG, font=("Consolas", 10))
        style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=5)
        style.configure("Action.TButton", font=("Segoe UI", 10, "bold"), foreground="#121212")

    def _create_widgets(self) -> None:
        main_frame = ttk.Frame(self._root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ---------------- Panel Superior: Monitores de Entrada Real ----------------
        top_bar = ttk.Frame(main_frame)
        top_bar.pack(fill=tk.X, side=tk.TOP, pady=(0, 10))

        # Sección de monitoreo de luz ambiente
        monitor_frame = ttk.LabelFrame(top_bar, text=" Monitoreo del Sensor en Vivo (Filtro Luz Ambiente) ", padding=5)
        monitor_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))

        self.lbl_live_voltage = ttk.Label(monitor_frame, text="Voltaje: --- V", font=("Consolas", 12, "bold"), foreground="#FFEB3B")
        self.lbl_live_voltage.pack(side=tk.LEFT, padx=15)

        self.lbl_live_current = ttk.Label(monitor_frame, text="Fotocorriente: --- µA", font=("Consolas", 12, "bold"), foreground="#4CAF50")
        self.lbl_live_current.pack(side=tk.LEFT, padx=15)

        self.lbl_live_lambda = ttk.Label(monitor_frame, text="λ Asignada: --- nm", font=("Consolas", 12, "bold"), foreground=ACCENT_CORR)
        self.lbl_live_lambda.pack(side=tk.LEFT, padx=15)

        # Ajuste dinámico del Trigger
        trigger_ctrl_frame = ttk.Frame(top_bar)
        trigger_ctrl_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        ttk.Label(trigger_ctrl_frame, text="Ajustar Umbral Trigger (V): ").pack(side=tk.TOP, anchor="w")
        self.spin_trigger = tk.Spinbox(trigger_ctrl_frame, from_=0.01, to=2.0, increment=0.01, 
                                       width=8, bg="#222222", fg=FG, buttonbackground="#333333",
                                       command=self._update_trigger_threshold)
        self.spin_trigger.delete(0, "end")
        self.spin_trigger.insert(0, f"{self.umbral_trigger_v:.3f}")
        self.spin_trigger.pack(side=tk.BOTTOM, pady=2)

        # ---------------- Panel Izquierdo: Controles del Motor ----------------
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)

        motor_frame = ttk.LabelFrame(content_frame, text=" Control de Motor Paso a Paso ", padding=10)
        motor_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        ttk.Button(motor_frame, text="▲ SUBIR (Giro+)", style="Action.TButton", 
                   command=lambda: self._send_motor_command("U")).pack(fill=tk.X, pady=8)
                   
        ttk.Button(motor_frame, text="■ PARAR MOTOR", style="Action.TButton", 
                   command=lambda: self._send_motor_command("S")).pack(fill=tk.X, pady=8)
                   
        ttk.Button(motor_frame, text="▼ BAJAR (Giro-)", style="Action.TButton", 
                   command=lambda: self._send_motor_command("D")).pack(fill=tk.X, pady=8)

        # Separador visual e instrucciones de control
        ttk.Separator(motor_frame, orient="horizontal").pack(fill=tk.X, pady=15)
        
        instrucciones = (
            "Instrucciones:\n"
            "1. Use bajar/subir para\n"
            "situar el haz en el\n"
            "extremo oscuro previo\n"
            "al azul.\n"
            "2. Ajuste el Umbral de\n"
            "manera que quede justo\n"
            "arriba del voltaje de\n"
            "la luz ambiente.\n"
            "3. Presione Reiniciar\n"
            "y comience el barrido."
        )
        ttk.Label(motor_frame, text=instrucciones, font=("Segoe UI", 9), justify=tk.LEFT).pack(anchor="w")

        # Contenedor de la gráfica (Derecha)
        self.plot_frame = ttk.Frame(content_frame)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ---------------- Barra Inferior de Funciones ----------------
        bot_bar = ttk.Frame(main_frame)
        bot_bar.pack(fill=tk.X, side=tk.BOTTOM, pady=(10, 0))

        self.lbl_status = ttk.Label(bot_bar, text="Iniciando...", font=("Segoe UI", 10, "italic"))
        self.lbl_status.pack(side=tk.LEFT)

        ttk.Button(bot_bar, text="Reiniciar Captura Espectral", command=self._reset_capture).pack(side=tk.RIGHT, padx=5)
        ttk.Button(bot_bar, text="Exportar Gráfica (PNG)", command=self._export_png).pack(side=tk.RIGHT, padx=5)
        
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_plots(self) -> None:
        self.fig, (self.ax_raw, self.ax_corr) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        self.fig.patch.set_facecolor(BG)

        for ax in (self.ax_raw, self.ax_corr):
            ax.set_facecolor("#1A1A1A")
            ax.xaxis.label.set_color(FG)
            ax.yaxis.label.set_color(FG)
            ax.tick_params(colors=FG, labelsize=9)
            ax.grid(True, color="#333333", linestyle="--")
            ax.set_xlim(WAVELENGTH_START_NM - 20, WAVELENGTH_END_NM + 20)

        self.ax_raw.set_ylabel("Voltaje Crudo (V)", color=ACCENT_RAW)
        self.ax_corr.set_ylabel("Intensidad Corregida (u.a.)", color=ACCENT_CORR)
        self.ax_corr.set_xlabel("Longitud de Onda (nm)")

        self.line_raw, = self.ax_raw.plot([], [], color=ACCENT_RAW, lw=1.5, label="V_out")
        self.line_corr, = self.ax_corr.plot([], [], color=ACCENT_CORR, lw=1.5, label="I_corregida")
        
        # Línea guía interactiva del trigger en el plot superior
        self.hl_trigger = self.ax_raw.axhline(self.umbral_trigger_v, color="#FFEB3B", linestyle=":", alpha=0.7, label="Umbral")

        self.ax_raw.legend(loc="upper right", facecolor="#222222", edgecolor="none", labelcolor=FG)
        self.ax_corr.legend(loc="upper right", facecolor="#222222", edgecolor="none", labelcolor=FG)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _connect_serial(self) -> None:
        if self._serial_mod is None:
            self._set_status("Error: Módulo serial no disponible.")
            return
        try:
            self._esp32 = self._serial_mod.Serial(self._port, BAUD_RATE, timeout=0.05)
            if self._esp32 is not None and self._esp32.is_open:
                self._set_status("Conectado con éxito al ESP32.")
            else:
                self._set_status(f"No se pudo abrir {self._port}.")
        except Exception as e:
            self._set_status(f"Puerto {self._port} no disponible. Conecta el ESP32 o cambia el puerto.")
            messagebox.showwarning(
                "Sin conexión serial",
                f"No se pudo abrir {self._port}. Verifica que el ESP32 esté conectado y que sea el COM4 correcto.\n\nDetalle: {e}"
            )

    def _send_motor_command(self, cmd: str) -> None:
        """Envía comandos compatibles con el firmware actual del ESP32."""
        if self._esp32 is not None and self._esp32.is_open:
            try:
                # El firmware actual de la placa responde a: w / s / x
                # El panel GUI usa U / D / S, así que lo mapeamos aquí.
                mapped_cmd = {
                    "U": "w",
                    "D": "s",
                    "S": "x",
                }.get(cmd, cmd)

                self._esp32.write(mapped_cmd.encode("utf-8"))
                self._set_status(f"Comando del motor enviado: {cmd} → {mapped_cmd}")
            except Exception as e:
                print(f"Error enviando comando: {e}")
        else:
            messagebox.showwarning("Sin Conexión", "El puerto serial no está abierto.")

    def _update_trigger_threshold(self) -> None:
        try:
            val = float(self.spin_trigger.get())
            self.umbral_trigger_v = val
            self.hl_trigger.set_ydata([val, val]) # Actualiza la línea en la gráfica
            self.canvas.draw_idle()
        except ValueError:
            pass

    def _set_status(self, text: str) -> None:
        self.lbl_status.config(text=text)

    def _reset_capture(self) -> None:
        self.samples.clear()
        self.trigger_activated = False
        self.t_inicio_espectro = 0.0
        self.start_app_time = time.time()
        
        self.line_raw.set_data([], [])
        self.line_corr.set_data([], [])
        self.canvas.draw()
        self._set_status("Captura reiniciada. Esperando cruce de umbral de luz...")

    def _poll_serial(self) -> None:
        if self._esp32 is not None and self._esp32.is_open:
            try:
                while self._esp32.in_waiting > 0:
                    line = self._esp32.readline().decode("utf-8", errors="ignore").strip()
                    if not line or "raw:" not in line:
                        continue
                    
                    if "|" in line:
                        try:
                            parts = line.split("|")
                            raw_part = parts[0].split("raw:")[1].strip()
                            v_part = parts[1].replace("V", "").strip()
                            
                            raw_val = int(raw_part)
                            v_val = float(v_part)
                            
                            t_actual = time.time() - self.start_app_time
                            sample = Sample(raw=raw_val, voltage_v=v_val, timestamp_s=t_actual)
                            
                            # --- CONTROL DEL TRIGGER DINÁMICO ---
                            if not self.trigger_activated:
                                if v_val >= self.umbral_trigger_v:
                                    self.trigger_activated = True
                                    self.t_inicio_espectro = t_actual
                                    self._set_status("¡Barrido óptico detectado superando la luz ambiente!")
                            
                            if self.trigger_activated:
                                delta_t = t_actual - self.t_inicio_espectro
                                lambda_dinamica = WAVELENGTH_START_NM + (delta_t * FACTOR_NM_POR_SEGUNDO)
                                sample.wavelength_nm = lambda_dinamica
                            else:
                                sample.wavelength_nm = 380.0  # Fuera del plot visible
                            
                            self.samples.append(sample)
                            
                            # Actualización de los indicadores en vivo solicitados
                            self.lbl_live_voltage.config(text=f"Voltaje: {sample.voltage_v:.3f} V")
                            self.lbl_live_current.config(text=f"Fotocorriente: {sample.current_ua:.3f} µA")
                            if self.trigger_activated and sample.wavelength_nm <= WAVELENGTH_END_NM:
                                self.lbl_live_lambda.config(text=f"λ Asignada: {sample.wavelength_nm:.1f} nm")
                            else:
                                self.lbl_live_lambda.config(text="λ Asignada: --- nm (Oscuro)")
                                
                        except (ValueError, IndexError):
                            pass
                
                if self.samples:
                    self._refresh_plots()
                    
            except Exception as e:
                print(f"Error de lectura serial: {e}")
                
        self._poll_id = self._root.after(10, self._poll_serial)

    def _refresh_plots(self) -> None:
        valid_samples = [s for s in self.samples if WAVELENGTH_START_NM <= s.wavelength_nm <= WAVELENGTH_END_NM]
        if not valid_samples:
            return

        x_wavelen = [s.wavelength_nm for s in valid_samples]
        y_raw = [s.voltage_v for s in valid_samples]
        y_corr = [s.corrected_intensity for s in valid_samples]

        self.line_raw.set_data(x_wavelen, y_raw)
        self.line_corr.set_data(x_wavelen, y_corr)

        self.ax_raw.set_ylim(0, max(y_raw) * 1.1 if max(y_raw) > 0 else 1.0)
        self.ax_corr.set_ylim(0, max(y_corr) * 1.1 if max(y_corr) > 0 else 1.0)
        self.canvas.draw_idle()

    def _export_png(self) -> None:
        if not self.samples:
            return
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"espectro_laboratorio_{timestamp}.png"
            self.fig.savefig(filename, dpi=150, facecolor=BG)
            self._set_status(f"Imagen guardada: {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar la imagen: {e}")

    def _on_close(self) -> None:
        if self._poll_id is not None:
            try:
                self._root.after_cancel(self._poll_id)
            except Exception:
                pass

        if self.samples:
            try:
                save_csv(self.samples, CSV_OUTPUT)
            except Exception as e:
                print(f"No se pudo exportar CSV al cerrar: {e}")

        if self._esp32 is not None:
            try:
                if hasattr(self._esp32, "is_open") and self._esp32.is_open:
                    self._esp32.close()
            except Exception:
                pass

        self._root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Módulo de Espectroscopía y Control de Motores.")
    parser.add_argument("port", nargs="?", default=SERIAL_PORT)
    args = parser.parse_args()

    try:
        import serial
    except ModuleNotFoundError:
        print("Instale pyserial usando: pip install pyserial matplotlib")
        sys.exit(1)

    root = tk.Tk()
    Opt101App(root, serial, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()